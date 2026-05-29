/**
 * @name:    Driving.cpp
 * @date:    30.03.2026
 * @authors: Vincent Rohkamm, Florian Wiesner
 * @details: Implementation of the Driving class.
 */
#include "Driving.h"
#include <Vcameras.h>

#ifdef _MSC_VER
#pragma region DEBUG
#endif

//#define DEBUG_RAMP
//#define DEBUG_RAMP_ARRAY
//#define DEBUG_X64
//#define DEBUG_DRIVING
//#define DEBUG_TURN
//#define DEBUG_DRIVING_1

#ifdef _MSC_VER
#pragma endregion DEBUG
#pragma region TURN
#endif

ErrorCodes Driving::StartTurn(float angle) {
	registeredBumps = 0;

	if (angle > 360)
		return ErrorCodes::ERROR;
	
	#ifdef DEBUG_TURN
		Serial.println("Starting turn");
		Serial.print("TargetAngle: ");
		Serial.println(angle);
	#endif
	
	ts_startTime = millis();
	DisableBumpers();	// Disable bumpers during turn

	if (!_CAM_ALERT_TURN) maxTurnTime = DEFAULT_MAX_TURN_TIME;

	_TURNING = true;
	p_colorSensing->Freeze(true);

	p_gyro->GetAngleAdvanced(angle, GyroAxles::Axis_X);	// Read gyro to determine turn direction
	if (abs(p_gyro->data.angle_error) > 150) _TURN_180_DEGREE = true;
	else _TURN_180_DEGREE = false;	// Flag for 180° turn speed limiting
	return ErrorCodes::OK;
}

ErrorCodes Driving::ControlTurn(float angle) {
	// Check if the turn takes too long, e.g. when stuck on a bumper
	if (_CAM_ALERT_TURN) maxTurnTime = 10000;
	if (((millis() - ts_startTime) >= maxTurnTime))	turnTimeout = true;
	else turnTimeout = false;

	if ((p_gyro->data.angle_abs >= (angle + 0.5) || (p_gyro->data.angle_abs <= (angle - 0.5))) && !turnTimeout) {
		// Get the gyro readings and calculate the control data
		p_gyro->GetAngleAdvanced(angle, GyroAxles::Axis_X);
		#ifdef DEBUG_TURN
			Serial.print("Angle error: ");
			Serial.print(p_gyro->data.angle_error);
		#endif

		int16_t turnSpeed = CalculateTurnSpeed();

		#ifdef DEBUG_TURN
			Serial.print("Turn speed: ");
			Serial.println(turnSpeed);
		#endif

		// Set the speed
		p_drivetrain->SetSpeedLeft(turnSpeed);
		p_drivetrain->SetSpeedRight(-turnSpeed);
	}
	else {
		// Stop all motors
		p_drivetrain->Stop();
		_TURNING = false;

		#ifdef DEBUG_TURN
			Serial.print("Final angle: ");
			Serial.println(gyro.getAngle(GyroAxles::AXIS_X));
			Serial.print("Time: ");
			Serial.println(millis() - ts_startTime);
		#endif

		return ErrorCodes::TURNED;
	}
	return ErrorCodes::OK;
}

ErrorCodes Driving::EndTurn(void){
	EnableBumpers();	// Enable bumpers
	_CAM_ALERT_TURN = false;
	maxTurnTime = DEFAULT_MAX_TURN_TIME;
	StartAlign();

	p_mapSys->Turn(p_gyro->GetOrientationFromAngle());
	_TURNING = false;
	p_colorSensing->Freeze(false);
	UpdateRampProximityFlags();
	return ErrorCodes::OK;
}

ErrorCodes Driving::Turn180Degree(void) {
	float newAngle = p_gyro->GetAngle(GyroAxles::Axis_X) + 180.0;	// Target angle is current + 180°
	if (newAngle > 360) newAngle -= 360;	// Wrap to [0, 360)
	StartTurn(newAngle);

	while (ControlTurn(newAngle) == ErrorCodes::OK) delay(1);
	EndTurn();
	return ErrorCodes::OK;
}


int8_t Driving::SelectAlignSide(void){
	// Picks the closer side wall, sets distanceFront/distanceBack and coeffSide as side effects.
	// Returns 1 for left, -1 for right, 0 if no usable wall found on either side.
	uint16_t sumDistanceLeft  = p_tof->GetRange(TofType::LEFT_BACK)  + p_tof->GetRange(TofType::LEFT_FRONT);
	uint16_t sumDistanceRight = p_tof->GetRange(TofType::RIGHT_BACK) + p_tof->GetRange(TofType::RIGHT_FRONT);

	if (sumDistanceLeft <= sumDistanceRight && p_tof->GetRange(TofType::LEFT_BACK) < 200 && p_tof->GetRange(TofType::LEFT_FRONT) < 200) {
		distanceFront = p_tof->GetRange(TofType::LEFT_FRONT);
		distanceBack  = p_tof->GetRange(TofType::LEFT_BACK);
		return 1;
	}
	else if (sumDistanceLeft > sumDistanceRight && p_tof->GetRange(TofType::RIGHT_BACK) < 200 && p_tof->GetRange(TofType::RIGHT_FRONT) < 200) {
		distanceFront = p_tof->GetRange(TofType::RIGHT_FRONT);
		distanceBack  = p_tof->GetRange(TofType::RIGHT_BACK);
		return -1;
	}
	return 0;
}

int16_t Driving::CalculateTurnSpeed(void) {
	// Computes exponential turn speed from gyro error. Sets _CAM_ALERT_TURN if camera is alerting.
	int16_t turnSpeed = 0;
	float baseSpeed = 70.0;
	if (p_cams->IsAlert()) {
		baseSpeed = 30.0;	// Reduce speed when camera is on alert
		_CAM_ALERT_TURN = true;
	}

	if (p_gyro->data.direction_right)     turnSpeed = -(baseSpeed * (1 - pow(EULER,  p_gyro->data.angle_error / 25.0)) + 20.0);
	else if (p_gyro->data.direction_left) turnSpeed =  (baseSpeed * (1 - pow(EULER, -p_gyro->data.angle_error / 25.0)) + 20.0);

	if (!_TURN_180_DEGREE) turnSpeed = constrain(turnSpeed, -100, 100);
	else                   turnSpeed = constrain(turnSpeed, -60, 60);	// Limit speed during 180° turn
	return turnSpeed;
}

ErrorCodes Driving::StartAlign(void) {
	p_tof->Update();
	_TURNING = true;
	uint32_t startTime = millis();

	#ifdef DEBUG_DRIVING
		Serial.print("LB: "); Serial.print(p_tof->GetRange(TofType::LEFT_BACK));
		Serial.print("\tLF: "); Serial.print(p_tof->GetRange(TofType::LEFT_FRONT));
		Serial.print("\tRF: "); Serial.print(p_tof->GetRange(TofType::RIGHT_FRONT));
		Serial.print("\tRB: "); Serial.println(p_tof->GetRange(TofType::RIGHT_BACK));
	#endif

	coeffSide = SelectAlignSide();
	if (coeffSide == 0) {
		_TURNING = false;
		return ErrorCodes::NOT_ALIGNING;
	}

	#ifdef DEBUG_DRIVING
		Serial.print("\tSide: "); Serial.println(coeffSide == 1 ? "LEFT" : "RIGHT");
	#endif

	// Rotate until front/back sensor pair reads equal distance to the wall
	do {
		p_tof->Update();

		if (coeffSide == 1) {
			distanceFront = (uint8_t)p_tof->GetRange(TofType::LEFT_FRONT);
			distanceBack  = (uint8_t)p_tof->GetRange(TofType::LEFT_BACK);
		}
		else {
			distanceFront = (uint8_t)p_tof->GetRange(TofType::RIGHT_FRONT);
			distanceBack  = (uint8_t)p_tof->GetRange(TofType::RIGHT_BACK);
		}

		if (distanceFront >= 255 || distanceBack >= 255) {
			p_drivetrain->Stop();
			_TURNING = false;
			return ErrorCodes::NOT_ALIGNING;
		}

		distanceError = distanceFront - distanceBack;

		if      (distanceError > 0) turnSpeedAlign = 40.0f * (pow(EULER, (float)-distanceError / 20.0f) - 1.0f) - 20.0f;
		else if (distanceError < 0) turnSpeedAlign = 40.0f * (1 - pow(EULER, (float)abs(distanceError) / -20.0f)) + 20.0f;
		else                        turnSpeedAlign = 0;

		#ifdef DEBUG_DRIVING
			Serial.print("Error: "); Serial.print(distanceError);
			Serial.print("\tSpeed: "); Serial.println(turnSpeedAlign);
		#endif

		p_drivetrain->SetSpeedLeft( turnSpeedAlign * coeffSide);
		p_drivetrain->SetSpeedRight(-turnSpeedAlign * coeffSide);
	} while (abs(distanceError) >= 5 && millis() - startTime < 1500);

	p_drivetrain->Stop();
	_TURNING = false;
	return ErrorCodes::OK;
}

#ifdef _MSC_VER
#pragma endregion
#pragma region DRIVE
#endif

ErrorCodes Driving::StartDrive(bool rampDown) {
	registeredBumps = 0;	// Reset bump counter
	_DRIVE_TIMEOUT = false;
	_CAM_VICTIM = false;
	maxDriveTime = DEFAULT_MAX_DRIVE_TIME;
	ts_driveStartTime = millis();

	sensor = GetOptimalSensor(rampDown);
	lastTargetDistance = nextTargetDistance;
	nextTargetDistance = CalculateNextTargetDistance();

	#ifdef DEBUG_DRIVING
	Serial.print("Target: ");
	Serial.println(nextTargetDistance);
	#endif

	if (sensor.type == ReferenceObj::ENCODER) {
		p_drivetrain->EnableEncoder();	// Enable encoder interrupts
		p_drivetrain->ResetEncoder();
	}
	else p_drivetrain->DisableEncoder();	// Disable encoder interrupts

	return ErrorCodes::OK;
}

uint16_t Driving::CalculateNextTargetDistance(void) {
	// Selects the stop threshold based on active reference sensor and current readings.
	if (sensor.type == ReferenceObj::BACK)
		if (sensor.back > 250) return sensor.back + 300;
		else return 320;
	else if (sensor.type == ReferenceObj::FRONT)
		if (sensor.front > 410) return sensor.front - 300;
		else return 110;
	else
		return 310;
}

ErrorCodes Driving::ControlDrive(int8_t driveSpeed, float angle) {
	if (_SLOW_SPEED)
		driveSpeed = 25;
	p_gyro->GetAngleAdvanced(angle, GyroAxles::Axis_X);	// Read gyro for current angle and error
	int8_t leftRightError = p_tof->CalculateLeftRightError(p_gyro->data.angle_error, TOF_SIDE_WALL_THRESHOLD, GAP_ROBOT_WALL);
	float error = -p_gyro->data.angle_error + (leftRightError * PID_LEFT_RIGHT_FACTOR);

	if (error != 0) integralError += error;	// Accumulate integral error
	else integralError = 0;	// Reset integral on zero error
	derivativeError = error - pidLastError;	// Calculate derivative error
	pidLastError = error;

	PID_Coefficients coeff;
	if (ts_lastPID) coeff = CalculatePIDCoefficients((float)(millis() - ts_lastPID) / 1000);
	else coeff = CalculatePIDCoefficients(PID_LOOP_DURATION);
	#ifdef DEBUG_DRIVING
		Serial.print("Time: ");
		Serial.println((float)(millis() - ts_lastPID));
	#endif

	correctionSpeed = (coeff.P * error) + (coeff.I * integralError) + (coeff.D * derivativeError);
	correctionSpeed = constrain(correctionSpeed, (-driveSpeed * 0.33), (driveSpeed * 0.33));	// Limit correction to 33% of drive speed

	if (driveSpeed >= 90) driveSpeed = 90;
	if (_ON_RAMP) driveSpeed = rampSpeed;
	float powerLeft = driveSpeed - correctionSpeed;
	float powerRight = driveSpeed + correctionSpeed;

	p_drivetrain->SetSpeedLeft(powerLeft);
	p_drivetrain->SetSpeedRight(powerRight);

	ts_lastPID = millis();

	#ifdef DEBUG_DRIVING_1
	Serial.print("Angle: ");
	Serial.print(gyro.data.angle_error);
	Serial.print("\tOffest: ");
	Serial.print(leftRightError);
	Serial.print("\tError: ");
	Serial.print(error);
	Serial.print("\tTimestamp: ");
	Serial.println(millis());
	#endif

	if (_CAM_VICTIM) maxDriveTime = 12000;
	if ((!_ON_RAMP) && (millis() - ts_driveStartTime > maxDriveTime)) return ErrorCodes::TIMEOUT;
	else if (!_ON_RAMP) return ErrorCodes::CHECK_DRIVE;
	else return ErrorCodes::OK;
}

ErrorCodes Driving::CheckDrive(void) {
	if (sensor.type == ReferenceObj::ENCODER)
		newValue = p_drivetrain->GetEncoderDistance();
	else
		if (sensor.type == ReferenceObj::FRONT)
			newValue = p_tof->GetRange(TofType::FRONT);
		else if (sensor.type == ReferenceObj::BACK)
			newValue = p_tof->GetRange(TofType::BACK);

	if ((newValue >= nextTargetDistance && (sensor.type == ReferenceObj::BACK || sensor.type == ReferenceObj::ENCODER))
		|| (newValue <= nextTargetDistance && sensor.type == ReferenceObj::FRONT)) {
		if (nextTargetDistance < 100 && sensor.type == ReferenceObj::FRONT) {
			p_drivetrain->Stop();
		}
		return ErrorCodes::SCAN_DRIVE;
	}
	else return ErrorCodes::CHECK_RAMP;
}

ErrorCodes Driving::EndDrive(void) {
	ts_lastPID = 0;
	p_drivetrain->Stop();
	return ErrorCodes::OK;
}

ErrorCodes Driving::TimeoutDrive(void) {
	EndDrive();
	StartAlign();
	StartAdjustment();

	_DRIVE_TIMEOUT = false;
	return ErrorCodes::OK;
}

ErrorCodes Driving::StartAdjustment(void) {
	// Position robot to fixed front-wall distance before turning
	p_tof->Update();
	if (p_tof->GetRange(TofType::FRONT) > 120)	return ErrorCodes::ERROR;

	int8_t posError;
	do {
		p_tof->Update();
		posError = p_tof->GetRange(TofType::FRONT) - ADJUST_WALL_DISTANCE;
		int8_t adjustSpeed = posError * ADJUSTMENT_SPEED_FACTOR;	// Calculate adjustment speed

		p_drivetrain->SetSpeed(adjustSpeed);
	} while (abs(posError) > 10);	// Wait until within tolerance

	p_drivetrain->Stop();
	return ErrorCodes::OK;
}

ErrorCodes Driving::ReverseBlackTile(void) {
	p_drivetrain->EnableEncoder();
	p_drivetrain->ResetEncoder();
	while (p_drivetrain->GetEncoderDistance() < 150) {	// Drive back ~15 cm
		p_drivetrain->SetSpeed(-30);
	}
	p_drivetrain->Stop();
	p_drivetrain->DisableEncoder();	// Disable motor interrupts
	return ErrorCodes::OK;
}

TOF_Optimal_Value Driving::GetOptimalSensor(bool rampDown){
	// Selects the best reference sensor (front/back/encoder) for the upcoming drive segment.
	TOF_Optimal_Value result;
	lastSensor = sensor;

	result.front = p_tof->GetRange(TofType::FRONT);
	result.back  = p_tof->GetRange(TofType::BACK);

	if (_RAMP_INSTRUCTION) {
		_RAMP_INSTRUCTION = false;
		result.type = ReferenceObj::ENCODER;
		return result;
	}

	#ifdef DEBUG_X64
	Serial.print("RAMP_BEHIND: ");
	Serial.println(_RAMP_BEHIND);
	Serial.print("\tRAMP_INFRONT: ");
	Serial.println(_RAMP_INFRONT);
	#endif

	ApplyRampFlagOverrides(result);

	#ifdef DEBUG_X64
	Serial.print("front: ");
	Serial.println(result.front);
	Serial.print("\back: ");
	Serial.println(result.back);
	#endif

	if      (result.front <= 850 && result.back  >  550) result.type = ReferenceObj::FRONT;
	else if (result.front >= 850 && result.back  <= 550) result.type = ReferenceObj::BACK;
	else if (result.front >= 850 && result.back  >= 550) result.type = ReferenceObj::ENCODER;
	else if (result.front <  result.back)                result.type = ReferenceObj::FRONT;
	else if (result.front >  result.back)                result.type = ReferenceObj::BACK;

	if (rampDown) result.type = ReferenceObj::ENCODER;

	#ifdef DEBUG_DRIVING
	Serial.print("OPTIMAL");
	Serial.print("\tType: ");
	Serial.print(int(result.type));
	Serial.print("\tFront: ");
	Serial.print(result.front);
	Serial.print("\tBack: ");
	Serial.println(result.back);
	#endif
	return result;
}

void Driving::ApplyRampFlagOverrides(TOF_Optimal_Value& result){
	// Overrides front/back readings with 1023 when a ramp flag is active, then clears the flags.
	if (_RAMP_INFRONT)     result.front = 1023;
	else if (_RAMP_BEHIND) result.back  = 1023;
	_RAMP_INFRONT = false;
	_RAMP_BEHIND  = false;
}

PID_Coefficients Driving::CalculatePIDCoefficients(float loopDuration){
	// Derives Ziegler-Nichols P/I/D coefficients scaled by measured loop duration.
	if (loopDuration >= 2 * PID_LOOP_DURATION) loopDuration = PID_LOOP_DURATION;	// Clamp to prevent derivative kick on long gaps
	float time_coeff = loopDuration / PID_LOOP_DURATION;
	PID_Coefficients pid;

	pid.P = 0.6 * PID_CRITICAL_GAIN;
	pid.I = time_coeff * 2 * pid.P * PID_LOOP_DURATION / PID_OSCILLATION_PERIOD;
	pid.D = time_coeff * pid.P * PID_OSCILLATION_PERIOD / (8 * PID_LOOP_DURATION);
	#ifdef DEBUG_DRIVING_1
	Serial.println(pid.P);
	Serial.println(pid.I);
	Serial.println(pid.D);
	#endif
	return pid;
}

#ifdef _MSC_VER
#pragma endregion
#pragma region RAMPS
#endif

ErrorCodes Driving::RampHandler(void){
	if (!_ON_RAMP) {
		CheckRamp();
	}
	else {
		p_gyro->GetAngleAdvanced(0, GyroAxles::Axis_Z);
		float incline = -p_gyro->data.angle_car;

		RecordInclineSample(incline);

		p_gyro->GetAngleAdvanced(0, GyroAxles::Axis_Y);

		if (incline <= 4 && incline >= -4) {
			rampEncoderDistance = p_drivetrain->GetEncoderDistance();
			EndDrive();
			ClassifyAndFinishRamp();
			return ErrorCodes::RAMP_END;
		}
	}
	return ErrorCodes::OK;
}

bool Driving::CheckStairRamp(void) {
	float aggregatedIncline = 0.0;
	float sumIncline = 0.0;

	for (int i = 0; i < arrInclineIndex - 1; i++) {
		aggregatedIncline += abs(arrIncline[i] - arrIncline[i+1]);
		sumIncline += arrIncline[i];
	}
	avgIncline = sumIncline / arrInclineIndex;	// Calculate the average incline

	#ifdef DEBUG_RAMP
	Serial.print("Average Stair Incline: ");
	Serial.println(avgIncline);
	#endif

	#ifdef DEBUG_RAMP
	Serial.print("Aggregated Incline: ");
	Serial.println(aggregatedIncline);
	#endif

	if (aggregatedIncline > RAMP_STAIRS_THRESHOLD) {
		#ifdef DEBUG_RAMP
		Serial.println("Stair Ramp detected!");
		#endif
		return true;	// Stair ramp detected
	}
	else {
		#ifdef DEBUG_RAMP
		Serial.println("No Stair Ramp detected!");
		#endif
		return false;	// Smooth ramp
	}
}

void Driving::UpdateRampProximityFlags(void) {
	// Sets _RAMP_INFRONT/_RAMP_BEHIND based on the 8×8 ToF ramp detection.
	if (p_tof->IsRampThere(false)) {
		_RAMP_INFRONT = true;
		_RAMP_BEHIND  = false;
	}
	else if (p_tof->IsRampThere(true)) {
		_RAMP_BEHIND  = true;
		_RAMP_INFRONT = false;
	}
	else {
		_RAMP_INFRONT = false;
		_RAMP_BEHIND  = false;
	}
}

ErrorCodes Driving::FinishRamp(uint8_t distance){
	// Drives forward a fixed distance to clear the ramp end, then aligns, re-enables bumpers, and reads ramp flags.
	p_drivetrain->ResetEncoder(0);	// Reset encoder
	p_drivetrain->EnableEncoder();	// Enable encoder interrupts
	ts_encoderStartTime = millis();	// Record start time

	while (p_drivetrain->GetEncoderDistance() < distance && millis() < (ts_encoderStartTime + DEFAULT_MAX_ENCODER_TIME)) {	// Drive forward to clear the ramp
		p_drivetrain->SetSpeed(50);
	}
	p_drivetrain->Stop();
	p_drivetrain->DisableEncoder();	// Disable encoder interrupts

	StartAlign();		// Align robot
	EnableBumpers();	// Re-enable bumpers after ramp traversal
	UpdateRampProximityFlags();
	return ErrorCodes::OK;
}

ErrorCodes Driving::CheckRamp(void){
	// Reads incline from gyro, updates counters, then evaluates ramp detection.
	p_gyro->GetAngleAdvanced(0, GyroAxles::Axis_Z);
	float incline = -p_gyro->data.angle_car;

	UpdateInclineCounters(incline);
	EvaluateRampDecision();

	return ErrorCodes::OK;
}

void Driving::UpdateInclineCounters(float incline){
	// Updates incline/non-incline cycle counters and adjusts rampCheckDuration based on current incline reading.
	if (!_ON_RAMP && (incline > RAMP_THRESHOLD_ANGLE) && inclineCycleCounter == 0) {
		ts_rampStartTime = millis();
		inclineCycleCounter++;
		nonInclineCycleCounter = 0;
	}
	else if (!_ON_RAMP && (incline < -RAMP_THRESHOLD_ANGLE) && inclineCycleCounter == 0) {
		ts_rampStartTime = millis();
		inclineCycleCounter--;
		nonInclineCycleCounter = 0;
	}
	else if (!_ON_RAMP && ((incline >= RAMP_THRESHOLD_ANGLE) || (incline <= -RAMP_THRESHOLD_ANGLE))) {
		if (incline > 0) inclineCycleCounter++;
		else             inclineCycleCounter--;
		if      ((incline < 15)  && (incline > -15))                                             rampCheckDuration = 1000;
		else if (((incline >= 15) && (incline < 20)) || ((incline <= -15) && (incline > -20)))  rampCheckDuration = 500;
		else if ((incline >= 20) || (incline <= -20))                                            rampCheckDuration = 200;
	}
	else if (!_ON_RAMP && ((incline < RAMP_THRESHOLD_ANGLE) && (incline > -RAMP_THRESHOLD_ANGLE)))
		nonInclineCycleCounter++;
}

void Driving::EvaluateRampDecision(void){
	// After the confidence window expires, decides whether a ramp was detected and triggers all ramp-entry side effects.
	if (!_ON_RAMP && millis() > (ts_rampStartTime + rampCheckDuration) && ts_rampStartTime != 0) {
		if (inclineCycleCounter >= nonInclineCycleCounter * RAMP_CONFIDENCE) {
			inclineCycleCounter = 0;
			#ifdef DEBUG_RAMP
			Serial.println("RAMP UP DETECTED!");
			#endif
			_ON_RAMP   = true;
			_RAMP_UP   = true;
			_RAMP_DOWN = false;
			p_mapSys->Move(true);
			ts_rampStartTime    = 0;
			rampEncoderDistance = 0;
			rampSpeed           = RAMP_SPEED_UP;
			DisableBumpers();
			p_drivetrain->ResetEncoder(0);
			p_drivetrain->EnableEncoder();
			p_colorSensing->Freeze(true);
		}
		else if (-inclineCycleCounter >= nonInclineCycleCounter * RAMP_CONFIDENCE) {
			inclineCycleCounter = 0;
			#ifdef DEBUG_RAMP
			Serial.println("RAMP DOWN DETECTED!");
			#endif
			_ON_RAMP   = true;
			_RAMP_DOWN = true;
			_RAMP_UP   = false;
			if (millis() < ts_lastSetTile + MIN_SETTILE_TIME) {
				p_mapSys->RollbackOne();
				Serial.println("CORRECTED RAMP DOWN!");
			}
			p_mapSys->Move(true);
			ts_rampStartTime    = 0;
			rampEncoderDistance = 0;
			rampSpeed           = RAMP_SPEED_DOWN;
			DisableBumpers();
			p_drivetrain->ResetEncoder(0);
			p_drivetrain->EnableEncoder();
			p_colorSensing->Freeze(true);
		}
		else {
			inclineCycleCounter    = 0;
			nonInclineCycleCounter = 0;
			ts_rampStartTime       = 0;
			rampCheckDuration      = DEFAULT_RAMP_CHECK_DURATION;
			#ifdef DEBUG_RAMP
			Serial.println("NO RAMP!");
			Serial.println("inc: " + String(inclineCycleCounter) + " non: " + String(nonInclineCycleCounter));
			#endif
		}
	}
}

void Driving::RecordInclineSample(float incline){
	// Appends the current incline reading to the sample array and tracks the peak incline.
	arrIncline[arrInclineIndex] = incline;
	arrInclineIndex++;
	if (_RAMP_UP   && incline >  maxRampIncline) maxRampIncline = incline;
	else if (_RAMP_DOWN && incline <  maxRampIncline) maxRampIncline = incline;
}

void Driving::ClassifyAndFinishRamp(void){
	// Classifies stair vs smooth, finalises traversal, computes geometry, resets ramp state.
	#ifdef DEBUG_RAMP_ARRAY
	Serial.println("ARRAY");
	for (uint8_t i = 0; i < arrInclineIndex; i++) {
		Serial.println(arrIncline[i]);
	}
	#endif

	if (CheckStairRamp()) {
		FinishRamp(75);
		_STAIR = true;
	}
	else FinishRamp(60);

	for (uint16_t i = 0; i < INCLINE_ARRAY_SIZE; i++) arrIncline[i] = 0.0;

	#ifdef DEBUG_RAMP
	Serial.print("Raw: ");
	Serial.print(rampEncoderDistance);
	Serial.print("\tStair: ");
	Serial.print(_STAIR);
	#endif

	CalculateRampGeometry(_RAMP_UP, _RAMP_DOWN, _STAIR);

	_RAMP_UP   = false;
	_RAMP_DOWN = false;
	_STAIR     = false;
	p_colorSensing->Freeze(false);
	arrInclineIndex = 0;

	#ifdef DEBUG_RAMP
	Serial.print("\tDistance: ");
	Serial.print(rampHypotenuse);
	Serial.print("\tRamp angle: ");
	Serial.print(rampAngle);
	Serial.print("\tHeight: ");
	Serial.print(rampHeight);
	Serial.print("\tLength: ");
	Serial.println(rampLength);
	#endif
}

void Driving::CalculateRampGeometry(bool rampUp, bool rampDown, bool isStair){
	// Applies K/d correction factors and computes rampHypotenuse, rampHeight, rampLength.
	if (rampUp && !isStair) {
		rampEncoderDistance = rampEncoderDistance * RAMP_UP_K + RAMP_UP_D;
		rampAngle = maxRampIncline;
		#ifdef DEBUG_RAMP
		Serial.print("\tRAMP UP");
		#endif
	}
	else if (rampDown && !isStair) {
		rampEncoderDistance = rampEncoderDistance * RAMP_DOWN_K + RAMP_DOWN_D;
		rampAngle = maxRampIncline + 2;
		#ifdef DEBUG_RAMP
		Serial.print("\tRAMP DOWN");
		#endif
	}
	else if (rampUp && isStair) {
		rampEncoderDistance = rampEncoderDistance * STAIR_UP_K + STAIR_UP_D;
		rampAngle = avgIncline + STAIR_UP_ANGLE_OFFSET;
		#ifdef DEBUG_RAMP
		Serial.print("\tSTAIR UP");
		#endif
	}
	else if (rampDown && isStair) {
		rampEncoderDistance = rampEncoderDistance * STAIR_DOWN_K + STAIR_DOWN_D;
		rampAngle = avgIncline + STAIR_DOWN_ANGLE_OFFSET;
		#ifdef DEBUG_RAMP
		Serial.print("\tSTAIR DOWN");
		#endif
	}
	else rampEncoderDistance = 0;

	if (!isStair) rampEncoderDistance *= 0.95;

	rampHypotenuse = rampEncoderDistance;
	rampHeight     = rampEncoderDistance * sinf(rampAngle * PI / 180.0);
	rampLength     = rampEncoderDistance * cosf(rampAngle * PI / 180.0);
}

#ifdef _MSC_VER
#pragma endregion
#pragma region BUMPERS
#endif

ErrorCodes Driving::ReverseBumper(uint16_t distance, int8_t speedLeft, int8_t speedRight){
	uint32_t buffer_beginTime = millis();	// Log begin time

	p_drivetrain->EnableEncoder();
	p_drivetrain->ResetEncoder();
	while (p_drivetrain->GetEncoderDistance() < distance && buffer_beginTime + REVERSE_BUMPER_TIMEOUT > millis()) {	// Drive back the distance, check timeout
		p_drivetrain->SetSpeedLeft(speedLeft);
		p_drivetrain->SetSpeedRight(speedRight);
	}
	p_drivetrain->Stop();

	if (sensor.type != ReferenceObj::ENCODER) p_drivetrain->DisableEncoder();
	if(buffer_beginTime + REVERSE_BUMPER_TIMEOUT < millis()) return ErrorCodes::TIMEOUT;	// Timeout reached
	return ErrorCodes::OK;
}

ErrorCodes Driving::EnableBumpers(void){
	_ENABLE_BUMPERS = true;	// Enable bumpers
	return ErrorCodes::OK;
}

ErrorCodes Driving::DisableBumpers(void){
	_ENABLE_BUMPERS = false;	// Disable bumpers
	return ErrorCodes::OK;
}

ErrorCodes Driving::BumperHandler(void){
	if (_ENABLE_BUMPERS) {
		if (digitalRead(BUMPER_LEFT_PIN) || digitalRead(BUMPER_RIGHT_PIN)) {
			p_drivetrain->Stop();
			delay(200);

			registeredBumps += 1;

			if (registeredBumps > BUMPER_TRYS) return HandleWallCollision();

			float saveDistance = 0;
			if (sensor.type == ReferenceObj::ENCODER) {
				saveDistance = p_drivetrain->GetEncoderDistance();
			}

			ExecuteBumperManeuver();

			StartAlign();
			StartAdjustment();

			if (sensor.type == ReferenceObj::ENCODER) {
				p_drivetrain->ResetEncoder(saveDistance);
			}
			integralError   = 0;
			derivativeError = 0;

			return ErrorCodes::OK;
		}
	}
	else return ErrorCodes::BUMPER_DISABLED;
	return ErrorCodes::UNKNOWN;
}

ErrorCodes Driving::HandleWallCollision(void){
	// Registers a wall collision: updates the map, resets bump counter, backs away.
	p_mapSys->Bumper();
	registeredBumps = 0;
	ReverseBumper(40, -40, -40);
	return ErrorCodes::BUMPER_WALL;
}

void Driving::ExecuteBumperManeuver(void){
	// Executes the avoidance S-curve based on which bumper was triggered.
	if (digitalRead(BUMPER_LEFT_PIN) && digitalRead(BUMPER_RIGHT_PIN)) {
		ReverseBumper(30, -40, -40);
	}
	else if (digitalRead(BUMPER_LEFT_PIN)) {
		ReverseBumper(70, -50,  50);
		ReverseBumper(40, -50, -50);
		ReverseBumper(90,  50, -50);
	}
	else if (digitalRead(BUMPER_RIGHT_PIN)) {
		ReverseBumper(70,  50, -50);
		ReverseBumper(40, -50, -50);
		ReverseBumper(90, -50,  50);
	}
}

#ifdef _MSC_VER
#pragma endregion
#pragma region GENERAL
#endif

void Driving::OnVictimDetected(void) {
	integralError   = 0;
	derivativeError = 0;
	if (_TURNING) _CAM_ALERT_TURN = true;
	else          _CAM_VICTIM     = true;
}

void Driving::Init(ColorSensing* p_colorSensing, TofSensors* p_tof, GyroBase* p_gyro, Mapping* p_mapSys, Vcameras* p_cams, Drivetrain* p_drivetrain) {
	this->p_colorSensing = p_colorSensing;
	this->p_tof          = p_tof;
	this->p_gyro         = p_gyro;
	this->p_mapSys       = p_mapSys;
	this->p_cams         = p_cams;
	this->p_drivetrain   = p_drivetrain;

	// Enable bumper input pins
	pinMode(BUMPER_LEFT_PIN, INPUT);
	pinMode(BUMPER_RIGHT_PIN, INPUT);
}

#ifdef _MSC_VER
#pragma endregion
#endif
