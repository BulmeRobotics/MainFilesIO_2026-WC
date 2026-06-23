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

#define DEBUG_RAMP
// #define DEBUG_RAMP_ARRAY
// #define DEBUG_RAMP_DATASET
// #define DEBUG_X64
// #define DEBUG_DRIVING
// #define DEBUG_TURN
// #define DEBUG_PID
// #define DEBUG_TURN_PID
// #define DEBUG_ALIGN

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

	integralTurnError = 0.0f;
	turnLastError     = 0.0f;
	ts_lastTurnPID    = 0;
	p_tof->DisableUpdate();
	return ErrorCodes::OK;
}

ErrorCodes Driving::ControlTurn(float angle) {
	// Check if the turn takes too long, e.g. when stuck on a bumper
	if (_CAM_ALERT_TURN) maxTurnTime = 10000;
	if (((millis() - ts_startTime) >= maxTurnTime))	turnTimeout = true;
	else turnTimeout = false;

	if ((p_gyro->data.angle_abs >= (angle + 1.5) || (p_gyro->data.angle_abs <= (angle - 1.5))) && !turnTimeout) {
		p_gyro->GetAngleAdvanced(angle, GyroAxles::Axis_X);

		// dt measurement
		float dt;
		float rawDt = 0.0f;
		if (ts_lastTurnPID == 0) {
			dt = PID_TURN.dt_nominal;
		}
		else {
			rawDt = (float)(millis() - ts_lastTurnPID) / 1000.0f;
			dt    = (rawDt > 2.0f * PID_TURN.dt_nominal) ? PID_TURN.dt_nominal : rawDt;
		}
		ts_lastTurnPID = millis();

		float error      = p_gyro->data.angle_error;
		float derivative = (error - turnLastError) / dt;
		float rawOutput  = PID_TURN.P * error + PID_TURN.I * integralTurnError + PID_TURN.D * derivative;

		int8_t limit = _CAM_ALERT_TURN ? TURN_CAM_SPEED : (_TURN_180_DEGREE ? TURN_180_SPEED : TURN_MAX_SPEED);
		if (abs(rawOutput) < (float)limit) integralTurnError += error * dt;
		turnLastError = error;

		int16_t turnSpeed = (int16_t)constrain(rawOutput, -(float)limit, (float)limit);

		#ifdef DEBUG_TURN_PID
			Serial.print("E:"); Serial.print(error, 2);
			Serial.print("\tI:"); Serial.print(integralTurnError, 2);
			Serial.print("\tD:"); Serial.print(derivative, 2);
			Serial.print("\tC:"); Serial.print(turnSpeed);
			Serial.print("\tdt:"); Serial.print(dt * 1000.0f, 1);
			Serial.print("\tR:"); Serial.println(rawDt * 1000.0f, 1);
		#endif

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
	p_tof->EnableUpdate();
	EnableBumpers();	// Enable bumpers
	_CAM_ALERT_TURN = false;
	maxTurnTime = DEFAULT_MAX_TURN_TIME;
	if (StartAlign() == ErrorCodes::OK) {
		Orientations aligned = p_gyro->GetOrientationFromAngle();
		p_gyro->SetAngle(GyroAxles::Axis_X, p_gyro->GetAngleFromOrientation(aligned));
	}

	p_mapSys->Turn(p_gyro->GetOrientationFromAngle());
	_TURNING = false;
	p_colorSensing->Freeze(false);
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

ErrorCodes Driving::StartAlign(void) {
	p_tof->Update();
	_TURNING = true;
	uint32_t startTime = millis();

	#ifdef DEBUG_ALIGN
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

	#ifdef DEBUG_ALIGN
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

		#ifdef DEBUG_ALIGN
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
	filteredLR = 0.0f;
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

	// Read sensor once; use nominal angle for ToF suppression check, biased angle for PID error
	float actualAngle = p_gyro->GetAngle(GyroAxles::Axis_X);
	p_gyro->GetAngleAdvanced(angle, actualAngle);
	int8_t leftRightError = p_tof->CalculateLeftRightError(p_gyro->data.angle_error, TOF_SIDE_WALL_THRESHOLD, GAP_ROBOT_WALL);
	filteredLR = 0.7f * filteredLR + 0.3f * (float)leftRightError;

	float targetAngle = angle - (filteredLR * LATERAL_TO_ANGLE_FACTOR);
	if (targetAngle > 360.0f) targetAngle -= 360.0f;
	if (targetAngle < 0.0f)   targetAngle += 360.0f;
	p_gyro->GetAngleAdvanced(targetAngle, actualAngle);
	float error = -p_gyro->data.angle_error;

	// Measure dt; guard first iteration against derivative kick
	float dt;
	float rawDt = 0.0f;
	if (ts_lastPID == 0) {
		dt              = PID_DRIVE.dt_nominal;
		derivativeError = 0.0f;
		pidLastError    = error;
	}
	else {
		rawDt = (float)(millis() - ts_lastPID) / 1000.0f;
		dt    = (rawDt > 2.0f * PID_DRIVE.dt_nominal) ? PID_DRIVE.dt_nominal : rawDt;
		derivativeError = (error - pidLastError) / dt;
	}
	pidLastError = error;

	// Conditional anti-windup: only commit integral when output is not saturated
	float lim              = driveSpeed * 0.33f;
	float candidateIntegral = integralError + error * dt;
	float output           = PID_DRIVE.P * error + PID_DRIVE.I * candidateIntegral + PID_DRIVE.D * derivativeError;
	if (output <= lim && output >= -lim) integralError = candidateIntegral;
	correctionSpeed = constrain(output, -lim, lim);

	if (driveSpeed >= 90) driveSpeed = 90;
	if (_ON_RAMP) driveSpeed = rampSpeed;
	float powerLeft = driveSpeed - correctionSpeed;
	float powerRight = driveSpeed + correctionSpeed;

	p_drivetrain->SetSpeedLeft(powerLeft);
	p_drivetrain->SetSpeedRight(powerRight);

	ts_lastPID = millis();

	#ifdef DEBUG_PID
	// A=actual heading  T=biased target  LR=lateral mm  E=error(deg)  C=correction  dt=clamped ms  R=raw ms
	Serial.print("A:"); Serial.print(actualAngle, 1);
	Serial.print("\tT:"); Serial.print(targetAngle, 1);
	Serial.print("\tLR:"); Serial.print(leftRightError); Serial.print("\tFLR:"); Serial.print(filteredLR, 1);
	Serial.print("\tE:"); Serial.print(error, 2);
	Serial.print("\tC:"); Serial.print(correctionSpeed, 2);
	Serial.print("\tdt:"); Serial.print(dt * 1000.0f, 1);
	Serial.print("\tR:"); Serial.println(rawDt * 1000.0f, 1);
	#endif

	if (_CAM_VICTIM) maxDriveTime = 12000;
	if ((!_ON_RAMP) && (millis() - ts_driveStartTime > maxDriveTime)) return ErrorCodes::TIMEOUT;
	else if (!_ON_RAMP) return ErrorCodes::CHECK_DRIVE;
	else return ErrorCodes::OK;
}

ErrorCodes Driving::CheckDrive(void) {
	// While climbing, the distance target is meaningless — RampHandler ends the drive on incline-flat.
	// Without this, an ENCODER-referenced ramp approach hits its target mid-climb and cuts it short.
	if (_ON_RAMP) return ErrorCodes::CHECK_RAMP;

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

	uint32_t startTime = millis();
	int8_t posError;
	do {
		p_tof->Update();
		posError = p_tof->GetRange(TofType::FRONT) - ADJUST_WALL_DISTANCE;
		int8_t adjustSpeed = posError * ADJUSTMENT_SPEED_FACTOR;	// Calculate adjustment speed

		p_drivetrain->SetSpeed(adjustSpeed);
	} while (abs(posError) > 10 && millis() - startTime < ADJUSTMENT_TIMEOUT);	// Within tolerance or timeout (abort on stall)

	p_drivetrain->Stop();
	return ErrorCodes::OK;
}

ErrorCodes Driving::ReverseBlackTile(void) {
	p_drivetrain->EnableEncoder();
	p_drivetrain->ResetEncoder();
	ts_encoderStartTime = millis();
	// Drive back ~17 cm; abort if the encoder stalls (blocked wheel) so the robot never reverses forever
	while (p_drivetrain->GetEncoderDistance() < 170 && millis() < (ts_encoderStartTime + DEFAULT_MAX_ENCODER_TIME)) {
		p_drivetrain->SetSpeed(-30);
	}
	p_drivetrain->Stop();
	p_drivetrain->DisableEncoder();	// Disable motor interrupts
	if (millis() >= (ts_encoderStartTime + DEFAULT_MAX_ENCODER_TIME)) return ErrorCodes::TIMEOUT;
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

	// Re-detect ramps every drive segment so straight tile-to-tile runs (not just turn/ramp ends)
	// keep the reference selection from picking a ramp surface. Consumed below by ApplyRampFlagOverrides.
	UpdateRampProximityFlags();

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
	aggregatedIncline = 0.0f;	// Reset each call so GVar does not accumulate across ramps
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
	// Sets _RAMP_INFRONT/_RAMP_BEHIND from the upper/lower VL53L4CD pair ramp detection.
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
	// Drives forward a fixed distance to clear the ramp end, then aligns and re-enables bumpers.
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
			ts_rampTraversalStart = ts_rampStartTime;	// Preserve start before reset, for duration logging
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
				#ifdef DEBUG_RAMP
				Serial.println("CORRECTED RAMP DOWN!");
				#endif
			}
			p_mapSys->Move(true);
			ts_rampTraversalStart = ts_rampStartTime;	// Preserve start before reset, for duration logging
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
	if (arrInclineIndex >= INCLINE_ARRAY_SIZE) return;	// Overflow guard: drop samples beyond capacity
	arrIncline[arrInclineIndex] = incline;
	arrInclineIndex++;
	if (_RAMP_UP   && incline >  maxRampIncline) maxRampIncline = incline;
	else if (_RAMP_DOWN && incline <  maxRampIncline) maxRampIncline = incline;
}

float Driving::ComputeMedianIncline(void){
	// Median of the recorded incline samples. Sorts arrIncline in place (order is no longer
	// needed after classification and the array is cleared immediately after).
	uint16_t n = arrInclineIndex;
	if (n == 0) return 0.0f;

	// Insertion sort: the robot is stopped here, so the one-shot cost is irrelevant.
	for (uint16_t i = 1; i < n; i++) {
		float key = arrIncline[i];
		int16_t j = i - 1;
		while (j >= 0 && arrIncline[j] > key) {
			arrIncline[j + 1] = arrIncline[j];
			j--;
		}
		arrIncline[j + 1] = key;
	}

	if (n & 1) return arrIncline[n / 2];
	return (arrIncline[n / 2 - 1] + arrIncline[n / 2]) / 2.0f;	// Even N: average the two middle values
}

void Driving::ClassifyAndFinishRamp(void){
	// Classifies stair vs smooth, finalises traversal, computes geometry, resets ramp state.

	#ifdef DEBUG_RAMP_DATASET
	// Dataset snapshot: raw (uncorrected) encoder distance and ramp duration, taken before
	// FinishRamp drives on and before CalculateRampGeometry mutates rampEncoderDistance.
	float    rawEncoderDistance = rampEncoderDistance;
	uint32_t rampDuration       = millis() - ts_rampTraversalStart;
	#endif

	#ifdef DEBUG_RAMP_ARRAY
	Serial.println("ARRAY");
	for (uint8_t i = 0; i < arrInclineIndex; i++) {
		Serial.println(arrIncline[i]);
	}
	#endif

	if (CheckStairRamp()) {	// Sets avgIncline and aggregatedIncline
		FinishRamp(75);
		_STAIR = true;
	}
	else FinishRamp(80);

	// Steady-state incline angle used for the geometry decomposition below. Computed here, before
	// the sample array is cleared (ComputeMedianIncline sorts arrIncline in place).
	medianIncline = ComputeMedianIncline();

	#ifdef DEBUG_RAMP_DATASET
	// Ramp dataset line. Direction is read from the sign of Mean; Dur measures incline-onset
	// (~10 deg) to flat-out (<=4 deg), not base-to-top.
	float gyroVar = (arrInclineIndex > 1) ? (aggregatedIncline / (arrInclineIndex - 1)) : 0.0f;
	Serial.print(F("RAMPDATA | Typ:"));
	Serial.print(_STAIR ? F("Stair") : F("Ramp"));
	Serial.print(F(" Dst:"));    Serial.print(rawEncoderDistance, 1);
	Serial.print(F(" Dur:"));    Serial.print(rampDuration);
	Serial.print(F("ms Mean:")); Serial.print(avgIncline, 1);
	Serial.print(F(" Med:"));    Serial.print(medianIncline, 1);
	Serial.print(F(" GVar:"));   Serial.println(gyroVar, 2);
	#endif

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
		rampAngle = medianIncline;	// Median is accurate vs tape; K/D are fit to the raw hypotenuse
		#ifdef DEBUG_RAMP
		Serial.print("\tRAMP UP");
		#endif
	}
	else if (rampDown && !isStair) {
		rampEncoderDistance = rampEncoderDistance * RAMP_DOWN_K + RAMP_DOWN_D;
		rampAngle = medianIncline;	// Median (negative for down) — accurate, no fudge offset needed
		#ifdef DEBUG_RAMP
		Serial.print("\tRAMP DOWN");
		#endif
	}
	else if (rampUp && isStair) {
		rampEncoderDistance = rampEncoderDistance * STAIR_UP_K + STAIR_UP_D;
		rampAngle = medianIncline;	// Median is accurate to ~1deg on stairs; K/D fit on the raw hypotenuse
		#ifdef DEBUG_RAMP
		Serial.print("\tSTAIR UP");
		#endif
	}
	else if (rampDown && isStair) {
		rampEncoderDistance = rampEncoderDistance * STAIR_DOWN_K + STAIR_DOWN_D;
		rampAngle = medianIncline;	// Median (negative for down)
		// 1-tile steep stair-down reads ~2.6° shallow; steepen it (corrected hyp ~330 vs ~650). Affects height, not tile count.
		if (rampEncoderDistance < STAIR_DOWN_1TILE_HYP_MAX) rampAngle -= STAIR_DOWN_1TILE_OFFSET;
		#ifdef DEBUG_RAMP
		Serial.print("\tSTAIR DOWN");
		#endif
	}
	else rampEncoderDistance = 0;

	// Note: the former (!isStair) *= 0.95 fudge is removed — it is folded into the new RAMP_*_K.

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
			else p_mapSys->Bumper(false);

			float saveDistance = 0;
			if (sensor.type == ReferenceObj::ENCODER) {
				saveDistance = p_drivetrain->GetEncoderDistance();
			}

			uint32_t maneuverStart = millis();

			ExecuteBumperManeuver();

			StartAlign();
			StartAdjustment();

			uint32_t elapsed = millis() - maneuverStart;
			ts_driveStartTime += (elapsed + 200); // Exclude recovery time + delay from drive time limit

			if (sensor.type == ReferenceObj::ENCODER) {
				p_drivetrain->ResetEncoder(saveDistance);
			}
			integralError   = 0;
			derivativeError = 0;
			ts_lastPID      = 0; // Reset PID timestamp to prevent derivative kick

			return ErrorCodes::OK;
		}
	}
	else return ErrorCodes::BUMPER_DISABLED;
	return ErrorCodes::UNKNOWN;
}

ErrorCodes Driving::HandleWallCollision(void){
	// Registers a wall collision: updates the map, resets bump counter, backs away.
	p_mapSys->Bumper(true);	//reset
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
	integralError  = 0.0f;
	ts_lastPID     = 0;      // force derivative restart on next ControlDrive cycle
	if (_TURNING) {
		_CAM_ALERT_TURN   = true;
		integralTurnError = 0.0f; // clear integral before speed limit drops to TURN_CAM_SPEED
		ts_lastTurnPID    = 0;    // force derivative restart on next ControlTurn cycle
	}
	else {
		_CAM_VICTIM = true;
	}
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
