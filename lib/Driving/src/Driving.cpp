#include "Driving.h"
#include <Vcameras.h>

#ifdef _MSC_VER
#pragma region BUMPERS
#endif
ErrorCodes Driving::reverseBumper(uint16_t distance, int8_t speedLeft, int8_t speedRight){
	uint32_t buffer_beginTime = millis();	//Log begin Time
	
    p_drivetrain->EnableEncoder();
	p_drivetrain->ResetEncoder();
	while (p_drivetrain->GetEncoderDistance() < distance && buffer_beginTime + reverseBumperTimeout > millis()) {	//Drive back the distance, Check Timeout
        p_drivetrain->SetSpeedLeft(speedLeft);
        p_drivetrain->SetSpeedRight(speedRight);
	}
	p_drivetrain->Stop();

	if (sensor.type != ReferenceObj::ENCODER) p_drivetrain->DisableEncoder();
	if(buffer_beginTime + reverseBumperTimeout < millis()) return ErrorCodes::TIMEOUT;	//Timeout reached
	return ErrorCodes::OK;
}
ErrorCodes Driving::enableBumpers(void){
    _ENABLE_BUMPERS = true;	//Enable Bumpers
    return ErrorCodes::OK;
}
ErrorCodes Driving::disableBumpers(void){
    _ENABLE_BUMPERS = false;	//Disable Bumpers
    return ErrorCodes::OK;
}
ErrorCodes Driving::bumperHandler(void){
    if (_ENABLE_BUMPERS) {
		if (digitalRead(BUMPER_LEFT_PIN) || digitalRead(BUMPER_RIGHT_PIN)) {
			p_drivetrain->Stop();
			delay(200);

			_registeredBumps += 1;	//add 1 to bump registered

			if(_registeredBumps > BUMPER_TRYS){
				p_mapSys->Bumper();
				_registeredBumps = 0;

				reverseBumper(40,-40,-40); //Drive back 40mm
				return ErrorCodes::BUMPER_WALL;
			} 

			float saveDistance = 0;
			if (sensor.type == ReferenceObj::ENCODER) {
				saveDistance = p_drivetrain->GetEncoderDistance();
			}

			if (digitalRead(BUMPER_LEFT_PIN) && digitalRead(BUMPER_RIGHT_PIN)) {	//both bumpers
				reverseBumper(30, -40, -40);	//Drive back 5cm
			}
			else if (digitalRead(BUMPER_LEFT_PIN)) {	//only left bumper
				//Reverse in S-curve
				reverseBumper(70, -50, 50);
				reverseBumper(40, -50, -50);
				reverseBumper(90, 50, -50);
			}
			else if (digitalRead(BUMPER_RIGHT_PIN)) {	//only right bumper
				//Reverse in S-curve
				reverseBumper(70, 50, -50);
				reverseBumper(40, -50, -50);
				reverseBumper(90, -50, 50);
			}

			startAlign();	//Align robot
			startAdjustment();	//Adjust robot to wall

			if (sensor.type == ReferenceObj::ENCODER) {
				p_drivetrain->ResetEncoder(saveDistance);	//Reset Encoder
			}
			integralError = 0;	//Reset integral error
			derivativeError = 0;	//Reset derivative error

			return ErrorCodes::OK;	//Bumper pressed
		}
	}
	else return ErrorCodes::BUMPER_DISABLED;	//no bumper pressed
	return ErrorCodes::UNKNOWN;
}
#ifdef _MSC_VER
#pragma endregion BUMPERS
#pragma region RAMPS
#endif
ErrorCodes Driving::checkRamp(void){
    p_gyro->GetAngleAdvanced(0, GyroAxles::Axis_Z);
    float incline = -p_gyro->data.angle_car;

    p_gyro->GetAngleAdvanced(0, GyroAxles::Axis_Y);
    float sideTilt = p_gyro->data.angle_car;

    // bool freezeColor = false;

	// if (!p_colorSensing->Freeze()) {
	//     if (abs(incline) > 3 || abs(sideTilt) > 4) p_colorSensing->Freeze(true);
	// }
	// else {
	//     if (abs(incline) < 2 && abs(sideTilt) < 3) p_colorSensing->Freeze(false);
	// }

    // if (p_colorSensing->Freeze()) {
    //     Serial.println(String(freezeColor) + "\tZ:" + String(incline) + "\tY:" + String(sideTilt));
    // }
	
    //Detect Ramps
	if (!_ON_RAMP && (incline > rampThresholdAngle) && inclineCycleCounter == 0) {
		rampStartTime = millis();
		inclineCycleCounter++;
		nonInclineCycleCounter = 0;
	}
	else if (!_ON_RAMP && (incline < -rampThresholdAngle) && inclineCycleCounter == 0) {
		rampStartTime = millis();
		inclineCycleCounter--;
		nonInclineCycleCounter = 0;
	}
	else if (!_ON_RAMP && ((incline >= rampThresholdAngle) || (incline <= -rampThresholdAngle)))	{
		if (incline > 0) inclineCycleCounter++;
		else inclineCycleCounter--;
		if ((incline < 15) && (incline > -15)) rampCheckDuration = 1000;
		else if (((incline >= 15) && (incline < 20)) || ((incline <= -15) && (incline > -20)))	rampCheckDuration = 500;
		else if ((incline >= 20) || (incline <= -20))	rampCheckDuration = 200;
	}

	else if (!_ON_RAMP && ((incline < rampThresholdAngle) && (incline > -rampThresholdAngle)))
		nonInclineCycleCounter++;

	if ( !_ON_RAMP && millis() > (rampStartTime + rampCheckDuration) && rampStartTime != 0) {
		if (inclineCycleCounter >= nonInclineCycleCounter * rampConfidence) {
			inclineCycleCounter = 0;
			#ifdef DEBUG_RAMP
			Serial.println("RAMP UP DETECTED!");
			#endif
			_ON_RAMP = true;	//Ramp detected
			p_mapSys->Move(true);
			_RAMP_UP = true;	//Ramp UP detected
			_RAMP_DOWN = false;	//Ramp DOWN not detected
			rampStartTime = 0;	//Reset Ramp Start Time
			rampEncoderDistance = 0;
			rampSpeed = rampSpeedUp;
			disableBumpers();	//Disable Bumpers
			p_drivetrain->ResetEncoder(0);	//Reset Encoder
			p_drivetrain->EnableEncoder();	//Enable Motor Interrupts
			p_colorSensing->Freeze(true);
			return ErrorCodes::OK;

		}
		else if (-inclineCycleCounter >= nonInclineCycleCounter * rampConfidence) {
			inclineCycleCounter = 0;
			#ifdef DEBUG_RAMP
			Serial.println("RAMP DOWN DETECTED!");
			#endif
			_ON_RAMP = true;	//Ramp detected
			
			_RAMP_DOWN = true;	//Ramp DOWN detected
			_RAMP_UP = false;	//Ramp UP not detected
			if (millis() < lastSetTile + MIN_SETTILE_TIME) {	//Check if settile occured shortly before Ramp detection
				p_mapSys->RollbackOne();
				Serial.println("CORRECTED RAMP DOWN!");
			}

			p_mapSys->Move(true);
			
			rampStartTime = 0;	//Reset Ramp Start Time
			rampEncoderDistance = 0;
			rampSpeed = rampSpeedDown;
			disableBumpers();	//Disable Bumpers
			p_drivetrain->ResetEncoder(0);	//Reset Encoder
			p_drivetrain->EnableEncoder();	//Enable Motor Interrupts
			p_colorSensing->Freeze(true);
			return ErrorCodes::OK;
		}
		else {
    		inclineCycleCounter = 0;
    		nonInclineCycleCounter = 0;
    		rampStartTime = 0;
    		rampCheckDuration = 1000;   // optional Default
    		#ifdef DEBUG_RAMP
    		Serial.println("NO RAMP!");
			Serial.println("inc: " + String(inclineCycleCounter) + " non: " + String(nonInclineCycleCounter));
    		#endif
		}
	}
	return ErrorCodes::OK;
}
ErrorCodes Driving::finishRamp(uint8_t distance){
    p_drivetrain->ResetEncoder(0);	//Reset Encoder
	p_drivetrain->EnableEncoder();	//Enable Motor Interrupts
	encoderStartTime = millis();	//Get the start time

	while (p_drivetrain->GetEncoderDistance() < distance && millis() < (encoderStartTime + maxEncoderTime)) {	//Drive up the ramp
		p_drivetrain->SetSpeed(50);
	}
	p_drivetrain->Stop();
	p_drivetrain->DisableEncoder();	//Disable Motor Interrupts

	startAlign();	//Align robot
	enableBumpers();	//Disable Bumpers

	// Check for ramps in front and back
	if (p_tof->IsRampThere(false)) {
		_RAMP_INFRONT = true;
		_RAMP_BEHIND = false;
	}
	else if (p_tof->IsRampThere(true)) {
		_RAMP_BEHIND = true;
		_RAMP_INFRONT = false;
	}
	else {
		_RAMP_INFRONT = false;
		_RAMP_BEHIND = false;
	}
	return ErrorCodes::OK;
}
ErrorCodes Driving::rampHandler(void){
    if (!_ON_RAMP) checkRamp();
	else {
		p_gyro->GetAngleAdvanced(0, GyroAxles::Axis_Z);
		float incline = -p_gyro->data.angle_car;
		arr_incline[arr_incline_index] = incline;	//Save the incline value in the array
		arr_incline_index++;


		p_gyro->GetAngleAdvanced(0, GyroAxles::Axis_Y);		

		if  (_RAMP_UP && incline > maxRampIncline)	maxRampIncline = incline;	//Ramp up
		else if (_RAMP_DOWN && incline < maxRampIncline) maxRampIncline = incline;	//Ramp down

		if (incline <= 4 && incline >= -4) {
			rampEncoderDistance = p_drivetrain->GetEncoderDistance();

			endDrive();

			#ifdef DEBUG_RAMP_ARRAY
			Serial.println("ARRAY");
			for (uint8_t i = 0; i < arr_incline_index; i++) {
				Serial.println(arr_incline[i]);
			}
			#endif

			if(checkStairRamp()) {
				finishRamp(75);
				_STAIR = true;
			}
			else finishRamp(60);
			
			
			for (uint16_t i = 0; i < INCLINE_ARRAY_SIZE; i++) {
				arr_incline[i] = 0.0;	//Reset the incline array
			}

			#ifdef DEBUG_RAMP
			Serial.print("Raw: ");
			Serial.print(rampEncoderDistance);

			Serial.print("\tStair: ");
			Serial.print(_STAIR);
			#endif // DEBUG_RAMP	
			
			if (_RAMP_UP && !_STAIR) {
				rampEncoderDistance = rampEncoderDistance * rampUp_K + rampUp_d;
				RAMP_ANGLE = maxRampIncline;
				#ifdef DEBUG_RAMP
				Serial.print("\tRAMP UP");
				#endif
			}
			else if (_RAMP_DOWN && !_STAIR) {
				rampEncoderDistance = rampEncoderDistance * rampDown_K + rampDown_d;
				RAMP_ANGLE = maxRampIncline + 2;
				#ifdef DEBUG_RAMP
				Serial.print("\tRAMP DOWN");
				#endif
			}
			else if (_RAMP_UP && _STAIR) {
				rampEncoderDistance = rampEncoderDistance * stairUp_K + stairUp_d;
				RAMP_ANGLE = avgIncline + stairUp_angle_offset;
				#ifdef DEBUG_RAMP
				Serial.print("\tSTAIR UP");
				#endif
			}
			else if (_RAMP_DOWN && _STAIR) {
				rampEncoderDistance = rampEncoderDistance * stairDown_K + stairDown_d;
				RAMP_ANGLE = avgIncline + stairDown_angle_offset;
				#ifdef DEBUG_RAMP
				Serial.print("\tSTAIR DOWN");
				#endif
			}
			else rampEncoderDistance = 0;	//No Ramp detected

			if (!_STAIR) rampEncoderDistance *= 0.95;

			RAMP_HYPOTENUSE = rampEncoderDistance;
			RAMP_HEIGHT = rampEncoderDistance * sinf(RAMP_ANGLE * PI / 180.0);
			RAMP_LENGTH = rampEncoderDistance * cosf(RAMP_ANGLE * PI / 180.0);
			
			
			_RAMP_UP = false;
			_RAMP_DOWN = false;
			_STAIR = false;
			p_colorSensing->Freeze(false);
			arr_incline_index = 0;	//Reset the incline array index

			#ifdef DEBUG_RAMP
			Serial.print("\tDistance: ");
			Serial.print(RAMP_HYPOTENUSE);
			Serial.print("\tRamp angle: ");
			Serial.print(RAMP_ANGLE);
			Serial.print("\tHeight: ");
			Serial.print(RAMP_HEIGHT);
			Serial.print("\tLength: ");
			Serial.println(RAMP_LENGTH);
			#endif // DEBUG_RAMP
			
			return ErrorCodes::RAMP_END;
		}
	}
	return ErrorCodes::OK;
}
bool Driving::checkStairRamp(void) {
	float aggregatedIncline = 0.0;
	float sumIncline = 0.0;

	for (int i = 0; i < arr_incline_index - 1; i++) {
		aggregatedIncline += abs(arr_incline[i] - arr_incline[i+1]);
		sumIncline += arr_incline[i];
	}
	avgIncline = sumIncline / arr_incline_index;	//Calculate the average incline

	#ifdef DEBUG_RAMP
	Serial.print("Average Stair Incline: ");
	Serial.println(avgIncline);
	#endif

	#ifdef DEBUG_RAMP
	Serial.print("Aggregated Incline: ");
	Serial.println(aggregatedIncline);
	#endif

	if (aggregatedIncline > rampStairsThreshold) {
		#ifdef DEBUG_RAMP
		Serial.println("Stair Ramp detected!");
		#endif
		return true;	//Stair Ramp detected
	}
	else {
		#ifdef DEBUG_RAMP
		Serial.println("No Stair Ramp detected!");
		#endif
		return false;	//No Stair Ramp detected
	}
}
#ifdef _MSC_VER
#pragma endregion RAMPS
#pragma region TURN
#endif

ErrorCodes Driving::startTurn(float angle) {
	_registeredBumps = 0;

	if (angle > 360) {
		return ErrorCodes::ERROR;
	}
	#ifdef DEBUG_TURN
		Serial.println("Starting turn");
		Serial.print("TargetAngle: ");
		Serial.println(angle);
	#endif
	startTime = millis();
	disableBumpers();	//Disable Bumpers

	if (!_CAM_ALERT_TURN) maxTurnTime = 3000;

	_TURNING = true;
	p_colorSensing->Freeze(true);

	p_gyro->GetAngleAdvanced(angle, GyroAxles::Axis_X);	//Get the gyro readings
	if (abs(p_gyro->data.angle_error) > 150) _TURN_180_DEGREE = true;
	else _TURN_180_DEGREE = false;	//Check if the turn is a 180 degree turn
	return ErrorCodes::OK;
}
ErrorCodes Driving::controlTurn(float angle) {
	//Check if the turn takes to long, e.g. by being stuck on a bumper
	if (_CAM_ALERT_TURN) maxTurnTime = 10000;
	if (((millis() - startTime) >= maxTurnTime))	turnTimeout = true;
	else turnTimeout = false;

	if ((p_gyro->data.angle_abs >= (angle + 0.5) || (p_gyro->data.angle_abs <= (angle - 0.5))) && !turnTimeout) {
		//Get the gyro readings and calculate the control data
		p_gyro->GetAngleAdvanced(angle, GyroAxles::Axis_X);
		#ifdef DEBUG_TURN
			Serial.print("Angle error: ");
			Serial.print(p_gyro->data.angle_error);
		#endif

		//Calculate the turn speed with an exponential function
		int16_t turnSpeed = 0;
		float baseSpeed = 70.0;
		if (p_cams->IsAlert(ErrorCodes::left) || p_cams->IsAlert(ErrorCodes::right)) {
			baseSpeed = 30.0;	//Reduce speed if CS is on ALERT
			_CAM_ALERT_TURN = true;
		}

		if (p_gyro->data.direction_right)	turnSpeed = -(baseSpeed * 	(1 - pow(EULER, p_gyro->data.angle_error / 25.0)) + 20.0);
		else if (p_gyro->data.direction_left) turnSpeed = (baseSpeed * 	(1 - pow(EULER, -p_gyro->data.angle_error / 25.0)) + 20.0);

		if (!_TURN_180_DEGREE) turnSpeed = constrain(turnSpeed, -100, 100);
		else turnSpeed = constrain(turnSpeed, -60, 60);	//Limit the speed to 50

		#ifdef DEBUG_TURN
			Serial.print("Turn speed: ");
			Serial.println(turnSpeed);
		#endif

		//Set the speed
		p_drivetrain->SetSpeedLeft(turnSpeed);
		p_drivetrain->SetSpeedRight(-turnSpeed);
	}
	else {
		//Stop all motors
		p_drivetrain->Stop();
		_TURNING = false;

		#ifdef DEBUG_TURN
			Serial.print("Final angle: ");
			Serial.println(gyro.getAngle(GyroAxles::AXIS_X));
			Serial.print("Time: ");
			Serial.println(millis() - startTime);
		#endif
		
		return ErrorCodes::TURNED;
	}
	return ErrorCodes::OK;
}
ErrorCodes Driving::endTurn(){
	enableBumpers();	//Enable Bumpers
	_CAM_ALERT_TURN = false;
	maxTurnTime = 3000;
	startAlign();
	// p_gyro->ResetAngle(GyroAxles::Axis_Y);
	// p_gyro->ResetAngle(GyroAxles::Axis_Z);

	
	p_mapSys->Turn(p_gyro->GetOrientationFromAngle());
	_TURNING = false;
	p_colorSensing->Freeze(false);
	// Check for ramps in front and back
    if (p_tof->IsRampThere(false)) {
		_RAMP_INFRONT = true;
		_RAMP_BEHIND = false;
	}
	else if (p_tof->IsRampThere(true)) {
		_RAMP_BEHIND = true;
		_RAMP_INFRONT = false;
	}
	else {
		_RAMP_INFRONT = false;
		_RAMP_BEHIND = false;
	}
	return ErrorCodes::OK;
}
ErrorCodes Driving::turn180Degree(void) {
	float newAngle = p_gyro->GetAngle(GyroAxles::Axis_X) + 180.0;	//Get the current angle and add 180 degrees
	if (newAngle > 360) newAngle -= 360;	//Check if the angle is greater than 360 degrees
	startTurn(newAngle);	//Start the turn

	while (controlTurn(newAngle) == ErrorCodes::OK) delay(1);
	endTurn();	//End the turn
	return ErrorCodes::OK;
}
ErrorCodes Driving::startAlign(void) {
	//Reading all short TOFs on the side of the robot
	uint16_t startTime = millis();
	_TURNING = true;	
	p_tof->Update();
	uint16_t sumDistanceLeft 	= p_tof->GetRange(TofType::LEFT_BACK) + p_tof->GetRange(TofType::LEFT_FRONT);
	uint16_t sumDistanceRight 	= p_tof->GetRange(TofType::RIGHT_BACK) + p_tof->GetRange(TofType::RIGHT_FRONT);

	#ifdef DEBUG_DRIVING
		Serial.print("LB: ");
		Serial.print(p_tof->GetRange(TofType::LEFT_BACK));
		Serial.print("\tLF: ");
		Serial.print(p_tof->GetRange(TofType::LEFT_FRONT));
		Serial.print("\tRF: ");
		Serial.print(p_tof->GetRange(TofType::RIGHT_FRONT));
		Serial.print("\tRB: ");
		Serial.print(p_tof->GetRange(TofType::RIGHT_BACK));
	#endif // DEBUG_DRIVING

	//deciding which side to use for the procedure
	if (sumDistanceLeft <= sumDistanceRight && p_tof->GetRange(TofType::LEFT_BACK) < 200 && p_tof->GetRange(TofType::LEFT_FRONT) < 200) {
		side = "LEFT";
		distanceFront = p_tof->GetRange(TofType::LEFT_FRONT);
		distanceBack = p_tof->GetRange(TofType::LEFT_BACK);
		coeff_side = 1;
	}
	else if (sumDistanceLeft > sumDistanceRight && p_tof->GetRange(TofType::RIGHT_BACK) < 200 && p_tof->GetRange(TofType::RIGHT_FRONT) < 200) {
		side = "RIGHT";
		coeff_side = -1;
		distanceFront = p_tof->GetRange(TofType::RIGHT_FRONT);
		distanceBack = p_tof->GetRange(TofType::RIGHT_BACK);
	}
	else return ErrorCodes::NOT_ALIGNING;	//No alignment possible, no wall next to the robot

	#ifdef DEBUG_DRIVING
		Serial.print("\tSide: ");
		Serial.println(side);
	#endif // DEBUG_DRIVING

	do {
		p_tof->Update();

		if (side == "LEFT") {
			//read the distances
			distanceFront = (uint8_t)p_tof->GetRange(TofType::LEFT_FRONT);
			distanceBack = 	(uint8_t)p_tof->GetRange(TofType::LEFT_BACK);
		}
		else if (side == "RIGHT") {
			//read the distances
			distanceFront = (uint8_t)p_tof->GetRange(TofType::RIGHT_FRONT);
			distanceBack = 	(uint8_t)p_tof->GetRange(TofType::RIGHT_BACK);
		}

		if (distanceFront >= 255 || distanceBack >= 255) {
			//stop all motors
			p_drivetrain->Stop();
			return ErrorCodes::NOT_ALIGNING;	//No alignment possible, no wall next to the robot
		}

		distanceError = distanceFront - distanceBack;

		//direction of the correction, calculating the turn speed with an exponential function
		if (distanceError > 0)	turnSpeed_align = 40.0f * (pow(EULER, (float)-distanceError / 20.0f) - 1.0f) - 20.0f;
		else if (distanceError < 0)	turnSpeed_align = 40.0f * (1 - pow(EULER, (float)abs(distanceError) / -20.0f)) + 20.0f;
		else  turnSpeed_align = 0;

		#ifdef DEBUG_DRIVING
			Serial.print("Error: " + String(distanceError));
			Serial.println("\tSpeed: " + String(turnSpeed_align));
		#endif // DEBUG_DRIVING

		//turning the robot with the calculated speed, in first cycle = 0
		p_drivetrain->SetSpeedLeft(turnSpeed_align * coeff_side);
		p_drivetrain->SetSpeedRight(-turnSpeed_align * coeff_side);
	} while (abs(distanceError) >= 5 && millis() - startTime < 1500);
	//stoping all motors
	p_drivetrain->Stop();
	_TURNING = false;	
	return ErrorCodes::OK;
}
#ifdef _MSC_VER
#pragma endregion TURN
#pragma region DRIVE
#endif
ErrorCodes Driving::startDrive(bool rampDown) {
	_registeredBumps = 0;	//add 1 to bump registered
	_DRIVE_TIMEOUT = false;
	_CAM_VICTIM = false;
	maxDriveTime = 5000;
	driveStartTime = millis();

	sensor = getOptimalSensor(rampDown);
	lastTargetDistance = nextTargetDistance;

	if (sensor.type == ReferenceObj::BACK)
		if (sensor.back > 250) nextTargetDistance = sensor.back + 300;
		else nextTargetDistance = 320;
	else if (sensor.type == ReferenceObj::FRONT)
		if (sensor.front > 410) nextTargetDistance = sensor.front - 300;
		else nextTargetDistance = 110;
	else
		nextTargetDistance = 310;

	#ifdef DEBUG_DRIVING
	Serial.print("Target: ");
	Serial.println(nextTargetDistance);
	#endif // DEBUG_DRIVING

	if (sensor.type == ReferenceObj::ENCODER) {
		p_drivetrain->EnableEncoder();	//Enable Motor Interrupts
		p_drivetrain->ResetEncoder();
	}
	else p_drivetrain->DisableEncoder();	//Disable Motor Interrupts

	return ErrorCodes::OK;
}
ErrorCodes Driving::controlDrive(int8_t driveSpeed, float angle) {
	if (_SLOW_SPEED)
		driveSpeed = 25;
	p_gyro->GetAngleAdvanced(angle, GyroAxles::Axis_X);	//Gyro auslesen, für akutellen Winkel und Fehler
	int8_t leftRightError = p_tof->CalculateLeftRightError(p_gyro->data.angle_error, tof_sideWallThreshold, gap_robot_wall);
	float error = -p_gyro->data.angle_error + (leftRightError * pid_LeftRightFactor);

	if (error != 0) integralError += error;	//Summenfehler berechnen
	else integralError = 0;	//Summe zurücksetzen bei Sollwert
	derivativeError = error - pid_lastError;	//Änderung des Winkels berechnen
	pid_lastError = error;

	PID_Coefficients coeff;
	if (lastPID_timestamp) coeff = calculatePIDCoefficients((float)(millis() - lastPID_timestamp) / 1000);
	else coeff = calculatePIDCoefficients(pid_LoopDuration);
	#ifdef DEBUG_DRIVING
		Serial.print("Time: ");
		Serial.println((float)(millis() - lastPID_timestamp));
	#endif // DEBUG_DRIVING

	correctionSpeed = (coeff.P * error) + (coeff.I * integralError) + (coeff.D * derivativeError);
	correctionSpeed = constrain(correctionSpeed, (-driveSpeed * 0.33), (driveSpeed * 0.33));	//Limit the correction speed to 33% of the max speed

	if (driveSpeed >= 90) driveSpeed = 90;
	if (_ON_RAMP) driveSpeed = rampSpeed;
	float powerLeft = driveSpeed - correctionSpeed;
	float powerRight = driveSpeed + correctionSpeed;

	p_drivetrain->SetSpeedLeft(powerLeft);
	p_drivetrain->SetSpeedRight(powerRight);

	lastPID_timestamp = millis();

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
	if ((!_ON_RAMP) && (millis() - driveStartTime > maxDriveTime)) return ErrorCodes::TIMEOUT;
	else if (!_ON_RAMP) return ErrorCodes::CHECK_DRIVE;
	else return ErrorCodes::OK;
}
ErrorCodes Driving::checkDrive(void) {
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
	#ifdef DEBUG_DRIVING
	Serial.print("Type: ");
	Serial.print(int(sensor.type));
	Serial.print("\tRange: ");
	Serial.println(newValue);
	#endif // DEBUG_DRIVING
	return ErrorCodes::CHECK_RAMP;

}
ErrorCodes Driving::endDrive(void) {
	lastPID_timestamp = 0;
	p_drivetrain->Stop();


	return ErrorCodes::OK;
}
ErrorCodes Driving::timeoutDrive(void) {
	endDrive();
	startAlign();
	startAdjustment();

	_DRIVE_TIMEOUT = false;
	return ErrorCodes::OK;
}
ErrorCodes Driving::startAdjustment(void) {
	//Positon the robot in the middle of the field before turning
	p_tof->Update();
	if (p_tof->GetRange(TofType::FRONT) > 120)	return ErrorCodes::ERROR;

	int8_t posError;
	do {
		p_tof->Update();
		posError = p_tof->GetRange(TofType::FRONT) - adjust_wallDistance;
		int8_t adjustSpeed = posError * adjustmentSpeedFactor;	//Calculate the adjustment speed

		p_drivetrain->SetSpeed(adjustSpeed);
	} while (abs(posError) > 10);	//Wait until the robot is in the middle of the field

	p_drivetrain->Stop();
	return ErrorCodes::OK;
}
ErrorCodes Driving::reverseBlackTile(void) {
	p_drivetrain->EnableEncoder();
	p_drivetrain->ResetEncoder();
	while (p_drivetrain->GetEncoderDistance() < 150) {	//Drive back 10cm
		p_drivetrain->SetSpeed(-30);
	}
	p_drivetrain->Stop();
	p_drivetrain->DisableEncoder();	//Disable Motor Interrupts
	return ErrorCodes::OK;
}
#ifdef _MSC_VER
#pragma endregion DRIVE
#pragma region GENERAL
#endif
PID_Coefficients  Driving::calculatePIDCoefficients(float loopDuration){
    if (loopDuration >= 2 * pid_LoopDuration) loopDuration = pid_LoopDuration;	//Limit the loop duration to 2*pid_LoopDuration
	float time_coeff = loopDuration / pid_LoopDuration;
	PID_Coefficients pid;

	pid.P = 0.6 * pid_CriticalGain;
	pid.I = time_coeff * 2 * pid.P * pid_LoopDuration / pid_OscillationPeriod;
	pid.D = time_coeff * pid.P * pid_OscillationPeriod / (8 * pid_LoopDuration);
    #ifdef DEBUG_DRIVING_1
	Serial.println(pid.P);
	Serial.println(pid.I);
	Serial.println(pid.D);
    #endif
	return pid;
}
TOF_Optimal_Value Driving::getOptimalSensor(bool rampDown){
    TOF_Optimal_Value result;
	lastSensor = sensor;

	result.front = p_tof->GetRange(TofType::FRONT);
	result.back = p_tof->GetRange(TofType::BACK);

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

	//Check for ramps in front and back
	if (_RAMP_INFRONT) result.front = 1023;	//Ramp in front
	else if (_RAMP_BEHIND) result.back = 1023;	//Ramp in back
	_RAMP_INFRONT = false;
	_RAMP_BEHIND = false;

	#ifdef DEBUG_X64
	Serial.print("front: ");
	Serial.println(result.front);
	Serial.print("\back: ");
	Serial.println(result.back);
	#endif

	if (result.front <= 850 && result.back > 550)	result.type = ReferenceObj::FRONT;
	else if (result.front >= 850 && result.back <= 550)	result.type = ReferenceObj::BACK;
	else if (result.front >= 850 && result.back >= 550)	result.type = ReferenceObj::ENCODER;
	else if (result.front < result.back)	result.type = ReferenceObj::FRONT;
	else if (result.front > result.back)	result.type = ReferenceObj::BACK;

	if (rampDown) result.type = ReferenceObj::ENCODER;

    #ifdef DEBUG_DRIVING
	Serial.print("OPTIMAL");
	Serial.print("\tType: ");
	Serial.print(int(result.type));
	Serial.print("\tFront: ");
	Serial.print(result.front);
	Serial.print("\tBack: ");
	Serial.println(result.back);
    #endif // DEBUG
	return result;
}
void Driving::init(ColorSensing* p_colorSensing, TofSensors* p_tof, Gyro* p_gyro, Mapping* mapSys_pointer, Vcameras* cam_pointer, Drivetrain* p_drivetrain) {
    this->p_colorSensing = p_colorSensing;
    this->p_tof = p_tof;
    this->p_gyro = p_gyro;
	this->p_mapSys = mapSys_pointer;
	this->p_cams = cam_pointer;
    this->p_drivetrain = p_drivetrain;	//Pointer to Motor 

    //ENABLE Bumper pins
    pinMode(BUMPER_LEFT_PIN, INPUT);
	pinMode(BUMPER_RIGHT_PIN, INPUT);
}
#ifdef _MSC_VER
#pragma endregion GENERAL
#endif
