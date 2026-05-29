/**
* @name:    Gyro.cpp
* @date:    22.05.2026
* @authors: Florian Wiesner
* @details: .cpp file for Gyro Sensor Classes (GyroBase, GyroBNO055, GyroBNO085)
*/

//----Libraries----
#include "Gyro.h"

#ifdef _MSC_VER
	#pragma region GyroBase — Angle //-------------------------------------------------------------------------------------------
#endif
float GyroBase::GetAngle(GyroAxles axis) {
	float diff = 0;
	if      (axis == GyroAxles::Axis_X) diff = diffX;
	else if (axis == GyroAxles::Axis_Y) diff = diffY;
	else if (axis == GyroAxles::Axis_Z) diff = diffZ;
	else return 0;

	float in = GetRawAngle(axis) + diff;
	if (in > 360) return in - 360;
	if (in < 0)   return in + 360;
	return in;
}

float GyroBase::GetAngleAdvanced(float targetAngle, float actualAngle) {
	// Get and constrain values
	targetAngle = constrain(targetAngle, 0, 360);
	data.angle_abs = constrain(actualAngle, 0, 360);

	// calculate angular error (always between -180° and +180°)
	float angleError = targetAngle - data.angle_abs;
	if (angleError > 180)  angleError -= 360;
	if (angleError < -180) angleError += 360;

	// store the angular error
	data.angle_error = angleError;

	// set the direction, based on shortest turn
	if (angleError > 0) {
		data.direction_left  = true;
		data.direction_right = false;
	}
	else if (angleError < 0) {
		data.direction_left  = false;
		data.direction_right = true;
	}
	else {
		data.direction_left  = false;
		data.direction_right = false;
	}

	// exactly 180°-turn: always left
	if (abs(angleError) == 180) {
		data.direction_left  = true;
		data.direction_right = false;
	}

	// calculate the cartesian angle (-180° to +180°)
	if (data.angle_abs <= 180) {
		data.angle_car = data.angle_abs;
	}
	else {
		data.angle_car = data.angle_abs - 360;
	}

	#ifdef DEBUG_GYRO_ADVANCED
	Serial.print("target: ");
	Serial.print(targetAngle);
	Serial.print("\tabs: ");
	Serial.print(data.angle_abs);
	Serial.print("\tcar: ");
	Serial.print(data.angle_car);
	Serial.print("\terror: ");
	Serial.print(data.angle_error);
	Serial.print("\tleft: ");
	Serial.print(data.direction_left);
	Serial.print("\tright: ");
	Serial.println(data.direction_right);
	#endif

	return data.angle_abs;
}

float GyroBase::GetAngleAdvanced(float targetAngle, GyroAxles axis) {
	return GetAngleAdvanced(targetAngle, GetAngle(axis));
}

#ifdef _MSC_VER
	#pragma endregion
	#pragma region GyroBase — Orientation //--------------------------------------------------------------------------------------
#endif
float GyroBase::GetAngleFromOrientation(Orientations orientation) {
	switch (orientation) {
	case Orientations::North: return 0;
	case Orientations::East:  return 90;
	case Orientations::South: return 180;
	case Orientations::West:  return 270;
	default: return 0;
	}
}

Orientations GyroBase::GetOrientationFromAngle(float angle) {
	angle = constrain(angle, 0, 360);
	if      (angle >= 315 || angle < 45)  return Orientations::North;
	else if (angle >= 45  && angle < 135) return Orientations::East;
	else if (angle >= 135 && angle < 225) return Orientations::South;
	else if (angle >= 225 && angle < 315) return Orientations::West;
	return Orientations::North;
}

Orientations GyroBase::GetOrientationFromAngle(void) {
	return GetOrientationFromAngle(GetAngle(GyroAxles::Axis_X));
}

#ifdef _MSC_VER
	#pragma endregion
	#pragma region GyroBase — Reset //--------------------------------------------------------------------------------------------
#endif
void GyroBase::ResetAngle(GyroAxles axis) {
	if      (axis == GyroAxles::Axis_X) diffX = -GetRawAngle(GyroAxles::Axis_X);
	else if (axis == GyroAxles::Axis_Y) diffY = -GetRawAngle(GyroAxles::Axis_Y);
	else if (axis == GyroAxles::Axis_Z) diffZ = -GetRawAngle(GyroAxles::Axis_Z);
}

void GyroBase::ResetAllAngles(void) {
	ResetAngle(GyroAxles::Axis_X);
	ResetAngle(GyroAxles::Axis_Y);
	ResetAngle(GyroAxles::Axis_Z);
}

void GyroBase::SetAngle(GyroAxles axis, float value) {
	value = constrain(value, 0, 360);

	if (axis == GyroAxles::Axis_X) {
		ResetAngle(axis);
		diffX = diffX + value;
		if (diffX > 360) diffX -= 360;
		if (diffX < 0)   diffX += 360;
	}
	else if (axis == GyroAxles::Axis_Y) {
		ResetAngle(axis);
		diffY = diffY + value;
		if (diffY > 360) diffY -= 360;
		if (diffY < 0)   diffY += 360;
	}
	else if (axis == GyroAxles::Axis_Z) {
		ResetAngle(axis);
		diffZ = diffZ + value;
		if (diffZ > 360) diffZ -= 360;
		if (diffZ < 0)   diffZ += 360;
	}
}

#ifdef _MSC_VER
	#pragma endregion
	#pragma region GyroBNO055 — Init //-------------------------------------------------------------------------------------------
#endif
ErrorCodes GyroBNO055::Init(void) {
	if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
		#ifdef DEBUG
		Serial.println("Could not find a valid BNO055 sensor!");
		#endif
		return ErrorCodes::ERROR;
	}
	bno.setExtCrystalUse(true);
	delay(500);
	ResetAllAngles();
	return ErrorCodes::OK;
}

#ifdef _MSC_VER
	#pragma endregion
	#pragma region GyroBNO055 — Accel & Gravity //--------------------------------------------------------------------------------
#endif
float GyroBNO055::GetAcceleration(GyroAxles axis) {
	if (!_ACCEL_ENABLED) return 0;
	sensors_event_t event;
	bno.getEvent(&event, Adafruit_BNO055::VECTOR_ACCELEROMETER);

	if      (axis == GyroAxles::Axis_X) return event.acceleration.x;
	else if (axis == GyroAxles::Axis_Y) return event.acceleration.y;
	else if (axis == GyroAxles::Axis_Z) return event.acceleration.z;
	return 0;
}

float GyroBNO055::GetGravity(GyroAxles axis) {
	if (!_GRAVITY_ENABLED) return 0;
	sensors_event_t event;
	bno.getEvent(&event, Adafruit_BNO055::VECTOR_GRAVITY);

	if      (axis == GyroAxles::Axis_X) return event.acceleration.x;
	else if (axis == GyroAxles::Axis_Y) return event.acceleration.y;
	else if (axis == GyroAxles::Axis_Z) return event.acceleration.z;
	return 0;
}

#ifdef _MSC_VER
	#pragma endregion
	#pragma region GyroBNO055 — Temp //-------------------------------------------------------------------------------------------
#endif
int8_t GyroBNO055::GetTemp(void) {
	return bno.getTemp();
}

#ifdef _MSC_VER
	#pragma endregion
	#pragma region GyroBNO055 — Private //----------------------------------------------------------------------------------------
#endif
// Private method to get raw angle from sensor; not for direct use in main program
float GyroBNO055::GetRawAngle(GyroAxles axis) {
	sensors_event_t event;
	bno.getEvent(&event);

	if      (axis == GyroAxles::Axis_X) return event.orientation.x;
	else if (axis == GyroAxles::Axis_Y) return event.orientation.y;
	else if (axis == GyroAxles::Axis_Z) return INVERT_AXIS_Z ? -event.orientation.z : event.orientation.z;
	return 0;
}

#ifdef _MSC_VER
	#pragma endregion
	#pragma region GyroBNO085 — Init //-------------------------------------------------------------------------------------------
#endif
ErrorCodes GyroBNO085::Init(void) {
	if (!bno.begin_I2C(I2C_ADDRESS, &Wire)) {
		#ifdef DEBUG
		Serial.println("Could not find a valid BNO085 sensor!");
		#endif
		return ErrorCodes::ERROR;
	}
	bno.wasReset(); // clear reset flag set by begin_I2C
	bno.enableReport(SH2_GAME_ROTATION_VECTOR);

	// Actively poll until first valid orientation packet arrives
	sh2_SensorValue_t val;
	uint32_t deadline = millis() + 2000;
	while (millis() < deadline) {
		if (bno.getSensorEvent(&val) && val.sensorId == SH2_GAME_ROTATION_VECTOR) break;
		delay(1);
	}
	ResetAllAngles();
	return ErrorCodes::OK;
}

#ifdef _MSC_VER
	#pragma endregion
	#pragma region GyroBNO085 — Accel & Gravity //--------------------------------------------------------------------------------
#endif
float GyroBNO085::GetAcceleration(GyroAxles axis) {
	if (!_ACCEL_ENABLED) return 0;
	PollEvents();
	if      (axis == GyroAxles::Axis_X) return accX;
	else if (axis == GyroAxles::Axis_Y) return accY;
	else if (axis == GyroAxles::Axis_Z) return accZ;
	return 0;
}

float GyroBNO085::GetGravity(GyroAxles axis) {
	if (!_GRAVITY_ENABLED) return 0;
	PollEvents();
	if      (axis == GyroAxles::Axis_X) return grvX;
	else if (axis == GyroAxles::Axis_Y) return grvY;
	else if (axis == GyroAxles::Axis_Z) return grvZ;
	return 0;
}

#ifdef _MSC_VER
	#pragma endregion
	#pragma region GyroBNO085 — Temp //-------------------------------------------------------------------------------------------
#endif
int8_t GyroBNO085::GetTemp(void) {
	return 0;
}

#ifdef _MSC_VER
	#pragma endregion
	#pragma region GyroBNO085 — Enable/Disable //---------------------------------------------------------------------------------
#endif
void GyroBNO085::EnableAccelerometer(void) {
	GyroBase::EnableAccelerometer();
	bno.enableReport(SH2_ACCELEROMETER, 5000);
}

void GyroBNO085::DisableAccelerometer(void) {
	GyroBase::DisableAccelerometer();
	bno.enableReport(SH2_ACCELEROMETER, 0);
}

void GyroBNO085::EnableGravity(void) {
	GyroBase::EnableGravity();
	bno.enableReport(SH2_GRAVITY, 5000);
}

void GyroBNO085::DisableGravity(void) {
	GyroBase::DisableGravity();
	bno.enableReport(SH2_GRAVITY, 0);
}

#ifdef _MSC_VER
	#pragma endregion
	#pragma region GyroBNO085 — Private //----------------------------------------------------------------------------------------
#endif
// Drains the BNO085 event queue and updates the per-report caches; re-enables reports on sensor reset
void GyroBNO085::PollEvents(void) {
	if (bno.wasReset()) {
		bno.enableReport(SH2_GAME_ROTATION_VECTOR);
		if (_ACCEL_ENABLED)   bno.enableReport(SH2_ACCELEROMETER, 5000);
		if (_GRAVITY_ENABLED) bno.enableReport(SH2_GRAVITY, 5000);
	}

	sh2_SensorValue_t val;
	while (bno.getSensorEvent(&val)) {
		if (val.sensorId == SH2_GAME_ROTATION_VECTOR) {
			float qr = val.un.gameRotationVector.real;
			float qi = val.un.gameRotationVector.i;
			float qj = val.un.gameRotationVector.j;
			float qk = val.un.gameRotationVector.k;
			yaw   = atan2(2*(qi*qj + qk*qr),  sq(qi) - sq(qj) - sq(qk) + sq(qr)) * RAD_TO_DEG;
			pitch = asin(-2*(qi*qk - qj*qr) / (sq(qi) + sq(qj) + sq(qk) + sq(qr))) * RAD_TO_DEG;
			roll  = atan2(2*(qj*qk + qi*qr), -sq(qi) - sq(qj) + sq(qk) + sq(qr))  * RAD_TO_DEG;
			if (yaw < 0) yaw += 360;
		}
		else if (val.sensorId == SH2_ACCELEROMETER) {
			accX = val.un.accelerometer.x;
			accY = val.un.accelerometer.y;
			accZ = val.un.accelerometer.z;
		}
		else if (val.sensorId == SH2_GRAVITY) {
			grvX = val.un.gravity.x;
			grvY = val.un.gravity.y;
			grvZ = val.un.gravity.z;
		}
	}
}

// Private method to get raw angle from sensor; not for direct use in main program
float GyroBNO085::GetRawAngle(GyroAxles axis) {
	PollEvents();
	if      (axis == GyroAxles::Axis_X) return yaw;
	else if (axis == GyroAxles::Axis_Y) return pitch;
	else if (axis == GyroAxles::Axis_Z) return roll;
	return 0;
}

#ifdef _MSC_VER
	#pragma endregion
#endif
