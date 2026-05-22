/**
* @name:    Gyro.cpp
* @date:    22.05.2026
* @authors: Florian Wiesner
* @details: .cpp file for Gyro Sensor Classes (GyroBase, Gyro_BNO055, Gyro_BNO085)
*/

//----Libraries----
#include "Gyro.h"

#ifdef _MSC_VER
	#pragma region GyroBase — Angle //-------------------------------------------------------------------------------------------
#endif
float GyroBase::GetAngle(GyroAxles axis) {
	float diff = 0;
	if      (axis == GyroAxles::Axis_X) diff = diff_x;
	else if (axis == GyroAxles::Axis_Y) diff = diff_y;
	else if (axis == GyroAxles::Axis_Z) diff = diff_z;
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
	if      (axis == GyroAxles::Axis_X) diff_x = -GetRawAngle(GyroAxles::Axis_X);
	else if (axis == GyroAxles::Axis_Y) diff_y = -GetRawAngle(GyroAxles::Axis_Y);
	else if (axis == GyroAxles::Axis_Z) diff_z = -GetRawAngle(GyroAxles::Axis_Z);
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
		diff_x = diff_x + value;
		if (diff_x > 360) diff_x -= 360;
		if (diff_x < 0)   diff_x += 360;
	}
	else if (axis == GyroAxles::Axis_Y) {
		ResetAngle(axis);
		diff_y = diff_y + value;
		if (diff_y > 360) diff_y -= 360;
		if (diff_y < 0)   diff_y += 360;
	}
	else if (axis == GyroAxles::Axis_Z) {
		ResetAngle(axis);
		diff_z = diff_z + value;
		if (diff_z > 360) diff_z -= 360;
		if (diff_z < 0)   diff_z += 360;
	}
}

#ifdef _MSC_VER
	#pragma endregion
	#pragma region Gyro_BNO055 — Init //-----------------------------------------------------------------------------------------
#endif
ErrorCodes Gyro_BNO055::Init(void) {
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
	#pragma region Gyro_BNO055 — Accel & Gravity //------------------------------------------------------------------------------
#endif
float Gyro_BNO055::GetAcceleration(GyroAxles axis) {
	sensors_event_t event;
	bno.getEvent(&event, Adafruit_BNO055::VECTOR_ACCELEROMETER);

	if      (axis == GyroAxles::Axis_X) return event.acceleration.x;
	else if (axis == GyroAxles::Axis_Y) return event.acceleration.y;
	else if (axis == GyroAxles::Axis_Z) return event.acceleration.z;
	return 0;
}

float Gyro_BNO055::GetGravity(GyroAxles axis) {
	sensors_event_t event;
	bno.getEvent(&event, Adafruit_BNO055::VECTOR_GRAVITY);

	if      (axis == GyroAxles::Axis_X) return event.acceleration.x;
	else if (axis == GyroAxles::Axis_Y) return event.acceleration.y;
	else if (axis == GyroAxles::Axis_Z) return event.acceleration.z;
	return 0;
}

#ifdef _MSC_VER
	#pragma endregion
	#pragma region Gyro_BNO055 — Temp //-----------------------------------------------------------------------------------------
#endif
int8_t Gyro_BNO055::GetTemp(void) {
	return bno.getTemp();
}

#ifdef _MSC_VER
	#pragma endregion
	#pragma region Gyro_BNO055 — Private //--------------------------------------------------------------------------------------
#endif
// Private method to get raw angle from sensor; not for direct use in main program
float Gyro_BNO055::GetRawAngle(GyroAxles axis) {
	sensors_event_t event;
	bno.getEvent(&event);

	if      (axis == GyroAxles::Axis_X) return event.orientation.x;
	else if (axis == GyroAxles::Axis_Y) return event.orientation.y;
	else if (axis == GyroAxles::Axis_Z) return event.orientation.z;
	return 0;
}

#ifdef _MSC_VER
	#pragma endregion
	#pragma region Gyro_BNO085 — Init //-----------------------------------------------------------------------------------------
#endif
ErrorCodes Gyro_BNO085::Init(void) {
	if (!bno.begin_I2C(I2C_ADDRESS, &Wire)) {
		#ifdef DEBUG
		Serial.println("Could not find a valid BNO085 sensor!");
		#endif
		return ErrorCodes::ERROR;
	}
	if (bno.wasReset()) {
		bno.enableReport(SH2_ARVR_STABILIZED_RV, 5000);
		bno.enableReport(SH2_ACCELEROMETER);
		bno.enableReport(SH2_GRAVITY);
	}
	delay(500);
	ResetAllAngles();
	return ErrorCodes::OK;
}

#ifdef _MSC_VER
	#pragma endregion
	#pragma region Gyro_BNO085 — Accel & Gravity //------------------------------------------------------------------------------
#endif
float Gyro_BNO085::GetAcceleration(GyroAxles axis) {
	PollEvents();
	if      (axis == GyroAxles::Axis_X) return acc_x_;
	else if (axis == GyroAxles::Axis_Y) return acc_y_;
	else if (axis == GyroAxles::Axis_Z) return acc_z_;
	return 0;
}

float Gyro_BNO085::GetGravity(GyroAxles axis) {
	PollEvents();
	if      (axis == GyroAxles::Axis_X) return grv_x_;
	else if (axis == GyroAxles::Axis_Y) return grv_y_;
	else if (axis == GyroAxles::Axis_Z) return grv_z_;
	return 0;
}

#ifdef _MSC_VER
	#pragma endregion
	#pragma region Gyro_BNO085 — Temp //-----------------------------------------------------------------------------------------
#endif
int8_t Gyro_BNO085::GetTemp(void) {
	return 0;
}

#ifdef _MSC_VER
	#pragma endregion
	#pragma region Gyro_BNO085 — Private //--------------------------------------------------------------------------------------
#endif
// Drains the BNO085 event queue and updates the per-report caches; re-enables reports on sensor reset
void Gyro_BNO085::PollEvents(void) {
	if (bno.wasReset()) {
		bno.enableReport(SH2_ARVR_STABILIZED_RV, 5000);
		bno.enableReport(SH2_ACCELEROMETER);
		bno.enableReport(SH2_GRAVITY);
	}

	sh2_SensorValue_t val;
	while (bno.getSensorEvent(&val)) {
		if (val.sensorId == SH2_ARVR_STABILIZED_RV) {
			float qr = val.un.arvrStabilizedRV.real;
			float qi = val.un.arvrStabilizedRV.i;
			float qj = val.un.arvrStabilizedRV.j;
			float qk = val.un.arvrStabilizedRV.k;
			yaw_   = atan2(2*(qi*qj + qk*qr),  sq(qi) - sq(qj) - sq(qk) + sq(qr)) * RAD_TO_DEG;
			pitch_ = asin(-2*(qi*qk - qj*qr) / (sq(qi) + sq(qj) + sq(qk) + sq(qr))) * RAD_TO_DEG;
			roll_  = atan2(2*(qj*qk + qi*qr), -sq(qi) - sq(qj) + sq(qk) + sq(qr))  * RAD_TO_DEG;
			if (yaw_ < 0) yaw_ += 360;
		}
		else if (val.sensorId == SH2_ACCELEROMETER) {
			acc_x_ = val.un.accelerometer.x;
			acc_y_ = val.un.accelerometer.y;
			acc_z_ = val.un.accelerometer.z;
		}
		else if (val.sensorId == SH2_GRAVITY) {
			grv_x_ = val.un.gravity.x;
			grv_y_ = val.un.gravity.y;
			grv_z_ = val.un.gravity.z;
		}
	}
}

// Private method to get raw angle from sensor; not for direct use in main program
float Gyro_BNO085::GetRawAngle(GyroAxles axis) {
	PollEvents();
	if      (axis == GyroAxles::Axis_X) return yaw_;
	else if (axis == GyroAxles::Axis_Y) return pitch_;
	else if (axis == GyroAxles::Axis_Z) return roll_;
	return 0;
}

#ifdef _MSC_VER
	#pragma endregion
#endif
