#pragma once
/**
* @name:    Gyro.h
* @date:    22.05.2026
* @authors: Florian Wiesner
* @details: Header file for Gyro Sensor Classes (GyroBase, GyroBNO055, GyroBNO085)
*/

#ifdef _MSC_VER
	#pragma region Libraries //--------------------------------------------------------------------------------------------------
#endif
#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include "CustomDatatypes.h"

#ifdef _MSC_VER
	#pragma endregion
	#pragma region GyroBase Class //----------------------------------------------------------------------------------------------
#endif
class GyroBase {
	protected:
		//----Members----
		float diffX = 0, diffY = 0, diffZ = 0;
		bool _ACCEL_ENABLED   = false;
		bool _GRAVITY_ENABLED = false;

		//----Methods----
		virtual float GetRawAngle(GyroAxles axis) = 0;

	public:
		virtual ~GyroBase() = default;

		/**
		* @brief  Initializes and configures the IMU.
		* @return OK if the sensor was initialized successfully. ERROR otherwise.
		*/
		virtual ErrorCodes Init(void) = 0;

		/**
		* @brief  Method to get an (absolute) angle measurement from the IMU (0-360°).
		* @param  axis enum to specify the axis (Axis_X, Axis_Y, Axis_Z).
		* @return current angle in degrees (0-360°).
		*/
		float GetAngle(GyroAxles axis);

		/**
		* @brief  Calculates control loop values from an existing angle measurement.
		*         Stores absolute angle, cartesian angle, angular error and turn direction in 'data'.
		* @param  targetAngle target angular value of the control loop (in degrees).
		* @param  actualAngle current angle to calculate from (in degrees).
		* @return absolute angle in degrees (0-360°).
		*/
		float GetAngleAdvanced(float targetAngle, float actualAngle);

		/**
		* @brief  Calculates control loop values by reading the specified IMU axis.
		*         Stores absolute angle, cartesian angle, angular error and turn direction in 'data'.
		* @param  targetAngle target angular value of the control loop (in degrees).
		* @param  axis enum to specify the axis to measure (Axis_X, Axis_Y, Axis_Z).
		* @return absolute angle in degrees (0-360°).
		*/
		float GetAngleAdvanced(float targetAngle, GyroAxles axis);

		/**
		* @brief  Method to get the angular value from a given orientation (NESW).
		* @param  orientation enum for the orientation (North, East, South, West).
		* @return corresponding angle in degrees (0, 90, 180, 270).
		*/
		float GetAngleFromOrientation(Orientations orientation);

		/**
		* @brief  Returns the orientation (NESW) corresponding to a given angular value.
		* @param  angle angular value in degrees (0-360°).
		* @return corresponding orientation (North, East, South, West).
		*/
		Orientations GetOrientationFromAngle(float angle);

		/**
		* @brief  Returns the current orientation (NESW) by reading the IMU (Axis_X).
		* @return corresponding orientation (North, East, South, West).
		*/
		Orientations GetOrientationFromAngle(void);

		/**
		* @brief  Method to get an acceleration measurement from the IMU. DISCLAIMER - not well tested.
		* @param  axis enum to specify the axis (Axis_X, Axis_Y, Axis_Z).
		* @return current acceleration, or 0 if the accelerometer is disabled.
		*/
		virtual float GetAcceleration(GyroAxles axis) = 0;

		/**
		* @brief  Method to get a gravity measurement from the IMU. DISCLAIMER - not well tested.
		* @param  axis enum to specify the axis (Axis_X, Axis_Y, Axis_Z).
		* @return current gravity, or 0 if gravity readings are disabled.
		*/
		virtual float GetGravity(GyroAxles axis) = 0;

		/**
		* @brief  Method to reset the angle of a specific axis. Zeros an axis.
		* @param  axis enum to specify the axis (Axis_X, Axis_Y, Axis_Z).
		*/
		void ResetAngle(GyroAxles axis);

		/**
		* @brief  Method to reset all angles of the IMU. Zeros the IMU.
		*/
		void ResetAllAngles(void);

		/**
		* @brief  Method to set an axis to a specific angle.
		* @param  axis enum to specify the axis (Axis_X, Axis_Y, Axis_Z).
		* @param  value angular value in degrees (0-360°).
		*/
		void SetAngle(GyroAxles axis, float value);

		/**
		* @brief  Method to get a temperature measurement from the IMU.
		* @return temperature in °C
		*/
		virtual int8_t GetTemp(void) = 0;

		/**
		* @brief  Enables accelerometer readings. On the BNO085 this also activates the SH2 report.
		*/
		virtual void EnableAccelerometer(void)  { _ACCEL_ENABLED = true;  }

		/**
		* @brief  Disables accelerometer readings. GetAcceleration() returns 0. On the BNO085 the SH2 report is stopped.
		*/
		virtual void DisableAccelerometer(void) { _ACCEL_ENABLED = false; }

		/**
		* @brief  Enables gravity readings. On the BNO085 this also activates the SH2 report.
		*/
		virtual void EnableGravity(void)        { _GRAVITY_ENABLED = true;  }

		/**
		* @brief  Disables gravity readings. GetGravity() returns 0. On the BNO085 the SH2 report is stopped.
		*/
		virtual void DisableGravity(void)       { _GRAVITY_ENABLED = false; }

		/**
		* @brief  Struct to store values used for control loops.
		* @param  angle_abs absolute angle in degrees (0-360°).
		* @param  angle_car cartesian angle in degrees (-180° to +180°)
		* @param  angle_error angular error between the target angle and the measurement (target-actual).
		* @param  direction_left true if a left-turn is the shortest option
		* @param  direction_right true if a right-turn is the shortest option
		*/
		GyroData data;
};

#ifdef _MSC_VER
	#pragma endregion
	#pragma region GyroBNO055 Class //-------------------------------------------------------------------------------------------
#endif
class GyroBNO055 : public GyroBase {
	private:
		//----Members----
		static constexpr uint8_t I2C_ADDRESS  = 0x28;
		// Flip to true if the IMU is mounted with Axis_Z polarity inverted (symptom: ramp-up and ramp-down detection are swapped).
		static constexpr bool    INVERT_AXIS_Z = true;
		Adafruit_BNO055 bno = Adafruit_BNO055(-1, I2C_ADDRESS, &Wire);

		//----Methods----
		float GetRawAngle(GyroAxles axis) override;

	public:
		GyroBNO055() = default;

		//----Methods----
		/**
		* @brief  Initializes and configures the BNO055 IMU.
		* @return OK if the sensor was initialized successfully. ERROR otherwise.
		*/
		ErrorCodes Init(void) override;
		/**
		* @brief  Returns the linear acceleration on the given axis.
		* @param  axis enum to specify the axis (Axis_X, Axis_Y, Axis_Z).
		* @return current acceleration, or 0 if the accelerometer is disabled.
		*/
		float GetAcceleration(GyroAxles axis) override;
		/**
		* @brief  Returns the gravity vector component on the given axis.
		* @param  axis enum to specify the axis (Axis_X, Axis_Y, Axis_Z).
		* @return current gravity, or 0 if gravity readings are disabled.
		*/
		float GetGravity(GyroAxles axis) override;
		/**
		* @brief  Returns the current temperature from the BNO055.
		* @return temperature in °C.
		*/
		int8_t GetTemp(void) override;
};

#ifdef _MSC_VER
	#pragma endregion
	#pragma region GyroBNO085 Class //-------------------------------------------------------------------------------------------
#endif
/**
* @warning  Do not use GyroBNO085 for the robot. Reading results requires draining the SH2 event
*           queue on every call (PollEvents), which has inconsistent timing and is measurably
*           slower than the BNO055 polling path. Use GyroBNO055 instead.
*/
class GyroBNO085 : public GyroBase {
	private:
		//----Members----
		static constexpr uint8_t I2C_ADDRESS = 0x4A;
		Adafruit_BNO08x bno = Adafruit_BNO08x(-1);
		float yaw = 0, pitch = 0, roll = 0;
		float accX = 0, accY = 0, accZ = 0;
		float grvX = 0, grvY = 0, grvZ = 0;

		//----Methods----
		void PollEvents(void);
		float GetRawAngle(GyroAxles axis) override;

	public:
		GyroBNO085() = default;

		//----Methods----
		/**
		* @brief  Initializes and configures the BNO085 IMU.
		* @return OK if the sensor was initialized successfully. ERROR otherwise.
		*/
		ErrorCodes Init(void) override;
		/**
		* @brief  Returns the linear acceleration on the given axis.
		* @param  axis enum to specify the axis (Axis_X, Axis_Y, Axis_Z).
		* @return current acceleration, or 0 if the accelerometer is disabled.
		*/
		float GetAcceleration(GyroAxles axis) override;
		/**
		* @brief  Returns the gravity vector component on the given axis.
		* @param  axis enum to specify the axis (Axis_X, Axis_Y, Axis_Z).
		* @return current gravity, or 0 if gravity readings are disabled.
		*/
		float GetGravity(GyroAxles axis) override;
		/**
		* @brief  Returns 0; temperature reading is not implemented on the BNO085.
		* @return 0.
		*/
		int8_t GetTemp(void) override;
		/**
		* @brief  Enables accelerometer readings and activates the SH2 accelerometer report.
		*/
		void EnableAccelerometer(void) override;
		/**
		* @brief  Disables accelerometer readings and stops the SH2 accelerometer report.
		*/
		void DisableAccelerometer(void) override;
		/**
		* @brief  Enables gravity readings and activates the SH2 gravity report.
		*/
		void EnableGravity(void) override;
		/**
		* @brief  Disables gravity readings and stops the SH2 gravity report.
		*/
		void DisableGravity(void) override;
};
#ifdef _MSC_VER
	#pragma endregion
#endif
