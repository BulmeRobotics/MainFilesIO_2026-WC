#pragma once
/**
* @name:    Gyro.h
* @date:    22.05.2026
* @authors: Florian Wiesner
* @details: Header file for Gyro Sensor Classes (GyroBase, Gyro_BNO055, Gyro_BNO085)
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
		float diff_x = 0, diff_y = 0, diff_z = 0;

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
		* @return current acceleration.
		*/
		virtual float GetAcceleration(GyroAxles axis) = 0;

		/**
		* @brief  Method to get a gravity measurement from the IMU. DISCLAIMER - not well tested.
		* @param  axis enum to specify the axis (Axis_X, Axis_Y, Axis_Z).
		* @return current gravity.
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
	#pragma region Gyro_BNO055 Class //-------------------------------------------------------------------------------------------
#endif
class Gyro_BNO055 : public GyroBase {
	private:
		//----Configuration----
		static constexpr uint8_t I2C_ADDRESS = 0x28;

		//----Object----
		Adafruit_BNO055 bno = Adafruit_BNO055(-1, I2C_ADDRESS, &Wire);

		//----Methods----
		float GetRawAngle(GyroAxles axis) override;

	public:
		//----Constructor----
		Gyro_BNO055() = default;

		//----Methods----
		ErrorCodes Init(void) override;
		float GetAcceleration(GyroAxles axis) override;
		float GetGravity(GyroAxles axis) override;
		int8_t GetTemp(void) override;
};

#ifdef _MSC_VER
	#pragma endregion
	#pragma region Gyro_BNO085 Class //-------------------------------------------------------------------------------------------
#endif
class Gyro_BNO085 : public GyroBase {
	private:
		//----Configuration----
		static constexpr uint8_t I2C_ADDRESS = 0x4A;

		//----Object----
		Adafruit_BNO08x bno = Adafruit_BNO08x(-1);

		//----Members----
		float yaw_ = 0, pitch_ = 0, roll_ = 0;
		float acc_x_ = 0, acc_y_ = 0, acc_z_ = 0;
		float grv_x_ = 0, grv_y_ = 0, grv_z_ = 0;

		//----Methods----
		void PollEvents(void);
		float GetRawAngle(GyroAxles axis) override;

	public:
		//----Constructor----
		Gyro_BNO085() = default;

		//----Methods----
		ErrorCodes Init(void) override;
		float GetAcceleration(GyroAxles axis) override;
		float GetGravity(GyroAxles axis) override;
		int8_t GetTemp(void) override;
};
#ifdef _MSC_VER
	#pragma endregion
#endif
