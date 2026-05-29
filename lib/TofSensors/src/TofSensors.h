#pragma once
/**
* @name:    TofSensors.h
* @date:    18.11.2025
* @authors: Florian Wiesner
* @details: Header file for Time-of-Flight-Sensors
*/

#ifdef _MSC_VER
    #pragma region Libraries //--------------------------------------------------------------------------------------------------
#endif
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>
#include "VL6180X_Custom/VL6180X_Custom.h"
#include "VL53L4CD_Custom/src/vl53l4cd_class.h"
#include "VL53L4CD_Custom/src/vl53l4cd_api.h"
#include "CustomDatatypes.h"

static constexpr uint8_t RANGING_BUDGET_SHORT = 30;
static constexpr uint8_t RANGING_BUDGET_MID   = 10;

#ifdef _MSC_VER
    #pragma endregion
    #pragma region Parent Class //-----------------------------------------------------------------------------------------------
#endif
class TofParent {
    public:
        //----Constructor----
        /**
        * @brief  Constructs a ToF sensor base object.
        * @param  i2cAddress  I2C address to assign to this sensor.
        * @param  xshutPin    GPIO pin connected to the sensor's XSHUT line.
        */
        TofParent(uint8_t i2cAddress, uint8_t xshutPin);

        //----Destructor----
        virtual ~TofParent() = default;

        //----Methods----
        /**
        * @brief  Initializes and configures the sensor.
        * @return true if the sensor was initialized successfully.
        */
        virtual bool Init(void);

        /**
        * @brief  Checks if a new measurement is available; if so, retrieves the data.
        * @return OK if no new data is available.
        *         NEW_DATA if data was retrieved and stored.
        *         NO_NEW_DATA if the sensor is not ready yet.
        *         OUT_OF_RANGE if measurement was out of range.
        *         TIMEOUT if a timeout occurred.
        *         ERROR if a sensor communication error occurred.
        */
        virtual ErrorCodes Read(void);

        /**
        * @brief  Getter-method to get the value of the last measurement.
        * @return Last measurement value.
        */
        virtual uint16_t GetRange(void);

        /**
        * @brief  Getter-method to get the status of the last measurement.
        * @return VALID if the measurement is within range and no timeout occurred.
        *         OUT_OF_RANGE if the measurement was out of range.
        *         TIMEOUT if the sensor is in a timeout.
        *         ERROR if a sensor communication error occurred.
        */
        virtual TofStatus GetStatus(void);

        /**
        * @brief  Method to check if a range value has yet been retrieved.
        * @return true if value is new.
        *         false if the value was already used.
        */
        virtual bool IsDataNew(void);

        /**
        * @brief  Method to check if a timeout has occurred.
        * @return true if a timeout occurred.
        *         false if no timeout occurred.
        */
        virtual bool TimeoutOccured(void);

        /**
        * @brief  Stops the ranging of the sensor.
        * @return OK if the sensor was stopped successfully.
        *         ERROR if something went wrong.
        */
        virtual ErrorCodes Stop(void);

        /**
        * @brief  Continues the ranging of the sensor.
        * @return OK if the sensor is working correctly.
        *         ERROR if something went wrong.
        */
        virtual ErrorCodes Continue(void);

        /**
        * @brief  Disables the sensor by turning the XSHUT pin LOW.
        */
        void Off(void);

        /**
        * @brief  Enables the sensor by releasing the XSHUT pin.
        */
        void On(void);

    protected:
        //----Members----
        uint8_t xshutPin;               // to store the XSHUT Pin of each sensor
        uint8_t i2cAddress;             // address of each sensor
        uint16_t lastMeasurement;       // value of the last range measurement
        TofStatus lastStatus;           // status of the last measurement
        uint32_t measurementCount;      // number of measurements since start
        uint32_t ts_lastMeasurement;    // timestamp of the last measurement; to detect a TIMEOUT
        uint32_t ts_waitStart;          // start of the current wait for the data
        uint32_t ts_lastReadCall;       // last call of Read()
        bool timeoutOccured;
        bool waitingForMeasurement;
        bool newData;                   // bool to store if a measurement is new or has been read used already

        //----Configuration----
        static constexpr uint16_t TIMEOUT_TIME_MS    = 500;
        static constexpr uint32_t INVALID_MEASUREMENT = 1023;
};

#ifdef _MSC_VER
    #pragma endregion
    #pragma region VL6180X //----------------------------------------------------------------------------------------------------
#endif
class TofVL6180X : public TofParent {
    private:
        //----Object----
        VL6180X sensor;

    public:
        //----Constructor----
        /**
        * @brief  Constructs a VL6180X ToF sensor object.
        * @param  i2cAddress  I2C address to assign to this sensor.
        * @param  xshutPin    GPIO pin connected to the sensor's XSHUT line.
        */
        TofVL6180X(uint8_t i2cAddress, uint8_t xshutPin);

        //----Methods----
        /**
        * @brief  Initializes and configures the VL6180X sensor.
        * @return true if the sensor was initialized successfully.
        */
        bool Init(void) override;

        /**
        * @brief  Checks if a new measurement is available; if so, retrieves the data.
        * @return NEW_DATA if data was retrieved and stored.
        *         OUT_OF_RANGE if measurement was out of range.
        *         TIMEOUT if a timeout occurred.
        *         OK if no new data is available.
        */
        ErrorCodes Read(void) override;

        /**
        * @brief  Getter-method to get the value of the last measurement.
        * @return Last measurement value (0-255).
        */
        uint16_t GetRange(void) override;

        /**
        * @brief  Stops the ranging of the VL6180X sensor.
        * @return OK if the sensor was stopped successfully.
        *         TIMEOUT if a timeout was detected on stop.
        */
        ErrorCodes Stop(void) override;

        /**
        * @brief  Continues the ranging of the VL6180X sensor.
        * @return OK always.
        */
        ErrorCodes Continue(void) override;
};

#ifdef _MSC_VER
    #pragma endregion
    #pragma region VL53L4CD //---------------------------------------------------------------------------------------------------
#endif
class TofVL53L4CD : public TofParent {
    private:
        //----Object----
        VL53L4CD sensor;

    public:
        //----Constructor----
        /**
        * @brief  Constructs a VL53L4CD ToF sensor object.
        * @param  i2cAddress  I2C address to assign to this sensor.
        * @param  xshutPin    GPIO pin connected to the sensor's XSHUT line.
        */
        TofVL53L4CD(uint8_t i2cAddress, uint8_t xshutPin);

        //----Methods----
        /**
        * @brief  Initializes and configures the VL53L4CD sensor.
        * @return true if the sensor was initialized successfully.
        */
        bool Init(void) override;

        /**
        * @brief  Checks if a new measurement is available; if so, retrieves the data.
        * @return NEW_DATA if data was retrieved and stored (both valid and out-of-range results).
        *         NO_NEW_DATA if the sensor is not ready yet.
        *         TIMEOUT if the sensor did not respond within the timeout window.
        *         ERROR if a sensor communication error occurred.
        */
        ErrorCodes Read(void) override;

        /**
        * @brief  Getter-method to get the value of the last measurement. Clears the new-data flag.
        * @return Last measurement value in mm (INVALID_MEASUREMENT = 1023 if out of range).
        */
        uint16_t GetRange(void) override;

        /**
        * @brief  Stops the ranging of the VL53L4CD sensor.
        * @return OK always.
        */
        ErrorCodes Stop(void) override;

        /**
        * @brief  Continues the ranging of the VL53L4CD sensor.
        * @return OK always.
        */
        ErrorCodes Continue(void) override;
};

#ifdef _MSC_VER
    #pragma endregion
    #pragma region VL53L5CX //---------------------------------------------------------------------------------------------------
#endif
class TofVL53L5CX : public TofParent {
    private:
        //----Configuration----
        static constexpr uint16_t FRONT_MAX_DETECTION_DISTANCE = 450;
        static constexpr uint8_t  FRONT_DIFF_DETECTION_VALUE   = 25;

        //----Object----
        SparkFun_VL53L5CX sensor;

        //----Members----
        VL53L5CX_ResultsData measurementData;
        uint16_t imageResolution = 64;
        uint16_t imageWidth = sqrt(imageResolution);
        uint8_t messurePointsUpper[2] = {35, 27};
        uint8_t messurePointsLower[2] = {36, 28};

    public:
        //----Constructor----
        /**
        * @brief  Constructs a VL53L5CX 8×8 ToF sensor object.
        * @param  i2cAddress  I2C address to assign to this sensor.
        * @param  xshutPin    GPIO pin connected to the sensor's XSHUT line.
        */
        TofVL53L5CX(uint8_t i2cAddress, uint8_t xshutPin);

        //----Methods----
        /**
        * @brief  Initializes and configures the specific Time-of-Flight-Sensor.
        * @return true if the sensor was initialized successfully.
        */
        bool Init(void) override;

        /**
        * @brief  Checks if a ramp is in front of or behind the robot.
        * @return true if a ramp was detected.
        *         false if no ramp was detected.
        */
        bool IsRamp(void);
};

#ifdef _MSC_VER
    #pragma endregion
    #pragma region TOF Class //--------------------------------------------------------------------------------------------------
#endif
class TofSensors {
    private:
        //----Configuration----
        static constexpr uint8_t I2C_ADDRESS_MLF  = 0x64;
        static constexpr uint8_t I2C_ADDRESS_MLB  = 0x68;
        static constexpr uint8_t I2C_ADDRESS_MRF  = 0x6C;
        static constexpr uint8_t I2C_ADDRESS_MRB  = 0x70;
        static constexpr uint8_t I2C_ADDRESS_MF   = 0x74;
        static constexpr uint8_t I2C_ADDRESS_MB   = 0x78;
        static constexpr uint8_t I2C_ADDRESS_MFW  = 0x7C;
        static constexpr uint8_t I2C_ADDRESS_MBW  = 0x80;
        // static constexpr uint8_t I2C_ADDRESS_Fx64 = 0x46;
        // static constexpr uint8_t I2C_ADDRESS_Bx64 = 0x47;

        static constexpr int XSHUT_PIN_MLF  = A2;
        static constexpr int XSHUT_PIN_MLB  = A5;
        static constexpr int XSHUT_PIN_MRF  = A3;
        static constexpr int XSHUT_PIN_MRB  = A4;
        static constexpr int XSHUT_PIN_MF   = A7;
        static constexpr int XSHUT_PIN_MB   = A6;
        static constexpr int XSHUT_PIN_MFW  = 32;
        static constexpr int XSHUT_PIN_MBW  = 50;
        // static constexpr int XSHUT_PIN_Fx64 = 32;
        // static constexpr int XSHUT_PIN_Bx64 = 26;

        //----Members----
        TofVL53L4CD leftBack   = TofVL53L4CD(I2C_ADDRESS_MLB, XSHUT_PIN_MLB);
        TofVL53L4CD leftFront  = TofVL53L4CD(I2C_ADDRESS_MLF, XSHUT_PIN_MLF);
        TofVL53L4CD rightFront = TofVL53L4CD(I2C_ADDRESS_MRF, XSHUT_PIN_MRF);
        TofVL53L4CD rightBack  = TofVL53L4CD(I2C_ADDRESS_MRB, XSHUT_PIN_MRB);
        TofVL53L4CD front      = TofVL53L4CD(I2C_ADDRESS_MF, XSHUT_PIN_MF);
        TofVL53L4CD back       = TofVL53L4CD(I2C_ADDRESS_MB, XSHUT_PIN_MB);
        TofVL53L4CD frontWall  = TofVL53L4CD(I2C_ADDRESS_MFW, XSHUT_PIN_MFW);
        TofVL53L4CD backWall   = TofVL53L4CD(I2C_ADDRESS_MBW, XSHUT_PIN_MBW);
        
        bool _UPDATE_ENABLED = true;

        //----Methods----
        void DisableAll(void);

    public:
        //----Constructor----
        TofSensors() = default;

        //----Methods----
        /**
        * @brief  Initializes and configures all Time-of-Flight-Sensors.
        * @return OK if all sensors were initialized successfully.
        *         ERROR if something went wrong.
        */
        ErrorCodes Init(void);

        /**
        * @brief  Updates all Time-of-Flight-Sensors. If a new measurement is ready, it is stored in the corresponding sensor object.
        * @return NEW_DATA if there were new values.
        *         NO_NEW_DATA if no new values were ready.
        *         OK if update is disabled.
        */
        ErrorCodes Update(void);

        /**
        * @brief  Blocks the Update-Method from running.
        */
        void DisableUpdate(void);

        /**
        * @brief  Re-enables the Update-Method.
        */
        void EnableUpdate(void);

        /**
        * @brief  Getter-method to get the last range measurement of a Time-of-Flight-Sensor.
        * @param  sensor  enum to choose the sensor to get the value from.
        * @return Last range measurement in mm. 0 if sensor type is unknown.
        */
        uint16_t GetRange(TofType sensor);

        /**
        * @brief  Getter-method to get the status of the last measurement of a Time-of-Flight-Sensor.
        * @param  sensor  enum to choose the sensor to get the value from.
        * @return VALID if the measurement is within range and no timeout occurred.
        *         OUT_OF_RANGE if the measurement was out of range.
        *         TIMEOUT if the sensor is in a timeout or the sensor type is unknown.
        *         ERROR if a sensor communication error occurred.
        */
        TofStatus GetStatus(TofType sensor);

        /**
        * @brief  Method to check if a range value has yet been retrieved.
        * @param  sensor  enum to choose the sensor to get the value from.
        * @return true if value is new.
        *         false if the value was already used.
        */
        bool IsDataNew(TofType sensor);

        /**
        * @brief  Method to check if a timeout occurred on any sensor.
        * @return true if a timeout occurred on at least one sensor.
        *         false if no timeout occurred.
        */
        bool AnyTimeoutOccured(void);

        /**
        * @brief  Calculates the left/right position error based on side wall distances.
        * @param  angleError         current gyro angle error.
        * @param  sideWallThreshold  distance threshold to consider a wall present.
        * @param  gapRobotWall       desired distance from robot to wall.
        * @return signed error; positive = too far right, negative = too far left. 0 if no walls.
        */
        int8_t CalculateLeftRightError(float angleError, uint8_t sideWallThreshold, uint8_t gapRobotWall);

        /**
        * @brief  Reads all side and front/back sensors and returns a wall bitmask.
        * @param  rampInfront  true if a ramp is detected in front (suppresses front wall bit).
        * @param  rampBehind   true if a ramp is detected behind (suppresses back wall bit).
        * @return Wall bitmask: bit 0 = front, bit 1 = right, bit 2 = back, bit 3 = left.
        */
        uint8_t GetWalls(bool rampInfront, bool rampBehind);

        /**
        * @brief  Method to check if a ramp is in front of or behind the robot.
        * @param  side  false = front; true = back.
        * @return true if a ramp is detected.
        *         false if no ramp is detected.
        */
        bool IsRampThere(bool side);
};
#ifdef _MSC_VER
    #pragma endregion
#endif