#pragma once
/**
* @name:    Motor.h
* @date:	16.01.2026
* @authors: Florian Wiesner
* @details: Header file for Motor driver class and the Drivetrain class
*/

#ifdef _MSC_VER
    #pragma region Libraries //--------------------------------------------------------------------------------------------------
#endif
#include <Arduino.h>
#include <cmath>
#include "CustomDatatypes.h"

// Encoder ISR - outside class
void encoder_ISR(void);

#ifdef _MSC_VER
    #pragma endregion
    #pragma region Motor Class //------------------------------------------------------------------------------------------------
#endif
class Motor {
    private:
        //----Configuration----
        static constexpr float   WHEEL_DIAMETER            = 93.0f;
        static constexpr float   MAX_VOLTAGE_PWM           = 8.26f;
        static constexpr float   GEAR_REDUCTION_RATIO      = 74.83f;
        static constexpr uint8_t CLICKS_PER_MOTOR_ROTATION = 11;
        static constexpr float   CLICKS_PER_SHAFT_ROTATION = GEAR_REDUCTION_RATIO * CLICKS_PER_MOTOR_ROTATION;

        //----Members----
        uint8_t cwPinA, ccwPinA, cwPinB, ccwPinB, pwmPin, encoderPhaseA;
        uint8_t driver;
        float maxMotorVoltage;
        int16_t maxPWM;

        //----Methods----
        void SetPositiveDirection(void);
        void SetNegativeDirection(void);
        void InitDriverPins(uint8_t pinA, uint8_t pinB);

    public:
        //----Definitions----
        static constexpr uint8_t DRIVER_A = 0;
        static constexpr uint8_t DRIVER_B = 1;

        //----Constructor----
        /**
        * @brief  Constructor of the motor driver class.
        * @param  pwmPin PWM pin that controls the motor.
        * @param  driver driver channel: 0 = Driver A, 1 = Driver B.
        * @param  driverPinCW pin connected to the clockwise direction-pin of the driver.
        * @param  driverPinCCW pin connected to the counter-clockwise direction-pin of the driver.
        * @param  encoderPhaseA one of the two encoder pins of the motor.
        * @param  maxMotorVoltage maximum motor voltage; used to calculate max PWM duty-cycle (max. 8.26V).
        */
        Motor(uint8_t pwmPin, uint8_t driver, uint8_t driverPinCW, uint8_t driverPinCCW, uint8_t encoderPhaseA, float maxMotorVoltage = 8.26);

        //----Methods----
        /**
        * @brief  Method to set the speed of the motor in percent.
        * @param  speed percentage of the speed (-100% to +100%).
        */
        void SetSpeed(int8_t speed);

        /**
        * @brief  Method to stop the motor.
        */
        void Stop(void);

        /**
        * @brief  Method to reset the encoder to a value(to mostly 0).
        * @param  value new number of the counter (default = 0).
        */
        void ResetEncoder(uint64_t value = 0);

        /**
        * @brief  Returns the current angle of the motor shaft based on encoder count.
        * @return shaft angle in degrees.
        */
        float GetEncoderAngle(void);

        /**
        * @brief  Method to get the distance the wheel has driven since the last reset.
        * @return travelled distance of the wheel (in mm).
        */
        float GetEncoderDistance(void);

        /**
        * @brief  Method to enable the encoder. Enables the interrupt.
        */
        void EnableEncoder(void);

        /**
        * @brief  Method to disable the encoder. Disables the interrupt.
        */
        void DisableEncoder(void);
};

#ifdef _MSC_VER
    #pragma endregion
    #pragma region Drivetrain Class //------------------------------------------------------------------------------------------------
#endif
class Drivetrain {
    private:
        //----Configuration----
        static constexpr uint8_t MOTOR_LB_PWM     = 13;
        static constexpr uint8_t MOTOR_LB_CW      = 36;
        static constexpr uint8_t MOTOR_LB_CCW     = 39;
        static constexpr uint8_t MOTOR_LB_ENCODER = 5;
        static constexpr uint8_t MOTOR_LB_DRIVER  = Motor::DRIVER_A;

        static constexpr uint8_t MOTOR_LF_PWM     = 11;
        static constexpr uint8_t MOTOR_LF_CW      = 36;
        static constexpr uint8_t MOTOR_LF_CCW     = 39;
        static constexpr uint8_t MOTOR_LF_ENCODER = 6;  // Not-existent
        static constexpr uint8_t MOTOR_LF_DRIVER  = Motor::DRIVER_A;

        static constexpr uint8_t MOTOR_RF_PWM     = 10;
        static constexpr uint8_t MOTOR_RF_CW      = 37;
        static constexpr uint8_t MOTOR_RF_CCW     = 38;
        static constexpr uint8_t MOTOR_RF_ENCODER = 3;
        static constexpr uint8_t MOTOR_RF_DRIVER  = Motor::DRIVER_B;

        static constexpr uint8_t MOTOR_RB_PWM     = 9;
        static constexpr uint8_t MOTOR_RB_CW      = 37;
        static constexpr uint8_t MOTOR_RB_CCW     = 38;
        static constexpr uint8_t MOTOR_RB_ENCODER = 2;  // Not-existent
        static constexpr uint8_t MOTOR_RB_DRIVER  = Motor::DRIVER_B;

        //----Members----
        Motor motorLB = Motor(MOTOR_LB_PWM, MOTOR_LB_DRIVER, MOTOR_LB_CW, MOTOR_LB_CCW, MOTOR_LB_ENCODER);
        Motor motorLF = Motor(MOTOR_LF_PWM, MOTOR_LF_DRIVER, MOTOR_LF_CW, MOTOR_LF_CCW, MOTOR_LF_ENCODER);
        Motor motorRF = Motor(MOTOR_RF_PWM, MOTOR_RF_DRIVER, MOTOR_RF_CW, MOTOR_RF_CCW, MOTOR_RF_ENCODER);
        Motor motorRB = Motor(MOTOR_RB_PWM, MOTOR_RB_DRIVER, MOTOR_RB_CW, MOTOR_RB_CCW, MOTOR_RB_ENCODER);

    public:
        //----Constructor----
        Drivetrain() = default;

        //----Methods----
        /**
        * @brief  Sets the speed of each motor individually.
        * @param  speedLB Left-Back motor speed (-100% to +100%).
        * @param  speedLF Left-Front motor speed (-100% to +100%).
        * @param  speedRF Right-Front motor speed (-100% to +100%).
        * @param  speedRB Right-Back motor speed (-100% to +100%).
        */
        void SetSpeed(int8_t speedLB, int8_t speedLF, int8_t speedRF, int8_t speedRB);

        /**
        * @brief  Sets the speed of all four motors to the same value.
        * @param  speed percentage of the speed (-100% to +100%).
        */
        void SetSpeed(int8_t speed);

        /**
        * @brief  Method to set the speed of the left-back motor in percent.
        * @param  speed percentage of the speed (-100% to +100%).
        */
        void SetSpeedLB(int8_t speed);

        /**
        * @brief  Method to set the speed of the left-front motor in percent.
        * @param  speed percentage of the speed (-100% to +100%).
        */
        void SetSpeedLF(int8_t speed);

        /**
        * @brief  Method to set the speed of the right-front motor in percent.
        * @param  speed percentage of the speed (-100% to +100%).
        */
        void SetSpeedRF(int8_t speed);

        /**
        * @brief  Method to set the speed of the right-back motor in percent.
        * @param  speed percentage of the speed (-100% to +100%).
        */
        void SetSpeedRB(int8_t speed);

        /**
        * @brief  Method to set the speed of the left-sided motors in percent.
        * @param  speed percentage of the speed (-100% to +100%).
        */
        void SetSpeedLeft(int8_t speed);

        /**
        * @brief  Method to set the speed of the right-sided motors in percent.
        * @param  speed percentage of the speed (-100% to +100%).
        */
        void SetSpeedRight(int8_t speed);
        
        /**
        * @brief  Method to stop all motors.
        */
        void Stop(void);

        /**
        * @brief  Method to reset the encoder to a value(to mostly 0).
        * @param  value new number of the counter (default = 0).
        */
        void ResetEncoder(uint64_t value = 0);


        /**
        * @brief  Method to get the distance the robot has driven since the last reset. Uses only Left-Back!
        * @return travelled distance of the robot (in mm).
        */
        float GetEncoderDistance(void);

        /**
        * @brief  Method to enable the encoder. Enables the interrupt (only Left-Back).
        */
        void EnableEncoder(void);

        /**
        * @brief  Method to disable the encoder. Disables the interrupt (only Left-Back).
        */
        void DisableEncoder(void);

};
#ifdef _MSC_VER
    #pragma endregion
#endif