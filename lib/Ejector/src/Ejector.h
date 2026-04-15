#pragma once
/**
* @name:    Ejector.h
* @date:	16.01.2026
* @authors: Florian Wiesner, Paul Charusa
* @details: Header file for the ejector class
*/

//----Forward-Declaration----
class Driving;

#ifdef _MSC_VER
    #pragma region Libraries //--------------------------------------------------------------------------------------------------
#endif
#include <Arduino.h>
#include <Servo.h>
#include "CustomDatatypes.h"

#ifdef _MSC_VER
    #pragma endregion
    #pragma region Ejector Class //----------------------------------------------------------------------------------------------
#endif
class Ejector {
    private:
        //----Pins----
        static constexpr uint8_t PIN_SERVO_LEFT = 4;
        static constexpr uint8_t PIN_SERVO_RIGHT= 7;

        //----Positions----
        static constexpr int POS_CLOSED_LEFT    = 100;
        static constexpr int POS_OPEN_LEFT      = 18;
        static constexpr int POS_CLOSED_RIGHT   = 85;
        static constexpr int POS_OPEN_RIGHT     = 170;

        //----Timing----
        static constexpr uint16_t DELAY_OPEN    = 500;
        static constexpr uint16_t DELAY_CLOSE   = 250;

        //----Variables----
        uint8_t remainingPacks;    //b0-3 right, b4-7 left

        //----Objects----
        Servo servoLeft;
        Servo servoRight;

        //----Dependencies----
        Driving* p_driving = nullptr;

    public:
        //----Constructor----
        Ejector(uint8_t rescuePacks = 12);

        //----Methods----
        /**
        * @brief  Initializes and configures the Ejectors.
        * @param  robot pointer to Driving instance; used for 180° turns during fallback ejection.
        */
        void Init(Driving* robot);

        /**
        * @brief  Method to eject a certain amount of rescue kits with either the left or right Ejector.
        * @param  side ErrorCodes: left / right
        * @param  amount the number of rescue kits that will be ejected (1-6).
        * @return OK if ejection completed successfully.
        *         empty if no rescue packs remain.
        *         invalid if side is neither left nor right.
        */
        ErrorCodes Eject(ErrorCodes side, uint8_t amount);

        /**
        * @brief  Returns the number of remaining rescue kits for a given side.
        * @param  side ErrorCodes: left / right
        * @return Number of remaining kits on the requested side.
        */
        uint8_t GetRemaining(ErrorCodes side);
    };