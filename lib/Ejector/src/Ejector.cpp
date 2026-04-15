/**
* @name:    Ejector.cpp
* @date:	16.01.2026
* @authors: Florian Wiesner, Paul Charusa
* @details: .cpp file for the ejector class
*/

//----Libraries----
#include "Ejector.h"
#include <Driving.h>

Ejector::Ejector(uint8_t rescuePacks) {
	if(rescuePacks > 14) rescuePacks = 14;
	rescuePacks /= 2;
	remainingPacks = rescuePacks;
	remainingPacks |= rescuePacks << 4;
}

#ifdef _MSC_VER
    #pragma region Init //-------------------------------------------------------------------------------------------------------
#endif
void Ejector::Init(Driving* robot) {
    p_driving = robot;
	pinMode(PIN_SERVO_LEFT, OUTPUT);
	pinMode(PIN_SERVO_RIGHT, OUTPUT);
	
	servoLeft.attach(PIN_SERVO_LEFT);
	servoRight.attach(PIN_SERVO_RIGHT);
	servoLeft.write(POS_CLOSED_LEFT);
	servoRight.write(POS_CLOSED_RIGHT);
}

#ifdef _MSC_VER
    #pragma endregion
    #pragma region Eject //------------------------------------------------------------------------------------------------------
#endif
ErrorCodes Ejector::Eject(ErrorCodes side, uint8_t amount) {
    Driving* robot = p_driving;
	if(amount == 0) return ErrorCodes::OK;
	amount = constrain(amount, 1, 6);

	if(remainingPacks <= 0) return ErrorCodes::empty;

	//extract packs
	uint8_t rLeft = remainingPacks >> 4;
	uint8_t rRight = remainingPacks & 0x0f;
	
	if(side == ErrorCodes::left){
		uint8_t priAmount = min(amount, rLeft);
		uint8_t secAmount = min((uint8_t)(amount - priAmount), rRight);

		//eject left
		for (uint8_t i = 0; i < priAmount; i++)
		{
			servoLeft.write(POS_OPEN_LEFT);
			delay(DELAY_OPEN);
			servoLeft.write(POS_CLOSED_LEFT);
			delay(DELAY_CLOSE);
		}
		if(secAmount>0){
			//eject remaining right
			robot->turn180Degree();
			for (uint8_t i = 0; i < secAmount; i++) {
				servoRight.write(POS_OPEN_RIGHT);
				delay(DELAY_OPEN);
				servoRight.write(POS_CLOSED_RIGHT);
				delay(DELAY_CLOSE);
			}
			robot->turn180Degree();
		}
		rLeft -= priAmount;
		rRight -= secAmount;
		remainingPacks = (rLeft << 4) | (rRight & 0x0f);
		return ErrorCodes::OK;
	} else if(side == ErrorCodes::right){
		uint8_t priAmount = min(amount, rRight);
		uint8_t secAmount = min((uint8_t)(amount - priAmount), rLeft);

		//eject right
		for (uint8_t i = 0; i < priAmount; i++)
		{
			servoRight.write(POS_OPEN_RIGHT);
			delay(DELAY_OPEN);
			servoRight.write(POS_CLOSED_RIGHT);
			delay(DELAY_CLOSE);
		}
		if(secAmount>0){
			//eject remaining left
			robot->turn180Degree();
			for (uint8_t i = 0; i < secAmount; i++) {
				servoLeft.write(POS_OPEN_LEFT);
				delay(DELAY_OPEN);
				servoLeft.write(POS_CLOSED_LEFT);
				delay(DELAY_CLOSE);
			}
			robot->turn180Degree();
		}
		rRight -= priAmount;
		rLeft -= secAmount;
		remainingPacks = (rLeft << 4) | (rRight & 0x0f);
		return ErrorCodes::OK;
	}
	return ErrorCodes::invalid;
}

uint8_t Ejector::GetRemaining(ErrorCodes side) {
	return (side == ErrorCodes::left) ? (remainingPacks >> 4) : (remainingPacks & 0x0F);
}