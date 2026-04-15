/**
* @name:    Motor.cpp
* @date:	16.01.2026
* @authors: Florian Wiesner
* @details: .cpp file for Motor driver class and the Drivetrain class
*/

//----Libraries----
#include "Motor.h"

#ifdef _MSC_VER
    #pragma region ISR //--------------------------------------------------------------------------------------------------------
#endif
volatile uint64_t counter = 0;  // counter for the encoder
// Function that is called by the interrupt; increases the counter by 1
void encoder_ISR(void) {
	counter++;
}

#ifdef _MSC_VER
    #pragma endregion
    #pragma region Motor //------------------------------------------------------------------------------------------------
#endif
Motor::Motor(uint8_t pwmPin, uint8_t driver, uint8_t driverPinCW, uint8_t driverPinCCW, uint8_t encoderPhaseA, float maxMotorVoltage) {
	this->pwmPin = pwmPin;
	this->driver = driver;
	this->maxMotorVoltage = constrain(maxMotorVoltage, 5, 8.26);
	this->encoderPhaseA = encoderPhaseA;

    maxPWM = (float)255 * (maxMotorVoltage / MAX_VOLTAGE_PWM);  // calculate max duty-cycle

	if (driver == DRIVER_A) {
		cwPinA = driverPinCW;
		ccwPinA = driverPinCCW;
	}
	else if (driver == DRIVER_B) {
		cwPinB = driverPinCW;
		ccwPinB = driverPinCCW;
	}
	
	InitDriverPins(driverPinCW, driverPinCCW);
}

void Motor::SetSpeed(int8_t speed) {
	speed = constrain(speed, -100, 100);
	int16_t speedPWM = map(speed, (int16_t)-100, (int16_t)100, -maxPWM, maxPWM);

	if (speedPWM >= 0)	SetPositiveDirection();
	else  SetNegativeDirection();

	analogWrite(pwmPin, abs(speedPWM));
}

void Motor::Stop(void) {
	SetSpeed(0);
}

void Motor::ResetEncoder(uint64_t value) {
	counter = value;
}

float Motor::GetEncoderAngle(void) {
	float degrees = (float)counter / CLICKS_PER_SHAFT_ROTATION * 360;
	return degrees;
}

float Motor::GetEncoderDistance(void) {
	float distance = (GetEncoderAngle() / 360) * WHEEL_DIAMETER * PI;
	return distance;
}

void Motor::EnableEncoder(void) {
	attachInterrupt(digitalPinToInterrupt(encoderPhaseA), encoder_ISR, RISING);
}

void Motor::DisableEncoder(void) {
	detachInterrupt(digitalPinToInterrupt(encoderPhaseA));
}

// Private method to set the H-Bridge for positive direction.
void Motor::SetPositiveDirection(void) {
	if (driver == DRIVER_A) {
		digitalWrite(ccwPinA, LOW);
		digitalWrite(cwPinA, HIGH);
	}
	else if (driver == DRIVER_B) {
		digitalWrite(ccwPinB, LOW);
		digitalWrite(cwPinB, HIGH);
	}
}

// Private method to set the H-Bridge for negative direction.
void Motor::SetNegativeDirection(void) {
	if (driver == DRIVER_A) {
		digitalWrite(cwPinA, LOW);
		digitalWrite(ccwPinA, HIGH);
	}
	else if (driver == DRIVER_B) {
		digitalWrite(cwPinB, LOW);
		digitalWrite(ccwPinB, HIGH);
	}
}

// Private method to settup the pins of the motor driver.
void Motor::InitDriverPins(uint8_t pinA, uint8_t pinB) {
	pinMode(pinA, OUTPUT);
	pinMode(pinB, OUTPUT);
	pinMode(encoderPhaseA, INPUT_PULLDOWN);
}


#ifdef _MSC_VER
    #pragma endregion
    #pragma region Drivetrain //----------------------------------------------------------------------------------------------------
#endif
void Drivetrain::SetSpeed(int8_t speedLB, int8_t speedLF, int8_t speedRF, int8_t speedRB) {
	motorLB.SetSpeed(speedLB);
	motorLF.SetSpeed(speedLF);
	motorRF.SetSpeed(speedRF);
	motorRB.SetSpeed(speedRB);
}

void Drivetrain::SetSpeed(int8_t speed) {
	SetSpeed(speed, speed, speed, speed);
}

void Drivetrain::SetSpeedLB(int8_t speed) {
	motorLB.SetSpeed(speed);
}

void Drivetrain::SetSpeedLF(int8_t speed) {
	motorLF.SetSpeed(speed);
}

void Drivetrain::SetSpeedRF(int8_t speed) {
	motorRF.SetSpeed(speed);
}

void Drivetrain::SetSpeedRB(int8_t speed) {
	motorRB.SetSpeed(speed);
}

void Drivetrain::SetSpeedLeft(int8_t speed) {
	motorLB.SetSpeed(speed);
	motorLF.SetSpeed(speed);
}

void Drivetrain::SetSpeedRight(int8_t speed) {
	motorRB.SetSpeed(speed);
	motorRF.SetSpeed(speed);
}

void Drivetrain::Stop(void) {
	SetSpeed(0, 0, 0, 0);
}

void Drivetrain::ResetEncoder(uint64_t value) {
	motorLB.ResetEncoder(value);
}

float Drivetrain::GetEncoderDistance(void) {
	return motorLB.GetEncoderDistance();
}

void Drivetrain::EnableEncoder(void) {
	motorLB.EnableEncoder();
}

void Drivetrain::DisableEncoder(void) {
	motorLB.DisableEncoder();
}