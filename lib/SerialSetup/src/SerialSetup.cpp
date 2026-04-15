/**
* @name:    SerialSetup.cpp
* @date:    15.12.2023
* @authors: Florian Wiesner, Paul Charusa
* @details: .cpp file with the functions for initialisation and setup
*           for the Serial Monitor. This way, because the Serial Monitor
*           on Arduino GIGA R1 doesn't work by default with main() - structure
*/

//----Libraries----
#include "SerialSetup.h"
#include <USB/PluggableUSBSerial.h>

//----Definitions----
#undef main


//----Functions----
// Function to initialize the Serial monitor for the Arduino GIGA R1
void serialSetup(int baudrate_f) {
	init();
	initVariant();

#if defined(SERIAL_CDC)
	PluggableUSBD().begin();
	_SerialUSB.begin(baudrate_f);
#endif

	Serial.begin(baudrate_f);
	Serial2.begin(baudrate_f);
	Serial3.begin(baudrate_f);
}

// Function that calls the SerialEventRunner every cycle of the loop
void serialLoop(void) {
	if (arduino::serialEventRun) {
		arduino::serialEventRun();
	}
}