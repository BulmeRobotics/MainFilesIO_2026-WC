#pragma once
/**
* @name:    SerialSetup.h
* @date:    15.12.2023
* @authors: Florian Wiesner, Paul Charusa
* @details: Header file for initialisation and setup for the Serial Monitor.
*           This way, because the Serial Monitor on Arduino GIGA R1 doesn't
*           work by default with main() - structure
*/

//----Libraries----
#include <Arduino.h>
#include <USB/PluggableUSBSerial.h>

//----Definitions----
#undef main

//----Functions----
/**
* @brief  Initializes all serial interfaces for the Arduino GIGA R1.
*         Must be called at the very beginning of setup(). Required because
*         the GIGA R1 does not initialize Serial correctly in a main()-based
*         structure.
* @param  baudrate_f  Baud rate to configure for all serial interfaces.
*/
void serialSetup(int baudrate_f);

/**
* @brief  Runs the serial event handler for the current loop iteration.
*         Must be called at the very beginning of loop(). Required because
*         the GIGA R1 does not call serialEventRun() automatically in a
*         main()-based structure — this function replaces that call.
*/
void serialLoop(void);

// Required by the Arduino framework internals — no direct call needed
int atexit(void (* /*func*/)()) { return 0; }
void initVariant() __attribute__((weak));
void initVariant() { }
void setupUSB() __attribute__((weak));
void setupUSB() { }