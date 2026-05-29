#pragma once

// @author  Vincent Rohkamm
// @date     07.07.2025
// @brief Header file for Bluetooth functionality for ROBOT

#ifdef _MSC_VER
#pragma region Includes
#endif

//Standard Libraries
#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Stream.h>

//Custom Libraries
#include <CustomDatatypes.h>
#include <UserInterface.h>


#ifdef _MSC_VER
#pragma endregion Includes
#pragma region BLE Class
#endif

class BLE_UART : public Stream{
private:
    //Settings
    uint16_t timeoutTime;
    bool _CONNECTED = false;

    //UUIDs for Service
    const String serviceUUID = "FF000000-FFFF-EEEE-99FF-123456789000"; //Service UUID for ESP32-INFO
    #define rxUUID              "FF000000-FFFF-EEEE-99FF-123456789002" //RX UUID for ESP32-INFO
    #define txUUID              "FF000000-FFFF-EEEE-99FF-123456789001" //TX UUID for ESP32-INFO

    const String LOCAL_NAME = "ESP32-INFO_BROBOT";

    //DEBUG
    Stream* _debug_ifc = nullptr;

    //Objects
    BLEDevice peripheral;
    BLECharacteristic rxCharacteristic;
    BLECharacteristic txCharacteristic;

    UserInterface* _ui = nullptr;

    //Ringbuffer
    static constexpr int RX_BUFFER_SIZE = 256;
    uint8_t _rx_buffer[RX_BUFFER_SIZE];
    uint16_t _rx_head = 0;
    uint16_t _rx_tail = 0;

    void pollBLE();
    
public:
    BLE_UART(uint16_t timeout_ms = 5000, Stream* debugPort = nullptr) : _debug_ifc(debugPort), timeoutTime(timeout_ms) {}

    /**
     * @brief Initializes BLE class.
     * @param ui User Interface Pointer
     * @return OK - success, else ErrorCodes
     */
    ErrorCodes init(UserInterface* ui = nullptr);

    /**
     * @brief tries connecting to UART Server
     * @return success / timeout
     */
    ErrorCodes connect();

    virtual int available() override;
    virtual int read() override;
    virtual int peek() override;
    virtual size_t write(uint8_t c) override;
    virtual size_t write(const uint8_t *buffer, size_t size) override;
    virtual void flush() override;
};