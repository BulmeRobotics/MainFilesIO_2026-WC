#pragma once

/**
 * @author  Vincent Rohkamm
 * @brief   camera class for communication between Camera (either RASPI or OPENMV) and uC
 * @date    01.04.2026
 * @todo    -UI Update for new Alert System
 */

#include <Arduino.h>
#include <mbed.h>

#include <CustomDatatypes.h>
#include <Ejector.h>
#include <Driving.h>
#include <Mapping.h>
#include <Motor.h>

class UserInterface;

class Vcameras
{
private:
    // --- Hardware Info ---
    static constexpr unsigned long CAM_BAUD = 115200;

    static constexpr uint8_t CAMERA_PIN_INT = 31;   //GPIO1 CM5 -> RASPI GPIO17
    static constexpr uint8_t CAMERAL_PIN_EN = 29;   //GPIO2 CM5 -> RASPI GPIO27
    static constexpr uint8_t CAMERAR_PIN_EN = 28;   //GPIO3 CM5 -> RASPI GPIO22

    static constexpr uint32_t CAM_TIMEOUT = 300;
    static constexpr uint32_t CAM_TIMEOUT_INIT = 60000;

    //Serial
    static constexpr UART* _cam = &Serial2;

    // --- related Objects ---
    Ejector* _ejector = nullptr;
    Driving* _robot = nullptr;
    Mapping* _mapper = nullptr;
    UserInterface* _ui = nullptr;
    Drivetrain* _drivetrain = nullptr;

    Stream* _debug_ifc = nullptr;

    //Reset after detection
    bool _victimFound = false;
    uint32_t _timeFound;
    static constexpr uint32_t DEACT_TIME_VICTIM = 2000;
    ErrorCodes HandleReset();

    // --- Interface ---
    bool _connected = false;
    bool _enabled = false;

    // --- State Fields ---
    bool _LeftEnabled = false, _RightEnabled = false;
    bool _Alert = false;
    bool _oldRed = false;
    bool _oldOnRamp = false;

    // Enable command state per camera (allows concurrent non-blocking left/right updates)
    bool _pending = false;
    bool _enTarget = false;
    bool _allowEN = false;
    uint32_t _enStart = 0;
    String _rxAsync = "";

    ErrorCodes EnableNonBlockingStep();
    bool TryReceivePacketNonBlocking();

    // --- Response ---
    static volatile char _buffL[10];
    static volatile char _buffR[10];

    /**
     * @brief Recieves Commandos
     * @param waittime time to block in ms
     * @return Commandostring
     */
    String Recieve(uint32_t waittime = 0);


    /**
     * @brief Start Camera Reset
     * @return success
     */
    bool ResetCam();
    
public:

    Vcameras(Stream* debugPort = nullptr) : _debug_ifc(debugPort) {}

    /**
     * @brief Initializes Cam class. Tries connecting to cameras
     * @return OK - success, else ErrorCodes
     */
    ErrorCodes Init(Ejector* ejector, Mapping* mapper, Driving* robot, UserInterface* ui, Drivetrain* drivetrain);

    /**
     * @brief enables or disables Camera
     * @param en true...on, false...off
     * @param side left / right / both
     * @return OK / Error
     */
    ErrorCodes Enable(bool en, bool blocking = true);

    /**
     * @brief camera handler has to be called periodically
     * @param onRed is robot on Red Tile?
     * @param wallL is a wall Left?
     * @param wallR is a wall Right?
     * @return ErrorCodes for debugging
     */
    ErrorCodes Update(bool onRed, bool onRamp = false);

    /**
     * @brief Getter if Cam is enabled
     * @param cam ErrorCodes::left / ErrorCodes::right
     * @return true...enabled, false...disabled
     */
    bool IsEnabled(ErrorCodes cam){
        bool enBuff = (cam == ErrorCodes::left) ? _LeftEnabled : _RightEnabled;
        // if(_debug_ifc != nullptr) {
        //     _debug_ifc->print("CAM: ");
        //     _debug_ifc->print((cam == ErrorCodes::left) ? "L" : "R");
        //     _debug_ifc->print("  ");
        //     _debug_ifc->println((enBuff) ? "En" : "Dis");
        // }
        return enBuff;
    }

    /**
     * @brief Getter if Cam is in Alert
     * @return true...ALERT, false...not ALERT
     */
    bool IsAlert(){
        return _Alert;
    }

    void AllowEnable(){ _allowEN = true; }
};
