#include "Vcameras.h"
#include <UserInterface.h>

//---------------------------------------------------------------------------------------------------------
// Initialization
//---------------------------------------------------------------------------------------------------------

ErrorCodes Vcameras::Init(Ejector* ejector, Mapping* mapper, Driving* robot, UserInterface* ui, Drivetrain* drivetrain){
    _ejector = ejector;
    _mapper = mapper;
    _robot = robot;
    _ui = ui;
    _drivetrain = drivetrain;

    pinMode(CAMERAL_PIN_INT, INPUT);
    pinMode(CAMERAR_PIN_INT, INPUT);

    _camL->begin(115200);
    _camR->begin(115200);

    if(_debug_ifc != nullptr) _debug_ifc->println("Start Cam INIT");

    String str;

    _camL->print("<I>");
    str = Recieve(ErrorCodes::left, CAM_TIMEOUT);
    _connectedL = (str.indexOf("OK") != -1) ? true : false;
    
    _camR->print("<I>");
    str = Recieve(ErrorCodes::right, CAM_TIMEOUT);
    _connectedR = (str.indexOf("OK") != -1) ? true : false;

    if(!_connectedL || !_connectedR) return ErrorCodes::no_connection;
    return ErrorCodes::OK;  
}

//---------------------------------------------------------------------------------------------------------
// Recieve UART
//---------------------------------------------------------------------------------------------------------

String Vcameras::Recieve(ErrorCodes side, uint32_t waittime){
    UART* _ifc = (side == ErrorCodes::left) ? _camL : _camR;
    //Wait if wanted
    String str = "";
    uint32_t startTime = millis();

    while (millis() - startTime <= waittime || waittime == 0) {
        while (_ifc->available()) {
            char c = _ifc->read();
            if (c == '<') {
                str = ""; // String bei neuem Paket zurücksetzen
            } else if (c == '>') {
                if(_debug_ifc != nullptr) _debug_ifc->println("Rec: " + str);
                return str; // Komplettes Paket empfangen!
            } else {
                str += c;
            }
        }
        if (waittime == 0) break; // Kein Blockieren, wenn waittime 0 ist
        delay(1);
    }
    return " "; // Timeout
}

bool Vcameras::TryReceivePacketNonBlocking(ErrorCodes side, String& packet){
    UART* ifc = (side == ErrorCodes::left) ? _camL : _camR;
    String& rx = (side == ErrorCodes::left) ? _rxEnableL : _rxEnableR;

    while (ifc->available()) {
        char c = ifc->read();
        if (c == '<') {
            rx = "";
        } else if (c == '>') {
            packet = rx;
            rx = "";
            if(_debug_ifc != nullptr) _debug_ifc->println("Rec(NB): " + packet);
            return true;
        } else {
            rx += c;
        }
    }
    return false;
}

ErrorCodes Vcameras::EnableNonBlockingStep(ErrorCodes side){
    bool& pending = (side == ErrorCodes::left) ? _enPendingL : _enPendingR;
    bool& target = (side == ErrorCodes::left) ? _enTargetL : _enTargetR;
    bool& enState = (side == ErrorCodes::left) ? _LeftEnabled : _RightEnabled;
    uint32_t& startTs = (side == ErrorCodes::left) ? _enStartL : _enStartR;
    UART* ifc = (side == ErrorCodes::left) ? _camL : _camR;

    if (!pending) return ErrorCodes::OK;

    String packet;
    if (TryReceivePacketNonBlocking(side, packet)) {
        pending = false;
        if (packet.indexOf("OK") != -1) {
            enState = target;
            return ErrorCodes::OK;
        }
        //_ui->ShowPopup("cams enable error", ErrorCodes::ERROR);
        return ErrorCodes::invalid;
    }

    if ((millis() - startTs) > CAM_TIMEOUT) {
        pending = false;
        _ui->ShowPopup("cams enable timeout", ErrorCodes::warning, 2);
        return ErrorCodes::TIMEOUT;
    }

    return ErrorCodes::NO_NEW_DATA;
}

//---------------------------------------------------------------------------------------------------------
// Enable
//---------------------------------------------------------------------------------------------------------

ErrorCodes Vcameras::Enable(bool en, ErrorCodes side, bool blocking){
    if(en && _victimFound) return ErrorCodes::OK;
    bool conn = (side == ErrorCodes::left) ? _connectedL : _connectedR;
    if(!conn) return ErrorCodes::no_connection; //Return if no connection

    //Create buffers
    bool& enState =  (side == ErrorCodes::left) ? _LeftEnabled : _RightEnabled;
    bool& pending = (side == ErrorCodes::left) ? _enPendingL : _enPendingR;
    bool& target = (side == ErrorCodes::left) ? _enTargetL : _enTargetR;
    uint32_t& startTs = (side == ErrorCodes::left) ? _enStartL : _enStartR;
    UART* ifc = (side == ErrorCodes::left) ? _camL : _camR;

    if (pending) {
        if (target != en) {
            // Command changed while waiting -> restart request with latest target.
            pending = false;
        } else {
            ErrorCodes step = EnableNonBlockingStep(side);
            if (!blocking || step != ErrorCodes::NO_NEW_DATA) return step;
        }
    }

    if(enState == en) return ErrorCodes::OK;

    //Send command
    const char* cmd = en ? "<E>" : "<D>";
    ifc->print(cmd);

    // Prepare async wait state
    pending = true;
    target = en;
    startTs = millis();

    if(!blocking) return ErrorCodes::NO_NEW_DATA;

    // Blocking mode: keep stepping until done or timeout
    while (true) {
        ErrorCodes step = EnableNonBlockingStep(side);
        if (step == ErrorCodes::NO_NEW_DATA) {
            delay(1);
            continue;
        }
        return step;
    }
}

//---------------------------------------------------------------------------------------------------------
// Handle Reset
//---------------------------------------------------------------------------------------------------------

ErrorCodes Vcameras::HandleReset(){
    if(!_victimFound) return ErrorCodes::OK;
    if(_timeFound + DEACT_TIME_VICTIM < millis()){
        _victimFound = false;
        //Enable cams
        Enable(true,ErrorCodes::left, false);
        Enable(true,ErrorCodes::right, false);
        return ErrorCodes::OK;
    }
    return ErrorCodes::disabled;
}

//---------------------------------------------------------------------------------------------------------
// Update
//---------------------------------------------------------------------------------------------------------

ErrorCodes Vcameras::Update(bool onRed, bool wallL, bool wallR){
    if(!_connectedL || !_connectedR) return ErrorCodes::no_connection;

    // Progress pending async enable commands for both cameras each cycle.
    EnableNonBlockingStep(ErrorCodes::left);
    EnableNonBlockingStep(ErrorCodes::right);

    if(HandleReset() == ErrorCodes::disabled) return ErrorCodes::disabled;

    if(_oldRed && !onRed) {
        Enable(true, ErrorCodes::left, false);
        Enable(true, ErrorCodes::right, false);
    } else if (!_oldRed && onRed){
        Enable(false, ErrorCodes::left, false);
        Enable(false, ErrorCodes::right, false);
    }
    _oldRed = onRed;
    if(onRed) return ErrorCodes::OK;

    //Wände überprüfen
    if(wallL && !_LeftEnabled)      Enable(true,  ErrorCodes::left, false);
    else if(!wallL && _LeftEnabled) Enable(false, ErrorCodes::left, false);

    if(wallR && !_RightEnabled)      Enable(true,  ErrorCodes::right, false);
    else if(!wallR && _RightEnabled) Enable(false, ErrorCodes::right, false);

    //Abfrage auf alert
    if(_LeftEnabled) _LeftAlert = digitalRead(CAMERAL_PIN_INT);
    if(_RightEnabled) _RightAlert = digitalRead(CAMERAR_PIN_INT);

    //Continue when no camera is reporting
    if(!_LeftAlert && !_RightAlert) return ErrorCodes::OK;

    String str = "";
    ErrorCodes side;
    //Wait for new Data
    if(_LeftAlert){
        str = Recieve(ErrorCodes::left);
        if(str[0] != ' ')
            side = ErrorCodes::left;
        else return ErrorCodes::OK;
    } else if(_RightAlert){
        str = Recieve(ErrorCodes::right);
        if(str[0] != ' ')
            side = ErrorCodes::right;
        else return ErrorCodes::OK;
    }
    
    //Dissect to side, and Victim Type
    char victim = str[0];

    //Check if Victim is allowed:
    if(!(victim == 'H' || victim == 'S' || victim == 'U')){
        _ui->ShowPopup("Invalid victim!", ErrorCodes::warning, 2);
        return ErrorCodes::ERROR;
    }

    //Mapping call
    ErrorCodes err = _mapper->SetVictim();
    if(err != ErrorCodes::OK) {
        if(err == ErrorCodes::already_found) _ui->ShowPopup("Victim alr found",ErrorCodes::warning);
        return err;
    }

    //_robot->endDrive(); //Stops robot
    _drivetrain->Stop();

    //Reset cams
    _victimFound = true;
    _timeFound = millis();
    Enable(false, ErrorCodes::left, false);
    Enable(false, ErrorCodes::right, false);

    //Get Amount of dropped Rescue Packs
    uint8_t amount;
    switch (victim) {
    case 'H':   //Harmed
        amount = 2;
        break;
    case 'S':   //Stable
        amount = 1;
        break;
    default:    //Unharmed / alles andere
        amount = 0;
        break;
    }
    char buffer[20];
    sprintf(buffer,"VICTIM Detected: %c", victim);
    _ui->ShowPopup(buffer, ErrorCodes::info, 5);
    _ui->LED_BUZZER_Signal(500,500,1);
    _ui->Update();
    _ui->LED_BUZZER_Signal(500,500,4);
    _ejector->Eject(side, amount);
    _ui->Update();
    
    _ui->Update();
    _robot->integralError = 0;
    _robot->derivativeError = 0;

    if(_robot->_TURNING)
        _robot->_CAM_ALERT_TURN = true;
    else
        _robot->_CAM_VICTIM = true;

    //2RC - Harmed
    //1RC - Stable
    //0RC - Unharmed
    return ErrorCodes::OK;
}