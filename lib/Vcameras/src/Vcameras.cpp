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

    // Set Alert Pins as Input
    pinMode(CAMERA_PIN_INT, INPUT);

    //Set Enabled Pins as Input
    pinMode(CAMERAL_PIN_EN,INPUT);
    pinMode(CAMERAR_PIN_EN,INPUT);

    //Start Communication
    _cam->begin(CAM_BAUD);

    if(_debug_ifc != nullptr) _debug_ifc->println("Start Cam INIT");

    String str;

    uint8_t repeats = CAM_TIMEOUT_INIT / CAM_TIMEOUT;
    for (uint8_t i = 0; i < repeats; i++)
    {
        _cam->print("<I>");
        str = Recieve(CAM_TIMEOUT);
        _connected = (str.indexOf("OK") != -1) ? true : false;
        if (str.indexOf("OK") != -1) break;
    }
    
    if(!_connected) return ErrorCodes::no_connection;
    return ErrorCodes::OK;  
}

//---------------------------------------------------------------------------------------------------------
// Recieve UART
//---------------------------------------------------------------------------------------------------------

String Vcameras::Recieve(uint32_t waittime){
    String str = "";
    uint32_t startTime = millis();

    do {
        while (_cam->available()){  //Is new Character available?
            char c = _cam->read();  //Get char from UART
            if (c == '<') { //Msg start
                str = ""; 
            } else if (c == '>') {  //Msg end
                if(_debug_ifc != nullptr) _debug_ifc->println("Rec: " + str);
                return str;
            } else { 
                str += c;
            }
        }
        if(waittime != 0) delay(1);
    } while (millis() - startTime < waittime);

    return " "; //Timeout
}

bool Vcameras::TryReceivePacketNonBlocking(){
    while (_cam->available()) {
        char c = _cam->read();
        if (c == '<') {
            _rxAsync = "";
        } else if (c == '>') {
            if(_debug_ifc != nullptr) _debug_ifc->println("Rec(NB): " + _rxAsync);
            return true;
        } else {
            _rxAsync += c;
        }
    }
    return false;
}

ErrorCodes Vcameras::EnableNonBlockingStep(){
    if (!_pending) return ErrorCodes::OK;

    if (TryReceivePacketNonBlocking()) {
        String packet = _rxAsync;
        _pending = false;
        _rxAsync = "";
        if (packet.indexOf("OK") != -1) {
            _enabled = _enTarget;
            if(_enabled == false) { _LeftEnabled = false; _RightEnabled = false; }
            return ErrorCodes::OK;
        }
        //_ui->ShowPopup("cams enable error", ErrorCodes::ERROR);
        return ErrorCodes::invalid;
    }

    if ((millis() - _enStart) > CAM_TIMEOUT) {
        _pending = false;
        _ui->ShowPopup("cams enable timeout", ErrorCodes::warning, 2);
        return ErrorCodes::TIMEOUT;
    }

    return ErrorCodes::NO_NEW_DATA;
}

//---------------------------------------------------------------------------------------------------------
// Enable
//---------------------------------------------------------------------------------------------------------

ErrorCodes Vcameras::Enable(bool en, bool blocking){
    if(!_connected) return ErrorCodes::no_connection;   //Check for connection first
    if(en && _victimFound) return ErrorCodes::OK;

    if(_debug_ifc != nullptr){
        _debug_ifc->print(en ? "En" : "Dis");
        _debug_ifc->println(" Cams");
    }

    if (_pending) {
        if (_enTarget != en) {
            // Command changed while waiting -> restart request with latest target.
            _pending = false;
        } else {
            ErrorCodes step = EnableNonBlockingStep();
            if (!blocking || step != ErrorCodes::NO_NEW_DATA) return step;
        }
    }

    if(_enabled == en) return ErrorCodes::OK;

    //Send command
    const char* cmd = en ? "<E>" : "<D>";
    _cam->print(cmd);

    // Prepare async wait state
    _pending = true;
    _enTarget = en;
    _enStart = millis();

    if(!blocking) return ErrorCodes::NO_NEW_DATA;

    // Blocking mode: keep stepping until done or timeout
    while (true) {
        ErrorCodes step = EnableNonBlockingStep();
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

bool Vcameras::ResetCam(){
    _victimFound = true;
    _timeFound = millis();
    _allowEN = false;
    Enable(false, false);
    return true;
}

ErrorCodes Vcameras::HandleReset(){
    if(!_victimFound || !_allowEN) return ErrorCodes::OK;
    if(_timeFound + DEACT_TIME_VICTIM < millis()){
        _victimFound = false;
        //Enable cams
        Enable(true, false);
        return ErrorCodes::OK;
    }
    return ErrorCodes::disabled;
}

//---------------------------------------------------------------------------------------------------------
// Update
//---------------------------------------------------------------------------------------------------------

ErrorCodes Vcameras::Update(bool onRed, bool onRamp){
    if(!_connected) {if(_debug_ifc!=nullptr) _debug_ifc->println("Cams no connection");return ErrorCodes::no_connection;}

    if(onRamp != _oldOnRamp){
        if(onRamp && !_oldOnRamp) {
            ResetCam();
            _ui->ShowPopup("Ramp - disable Cam",ErrorCodes::info);
        }
        else _ui->ShowPopup("Ramp end - enable Cam", ErrorCodes::info,3);
        _oldOnRamp = onRamp;
    }

    if(onRamp) return ErrorCodes::ramp;

    // Progress pending async enable commands for both cameras each cycle.
    EnableNonBlockingStep();
    EnableNonBlockingStep();

    if(HandleReset() == ErrorCodes::disabled) {if(_debug_ifc!=nullptr) _debug_ifc->println("Cams Disabled");return ErrorCodes::disabled;}

    if(_oldRed && !onRed) {
        Enable(true, false);
    } else if (!_oldRed && onRed){
        Enable(false, false);
    }
    _oldRed = onRed;
    if(onRed) return ErrorCodes::OK;

    if(!_enabled) { if(_debug_ifc!=nullptr) _debug_ifc->println("Cams Disabled"); return ErrorCodes::disabled;}

    //check if cameras are enabled
    _LeftEnabled = digitalRead(CAMERAL_PIN_EN);
    _RightEnabled = digitalRead(CAMERAR_PIN_EN);

    //Check Alert State
    if(_LeftEnabled || _RightEnabled) _Alert = digitalRead(CAMERA_PIN_INT);

    //Continue when no camera is reporting
    if(!_Alert) return ErrorCodes::OK;

    String str = "";
    ErrorCodes side;

    //Wait for new Data
    str = Recieve();
    if(str[0] == ' ') return ErrorCodes::OK;    //Is data valid?

    ResetCam();

    //Determine victim side
    if(str[0] == 'L') side = ErrorCodes::left;
    else if(str[0] == 'R') side = ErrorCodes::right;
    else return ErrorCodes::invalid;
    

    //Determine Victim Type
    char victim = str[1];

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

    //Stops robot
    _drivetrain->Stop();

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

    //Signal Victim
    char buffer[29];
    sprintf(buffer,"VICTIM: %c, Side: %c", victim, ((side == ErrorCodes::left) ? 'L' : 'R'));
    _ui->ShowPopup(buffer, ErrorCodes::info, 5);

    _ui->Signal(ErrorCodes::BUZZER_LED, 500, 500 ,1);
    _ui->Update();
    _ui->Signal(ErrorCodes::LED, 500, 500 ,4);

    //Eject
    _ejector->Eject(side, amount);
    _ui->Update();
    
    _ui->Update();
    _robot->OnVictimDetected();

    //2RP - Harmed
    //1RP - Stable
    //0RP - Unharmed

    _timeFound = millis();  //reset reset time to compensate for waiting
    return ErrorCodes::OK;
}