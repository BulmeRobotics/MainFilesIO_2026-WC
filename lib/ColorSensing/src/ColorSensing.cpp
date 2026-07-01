#include "ColorSensing.h"
#include <CustomDatatypes.h>
#include <UserInterface.h>

ErrorCodes EEPROM::Init(){
    if(i2ceeprom.begin(EEPROM_ADDR)) return ErrorCodes::OK;
    else return ErrorCodes::ERROR;
}

int16_t EEPROM::GetStartAddr(PoI_Type type, char sensor){
    if (sensor == 'R')
        return EEPROM_START_ADD_REFLECTIVE;
    
    switch (type) {
    case PoI_Type::white:
        return (sensor == 'F') ? EEPROM_START_ADD_FRONT_WHITE : EEPROM_START_ADD_MIDDLE_WHITE;
	    break;
    case(PoI_Type::blue):
        return (sensor == 'F') ? EEPROM_START_ADD_FRONT_BLUE : EEPROM_START_ADD_MIDDLE_BLUE;
        break;
    case(PoI_Type::checkpoint):
        return (sensor == 'F') ? EEPROM_START_ADD_FRONT_CP : EEPROM_START_ADD_MIDDLE_CP;
        break;
    case(PoI_Type::black):
        return (sensor == 'F') ? EEPROM_START_ADD_FRONT_BLACK : EEPROM_START_ADD_MIDDLE_BlACK;
        break;
    case(PoI_Type::red):
        return (sensor == 'F') ? EEPROM_START_ADD_FRONT_DZ : EEPROM_START_ADD_MIDDLE_DZ;
        break;
    default:
        break;
    }
    return 0;
}

ErrorCodes EEPROM::ReadFromEEPROM(PoI_Type type, char sensor, uint16_t* buffer){
    int16_t curAddr = GetStartAddr(type, sensor);

    for(uint8_t i=0; i<EEPROM_PACKAGE_NUM; i++, curAddr += (EEPROM_PACKAGE_SIZE + EEPROM_PACKAGE_OVERHEAD)){
        uint8_t localBuffer[EEPROM_PACKAGE_SIZE];
		i2ceeprom.read(curAddr, localBuffer, EEPROM_PACKAGE_SIZE);
        memcpy((void*)&buffer[i], localBuffer, EEPROM_PACKAGE_SIZE);
    }
    return ErrorCodes::OK;
}

ErrorCodes EEPROM::WriteToEEPROM(PoI_Type type, char sensor, uint16_t* buffer){
    int16_t curAddr = GetStartAddr(type, sensor);
	for (int i = 0; i < EEPROM_PACKAGE_NUM; i++, curAddr += (EEPROM_PACKAGE_SIZE + EEPROM_PACKAGE_OVERHEAD)) {
		uint8_t localBuffer[EEPROM_PACKAGE_SIZE];
		memcpy(localBuffer, (void*)&buffer[i], EEPROM_PACKAGE_SIZE);
		i2ceeprom.write(curAddr, localBuffer, EEPROM_PACKAGE_SIZE);
	}
	return ErrorCodes::OK;
}

// COLOR SENSING: --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint8_t ColorSensing::Init(TwoWire* wire, UserInterface* ui, EEPROM* eeprom){
    _eeprom = eeprom;
    _ui = ui;
    uint8_t ebuff = 0;

    // --- AS7341 FRONT ---
    TCA9548A(0);
    if (!front.begin(0x39, wire, 0)) ebuff |= 1 << 0;
	//LED
    if (!(ebuff & 0x01)) {
        if (!front.setLEDCurrent(LED_CURRENT))	    ebuff |= 1 << 2;
        if (!front.enableLED(_ENABLE_LED_FRONT))	ebuff |= 1 << 2;
    }
    if (!front.setGain(AS7341_GAIN_256X))   ebuff |= 1 << 4;
    if (!front.setATIME(ATIME_Front))       ebuff |= 1 << 7;
    if (!front.setASTEP(ASTEP_Front))       ebuff |= 1 << 7;

    // --- AS7341 FRONT ---
    TCA9548A(1);
    if (!middle.begin(0x39, wire, 0))	ebuff |= 1 << 1;

    //Set LED Current and turn on
    if (!(ebuff & 1 << 1)) {
        if (!middle.setLEDCurrent(LED_CURRENT))	    ebuff |= 1 << 3;
        if (!middle.enableLED(_ENABLE_LED_MIDDLE))	ebuff |= 1 << 3;
    }
    if (!middle.setGain(AS7341_GAIN_256X))  ebuff |= 1 << 5;
    if (!middle.setATIME(ATIME_Middle))     ebuff |= 1 << 6;
    if (!middle.setASTEP(ASTEP_Middle))     ebuff |= 1 << 6;

    // --- Reflective Sensor ---
    pinMode(REFL_CTRL, OUTPUT);
    pinMode(REFL_READ, INPUT);

    // --- EEPROM ---
    _eeprom->ReadFromEEPROM(PoI_Type::white,      'F', (uint16_t*)&frontColorsCalibrated[WHITE]);
    _eeprom->ReadFromEEPROM(PoI_Type::black,      'F', (uint16_t*)&frontColorsCalibrated[BLACK]);
    _eeprom->ReadFromEEPROM(PoI_Type::blue,       'F', (uint16_t*)&frontColorsCalibrated[BLUE]);
    _eeprom->ReadFromEEPROM(PoI_Type::red,        'F', (uint16_t*)&frontColorsCalibrated[RED]);
    _eeprom->ReadFromEEPROM(PoI_Type::checkpoint, 'F', (uint16_t*)&frontColorsCalibrated[SILVER]);
    
    _eeprom->ReadFromEEPROM(PoI_Type::white,      'M', (uint16_t*)&middleColorsCalibrated[WHITE]);
    _eeprom->ReadFromEEPROM(PoI_Type::black,      'M', (uint16_t*)&middleColorsCalibrated[BLACK]);
    _eeprom->ReadFromEEPROM(PoI_Type::blue,       'M', (uint16_t*)&middleColorsCalibrated[BLUE]);
    _eeprom->ReadFromEEPROM(PoI_Type::red,        'M', (uint16_t*)&middleColorsCalibrated[RED]);
    _eeprom->ReadFromEEPROM(PoI_Type::checkpoint, 'M', (uint16_t*)&middleColorsCalibrated[SILVER]);

    return ebuff;
}

ErrorCodes ColorSensing::EnableRead(bool enable){
    //Turn On
    if(!_READING && enable){
        TCA9548A(0);
        if(!front.startReading()) return ErrorCodes::ERROR;
        TCA9548A(1);
        if(!middle.startReading()) return ErrorCodes::ERROR;
        _READING = true;
    }

    //Turn Off
    if(_READING && !enable){
        uint32_t time = millis();
        TCA9548A(0);
        while(!front.checkReadingProgress()){
            if(time + COLOR_TIMEOUT < millis()) return ErrorCodes::TIMEOUT;
            delay(5);
        }
        TCA9548A(1);
        while(!middle.checkReadingProgress()){
            if(time + COLOR_TIMEOUT < millis()) return ErrorCodes::TIMEOUT;
            delay(5);
        }
        _READING = false;
    }
    return ErrorCodes::OK;
}

void ColorSensing::UpdateHistory(uint16_t f5, uint16_t f6, uint16_t f7) {
    f5History[historyIndex] = f5;
    f6History[historyIndex] = f6;
    f7History[historyIndex] = f7;
    historyIndex++;

    // check for Checkpoint
    if (historyIndex >= WINDOW_SIZE) {
        historyIndex = 0;
        int8_t changes = 0;

        for (int i = 1; i < WINDOW_SIZE; i++) {
            // Read buffer                    
            int diff5 = (int)f5History[i] - (int)f5History[i-1];
            int diff6 = (int)f6History[i] - (int)f6History[i-1];
            int diff7 = (int)f7History[i] - (int)f7History[i-1];
            

            if (diff5 > NOISE_THRESHOLD || diff6 > NOISE_THRESHOLD || diff7 > NOISE_THRESHOLD) {
                changes++;
            } else if (diff5 < -NOISE_THRESHOLD || diff6 < -NOISE_THRESHOLD || diff7 < -NOISE_THRESHOLD) {
                changes++;
            }
        }
        // Wenn es oft genug hin und her geschwankt ist -> Silber!
        if (changes >= FLICKER_MIN_COUNT) {
            _checkpoint = true;
        }
    }
}

ErrorCodes ColorSensing::Update(){
    if(_READING){
        digitalWrite(REFL_CTRL, HIGH); //Set Pin High  for Reflective 
        // -- FRONT --
        TCA9548A(0);
        if(front.checkReadingProgress()){
            if(!_FREEZE_SENSOR) colorFront = checkFront();
            if(!front.startReading()) return ErrorCodes::ERROR;
        }
        // -- MIDDLE --
        TCA9548A(1);
        if(middle.checkReadingProgress()){
            if(!_FREEZE_SENSOR) colorMiddle = checkMiddle();
            if(!middle.startReading()) return ErrorCodes::ERROR;
        }
        return ErrorCodes::OK;
    }
    return ErrorCodes::disabled;
}

TileType ColorSensing::GetFloor(){
    if(_FREEZE_SENSOR) return TileType::obstacle;
    else if(colorFront == PoI_Type::black) return TileType::black;
    TileType floor = colorMiddle;
    if (floor == TileType::checkpoint && !_checkpoint) return TileType::visited;
    else return floor;
}

bool ColorSensing::GetAlert(){
    if(_FREEZE_SENSOR) return false;	// No drive-alert while frozen (ramp/turn) — prevents slow-speed sticking on stale _ALERT
    return _ALERT;
}

void ColorSensing::printDebugData(uint16_t* rawColor, char sensor){
    if(_debugPort != nullptr){
        _debugPort->print(sensor);
        _debugPort->print("\t");
        for (uint8_t i = 0; i < 10; i++)
        {
            _debugPort->print(rawColor[i]);
            _debugPort->print("\t");
        }
        _debugPort->println();    
    }
}

PoI_Type ColorSensing::checkFront(){
    //Read all channels
    TCA9548A(0);
    uint16_t colorRaw[10];
    colorRaw[0] = front.getChannel(AS7341_CHANNEL_415nm_F1);
    colorRaw[1] = front.getChannel(AS7341_CHANNEL_445nm_F2);
    colorRaw[2] = front.getChannel(AS7341_CHANNEL_480nm_F3);
    colorRaw[3] = front.getChannel(AS7341_CHANNEL_515nm_F4);
    colorRaw[4] = front.getChannel(AS7341_CHANNEL_555nm_F5);
    colorRaw[5] = front.getChannel(AS7341_CHANNEL_590nm_F6);
    colorRaw[6] = front.getChannel(AS7341_CHANNEL_630nm_F7);
    colorRaw[7] = front.getChannel(AS7341_CHANNEL_680nm_F8);
    colorRaw[8] = front.getChannel(AS7341_CHANNEL_CLEAR);
    colorRaw[9] = front.getChannel(AS7341_CHANNEL_NIR);

    //Only prints values when debugPort is set, otherwise does nothing
    printDebugData(colorRaw, 'F');

    // Silver Tile
    //UpdateHistory(colorRaw[4], colorRaw[5], colorRaw[6]);


    // Not white
    if( colorRaw[8] <= 13000){
        _ALERT = true;
        _checkpoint = false;
        
        //Check Black
        if(colorRaw[8] <= 2700 && colorRaw[1] <= 1200)  //pre WC: [8] 2500; [1] 1000
            return PoI_Type::black;
        else return PoI_Type::undef;
    }

    //Silver - checkpoint
    if(colorRaw[4] < 7000 && colorRaw[5] < 7000){
        _checkpoint = true;
        _ALERT = true;
        return PoI_Type::checkpoint;
    }

    _checkpoint = false;
    _ALERT = false;
    return PoI_Type::white; //TODO: Implement actual color checking
}

TileType ColorSensing::checkMiddle(){
    //Read all channels
    TCA9548A(1);
    uint16_t colorRaw[10];
    colorRaw[0] = middle.getChannel(AS7341_CHANNEL_415nm_F1);
    colorRaw[1] = middle.getChannel(AS7341_CHANNEL_445nm_F2);
    colorRaw[2] = middle.getChannel(AS7341_CHANNEL_480nm_F3);
    colorRaw[3] = middle.getChannel(AS7341_CHANNEL_515nm_F4);
    colorRaw[4] = middle.getChannel(AS7341_CHANNEL_555nm_F5);
    colorRaw[5] = middle.getChannel(AS7341_CHANNEL_590nm_F6);
    colorRaw[6] = middle.getChannel(AS7341_CHANNEL_630nm_F7);
    colorRaw[7] = middle.getChannel(AS7341_CHANNEL_680nm_F8);
    colorRaw[8] = middle.getChannel(AS7341_CHANNEL_CLEAR);
    colorRaw[9] = middle.getChannel(AS7341_CHANNEL_NIR);

    //Only prints values when debugPort is set, otherwise does nothing
    printDebugData(colorRaw, 'M');
    
    if(_checkpoint && colorRaw[8] > 61000) return TileType::checkpoint;

    //Blau:
    if(colorRaw[8] < 40000){
        return TileType::blue;
    } else if(colorRaw[8] < 61000){
        return TileType::dangerZone;
    }
    return TileType::visited; //TODO: Implement actual color checking
}

ErrorCodes ColorSensing::Calibrate(PoI_Type type){
    uint32_t time = millis();

    _ui->ShowCalibrationScreen(type);

    //Finish old reading
    TCA9548A(0);
    while(!front.checkReadingProgress() && time + COLOR_TIMEOUT > millis()){
        delay(10);
    }

    TCA9548A(1);
    while(!middle.checkReadingProgress()&& time + COLOR_TIMEOUT > millis()){
        delay(10);
    }


    //Update calibration screen:
    _ui->UpdateCalibrationProgress(1, RUNS_calibration + 1);

    uint32_t bufferFront[10] = {0};
    uint32_t bufferMiddle[10] = {0};

    //Read new values
    for (uint8_t i = 0; i<RUNS_calibration; i++){
        time = millis();

        //Middle Sensor --------------------------------------------------------
        TCA9548A(1);
        middle.startReading();
        while (!middle.checkReadingProgress()) {delay(10);};

        bufferMiddle[0] += middle.getChannel(AS7341_CHANNEL_415nm_F1);
        bufferMiddle[1] += middle.getChannel(AS7341_CHANNEL_445nm_F2);
        bufferMiddle[2] += middle.getChannel(AS7341_CHANNEL_480nm_F3);
        bufferMiddle[3] += middle.getChannel(AS7341_CHANNEL_515nm_F4);
        bufferMiddle[4] += middle.getChannel(AS7341_CHANNEL_555nm_F5);
        bufferMiddle[5] += middle.getChannel(AS7341_CHANNEL_590nm_F6);
        bufferMiddle[6] += middle.getChannel(AS7341_CHANNEL_630nm_F7);
        bufferMiddle[7] += middle.getChannel(AS7341_CHANNEL_680nm_F8);
        bufferMiddle[8] += middle.getChannel(AS7341_CHANNEL_CLEAR);
        bufferMiddle[9] += middle.getChannel(AS7341_CHANNEL_NIR);
        
        //Middle Sensor --------------------------------------------------------
        TCA9548A(0);
        front.startReading();

        while(!front.checkReadingProgress()) {delay(10);};

        bufferFront[0] += front.getChannel(AS7341_CHANNEL_415nm_F1);
        bufferFront[1] += front.getChannel(AS7341_CHANNEL_445nm_F2);
        bufferFront[2] += front.getChannel(AS7341_CHANNEL_480nm_F3);
        bufferFront[3] += front.getChannel(AS7341_CHANNEL_515nm_F4);
        bufferFront[4] += front.getChannel(AS7341_CHANNEL_555nm_F5);
        bufferFront[5] += front.getChannel(AS7341_CHANNEL_590nm_F6);
        bufferFront[6] += front.getChannel(AS7341_CHANNEL_630nm_F7);
        bufferFront[7] += front.getChannel(AS7341_CHANNEL_680nm_F8);
        bufferFront[8] += front.getChannel(AS7341_CHANNEL_CLEAR);
        bufferFront[9] += front.getChannel(AS7341_CHANNEL_NIR);

        if((uint32_t)(time + COLOR_TIMEOUT) < millis()) {
            _ui->FinishCalibration(false);
            return ErrorCodes::TIMEOUT;
        }
        
        _ui->UpdateCalibrationProgress(i + 2, RUNS_calibration + 1);
    }
    uint8_t floor = 0;
    switch (type)
    {
    case PoI_Type::black:
        floor = BLACK;
        break;
    case PoI_Type::blue:
        floor = BLUE;
        break;
    case PoI_Type::checkpoint:
        floor = SILVER;
        break;
    case PoI_Type::red:
        floor = RED;
        break;
    
    default:
        floor = WHITE;
        break;
    }

    frontColorsCalibrated[floor].F1     = (uint16_t)(bufferFront[0]/(uint32_t)RUNS_calibration);
    frontColorsCalibrated[floor].F2     = (uint16_t)(bufferFront[1]/(uint32_t)RUNS_calibration);
    frontColorsCalibrated[floor].F3     = (uint16_t)(bufferFront[2]/(uint32_t)RUNS_calibration);
    frontColorsCalibrated[floor].F4     = (uint16_t)(bufferFront[3]/(uint32_t)RUNS_calibration);
    frontColorsCalibrated[floor].F5     = (uint16_t)(bufferFront[4]/(uint32_t)RUNS_calibration);
    frontColorsCalibrated[floor].F6     = (uint16_t)(bufferFront[5]/(uint32_t)RUNS_calibration);
    frontColorsCalibrated[floor].F7     = (uint16_t)(bufferFront[6]/(uint32_t)RUNS_calibration);
    frontColorsCalibrated[floor].F8     = (uint16_t)(bufferFront[7]/(uint32_t)RUNS_calibration);
    frontColorsCalibrated[floor].Clear  = (uint16_t)(bufferFront[8]/(uint32_t)RUNS_calibration);
    frontColorsCalibrated[floor].NIR    = (uint16_t)(bufferFront[9]/(uint32_t)RUNS_calibration);
    
    middleColorsCalibrated[floor].F1     = (uint16_t)(bufferMiddle[0]/(uint32_t)RUNS_calibration);
    middleColorsCalibrated[floor].F2     = (uint16_t)(bufferMiddle[1]/(uint32_t)RUNS_calibration);
    middleColorsCalibrated[floor].F3     = (uint16_t)(bufferMiddle[2]/(uint32_t)RUNS_calibration);
    middleColorsCalibrated[floor].F4     = (uint16_t)(bufferMiddle[3]/(uint32_t)RUNS_calibration);
    middleColorsCalibrated[floor].F5     = (uint16_t)(bufferMiddle[4]/(uint32_t)RUNS_calibration);
    middleColorsCalibrated[floor].F6     = (uint16_t)(bufferMiddle[5]/(uint32_t)RUNS_calibration);
    middleColorsCalibrated[floor].F7     = (uint16_t)(bufferMiddle[6]/(uint32_t)RUNS_calibration);
    middleColorsCalibrated[floor].F8     = (uint16_t)(bufferMiddle[7]/(uint32_t)RUNS_calibration);
    middleColorsCalibrated[floor].Clear  = (uint16_t)(bufferMiddle[8]/(uint32_t)RUNS_calibration);
    middleColorsCalibrated[floor].NIR    = (uint16_t)(bufferMiddle[9]/(uint32_t)RUNS_calibration);

    _eeprom->WriteToEEPROM(type, 'F', (uint16_t*)&frontColorsCalibrated[floor]);
    _eeprom->WriteToEEPROM(type, 'M', (uint16_t*)&middleColorsCalibrated[floor]);

    printDebugData((uint16_t*)&middleColorsCalibrated[floor], 'C');
    printDebugData((uint16_t*)&frontColorsCalibrated[floor], 'c');

    _ui->FinishCalibration(true);
    return ErrorCodes::OK;
}
