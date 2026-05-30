//author: Vincent Rohkamm
//date: 18.11.2025
//description: User Interface for Robot

#include "UserInterface.h"
#include <ColorSensing.h>
#include <Vcameras.h>

// Definitionen der statischen Member
constexpr char UserInterface::UI_VERSION[4];
GDTpoint_t UserInterface::LastContact{};
bool UserInterface::NewContact = false;

#ifdef _MSC_VER
  #pragma region Constructor //-----------------------------------------------------------
#endif

UserInterface::UserInterface(uint8_t updateInterval){
    UPDATE_INTERVAL = updateInterval;
}


#ifdef _MSC_VER

  #pragma endregion Constructor //-----------------------------------------------------------
  #pragma region Icon Drawing 
#endif

void UserInterface::DrawIconLocation(GigaDisplay_GFX& display, uint16_t cx, uint16_t cy){
    display.fillCircle(cx, cy - 20, 25, 0);
    display.fillTriangle(cx - 25, cy - 20, cx, cy + 40, cx + 25, cy - 20, 0);
    display.fillCircle(cx, cy - 20, 13, BTN_COLOR);
}
void UserInterface::DrawIconSensor(GigaDisplay_GFX& display, uint16_t cx, uint16_t cy){
    display.fillCircle(cx, cy, 40, 0);
    display.fillCircle(cx, cy, 35, BTN_COLOR);
    // (Weitere Ringe deines alten Codes hier optional ergänzen)
    display.fillCircle(cx, cy, 10, 0);
}
void UserInterface::DrawIconSettings(GigaDisplay_GFX& display, uint16_t cx, uint16_t cy){

    display.fillCircle(cx, cy, 33, 0);

	display.fillCircle(cx, cy-30, 10, 0);	//Vertical
	display.fillCircle(cx, cy+30, 10, 0);

	display.fillCircle(cx-30, cy, 10, 0);	//Horizontal
	display.fillCircle(cx+30, cy, 10, 0);

	display.fillCircle(cx+21, cy-21, 10, 0);    //Rechtsoben
	display.fillCircle(cx-21, cy-21, 10, 0);     //Links oben

	display.fillCircle(cx+21, cy+21, 10, 0);    //Rechts unten
	display.fillCircle(cx-21, cy+21, 10, 0);     //Links unten
	display.fillCircle(cx, cy, 19, BTN_COLOR);
}
void UserInterface::DrawIconAbout(GigaDisplay_GFX& display, uint16_t cx, uint16_t cy){
    display.fillCircle(cx, cy, 38, 0);
    display.fillCircle(cx, cy, 33, BTN_COLOR);
    display.setCursor(cx - 10, cy - 15);
    display.setTextSize(4);
    display.setTextColor(TEXT_COLOR);
    display.print("I");
}

#ifdef _MSC_VER
  #pragma endregion Icon Drawing //-----------------------------------------------------------
  #pragma region MainMenu 
#endif

// ------------------------------------------------------------------
// HAUPTMENÜ (LINKER BALKEN)
// ------------------------------------------------------------------

void UserInterface::DrawMainMenuStatic() {
    // Grauer Hintergrundbalken
    display.fillRoundRect(0, 6, 120, 468, 15, HL_COLOR);

    btnMenuLocation.Draw(display);
    btnMenuSensor.Draw(display);
    btnMenuSettings.Draw(display);
    btnMenuAbout.Draw(display);
}

void UserInterface::HandleMainMenu(uint16_t tx, uint16_t ty) {
    if (btnMenuLocation.IsPressed(tx, ty) && *p_state != RobotState::INFO_VISUAL) {
        *p_state = RobotState::INFO_VISUAL;
        BuzzerSignal(5, 0, 1);
    }
    else if (btnMenuSensor.IsPressed(tx, ty) && *p_state != RobotState::INFO_SENSOR) {
        *p_state = RobotState::INFO_SENSOR;
        BuzzerSignal(5, 0, 1);
    }
    else if (btnMenuSettings.IsPressed(tx, ty) && *p_state != RobotState::SETTINGS) {
        *p_state = RobotState::SETTINGS;
        BuzzerSignal(5, 0, 1);
    }
    else if (btnMenuAbout.IsPressed(tx, ty) && *p_state != RobotState::ABOUT) {
        *p_state = RobotState::ABOUT;
        BuzzerSignal(5, 0, 1);
    }
}

#ifdef _MSC_VER
  #pragma endregion MainMenu 
  #pragma region About Menu  
#endif

// ------------------------------------------------------------------
// About Menu
// ------------------------------------------------------------------

void UserInterface::ConstructAboutMenu(){
    display.fillScreen(0);

    DrawMainMenuStatic();
	//prepare font
	display.setTextSize(10);
	display.setTextColor(TEXT_COLOR);

	//Robot Name
	//display.fillRoundRect(130, 6, 400, 100, 20, BTN_COLOR);
	display.setCursor(150, 21);
	display.println("B.Robots");

	//HTL BULME Graz-G�sting
	display.setTextSize(TEXT_SIZE);
	display.setCursor(150, display.getCursorY());
	display.println("HTL Bulme Graz-Gosting");

	//UI-Version
	display.setTextSize(3);
	display.setCursor(150, display.getCursorY());
	display.print("\tUI Version ");
	display.println(UI_VERSION);

	//Team:
	display.setCursor(150, display.getCursorY() + 15);
	display.setTextSize(5);
	display.println("Team:");
	//Mentor
	display.setTextSize(TEXT_SIZE);
	display.setCursor(160, display.getCursorY() + 5);
	display.drawLine(150, display.getCursorY() - 4, 750, display.getCursorY() - 4, TEXT_COLOR);
	display.println("Mentor:");
	display.setTextSize(3);
	display.setCursor(175, display.getCursorY());
	display.println("Peter Frauscher");

	//Teammembers
	display.setCursor(160, display.getCursorY() + 5);
	display.setTextSize(TEXT_SIZE);
	display.println("Members:");
	display.setTextSize(3);
	display.setCursor(175, display.getCursorY());
	display.println("Paul Charusa | Florian Wiesner");
	display.setCursor(175, display.getCursorY());
	display.println("Thomas Rauch | Vincent Rohkamm");
}

#ifdef _MSC_VER
  #pragma endregion About Menu 
  #pragma region Run Menu 
#endif

// ------------------------------------------------------------------
// Run Menu
// ------------------------------------------------------------------

void UserInterface::ConstructRunMenu() {
    display.fillScreen(BG_COLOR); // Komplettes Display schwarz für die Karte
    
    // Trennlinie zeichnen (X = 600)
    display.drawLine(MAP_AREA_WIDTH, 0, MAP_AREA_WIDTH, 480, HL_COLOR);
    display.drawLine(MAP_AREA_WIDTH + 1, 0, MAP_AREA_WIDTH + 1, 480, HL_COLOR); // Bisschen dicker

    // Hintergrund für das rechte Panel
    display.fillRoundRect(MAP_AREA_WIDTH + 5, 5, 190, 470, 10, HL_COLOR);

    // Statische Labels zeichnen
    display.setTextSize(3);
    display.setTextColor(TEXT_COLOR);

    // --- Kamera Status ---
    display.setCursor(MAP_AREA_WIDTH + 15, 20);
    display.print("CAMERAS");
    display.drawLine(MAP_AREA_WIDTH + 15, 45, 780, 45, TEXT_COLOR);
    
    display.setCursor(MAP_AREA_WIDTH + 15, 60);
    display.print("Left:");
    display.setCursor(MAP_AREA_WIDTH + 15, 120);
    display.print("Right:");

    // --- Color Sensor Status ---
    display.setCursor(MAP_AREA_WIDTH + 15, 220);
    display.print("COLOR SENS");
    display.drawLine(MAP_AREA_WIDTH + 15, 245, 780, 245, TEXT_COLOR);
    
    display.setCursor(MAP_AREA_WIDTH + 15, 260);
    display.print("State:");
}

void UserInterface::UpdateRunMenu() {
    // 1. KAMERA STATUS UPDATEN
    display.setTextSize(3);
    
    //Kamera Alert
    if(p_camera->IsAlert()){
        //ALERT
    }


    // Linke Kamera
    display.setCursor(MAP_AREA_WIDTH + 15, 85);
    if (!p_camera->IsEnabled(ErrorCodes::left)) {
        display.setTextColor(0x7BEF, HL_COLOR); // Grau
        display.print("DISABLED ");
    } else if (false) {
        display.setTextColor(0xF800, HL_COLOR); // Rot
        display.print("ALERT!   ");
    } else {
        display.setTextColor(0x07E0, HL_COLOR); // Grün
        display.print("CLEAR    ");
    }

    // Rechte Kamera
    display.setCursor(MAP_AREA_WIDTH + 15, 145);
    if (!p_camera->IsEnabled(ErrorCodes::right)) {
        display.setTextColor(0x7BEF, HL_COLOR); 
        display.print("DISABLED ");
    } else if (false) {
        display.setTextColor(0xF800, HL_COLOR); 
        display.print("ALERT!   ");
    } else {
        display.setTextColor(0x07E0, HL_COLOR); 
        display.print("CLEAR    ");
    }

    // 2. COLOR SENSOR STATUS UPDATEN
    display.setCursor(MAP_AREA_WIDTH + 15, 285);
    
    if (p_colorSens != nullptr) {
        if (p_colorSens->Freeze()) {
            display.setTextColor(0x001F, HL_COLOR); // Blau für Freeze
            display.print("FROZEN   ");
        } else if (p_colorSens->GetAlert()) {
            display.setTextColor(0xF800, HL_COLOR); // Rot für Alert
            display.print("ALERT!   ");
        } else {
            display.setTextColor(0x07E0, HL_COLOR); // Grün für Normal
            display.print("NORMAL   ");
        }
    }

    // 3. KARTE ZEICHNEN
    DrawMap();
}

void UserInterface::DrawMap(){
    if(!_updateMap) return;
    _updateMap = false;
    
    if (p_mapping == nullptr) return;

    //Get Tile Data
    Tile* tiles = p_mapping->GetTiles();
    uint16_t currentPos = p_mapping->GetCurrentPosition();
    Orientations currentOri = p_mapping->GetCurrentOrientation();

    if (currentPos >= MAX_TILES || tiles[currentPos].type == TileType::inactive) return;

    // get Robot Position
    int16_t robX = tiles[currentPos].x;
    int16_t robY = tiles[currentPos].y;
    int16_t robZ = tiles[currentPos].z;

    int16_t centerX = MAP_AREA_WIDTH / 2; // 300
    int16_t centerY = 480 / 2;            // 240

    // viewX = left/right from Robotview (-4 to +4)
    // viewY = behind/front from Robotview (-3 to +3)
    for (int16_t viewY = 3; viewY >= -3; viewY--) {
        for (int16_t viewX = -4; viewX <= 4; viewX++) {
            
            int16_t targetX = robX;
            int16_t targetY = robY;

            // 1. Coordinate transformation (depending on view-angle -> Orientation)
            switch (currentOri) {
                case Orientations::North:
                    targetX += viewX;
                    targetY += viewY;
                    break;
                case Orientations::East:
                    targetX += viewY;
                    targetY -= viewX;
                    break;
                case Orientations::South:
                    targetX -= viewX;
                    targetY -= viewY;
                    break;
                case Orientations::West:
                    targetX -= viewY;
                    targetY += viewX;
                    break;
            }
            
            // find Display space
            int16_t drawX = centerX + (viewX * TILE_SIZE) - (TILE_SIZE / 2);
            int16_t drawY = centerY - (viewY * TILE_SIZE) - (TILE_SIZE / 2);

            // search for existing Tile
            uint16_t foundIndex = UINT16_MAX;
            for (uint16_t i = 0; i < MAX_TILES; i++) {
                if (tiles[i].type != TileType::inactive && 
                    tiles[i].x == targetX && 
                    tiles[i].y == targetY && 
                    tiles[i].z == robZ) { // only show current level
                    foundIndex = i;
                    break;
                }
            }

            if (foundIndex != UINT16_MAX) {
                // Tile found -> DRAW
                Tile& t = tiles[foundIndex];
                uint16_t tileColor = BG_COLOR;

                // get color from type
                switch (t.type) {
                    case TileType::unexplored: tileColor = 0x3186;     break; // Dunkelgrau
                    case TileType::visited:    tileColor = 0x7BEF;     break; // Hellgrau
                    case TileType::obstacle:   tileColor = TEXT_COLOR; break; // Gelb
                    case TileType::checkpoint: tileColor = 0x07E0;     break; // Grün
                    case TileType::dangerZone: tileColor = 0xF800;     break; // Rot
                    case TileType::blue:       tileColor = 0x1175;     break; // Blau
                    case TileType::black:      tileColor = 0x0000;     break; // Schwarz
                    default:                   tileColor = BG_COLOR;   break;
                }

                // draw Tile
                display.fillRect(drawX + 1, drawY + 1, TILE_SIZE - 2, TILE_SIZE - 2, tileColor);

                // transform walls
                // get which wall has to be drawn where
                bool drawTop = false, drawRight = false, drawBottom = false, drawLeft = false;

                switch(currentOri) {
                    case Orientations::North:
                        drawTop = (t.north == -1); drawRight = (t.east == -1); drawBottom = (t.south == -1); drawLeft = (t.west == -1);
                        break;
                    case Orientations::East:
                        drawTop = (t.east == -1); drawRight = (t.south == -1); drawBottom = (t.west == -1); drawLeft = (t.north == -1);
                        break;
                    case Orientations::South:
                        drawTop = (t.south == -1); drawRight = (t.west == -1); drawBottom = (t.north == -1); drawLeft = (t.east == -1);
                        break;
                    case Orientations::West:
                        drawTop = (t.west == -1); drawRight = (t.north == -1); drawBottom = (t.east == -1); drawLeft = (t.south == -1);
                        break;
                }

                uint16_t wallColor = TEXT_COLOR; // Gelb (Kontraststark auf dunklen Displays)
                uint8_t wallThickness = 4;

                if (drawTop)    display.fillRect(drawX, drawY, TILE_SIZE, wallThickness, wallColor);
                if (drawBottom) display.fillRect(drawX, drawY + TILE_SIZE - wallThickness, TILE_SIZE, wallThickness, wallColor);
                if (drawRight)  display.fillRect(drawX + TILE_SIZE - wallThickness, drawY, wallThickness, TILE_SIZE, wallColor);
                if (drawLeft)   display.fillRect(drawX, drawY, wallThickness, TILE_SIZE, wallColor);

            } else {
                // No Tile -> overwrite
                display.fillRect(drawX, drawY, TILE_SIZE, TILE_SIZE, BG_COLOR);
            }
        }
    }

    // 3. Roboter exakt in der Mitte zeichnen
    // Da sich die Map nun relativ dreht, zeigt der Roboter logischerweise IMMER nach oben ("vorwärts")!
    uint16_t rColor = 0xF800; // Rot
    display.fillTriangle(centerX, centerY - 15, centerX - 10, centerY + 10, centerX + 10, centerY + 10, rColor);
}

#ifdef _MSC_VER
  #pragma endregion Run Menu 
  #pragma region Settings 
#endif

// ------------------------------------------------------------------
// SETTINGS
// ------------------------------------------------------------------

void UserInterface::ConstructSettingsMenu() {
    display.fillScreen(BG_COLOR); // Komplettes Display löschen
    DrawMainMenuStatic();         // Linke Navigation malen

    // Label-Boxen (Statischer Text)
    display.fillRoundRect(140, 10, 362, 44, 10, HL_COLOR);

    //Color Sensors
    display.fillRoundRect(140, 158, 640, 152, 10, HL_COLOR);
    
    display.setTextSize(3);
    display.setTextColor(TEXT_COLOR, HL_COLOR);
    display.setCursor(150, 20);
    display.print("driveMode:");
    display.setCursor(150, 168);
    display.print("ColorSensor Calibration");

    //Hintergrund
    display.fillRect(150,64,200,84, BTN_COLOR);

    // Alle Buttons malen (Der Text wird durch die Klasse automatisch zentriert!)
    btnSpeedMinus.Draw(display, "-");
    btnSpeedPlus.Draw(display, "+");
    btnCalibWhite.Draw(display, "WHI");
    btnBleConnect.Draw(display, "BLE");
    
    btnCalibBlack.Draw(display, "Bla");
    btnCalibBlue.Draw(display, "Blu");
    btnCalibDZone.Draw(display, "Red");
    btnCalibCheckP.Draw(display, "Sil");
    btnLayerSetting.Draw(display,(p_mapping->GetLayerSetting() == ErrorCodes::single) ? "single" : "multi");
}

#ifdef _MSC_VER
  #pragma endregion Settings
  #pragma region SensorInfo 
#endif

// ------------------------------------------------------------------
// Sensor Info
// ------------------------------------------------------------------

void UserInterface::ConstructSensorInfoMenu(){
    display.fillScreen(BG_COLOR);
    DrawMainMenuStatic();
    display.fillRoundRect(400,180,120,120,10,BTN_COLOR);        //"Robot"
    display.fillTriangle(460,228,448,252,472,252,TEXT_COLOR);   //Direction
}

#ifdef _MSC_VER
  #pragma endregion SensorInfo
  #pragma region Battery 
#endif

// ------------------------------------------------------------------
// Battery
// ------------------------------------------------------------------
uint8_t UserInterface::GetCharge(){
    float voltage = 0.2 + analogRead(batteryPin) * (3.216/1023.0) * 4.0;
    if(voltage > 8.2) lastPercent = 100;
    else if(voltage < 6.8) lastPercent = 0;
    else {
        lastPercent = 10 * voltage;
        lastPercent = 10 * map(lastPercent, 68, 82, 0, 10);
    }
    return lastPercent;
}

void UserInterface::DrawBattery(){
    display.fillRoundRect(680, 10, 100, 44, 5, HL_COLOR); //Battery Background
	display.setTextColor(TEXT_COLOR);
	//Battery symbol
	display.fillRoundRect(685, 24, 3, 16, 5, BG_COLOR);
	display.fillRoundRect(690, 15, 85, 34, 5, BG_COLOR);
	display.fillRoundRect(693, 18, 79, 28, 5, HL_COLOR);

	//Current Charge
	display.fillRoundRect(768 - (lastPercent * 0.75), 18, 4 + (lastPercent * 0.75), 28, 5, BTN_COLOR);
	display.setTextSize(2);
	display.setCursor(715, 25);
	char buff[4];
	sprintf(buff, "%3d", lastPercent);
	display.print(buff);
	display.setTextSize(TEXT_SIZE);
}

#ifdef _MSC_VER
  #pragma endregion Battery 
  #pragma region Touch Handler 
#endif

// ------------------------------------------------------------------
// Touch Handlers
// ------------------------------------------------------------------

void UserInterface::gigaTouchHandler(uint8_t contacts, GDTpoint_t* points) {
    NewContact = true;
    LastContact = points[0];

}

bool UserInterface::GetValidTouch(uint16_t &touchX, uint16_t &touchY){
    if (NewContact) {
        NewContact = false;
        
        uint32_t time = millis();
        if (time - lastTouch >= touchDebounce) {
            lastTouch = time;
            touchX = LastContact.x;
            touchY = LastContact.y;
            return true;
        }
    }
    return false;
}

#ifdef _MSC_VER
  #pragma endregion Touch Handler
  #pragma region Calibration //------------------------------------------------------
#endif

// ------------------------------------------------------------------
// Calibration
// ------------------------------------------------------------------

void UserInterface::ShowCalibrationScreen(PoI_Type type){
    uint16_t popX = 150, popY = 150, popW = 500, popH = 180;
    display.fillRoundRect(popX, popY, popW, popH, 15, BG_COLOR);
    display.drawRoundRect(popX, popY, popW, popH, 15, HL_COLOR);
    display.drawRoundRect(popX+1, popY+1, popW-2, popH-2, 15, HL_COLOR); // Dickerer Rahmen
    
    // Titel schreiben
    display.setTextColor(TEXT_COLOR);
    display.setTextSize(4);
    display.setCursor(popX + 20, popY + 20);

    if(type == PoI_Type::ble){
        display.print("Connecting BLE");
    }
    else {
        display.print("Calibrating ");
        
        // Name der Farbe auflösen
        switch(type) {
            case PoI_Type::black:      display.print("BLACK"); break;
            case PoI_Type::blue:       display.print("BLUE"); break;
            case PoI_Type::white:      display.print("WHITE"); break;
            case PoI_Type::dangerZone: display.print("D-ZONE"); break;
            case PoI_Type::red:        display.print("RED"); break;
            case PoI_Type::checkpoint: display.print("CHECKP"); break;
            default:                   display.print("UNKNOWN"); break;
        }
    }
    
    // Trennlinie
    display.drawLine(popX, popY + 60, popX + popW, popY + 60, HL_COLOR);
    
    // Initiale Progress-Anzeige
    display.setTextSize(3);
    display.setCursor(popX + 20, popY + 80);
    display.print("Progress:   0%");
}
void UserInterface::UpdateCalibrationProgress(uint8_t step, uint8_t totalSteps){
    uint16_t popX = 150, popY = 150;
    
    // Prozentzahl überschreiben (Dank BG_COLOR schmiert der Text nicht!)
    display.setTextColor(TEXT_COLOR, BG_COLOR); 
    display.setTextSize(3);
    display.setCursor(popX + 182, popY + 80);
    
    uint8_t percent = (step * 100) / totalSteps;
    char buff[5];
    sprintf(buff, "%3d%%", percent);
    display.print(buff);
    
    // Dynamischen Fortschrittsbalken zeichnen
    display.fillRoundRect(popX + 20, popY + 130, (460 * step) / totalSteps, 20, 5, BTN_COLOR);
}
void UserInterface::FinishCalibration(bool success){
    uint16_t popX = 150, popY = 150;
    
    // Status-Text in Grün (Erfolg) oder Rot (Fehler) überschreiben
    display.setTextColor(success ? 0x07E0 : 0xF800, BG_COLOR); 
    display.setTextSize(3);
    display.setCursor(popX + 20, popY + 80);
    display.print(success ? "Done! Touch to close.   " : "ERROR! Touch to close.  ");
    
    // --- Warten: 3 Sekunden oder Touch ---
    uint32_t start = millis();
    NewContact = false; // Touch-Flag sicherheitshalber resetten
    
    while (millis() - start < 3000) {
        if (NewContact) {
            NewContact = false;
            BuzzerSignal(5, 0, 1);
            break; // Beendet das Warten sofort bei Touch
        }
        delay(10); // Verhindert Watchdog-Crashes
    }
    
    // Wenn alles vorbei ist, zwingen wir das UI, das Settings-Menü neu zu zeichnen
    *p_state = RobotState::SETTINGS;
    lastState = RobotState::CALIBRATION;
}


void UserInterface::ShowResetScreen(){
    uint16_t popX = 150, popY = 150, popW = 500, popH = 180;
    display.fillRoundRect(popX, popY, popW, popH, 15, BG_COLOR);
    display.drawRoundRect(popX, popY, popW, popH, 15, HL_COLOR);
    display.drawRoundRect(popX+1, popY+1, popW-2, popH-2, 15, HL_COLOR); // Dickerer Rahmen
    
    // Titel schreiben
    display.setTextColor(TEXT_COLOR);
    display.setTextSize(4);
    display.setCursor(popX + 20, popY + 20);
    display.print("Reset to Checkpoint");
        
    // Trennlinie
    display.drawLine(popX, popY + 60, popX + popW, popY + 60, HL_COLOR);
    
    // Initiale Progress-Anzeige
    display.setTextSize(3);
    display.setCursor(popX + 20, popY + 80);
    display.print("x:");
}
void UserInterface::UpdateResetProgress(char* message, uint8_t step, uint8_t totalSteps){
    uint16_t popX = 150, popY = 150;
    
    // Prozentzahl überschreiben (Dank BG_COLOR schmiert der Text nicht!)
    display.setTextColor(TEXT_COLOR, BG_COLOR); 
    display.setTextSize(3);
    display.setCursor(popX + 20, popY + 80);
    display.print(message);
    
    // Dynamischen Fortschrittsbalken zeichnen
    display.fillRoundRect(popX + 20, popY + 130, (460 * step) / totalSteps, 20, 5, BTN_COLOR);
}
void UserInterface::FinishReset(bool success){
    uint16_t popX = 150, popY = 150;
    
    // Status-Text in Grün (Erfolg) oder Rot (Fehler) überschreiben
    display.setTextColor(success ? 0x07E0 : 0xF800, BG_COLOR); 
    display.setTextSize(3);
    display.setCursor(popX + 20, popY + 80);
    display.print(success ? "START       " : "FEHLER!    ");
    
    // --- Warten: 3 Sekunden oder Touch ---
    uint32_t start = millis();
    NewContact = false; // Touch-Flag sicherheitshalber resetten
    
    while (millis() - start < 1000) {
        if (NewContact) {
            NewContact = false;
            BuzzerSignal(5, 0, 1);
            break; // Beendet das Warten sofort bei Touch
        }
        delay(10); // Verhindert Watchdog-Crashes
    }
    
    // Wenn alles vorbei ist, zwingen wir das UI, das Settings-Menü neu zu zeichnen
    lastState = RobotState::BOOT;
}

#ifdef _MSC_VER
  #pragma endregion Calibration
  #pragma region Init //------------------------------------------------------
#endif    

// ------------------------------------------------------------------
// Initialization
// ------------------------------------------------------------------

void UserInterface::Initialize(){
    // Initialize NeoPixel
    pixels.begin();
    pixels.clear();

    //Initialize Buzzer and Signal-LED
    pinMode(buzzerPin, OUTPUT);
    pinMode(ledPin, OUTPUT);

    //Initialize Display
    display.begin();
    display.setRotation(1);
    display.fillScreen(0x0000); // Black Screen

    //Display Boot Menu
    display.setTextColor(TEXT_COLOR);
	display.setTextSize(5);
	display.setCursor(10, 10);
	display.print("StartUp");

	//Display UI Version
	display.setTextSize(3);
	display.setCursor(574, 10);
	display.print("\tUI Ver. ");
	display.println(UI_VERSION);
	display.setTextSize(4);
	display.drawLine(0, 54, 800, 54, TEXT_COLOR);
	display.setCursor(15, 60);
	display.print("Display UI");
	display.setCursor(684, display.getCursorY());
	display.println("{OK}");

	//Start touch Detector
	if (!touchDetector.begin()) AddInfoMsg("TouchDetection", "Failed", false);
	
	touchDetector.onDetect(gigaTouchHandler);

    display.setCursor(15, display.getCursorY());

    //Check Battery at Startup
    if(GetCharge() <= 10) {
        AddInfoMsg("Battery", "LOW/ERROR!", false);
        while(true){
            BuzzerSignal(200,200,1);
        }
    } else AddInfoMsg("Battery", "OK", true);
    SetIllumination(0, 0, 0, 255); // Turn on Illumination at Startup
    driveMode = ErrorCodes::straight;
}

void UserInterface::ConnectPointer(RobotState* state, ColorSensing* cs, Mapping* mapping, Vcameras* camera){
    p_state = state;
    p_colorSens = cs;
    p_mapping = mapping;
    p_camera = camera;
    return;
}

#ifdef _MSC_VER
  #pragma endregion Init
  #pragma region Messages //------------------------------------------------------
#endif   

// ------------------------------------------------------------------
// Info Message (Boot / BLE)
// ------------------------------------------------------------------

void UserInterface::AddInfoMsg(String Info, String Message, bool success){
    if (p_state != nullptr && *p_state == RobotState::BOOT) display.setCursor(15, display.getCursorY());
    else if(p_state != nullptr && *p_state == RobotState::BT) display.setCursor(10, display.getCursorY());
	else if(p_state != nullptr && *p_state == RobotState::CHECKPOINT) display.setCursor(10, display.getCursorY());
    
    display.setTextSize(3);
	display.print(Info);          
	display.setCursor(800 - (68 + /*24 ursprünglich*/18 * Message.length()), display.getCursorY());
	
    if(success)	display.setTextColor(0x0fc0);
	else display.setTextColor(0xf800);

	display.print("[");
	display.setTextColor(TEXT_COLOR);
	display.print(Message);
	
    if(success)	display.setTextColor(0x0fc0);
	else display.setTextColor(0xf800);
	display.println("]");
    
	display.setTextColor(TEXT_COLOR);
}

void UserInterface::ShowPopup(const char* text, ErrorCodes type, uint16_t timeS) {
    _popupMsg = text;
    _popupType = type;
    _popupDurationMs = (uint32_t)timeS * 1000;
    _popupStartTime = millis();
    _popupActive = true;
}

void UserInterface::DrawPopup() {
    // coordinates
    uint16_t w = 500;
    uint16_t h = 200;
    uint16_t x = (800 - w) / 2;
    uint16_t y = (480 - h) / 2;

    uint16_t color;
    const char* title;

    // Mapping of error codes to title
    switch (_popupType) {
        case ErrorCodes::ERROR:
        case ErrorCodes::TIMEOUT:
        case ErrorCodes::Overflow:
            color = 0xF800; // Rot
            title = "ERROR";
            break;
        case ErrorCodes::warning:
        case ErrorCodes::invalid:
            color = 0xFFE0; // Gelb
            title = "WARNING";
            break;
        default:
            color = 0x07E0; // Grün
            title = "INFO";
            break;
    }

    // shadow and background
    display.fillRoundRect(x + 5, y + 5, w, h, 15, 0x0000);  // shadow
    display.fillRoundRect(x, y, w, h, 15, BG_COLOR);        // Box
    display.drawRoundRect(x, y, w, h, 15, color);           // border
    display.drawRoundRect(x+1, y+1, w-2, h-2, 15, color);   // thick border

    // Header
    display.setTextColor(color);
    display.setTextSize(4);
    display.setCursor(x + 20, y + 20);
    display.print(title);
    display.drawLine(x + 10, y + 65, x + w - 10, y + 65, color);

    // Message
    display.setTextColor(TEXT_COLOR);
    display.setTextSize(3);
    display.setCursor(x + 20, y + 85);
    display.print(_popupMsg);
}


#ifdef _MSC_VER
  #pragma endregion Messages
  #pragma region Update //------------------------------------------------------
#endif 

// ------------------------------------------------------------------
// Update
// ------------------------------------------------------------------

void UserInterface::Update(){
    uint32_t time = millis();
    if (time < lastUpdate + UPDATE_INTERVAL) return;

    lastUpdate = time;

    //Toch Handling
    uint16_t tx = 0, ty = 0;
    bool touched = GetValidTouch(tx, ty);

    //Rendering
    if(*p_state != lastState){
        switch (*p_state) {
        case RobotState::SETTINGS:
            ConstructSettingsMenu();
            break;
        case RobotState::ABOUT:
            ConstructAboutMenu();
            break;
        case RobotState::RUN:
            ConstructRunMenu();
            break;

        case RobotState::INFO_SENSOR:
            ConstructSensorInfoMenu();            
            break;
        
        default:
            break;
        }
        lastState = *p_state;
    }

    // Update Battery Status
    if(*p_state == RobotState::SETTINGS || *p_state == RobotState::INFO_SENSOR || *p_state == RobotState::INFO_VISUAL){
        GetCharge();
        DrawBattery();
    }

    //Draw and Handle mainMenu -> Menu Selector
    if (touched && (*p_state == RobotState::SETTINGS || *p_state == RobotState::ABOUT || *p_state == RobotState::INFO_SENSOR || *p_state == RobotState::INFO_VISUAL))
        HandleMainMenu(tx,ty);
    
    // Handle Menus
    if(*p_state == RobotState::RUN){
        //RUN
        UpdateRunMenu();

    } else if (*p_state == RobotState::INFO_VISUAL){
        //Info Visual

    } else if (*p_state == RobotState::INFO_SENSOR){
        //Sensor Information
        display.setTextColor(TEXT_COLOR, BG_COLOR);
        display.setTextSize(3);

        //Gyro Angles
        display.setCursor(658,374);
        display.println("Gyro");

        char buffer[20];
        display.setCursor(676, display.getCursorY());
        sprintf(buffer, "X: %3d", (int)(gyro_X));
        display.println(buffer);

        display.setCursor(676, display.getCursorY());
        sprintf(buffer, "Y: %3d", (int)(gyro_Y));
        display.println(buffer);

        display.setCursor(676, display.getCursorY());
        sprintf(buffer, "Z: %3d", (int)(gyro_Z));
        display.print(buffer);

        //ToF Values
            //LEFT
        display.setCursor(312,204);
        sprintf(buffer, "%4d", tof_LF);
        display.print(buffer);
        display.setCursor(312,252);
        sprintf(buffer,"%4d", tof_LB);
        display.print(buffer);

            //RIGHT
        display.setCursor(536,204);
        sprintf(buffer, "%4d", tof_RF);
        display.print(buffer);
        display.setCursor(536,252);
        sprintf(buffer,"%4d", tof_RB);
        display.print(buffer);

            //FRONT
        display.setCursor(433,122);
        sprintf(buffer, "%4d", tof_F);
        display.println(buffer);
        display.setCursor(415,display.getCursorY());
        sprintf(buffer, "W%4d", tof_FW);
        display.print(buffer);

            //BACK
        display.setCursor(433,310);
        sprintf(buffer, "%4d", tof_B);
        display.println(buffer);
        display.setCursor(415,display.getCursorY());
        sprintf(buffer, "W%4d", tof_BW);
        display.print(buffer);



    } else if(*p_state == RobotState::BOOT){
        if(touched){
            *p_state = RobotState::SETTINGS;
            BuzzerSignal(5,0,1);
        }
    } 
    else if (*p_state == RobotState::SETTINGS){
        //Settings

        //Update driveMode
        display.setCursor(348, 20);
        display.setTextColor(TEXT_COLOR, HL_COLOR);
        display.setTextSize(3);
        switch (driveMode) {
        case ErrorCodes::straight:
            display.print("straight");
            break;
        case ErrorCodes::north:
            display.print("   North");
            break;
        case ErrorCodes::east:
            display.print("    East");
            break;
        case ErrorCodes::south:
            display.print("   South");
            break;
        case ErrorCodes::west:
            display.print("    West");
            break;
        default:
            display.print("   ERROR");
            break;
        }

        //Update speed:
        display.setCursor(223, 94);
        char buff[6];
        sprintf(buff, "%3d", driveSpeed);
        display.print(buff);

        // Update Buttons
        if(touched){
            //Layer Settings
            if(btnLayerSetting.IsPressed(tx,ty)){
                BuzzerSignal(5,0,1);
                ErrorCodes newLayer = ErrorCodes::single;
                if(p_mapping->GetLayerSetting() == ErrorCodes::single) newLayer = ErrorCodes::multi;
                p_mapping->SetLayerSetting(newLayer);
                btnLayerSetting.Draw(display,(p_mapping->GetLayerSetting() == ErrorCodes::single) ? "single" : "multi");
            }

            //Speed
            if(btnSpeedMinus.IsPressed(tx,ty) && driveSpeed > 10){
                driveSpeed -= 10;
                BuzzerSignal(5, 0, 1);
            } else if(btnSpeedPlus.IsPressed(tx,ty) && driveSpeed < 100){
                driveSpeed += 10;
                BuzzerSignal(5, 0, 1);
            }

            //Calibration
            else if(btnCalibWhite.IsPressed(tx,ty)){
                BuzzerSignal(5, 0, 1);
                *p_state = RobotState::CALIBRATION;
                p_colorSens->Calibrate(PoI_Type::white);
            }
            else if(btnCalibBlack.IsPressed(tx,ty)){
                BuzzerSignal(5, 0, 1);
                *p_state = RobotState::CALIBRATION;
                p_colorSens->Calibrate(PoI_Type::black);
            }
            else if(btnCalibBlue.IsPressed(tx,ty)){
                BuzzerSignal(5, 0, 1);
                *p_state = RobotState::CALIBRATION;
                p_colorSens->Calibrate(PoI_Type::blue);
            }
            else if(btnCalibDZone.IsPressed(tx,ty)){
                BuzzerSignal(5, 0, 1);
                *p_state = RobotState::CALIBRATION;
                p_colorSens->Calibrate(PoI_Type::red);
            }
            else if(btnCalibCheckP.IsPressed(tx,ty)){
                BuzzerSignal(5, 0, 1);
                *p_state = RobotState::CALIBRATION;
                p_colorSens->Calibrate(PoI_Type::checkpoint);
            }
            //BLE
            else if(btnBleConnect.IsPressed(tx,ty) && _BLE_ENABLED){
                BuzzerSignal(5, 0, 1);
                *p_state = RobotState::BT;
            }            
        }
    }

    if(_popupActive){
        //check if time is up
        if(millis() - _popupStartTime >= _popupDurationMs){
            _popupActive = false;
            lastState = RobotState(255);    //forces redraw
        } else{
            DrawPopup();
        }
    }
}

ErrorCodes UserInterface::CycleDriveMode(){
    if (driveMode != ErrorCodes::west) driveMode = (ErrorCodes)((uint8_t)driveMode + 1);
	else driveMode = ErrorCodes::straight;
	p_mapping->SetPriority(driveMode);
    return driveMode;
}


#ifdef _MSC_VER
  #pragma endregion Public Methods  
  #pragma region Signal Unit //-----------------------------------------------------------------------
#endif

void UserInterface::SetIllumination(uint8_t red, uint8_t green, uint8_t blue, uint8_t white){
    pixels.clear();
    for(uint8_t i = 0; i < pixelNum; i++){
        pixels.setPixelColor(i, red, green, blue, white);
        pixels.show();
    }

}

void UserInterface::BuzzerSignal(uint16_t time_ms, uint16_t pause_ms, uint8_t repetitions){
    for(uint8_t i = 0; i < repetitions; i++){
        digitalWrite(buzzerPin, HIGH);
        delay(time_ms);
        digitalWrite(buzzerPin, LOW);
        delay(pause_ms);
    }
}

void UserInterface::LEDSignal(uint16_t time_ms, uint16_t pause_ms, uint8_t repetitions){
    for(uint8_t i = 0; i < repetitions; i++){
        digitalWrite(ledPin, HIGH);
        delay(time_ms);
        digitalWrite(ledPin, LOW);
        delay(pause_ms);
    }
}

void UserInterface::LED_BUZZER_Signal(uint16_t time_ms, uint16_t pause_ms, uint8_t repetitions){
    for(uint8_t i = 0; i < repetitions; i++){
        digitalWrite(ledPin, HIGH);
        digitalWrite(buzzerPin, HIGH);
        delay(time_ms);
        digitalWrite(ledPin, LOW);
        digitalWrite(buzzerPin, LOW);
        delay(pause_ms);
    }
}

#ifdef _MSC_VER
  #pragma endregion Signal Unit
#endif