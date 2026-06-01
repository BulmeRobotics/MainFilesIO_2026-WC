#pragma once

//author: Vincent Rohkamm
//date: 18.11.2025
//description: User Interface for Robot

#ifdef _MSC_VER
    #pragma region Includes //-----------------------------------------------------------------------
#endif

#include <Arduino.h>
#include <Arduino_GigaDisplay.h>
#include <Arduino_GigaDisplay_GFX.h>
#include <Arduino_GigaDisplayTouch.h>
#include "Adafruit_NeoPixel.h"

#include <CustomDatatypes.h>
#include <Mapping.h>
#include <Ejector.h>

class ColorSensing;
class Vcameras;

#ifdef _MSC_VER
    #pragma endregion Includes
    #pragma region Button Class//----------------------------------------------------------------
#endif

typedef void (*IconDrawFunc)(GigaDisplay_GFX& display, uint16_t btnX, uint16_t btnY);

class Button{
    private:
        uint16_t x, y, width, height;
        uint16_t bgColor, textColor;
        uint8_t textSize;
        IconDrawFunc drawFunction;
    public:
        Button(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t bgColor, uint16_t textColor, uint8_t textSize, IconDrawFunc drawFunction)
            : x(x), y(y), width(width), height(height), bgColor(bgColor), textColor(textColor), textSize(textSize), drawFunction(drawFunction) {}

        void Draw(GigaDisplay_GFX& display, const char* label = ""){
            display.fillRoundRect(x, y, width, height, 15, bgColor);
            if(label[0] != '\0'){
                display.setTextSize(textSize);
                display.setTextColor(textColor);

                uint16_t charWidth = 6 * textSize;
                uint16_t charHeight = 8 * textSize;
                uint16_t textWidth = strlen(label) * charWidth;

                display.setCursor(x + (width / 2) - (textWidth / 2), y + (height / 2) - (charHeight / 2));
                display.print(label);
            }
            if (drawFunction != nullptr) drawFunction(display, x + width / 2, y + height / 2);
        }
        bool IsPressed(uint16_t touchX, uint16_t touchY){
            uint16_t tx = touchY;
            uint16_t ty = 480 - touchX; 
            return (tx >= x && tx <= (x + width) && ty >= y && ty <= (y + height));
        }
};

#ifdef _MSC_VER
    #pragma endregion Button Class
    #pragma region UI Class//----------------------------------------------------------------
#endif


class UserInterface {
private:
    static constexpr char UI_VERSION[4] = "2.3";

    #define TEXT_SIZE 4
    #define TEXT_COLOR 0xce4a
    #define BG_COLOR 0
    #define HL_COLOR 0x1168
    #define BTN_COLOR 0x630c

    // --- Display Objects ---
    GigaDisplay_GFX display;
    Arduino_GigaDisplayTouch touchDetector;
    GigaDisplayRGB rgb;


    // --- Toch Detection ---
    static void gigaTouchHandler(uint8_t contacts, GDTpoint_t* points);
    static GDTpoint_t LastContact;
    static bool NewContact;

    uint32_t lastTouch;
    static constexpr uint16_t touchDebounce = 250; //in ms

    bool GetValidTouch(uint16_t &touchX, uint16_t &tocuhY);

    // --- LEDs and Buzzer ---
    static constexpr uint8_t neoPin = 60;
    static constexpr uint8_t pixelNum = 18;
    static constexpr uint8_t ledPin = 61;
    static constexpr uint8_t buzzerPin = 63;
    Adafruit_NeoPixel pixels = Adafruit_NeoPixel(pixelNum, neoPin, NEO_GRBW + NEO_KHZ800);

    // --- Sensor Data ---
    uint16_t tof_LF = 0;
    uint16_t tof_LB = 0;
    uint16_t tof_RF = 0;
    uint16_t tof_RB = 0;
    uint16_t tof_F = 0;
    uint16_t tof_FW = 0;
    uint16_t tof_B = 0;
    uint16_t tof_BW = 0;
    float gyro_X = 0;
    float gyro_Y = 0;
    float gyro_Z = 0;

    // --- Battery Measurement ---
    static constexpr uint8_t batteryPin = A0;
    uint8_t lastPercent = 0;

    // --- Settings ---
    uint8_t UPDATE_INTERVAL; //in ms
    uint32_t lastUpdate = 0;
    static constexpr bool _BLE_ENABLED = true;

    uint8_t driveSpeed = 50;

    // --- Pop-Up System ---
    bool _popupActive = false;
    uint32_t _popupStartTime = 0;
    uint32_t _popupDurationMs = 0;
    const char* _popupMsg = nullptr;
    ErrorCodes _popupType = ErrorCodes::OK;
    void DrawPopup();
    
    
    // --- Object Pointers ---
    ColorSensing* p_colorSens = nullptr;
    Mapping* p_mapping = nullptr;
    Vcameras* p_camera = nullptr;
    Ejector* p_ejector = nullptr;

    // --- States ---
    ErrorCodes driveMode;
    RobotState* p_state = nullptr;
    RobotState lastState = RobotState::BOOT;
    bool _lastCamAlert = false;
    bool _lastCamLeftEn = false;
    bool _lastCamRightEn = false;

    // --- Icon Drawing Functions ---
    static void DrawIconLocation(GigaDisplay_GFX& display, uint16_t cx, uint16_t cy);
    static void DrawIconSensor(GigaDisplay_GFX& display, uint16_t cx, uint16_t cy);
    static void DrawIconSettings(GigaDisplay_GFX& display, uint16_t cx, uint16_t cy);
    static void DrawIconAbout(GigaDisplay_GFX& display, uint16_t cx, uint16_t cy);

// --- Buttons ---

    // -- Menu Buttons --
    Button btnMenuLocation = Button(10, 16, 100, 100, BTN_COLOR, TEXT_COLOR, 3, DrawIconLocation);
    Button btnMenuSensor   = Button(10, 132, 100, 100, BTN_COLOR, TEXT_COLOR, 3, DrawIconSensor);
    Button btnMenuSettings = Button(10, 248, 100, 100, BTN_COLOR, TEXT_COLOR, 3, DrawIconSettings);
    Button btnMenuAbout    = Button(10, 364, 100, 100, BTN_COLOR, TEXT_COLOR, 3, DrawIconAbout);

    // -- Settings --
    Button btnSpeedMinus  = Button(140, 64, 160, 84, HL_COLOR, TEXT_COLOR, 4, nullptr);
    Button btnSpeedPlus   = Button(320, 64, 160, 84, HL_COLOR, TEXT_COLOR, 4, nullptr);
    Button btnBleConnect  = Button(665, 71, 105, 70, BTN_COLOR, TEXT_COLOR, 3, nullptr);

    Button btnCalibWhite  = Button(160, 200, 104, 100, BTN_COLOR, TEXT_COLOR, 3, nullptr);
    Button btnCalibBlack  = Button(284, 200, 104, 100, BTN_COLOR, 0x0000, 3, nullptr); // Schwarz
    Button btnCalibBlue   = Button(408, 200, 104, 100, BTN_COLOR, 0x001f, 3, nullptr); // Blau
    Button btnCalibDZone  = Button(532, 200, 104, 100, BTN_COLOR, 0xf800, 3, nullptr); // Rot
    Button btnCalibCheckP = Button(656, 200, 104, 100, BTN_COLOR, TEXT_COLOR, 3, nullptr); // Gelb/Text

    Button btnLayerSetting = Button(500, 71, 145, 70, BTN_COLOR, TEXT_COLOR, 3, nullptr);

// --- Functions ---
    // -- Main Menu --
    void DrawMainMenuStatic();
    void HandleMainMenu(uint16_t tx, uint16_t ty);

    // -- Run Menu --
    static constexpr uint16_t MAP_AREA_WIDTH = 600;
    static constexpr uint8_t TILE_SIZE = 60; // Größe einer Kachel in Pixeln auf dem Display

    bool _updateMap = false;

    void ConstructRunMenu();
    void UpdateRunMenu();
    void DrawMap();

    // -- Sensor Info --
    void ConstructSensorInfoMenu();

    // @brief Draws Battery Status on Display
    void DrawBattery();

    //Helper Functions for Constructing Menus
    void ConstructSettingsMenu();
    void ConstructAboutMenu();
    
public:

    /**
     * @brief Construct a new User Interface object
     * @param updateInterval Update Interval for the User Interface in milliseconds
     */
    UserInterface(uint8_t updateInterval = 50);    //CONSTRUCTOR

    /**
     * @brief Updates the User Interface (should be called cyclically)
     */
    void Update();

    // EVENTS ---------------------------------------------------------------------------------

    void Initialize();

    /**
     * @brief Connects the other classes to the User Interface
     */
    void ConnectPointer(RobotState* state, ColorSensing* cs, Mapping* mapping, Vcameras* camera, Ejector* ejector);

    /**
     * @brief Adds an Information to the Message Log in startup or BLE screen
     * @param Info Title of the Message
     * @param Message Detailed Status (OK / FAIL / etc.)
     * @param success If true, Message is shown in green, else in red
     */
    void AddInfoMsg(String Info, String Message, bool success);

    // POP-Ups / Error Messages ---------------------------------------------------------------------------------

    /**
     * @brief Erzeugt ein Pop-up Overlay über dem aktuellen Screen
     * @param text displayed Text
     * @param type ErrorCodes (ERROR / info / warning) 
     * @param timeS shown duration (Default 5)
     */
    void ShowPopup(const char* text, ErrorCodes type, uint16_t timeS = 5);

    // SIGNALS ---------------------------------------------------------------------------------

    /** 
     * @brief Triggers the Buzzer Signal; is Blocking!
     * @param time_ms Duration of the signal in milliseconds
     * @param pause_ms Pause between signals in milliseconds
     * @param repetitions Number of repetitions of the signal
     */
    void BuzzerSignal(uint16_t time_ms, uint16_t pause_ms, uint8_t repetitions);

    /**
     * @brief Triggers the LED Signal; is Blocking!
     * @param color Color of the LED signal (0xRRGGBBWW)
     * @param time_ms Duration of the signal in milliseconds
     * @param pause_ms Pause between signals in milliseconds
     * @param repetitions Number of repetitions of the signal
     */
    void LEDSignal(uint16_t time_ms, uint16_t pause_ms, uint8_t repetitions);

    /**
     * @brief Triggers the LED and Buzzer Signal; is Blocking!
     * @param color Color of the LED signal (0xRRGGBBWW)
     * @param time_ms Duration of the signal in milliseconds
     * @param pause_ms Pause between signals in milliseconds
     * @param repetitions Number of repetitions of the signal
     */
    void LED_BUZZER_Signal(uint16_t time_ms, uint16_t pause_ms, uint8_t repetitions);

    /**
     * @brief Set the LED Illumination of the Robot (Light for Camera)
     * @param red Value of the red RGB part
     * @param green Value of the green RGB part
     * @param blue Value of the blue RGB part
     * @param white Value of the white RGB part
     */
    void SetIllumination(uint8_t red, uint8_t green, uint8_t blue, uint8_t white);

    /**
     * @brief Gets the Battery Charge Level in Percent
     * @return Charge Level (0-100)
     */
    uint8_t GetCharge();

    /**
     * @brief Cycles through the Drive Modes in the Settings Menu
     * @return The new Drive Mode after cycling
     */
    ErrorCodes CycleDriveMode();

    /**
     * @brief returns current drive speed
     * @return drivespeed 0...100
     */
    uint8_t GetDriveSpeed(){ return driveSpeed; }

    // --- Calibration Overlay ---
    void ShowCalibrationScreen(PoI_Type type);
    void UpdateCalibrationProgress(uint8_t step, uint8_t totalSteps);
    void FinishCalibration(bool success);

    // --- Reset To Checkpoint Overlay ---
    void ShowResetScreen();
    void UpdateResetProgress(char* message, uint8_t step, uint8_t totalSteps);
    void FinishReset(bool success);

    /**
     * @brief called to update Map on Display
     */
    void UpdateMap() { _updateMap = true; };

    void UpdateSensors(uint16_t RF, uint16_t RB, uint16_t LF, uint16_t LB, uint16_t F, uint16_t FW, uint16_t B, uint16_t BW, float aX, float aY = 0, float aZ = 0) {
        tof_RF = RF; tof_RB = RB;
        tof_LB = LB; tof_LF = LF;
        tof_B = B; tof_BW = BW;
        tof_F = F; tof_FW = FW;
        gyro_X= aX;
        gyro_Y= aY;
        gyro_Z= aZ;
    }
};