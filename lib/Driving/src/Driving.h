#pragma once
/**
 * @name:    Driving.h
 * @date:    30.03.2026
 * @authors: Florian Wiesner
 * @details: High-level motion control — PID drive, wall alignment, turn control, ramp traversal, and bumper handling.
 */

#ifdef _MSC_VER
    #pragma region Libraries
#endif

// General
#include <Arduino.h>
#include <math.h>

// Custom
#include <CustomDatatypes.h>
#include <Motor.h>
#include <TofSensors.h>
#include <Gyro.h>
#include <ColorSensing.h>
#include <Mapping.h>

#ifdef _MSC_VER
    #pragma endregion Libraries
#endif

// Forward declaration
class Vcameras;


class Driving {
    public:
        #ifdef _MSC_VER
            #pragma region Init
        #endif
        /**
         * @brief  Initializes the Driving module and links all required subsystems.
         * @param  p_colorSensing  Pointer to the ColorSensing module.
         * @param  p_tof           Pointer to the TofSensors module.
         * @param  p_gyro          Pointer to the GyroBase module.
         * @param  p_mapSys        Pointer to the Mapping module.
         * @param  p_cams          Pointer to the Vcameras module.
         * @param  p_drivetrain    Pointer to the Drivetrain module.
         */
        void Init(ColorSensing* p_colorSensing, TofSensors* p_tof, GyroBase* p_gyro, Mapping* p_mapSys, Vcameras* p_cams, Drivetrain* p_drivetrain);

        #ifdef _MSC_VER
            #pragma endregion
            #pragma region Turn
        #endif
        /**
         * @brief  Initializes a turn to a target heading.
         * @param  angle  Target absolute angle in degrees.
         * @return OK if angle is valid (≤ 360°).
         *         ERROR if angle exceeds 360°.
         */
        ErrorCodes StartTurn(float angle);

        /**
         * @brief  Controls the ongoing turn each cycle. Call until TURNED is returned.
         * @param  angle  Target absolute angle in degrees.
         * @return OK while turning is in progress.
         *         TURNED when the target angle is reached or a timeout occurs.
         */
        ErrorCodes ControlTurn(float angle);

        /**
         * @brief  Finalizes a turn: re-enables bumpers, aligns, updates map orientation.
         * @return OK always.
         */
        ErrorCodes EndTurn(void);

        /**
         * @brief  Performs a blocking in-place 180° turn.
         * @return OK after completion.
         */
        ErrorCodes Turn180Degree(void);

        /**
         * @brief  Aligns the robot parallel to the nearest side wall using the ToF sensors.
         * @return OK if alignment succeeded.
         *         NOT_ALIGNING if no usable wall was found on either side, or if a side sensor goes out of range during alignment.
         */
        ErrorCodes StartAlign(void);

        #ifdef _MSC_VER
            #pragma endregion
            #pragma region Drive
        #endif
        /**
         * @brief  Initializes a forward driving sequence and selects the reference sensor.
         * @param  rampDown  true forces encoder reference (used when approaching a down-ramp).
         * @return OK always.
         */
        ErrorCodes StartDrive(bool rampDown);

        /**
         * @brief  Runs one PID drive cycle. Call each loop iteration while in DRIVE state.
         * @param  driveSpeed  Base forward speed (0–100).
         * @param  angle       Target heading angle in degrees.
         * @return CHECK_DRIVE  while normal forward driving continues.
         *         OK           while on a ramp (speed overridden internally).
         *         TIMEOUT      if the maximum drive time is exceeded.
         */
        ErrorCodes ControlDrive(int8_t driveSpeed, float angle);

        /**
         * @brief  Checks whether the target position has been reached.
         * @return SCAN_DRIVE if the reference sensor crossed the target threshold.
         *         CHECK_RAMP  if the target has not yet been reached.
         */
        ErrorCodes CheckDrive(void);

        /**
         * @brief  Stops the robot and resets the PID timestamp.
         * @return OK always.
         */
        ErrorCodes EndDrive(void);

        /**
         * @brief  Recovers from a drive timeout: stops, aligns, and adjusts to the front wall.
         * @return OK always.
         */
        ErrorCodes TimeoutDrive(void);

        /**
         * @brief  Nudges the robot to a fixed distance from the front wall.
         * @return OK if adjustment succeeded.
         *         ERROR if no wall was detected within range.
         */
        ErrorCodes StartAdjustment(void);

        /**
         * @brief  Drives backwards ~15 cm to escape a black tile.
         * @return OK always.
         */
        ErrorCodes ReverseBlackTile(void);

        #ifdef _MSC_VER
            #pragma endregion
            #pragma region Ramps
        #endif
        /**
         * @brief  Handles ramp detection and traversal. Call each cycle while in RAMP state.
         * @return OK while traversal is ongoing.
         *         RAMP_END when the ramp is fully cleared.
         */
        ErrorCodes RampHandler(void);

        /**
         * @brief  Analyses the incline sample array to decide if the ramp is stair-type.
         * @return true  if the ramp is classified as a staircase.
         *         false if it is a smooth ramp.
         */
        bool CheckStairRamp(void);

        #ifdef _MSC_VER
            #pragma endregion
            #pragma region Bumpers
        #endif
        /**
         * @brief  Drives the robot backwards for a defined distance using encoders.
         * @param  distance    Distance in mm to drive backwards.
         * @param  speedLeft   Speed of left motor.
         * @param  speedRight  Speed of right motor.
         * @return OK if movement finished successfully.
         *         TIMEOUT if the distance was not reached in time.
         */
        ErrorCodes ReverseBumper(uint16_t distance, int8_t speedLeft, int8_t speedRight);

        /**
         * @brief  Enables bumper handling.
         * @return OK always.
         */
        ErrorCodes EnableBumpers(void);

        /**
         * @brief  Disables bumper handling.
         * @return OK always.
         */
        ErrorCodes DisableBumpers(void);

        /**
         * @brief  Handles bumper events and executes avoidance maneuvers.
         * @return OK if bumper was triggered and handled.
         *         BUMPER_WALL if repeated collision detected (map update needed).
         *         BUMPER_DISABLED if bumpers are disabled.
         *         UNKNOWN if no bumper event occurred.
         */
        ErrorCodes BumperHandler(void);

        #ifdef _MSC_VER
            #pragma endregion
            #pragma region Getters
        #endif
        bool         IsOnRamp(void)            const { return _ON_RAMP; }
        float        GetRampHeight(void)       const { return rampHeight; }
        float        GetRampLength(void)       const { return rampLength; }
        int16_t      GetCurrentRobotHeight(void) const { return currentRobotHeight; }
        float        GetMaxRampIncline(void)   const { return maxRampIncline; }
        Orientations GetRobotTargetAngle(void) const { return robotTargetAngle; }

        #ifdef _MSC_VER
        #pragma endregion
        #pragma region Setters
        #endif
        void ClearOnRamp(void)                          { _ON_RAMP = false; }
        void SetCurrentRobotHeight(int16_t h)           { currentRobotHeight = h; }
        void SetMaxRampIncline(float v)                 { maxRampIncline = v; }
        void SetRobotTargetAngle(Orientations a)        { robotTargetAngle = a; }
        void SetSlowSpeed(bool s)                       { _SLOW_SPEED = s; }
        void SetLastSetTile(uint32_t t)                 { ts_lastSetTile = t; }

        #ifdef _MSC_VER
            #pragma endregion
            #pragma region Victim
        #endif
        /**
         * @brief  Called by Vcameras on victim detection. Resets PID error accumulators and
         *         sets the appropriate camera-alert flag based on current turn state.
         */
        void OnVictimDetected(void);

    private:
        #ifdef _MSC_VER
            #pragma endregion
            #pragma region Config
        #endif
        //----Behavior constants----
        static constexpr uint16_t MIN_SETTILE_TIME       = 750;
        static constexpr uint16_t INCLINE_ARRAY_SIZE     = 250;
        static constexpr uint16_t REVERSE_BUMPER_TIMEOUT = 2000;

        //----Driving tuning----
        static constexpr uint8_t TOF_SIDE_WALL_THRESHOLD = 170;
        static constexpr uint8_t GAP_ROBOT_WALL          = 75;
        static constexpr uint8_t STOP_WALL_DISTANCE      = 75;
        static constexpr uint8_t ADJUST_WALL_DISTANCE    = 70;
        static constexpr uint8_t ADJUSTMENT_SPEED_FACTOR = 1;
        static constexpr uint8_t STD_SPEED_MOD           = 2;
        static constexpr uint8_t SLOW_SPEED_MOD          = 4;

        //----Ramp tuning----
        static constexpr float   RAMP_THRESHOLD_ANGLE    = 10.0f;
        static constexpr float   RAMP_CONFIDENCE         = 0.5f;
        static constexpr uint8_t RAMP_SPEED_UP           = 75;
        static constexpr uint8_t RAMP_SPEED_DOWN         = 25;
        static constexpr uint8_t RAMP_SUB_DISTANCE       = 30;
        static constexpr float   RAMP_STAIRS_THRESHOLD   = 60.0f;
        static constexpr float   RAMP_UP_K               = 0.9f;
        static constexpr float   RAMP_UP_D               = 30.0f;
        static constexpr float   RAMP_DOWN_K             = 1.25f;
        static constexpr float   RAMP_DOWN_D             = 20.0f;
        static constexpr float   STAIR_UP_K              = 0.7f;
        static constexpr float   STAIR_UP_D              = 0.0f;
        static constexpr float   STAIR_DOWN_K            = 1.15f;
        static constexpr float   STAIR_DOWN_D            = 0.0f;
        static constexpr float   STAIR_UP_ANGLE_OFFSET   = 3.5f;
        static constexpr float   STAIR_DOWN_ANGLE_OFFSET = -3.5f;

        //----PID tuning----
        static constexpr float PID_CRITICAL_GAIN      = 7.5f;
        static constexpr float PID_LOOP_DURATION      = 0.025f;
        static constexpr float PID_OSCILLATION_PERIOD = 0.75f;
        static constexpr float PID_LEFT_RIGHT_FACTOR  = 0.35f;

        //----Bumper config----
        static constexpr uint8_t BUMPER_TRYS      = 5;
        static constexpr uint8_t BUMPER_LEFT_PIN  = 47;
        static constexpr uint8_t BUMPER_RIGHT_PIN = 45;

        //----State defaults----
        static constexpr uint32_t DEFAULT_MAX_DRIVE_TIME      = 5000;
        static constexpr uint16_t DEFAULT_MAX_ENCODER_TIME    = 3000;
        static constexpr uint16_t DEFAULT_MAX_TURN_TIME       = 3000;
        static constexpr uint16_t DEFAULT_RAMP_CHECK_DURATION = 1000;
        #ifdef _MSC_VER
            #pragma endregion
            #pragma region Members
        #endif
        //----Dependency pointers----
        ColorSensing*	p_colorSensing	= nullptr;
        TofSensors*		p_tof			= nullptr;
        GyroBase*		p_gyro			= nullptr;
        Drivetrain*		p_drivetrain	= nullptr;
        Mapping*		p_mapSys		= nullptr;
        Vcameras*		p_cams			= nullptr;

        //----Bumper state----
        bool    _ENABLE_BUMPERS = true;
        uint8_t registeredBumps = 0;

        //----Alignment state----
        int8_t  coeffSide;
        uint8_t distanceFront;
        uint8_t distanceBack;
        int16_t distanceError;
        int16_t turnSpeedAlign;

        //----Drive state----
        TOF_Optimal_Value sensor;
        TOF_Optimal_Value lastSensor;
        uint32_t          ts_startTime;
        bool              turnTimeout;
        uint16_t          lastTargetDistance;
        uint8_t           partsDriven;
        uint16_t          encoderDistance;
        uint16_t          nextTargetDistance;
        uint16_t          newValue;
        uint32_t          ts_encoderStartTime;
        uint32_t          ts_driveStartTime  = 0;
        uint32_t          maxDriveTime       = DEFAULT_MAX_DRIVE_TIME;
        uint16_t          maxEncoderTime     = DEFAULT_MAX_ENCODER_TIME;
        uint16_t          maxTurnTime        = DEFAULT_MAX_TURN_TIME;
        uint32_t          ts_lastSetTile;
        Orientations      robotTargetAngle   = Orientations::North;
        bool              _TURNING           = true;
        bool              _SLOW_SPEED        = false;
        bool              _CAM_ALERT_TURN    = false;
        bool              _CAM_VICTIM        = false;
        bool              _TURN_180_DEGREE   = false;
        bool              _DRIVE_TIMEOUT     = false;

        //----Ramp state----
        bool     _ON_RAMP               = false;
        bool     _RAMP_BEHIND           = false;
        bool     _RAMP_INFRONT          = false;
        bool     _RAMP_INSTRUCTION      = false;
        bool     _RAMP_UP               = false;
        bool     _RAMP_DOWN             = false;
        bool     _STAIR                 = false;
        int16_t  inclineCycleCounter    = 0;
        uint16_t nonInclineCycleCounter = 0;
        uint32_t ts_rampStartTime       = 0;
        uint16_t rampCheckDuration      = DEFAULT_RAMP_CHECK_DURATION;
        uint8_t  rampSpeed;
        float    rampEncoderDistance;
        float    rampHypotenuse         = 0.0f;
        float    rampAngle              = 0.0f;
        float    rampHeight             = 0.0f;
        float    rampLength             = 0.0f;
        float    maxRampIncline;
        int16_t  currentRobotHeight;
        float    avgIncline             = 0.0f;
        float    arrIncline[INCLINE_ARRAY_SIZE] = { 0.0f };
        uint16_t arrInclineIndex        = 0;

        //----PID state----
        float integralError   = 0.0f;
        float derivativeError = 0.0f;
        float pidLastError    = 0.0f;
        float correctionSpeed = 0.0f;
        long  ts_lastPID      = 0;
        #ifdef _MSC_VER
            #pragma endregion
            #pragma region Helpers
        #endif
        //----Turn helpers----
        int8_t  SelectAlignSide(void);
        int16_t CalculateTurnSpeed(void);

        //----Drive helpers----
        TOF_Optimal_Value   GetOptimalSensor(bool rampDown);
        void                ApplyRampFlagOverrides(TOF_Optimal_Value& result);
        PID_Coefficients    CalculatePIDCoefficients(float loopDuration);
        uint16_t            CalculateNextTargetDistance(void);

        //----Ramp helpers----
        ErrorCodes  FinishRamp(uint8_t distance);
        ErrorCodes  CheckRamp(void);
        void        UpdateRampProximityFlags(void);
        void        UpdateInclineCounters(float incline);
        void        EvaluateRampDecision(void);
        void        RecordInclineSample(float incline);
        void        ClassifyAndFinishRamp(void);
        void        CalculateRampGeometry(bool rampUp, bool rampDown, bool isStair);

        //----Bumper helpers----
        ErrorCodes  HandleWallCollision(void);
        void        ExecuteBumperManeuver(void);
        #ifdef _MSC_VER
            #pragma endregion
        #endif
};
