#pragma once

//Created:	21.04.2024
//Copied:   30.03.2026
//Authors:	Vincent Rohkamm, Florian Wiesner

#include <Arduino.h>
#include <math.h>

#include <CustomDatatypes.h>
#include <Motor.h>
#include <TofSensors.h>
#include <Gyro.h>
#include <ColorSensing.h>
#include <Mapping.h>

class Vcameras;


class Driving {
protected:

    //#define DEBUG_RAMP
    //#define DEBUG_RAMP_ARRAY
    //#define DEBUG_X64
    //#define DEBUG_DRIVING


private:
//CONFIGURATION -------------------------------------------------------------------------------------------------------------------------
    #define MIN_SETTILE_TIME 750
    #define INCLINE_ARRAY_SIZE 250
    #define reverseBumperTimeout 2000

    //Driving consants
    const uint8_t tof_sideWallThreshold = 170;
    const uint8_t gap_robot_wall = 75; //Distance between the robot and the wall
    const uint8_t stop_wallDistance = 75;
    const uint8_t adjust_wallDistance = 70;
    const uint8_t adjustmentSpeedFactor = 1.8;	//Speed factor for the adjustment speed

    //Ramp settings
    const float rampThresholdAngle = 10;	//Angle threshold for the ramp
    const float rampConfidence = 0.5;    //Confidence for the ramp detection
    const uint8_t rampSpeedUp = 75;
    const uint8_t rampSpeedDown = 25;
    const uint8_t rampSubDistance = 30;
    const float rampStairsThreshold = 60;
    const float rampUp_K = 0.9;
    const float rampUp_d = 30; 
    const float rampDown_K = 1.25;
    const float rampDown_d = 20;
    const float stairUp_K = 0.7;
    const float stairUp_d = 0;
    const float stairDown_K = 1.15;
    const float stairDown_d = 0;	
    const float stairUp_angle_offset = 3.5;
    const float stairDown_angle_offset = -3.5;

    //PID coefficents
    //const float pidCoeff_P
    const float pid_CriticalGain = 7.5;			//Maximum proportional amount for P-Controller
    const float pid_LoopDuration = 0.025;	//Duration of the control loop
    const float pid_OscillationPeriod = 0.75;	//Period time of the oscillation of the robot
    const float pid_LeftRightFactor = 0.35;

    const uint8_t BUMPER_TRYS = 5;
    const uint8_t BUMPER_LEFT_PIN = 47;
    const uint8_t BUMPER_RIGHT_PIN = 45;

//Obj Pointer -------------------------------------------------------------------------------------------------------------------------
    ColorSensing*  p_colorSensing = nullptr;
    TofSensors*    p_tof = nullptr;
    GyroBase*      p_gyro = nullptr;
    Drivetrain*    p_drivetrain = nullptr;
    Mapping*       p_mapSys = nullptr;
    Vcameras*      p_cams = nullptr;

//Objects -------------------------------------------------------------------------------------------------------------------------
    

//Private Variables -------------------------------------------------------------------------------------------------------------------------
    //FLAGS
    //Bumper
    bool    _ENABLE_BUMPERS = true;
    uint8_t _registeredBumps = 0;

    //Ramps
    ErrorCodes finishRamp(uint8_t distance);
    ErrorCodes checkRamp(void);
    
    TOF_Optimal_Value 	lastSensor;
    uint32_t 			startTime;
    bool 				turnTimeout;

    uint16_t 			lastTargetDistance;
    uint8_t 			partsDriven;
    uint16_t 			encoderDistance;
    String 				side;
    int8_t 				coeff_side;
    uint8_t 			distanceFront;
    uint8_t 			distanceBack;
    uint32_t            encoderStartTime;
    uint32_t            driveStartTime = 0;
    uint32_t            maxDriveTime = 5000;
    uint16_t            maxEncoderTime = 3000;

    int16_t 			distanceError;
    int16_t 			turnSpeed_align;
    uint16_t			maxTurnTime = 3000;
    int16_t 			inclineCycleCounter = 0;
    uint16_t 			nonInclineCycleCounter = 0;
    uint32_t 			rampStartTime = 0;
    uint16_t 			rampCheckDuration = 1000;
    uint8_t 			rampSpeed;
    float				rampEncoderDistance;
    float 				RAMP_HYPOTENUSE = 0.0;
    
    float 				RAMP_ANGLE = 0.0;
    float               avgIncline = 0.0;
    float 				arr_incline[INCLINE_ARRAY_SIZE] = { 0.0 };
    uint16_t 			arr_incline_index = 0;
    
    float 				pid_lastError;
    float 				correctionSpeed;
    long 				lastPID_timestamp = 0;
    
    bool				_RAMP_UP = false;
    bool				_RAMP_DOWN = false;
    bool                _STAIR = false;
    bool 				_TURN_180_DEGREE = false;
    bool                _DRIVE_TIMEOUT = false;
    
    //speedMOD
    const uint8_t stdSPEED_MOD = 2;
    const uint8_t slowSPEED_MOD = 4;
    

    //General
    PID_Coefficients calculatePIDCoefficients(float);
    TOF_Optimal_Value getOptimalSensor(bool);

public:
    //PUBLIC FLAGS
    bool	_ON_RAMP = false;
    bool 	_RAMP_BEHIND = false;
    bool	_RAMP_INFRONT = false;
    bool 	_RAMP_INSTRUCTION = false;
    bool    _TURNING = true;
    bool    _SLOW_SPEED = false;
    bool 	_CAM_ALERT_TURN = false;
    bool    _CAM_VICTIM = false;

    //Public Variables
    TOF_Optimal_Value 	sensor;
    uint16_t 			nextTargetDistance;
    uint16_t 			newValue;   //Actual distance to Target
    Orientations        robotTargetAngle = Orientations::North;

    //PID
    float 		integralError;
    float 		derivativeError;
    
    //RAMP
    float 		maxRampIncline;
    float 		RAMP_HEIGHT = 0.0;
    float 		RAMP_LENGTH = 0.0;
    int16_t		currentRobotHeight;
    uint32_t    lastSetTile;

    /**
     * @brief  Initializes the Driving module and links all required subsystems.
     * @param  Pointers to all required modules (sensors, mapping, drivetrain).
     */
    void init(ColorSensing* p_colorSensing, TofSensors* p_tof, GyroBase* p_gyro, Mapping* mapSys_pointer, Vcameras* cam_pointer, Drivetrain* p_drivetrain);

    //Bumpers:  
    /**
     * @brief  Drives the robot backwards for a defined distance using encoders.
     * @param  distance Distance in mm to drive backwards.
     * @param  speedLeft Speed of left motor.
     * @param  speedRight Speed of right motor.
     * @return OK if movement finished successfully.
     *         TIMEOUT if the distance was not reached in time.
     */    
    ErrorCodes reverseBumper(uint16_t distance, int8_t speedLeft, int8_t speedRight);

    /**
     * @brief  Enables bumper handling.
     * @return OK always.
     */
    ErrorCodes enableBumpers(void);

    /**
     * @brief  Disables bumper handling.
     * @return OK always.
     */
    ErrorCodes disableBumpers(void);

    /**
     * @brief  Handles bumper events and executes avoidance maneuvers.
     * @return OK if bumper was triggered and handled.
     *         BUMPER_WALL if repeated collision detected (map update).
     *         BUMPER_DISABLED if bumpers are disabled.
     *         UNKNOWN if no bumper event occurred.
     */  
    ErrorCodes bumperHandler(void);

    //Ramps:
    /**
     * @brief  Handles ramp detection and processing.
     * @return OK if still driving or no ramp event.
     *         RAMP_END if ramp traversal is completed.
     */
    ErrorCodes rampHandler(void);

    /**
     * @brief  Checks if current ramp is a stair-type ramp.
     * @return true if stair ramp detected.
     *         false otherwise.
     */   
    bool checkStairRamp(void);

    //Turn
    /**
     * @brief  Initializes a turn to a target angle.
     * @param  angle Target angle in degrees.
     * @return OK if initialization successful.
     *         INVALID if angle is out of range.
     */
    ErrorCodes startTurn(float angle);

    /**
     * @brief  Controls the ongoing turn motion.
     * @param  angle Target angle in degrees.
     * @return OK while turning is in progress.
     *         TURNED if target angle reached or timeout occurred.
     */
    ErrorCodes controlTurn(float angle);

    /**
     * @brief  Finalizes a turn and updates system state.
     * @return OK always.
     */
    ErrorCodes endTurn();

    /**
     * @brief  Performs a blocking 180° turn.
     * @return OK after completion.
     */
    ErrorCodes turn180Degree(void);

    /**
     * @brief  Aligns the robot parallel to a wall using side ToF sensors.
     * @return OK if alignment successful.
     *         NOT_ALIGNING if no valid wall detected or alignment failed.
     */
    ErrorCodes startAlign(void);

    //Drive
    /**
     * @brief  Initializes a forward driving sequence.
     * @return OK always.
     */    
    ErrorCodes startDrive(bool rampDown);

    /**
     * @brief  Controls forward movement using PID and sensor fusion.
     * @param  driveSpeed Base forward speed.
     * @param  angle Target heading angle.
     * @return OK if driving on ramp.
     *         CHECK_DRIVE if normal driving continues.
     *         TIMEOUT if max drive time exceeded.
     */
    ErrorCodes controlDrive(int8_t drivespeed, float angle);

    /**
     * @brief  Checks if target distance or condition is reached.
     * @return SCAN_DRIVE if target reached.
     *         CHECK_RAMP if still driving.
     */
    ErrorCodes checkDrive(void);

    /**
     * @brief  Stops the robot after driving.
     * @return OK always.
     */
    ErrorCodes endDrive(void);

    /**
     * @brief  Handles drive timeout recovery (align + adjust).
     * @return OK always.
     */
    ErrorCodes timeoutDrive(void);

    /**
     * @brief  Adjusts robot position relative to front wall.
     * @return OK if adjustment successful.
     *         INVALID if no wall detected in front.
     */
    ErrorCodes startAdjustment(void);

    /**
     * @brief  Drives backwards to escape a black tile.
     * @return OK always.
     */
    ErrorCodes reverseBlackTile(void);
};

