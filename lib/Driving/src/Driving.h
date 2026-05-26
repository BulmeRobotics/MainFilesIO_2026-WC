#pragma once
/**
 * @name:    Driving.h
 * @date:    30.03.2026
 * @authors: Vincent Rohkamm, Florian Wiesner
 * @details: High-level motion control — PID drive, wall alignment, turn control, ramp traversal, and bumper handling.
 */

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
public:
	//----Init----
	/**
	 * @brief  Initializes the Driving module and links all required subsystems.
	 * @param  p_colorSensing  Pointer to the ColorSensing module.
	 * @param  p_tof           Pointer to the TofSensors module.
	 * @param  p_gyro          Pointer to the GyroBase module.
	 * @param  mapSys_pointer  Pointer to the Mapping module.
	 * @param  cam_pointer     Pointer to the Vcameras module.
	 * @param  p_drivetrain    Pointer to the Drivetrain module.
	 */
	void Init(ColorSensing* p_colorSensing, TofSensors* p_tof, GyroBase* p_gyro, Mapping* mapSys_pointer, Vcameras* cam_pointer, Drivetrain* p_drivetrain);

	//----Bumpers----
	/**
	 * @brief  Drives the robot backwards for a defined distance using encoders.
	 * @param  distance    Distance in mm to drive backwards.
	 * @param  speedLeft   Speed of left motor.
	 * @param  speedRight  Speed of right motor.
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
	 *         BUMPER_WALL if repeated collision detected (map update needed).
	 *         BUMPER_DISABLED if bumpers are disabled.
	 *         UNKNOWN if no bumper event occurred.
	 */
	ErrorCodes bumperHandler(void);

	//----Ramps----
	/**
	 * @brief  Handles ramp detection and traversal. Call each cycle while in RAMP state.
	 * @return OK while traversal is ongoing.
	 *         RAMP_END when the ramp is fully cleared.
	 */
	ErrorCodes rampHandler(void);

	/**
	 * @brief  Analyses the incline sample array to decide if the ramp is stair-type.
	 * @return true  if the ramp is classified as a staircase.
	 *         false if it is a smooth ramp.
	 */
	bool checkStairRamp(void);

	//----Turn----
	/**
	 * @brief  Initializes a turn to a target heading.
	 * @param  angle  Target absolute angle in degrees.
	 * @return OK always.
	 *         ERROR if angle exceeds 360°.
	 */
	ErrorCodes startTurn(float angle);

	/**
	 * @brief  Controls the ongoing turn each cycle. Call until TURNED is returned.
	 * @param  angle  Target absolute angle in degrees.
	 * @return OK while turning is in progress.
	 *         TURNED when the target angle is reached or a timeout occurs.
	 */
	ErrorCodes controlTurn(float angle);

	/**
	 * @brief  Finalizes a turn: re-enables bumpers, aligns, updates map orientation.
	 * @return OK always.
	 */
	ErrorCodes endTurn(void);

	/**
	 * @brief  Performs a blocking in-place 180° turn.
	 * @return OK after completion.
	 */
	ErrorCodes turn180Degree(void);

	/**
	 * @brief  Aligns the robot parallel to the nearest side wall using the ToF sensors.
	 * @return OK if alignment succeeded.
	 *         NOT_ALIGNING if no usable wall was found on either side.
	 */
	ErrorCodes startAlign(void);

	//----Drive----
	/**
	 * @brief  Initializes a forward driving sequence and selects the reference sensor.
	 * @param  rampDown  true forces encoder reference (used when approaching a down-ramp).
	 * @return OK always.
	 */
	ErrorCodes startDrive(bool rampDown);

	/**
	 * @brief  Runs one PID drive cycle. Call each loop iteration while in DRIVE state.
	 * @param  driveSpeed  Base forward speed (0–100).
	 * @param  angle       Target heading angle in degrees.
	 * @return CHECK_DRIVE  while normal forward driving continues.
	 *         OK           while on a ramp (speed overridden internally).
	 *         TIMEOUT      if the maximum drive time is exceeded.
	 */
	ErrorCodes controlDrive(int8_t driveSpeed, float angle);

	/**
	 * @brief  Checks whether the target position has been reached.
	 * @return SCAN_DRIVE if the reference sensor crossed the target threshold.
	 *         CHECK_RAMP  if the target has not yet been reached.
	 */
	ErrorCodes checkDrive(void);

	/**
	 * @brief  Stops the robot and resets the PID timestamp.
	 * @return OK always.
	 */
	ErrorCodes endDrive(void);

	/**
	 * @brief  Recovers from a drive timeout: stops, aligns, and adjusts to the front wall.
	 * @return OK always.
	 */
	ErrorCodes timeoutDrive(void);

	/**
	 * @brief  Nudges the robot to a fixed distance from the front wall.
	 * @return OK if adjustment succeeded.
	 *         ERROR if no wall was detected within range.
	 */
	ErrorCodes startAdjustment(void);

	/**
	 * @brief  Drives backwards ~15 cm to escape a black tile.
	 * @return OK always.
	 */
	ErrorCodes reverseBlackTile(void);

	//----Public data (Phase 3: becomes private with getters/setters)----
	bool	_ON_RAMP          = false;
	bool	_RAMP_BEHIND      = false;
	bool	_RAMP_INFRONT     = false;
	bool	_RAMP_INSTRUCTION = false;
	bool	_TURNING          = true;
	bool	_SLOW_SPEED       = false;
	bool	_CAM_ALERT_TURN   = false;
	bool	_CAM_VICTIM       = false;

	TOF_Optimal_Value	sensor;
	uint16_t			nextTargetDistance;
	uint16_t			newValue;
	Orientations		robotTargetAngle    = Orientations::North;
	float				integralError;
	float				derivativeError;
	float				maxRampIncline;
	float				RAMP_HEIGHT         = 0.0;
	float				RAMP_LENGTH         = 0.0;
	int16_t				currentRobotHeight;
	uint32_t			lastSetTile;

private:
	//----Config constants----
	static constexpr uint16_t MIN_SETTILE_TIME       = 750;
	static constexpr uint16_t INCLINE_ARRAY_SIZE     = 250;
	static constexpr uint16_t REVERSE_BUMPER_TIMEOUT = 2000;

	//----Driving tuning----
	const uint8_t tof_sideWallThreshold  = 170;
	const uint8_t gap_robot_wall         = 75;
	const uint8_t stop_wallDistance      = 75;
	const uint8_t adjust_wallDistance    = 70;
	const uint8_t adjustmentSpeedFactor  = 1.8;
	const uint8_t stdSPEED_MOD           = 2;
	const uint8_t slowSPEED_MOD          = 4;

	//----Ramp tuning----
	const float   rampThresholdAngle     = 10;
	const float   rampConfidence         = 0.5;
	const uint8_t rampSpeedUp            = 75;
	const uint8_t rampSpeedDown          = 25;
	const uint8_t rampSubDistance        = 30;
	const float   rampStairsThreshold    = 60;
	const float   rampUp_K               = 0.9;
	const float   rampUp_d               = 30;
	const float   rampDown_K             = 1.25;
	const float   rampDown_d             = 20;
	const float   stairUp_K              = 0.7;
	const float   stairUp_d              = 0;
	const float   stairDown_K            = 1.15;
	const float   stairDown_d            = 0;
	const float   stairUp_angle_offset   = 3.5;
	const float   stairDown_angle_offset = -3.5;

	//----PID tuning----
	const float pid_CriticalGain      = 7.5;
	const float pid_LoopDuration      = 0.025;
	const float pid_OscillationPeriod = 0.75;
	const float pid_LeftRightFactor   = 0.35;

	//----Bumper config----
	const uint8_t BUMPER_TRYS      = 5;
	const uint8_t BUMPER_LEFT_PIN  = 47;
	const uint8_t BUMPER_RIGHT_PIN = 45;

	//----Dependency pointers----
	ColorSensing*	p_colorSensing	= nullptr;
	TofSensors*		p_tof			= nullptr;
	GyroBase*		p_gyro			= nullptr;
	Drivetrain*		p_drivetrain	= nullptr;
	Mapping*		p_mapSys		= nullptr;
	Vcameras*		p_cams			= nullptr;

	//----Bumper state----
	bool    _ENABLE_BUMPERS  = true;
	uint8_t _registeredBumps = 0;

	//----Alignment state----
	int8_t	coeff_side;
	uint8_t	distanceFront;
	uint8_t	distanceBack;
	int16_t	distanceError;
	int16_t	turnSpeed_align;

	//----Drive state----
	TOF_Optimal_Value	lastSensor;
	uint32_t			startTime;
	bool				turnTimeout;
	uint16_t			lastTargetDistance;
	uint8_t				partsDriven;
	uint16_t			encoderDistance;
	uint32_t			encoderStartTime;
	uint32_t			driveStartTime   = 0;
	uint32_t			maxDriveTime     = 5000;
	uint16_t			maxEncoderTime   = 3000;
	uint16_t			maxTurnTime      = 3000;
	bool				_TURN_180_DEGREE = false;
	bool				_DRIVE_TIMEOUT   = false;

	//----Ramp state----
	int16_t		inclineCycleCounter    = 0;
	uint16_t	nonInclineCycleCounter = 0;
	uint32_t	rampStartTime          = 0;
	uint16_t	rampCheckDuration      = 1000;
	uint8_t		rampSpeed;
	float		rampEncoderDistance;
	float		RAMP_HYPOTENUSE        = 0.0;
	float		RAMP_ANGLE             = 0.0;
	float		avgIncline             = 0.0;
	float		arr_incline[INCLINE_ARRAY_SIZE] = { 0.0 };
	uint16_t	arr_incline_index      = 0;
	bool		_RAMP_UP               = false;
	bool		_RAMP_DOWN             = false;
	bool		_STAIR                 = false;

	//----PID state----
	float	pid_lastError     = 0.0;
	float	correctionSpeed   = 0.0;
	long	lastPID_timestamp = 0;

	//----Private methods----
	ErrorCodes			finishRamp(uint8_t distance);
	ErrorCodes			checkRamp(void);
	PID_Coefficients	calculatePIDCoefficients(float loopDuration);
	TOF_Optimal_Value	getOptimalSensor(bool rampDown);
};
