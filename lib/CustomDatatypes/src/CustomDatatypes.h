#pragma once
#include <cstdint>

#ifdef _MSC_VER
#pragma region Robot States
#endif

enum class RobotState : uint8_t {
    BOOT, INFO_SENSOR, INFO_VISUAL, SETTINGS, ABOUT, CALIBRATION, RUN, BT, CHECKPOINT
};

enum class RunState : uint8_t {
    INITIAL, SETTILE, GET_INSTRUCTIONS, CHECKPOINT_RESET, END, TURN, ALIGN, CHECK_DRIVE, DRIVE, RAMP, SCAN,
};

enum class ErrorCodes : uint8_t {
    OK, ERROR, UNKNOWN,
    TIMEOUT,
    NEW_DATA,
    NO_NEW_DATA,
    OUT_OF_RANGE,
    BUMPER_WALL,
    BUMPER_DISABLED,
    RAMP_END,
    TURNED,
    NOT_ALIGNING,
    CHECK_DRIVE,
    SCAN_DRIVE,
    CHECK_RAMP,
    invalid, wall, straight, north, east, south, west, Overflow, up, down, same, single, multi, //Mapping specific
    disabled, left, right,
    info,warning,    //Popup System
    empty,               //ejector
    already_found, no_connection,
    start, stop, ready, //Checkpoint
    ramp, layer
};

#ifdef _MSC_VER
#pragma region ToF
#endif

enum class TofType : uint8_t {
    LEFT_FRONT,
    LEFT_BACK,
    RIGHT_FRONT,
    RIGHT_BACK,
    FRONT,
    FRONT_WALL,
    BACK,
    BACK_WALL
};

enum class TofStatus : uint8_t{
    VALID,
    OUT_OF_RANGE,
    TIMEOUT,
    ERROR
};

enum class ReferenceObj : uint8_t {
    FRONT,
    BACK,
    ENCODER
};

struct TOF_Optimal_Value {
	uint16_t front;
	uint16_t back;
	ReferenceObj type;
};

struct PID_Coefficients {
	float P;
	float I;
	float D;
	float dt_nominal; // nominal loop dt in seconds; clamp threshold = 2×dt_nominal
};
#ifdef _MSC_VER
#pragma region Gyro
#endif

struct GyroData {
	float angle_abs;
	float angle_car;
	float angle_error;
	bool direction_left;
	bool direction_right;
};

enum class GyroAxles : uint8_t {
	Axis_X,
	Axis_Y,
	Axis_Z
};
#ifdef _MSC_VER
#pragma region Mapping
#endif
enum class Orientations : uint8_t {
    North, East, South, West
};

enum class PoI_Type : uint8_t {
    undef = 0,
    white, blue, checkpoint, black, dangerZone,
    harmed, safe, unharmed,
    red, yellow, green,
    ble
};

enum class TileType : uint8_t {
    inactive = 0,
    unexplored,
    visited, obstacle,
    checkpoint, dangerZone, blue, black
};

#ifdef _MSC_VER
#pragma region ColorSensor
#endif

struct rawColor {
	uint16_t
		F1		= 0,
		F2		= 0,
		F3		= 0,
		F4		= 0,
		F5		= 0,
		F6		= 0,
		F7		= 0,
		F8		= 0,
		Clear	= 0,
        NIR     = 0;
};
