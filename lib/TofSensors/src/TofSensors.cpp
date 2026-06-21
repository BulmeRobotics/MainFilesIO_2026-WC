/**
* @name:    TofSensors.cpp
* @date:    18.11.2025
* @authors: Florian Wiesner
* @details: .cpp file for Time-of-Flight-Sensors
*/

//----Libraries----
#include "TofSensors.h"

//#define DEBUG_RAMP    // Uncomment to print IsRampThere() upper/lower/diff (used by RAMP_TEST_MODE)

#ifdef _MSC_VER
    #pragma region Parent Class //-----------------------------------------------------------------------------------------------
#endif
TofParent::TofParent(uint8_t i2cAddr, uint8_t xshut)
	: xshutPin(xshut),
	  i2cAddress(i2cAddr),
	  lastMeasurement(0),
	  lastStatus(static_cast<TofStatus>(0)),
	  measurementCount(0),
	  ts_lastMeasurement(0),
	  ts_waitStart(0),
	  ts_lastReadCall(0),
	  timeoutOccured(false),
	  waitingForMeasurement(false),
	  newData(false) {}

void TofParent::Off(void) {
	pinMode(xshutPin, OUTPUT);
	digitalWrite(xshutPin, LOW);
}

void TofParent::On(void) {
	pinMode(xshutPin, INPUT_PULLUP);
}

bool TofParent::Init(void) { return false; }
ErrorCodes TofParent::Read(void) { return ErrorCodes::ERROR; }
uint16_t TofParent::GetRange(void) { return 0; }
TofStatus TofParent::GetStatus(void) { return lastStatus; }
bool TofParent::IsDataNew(void) { return newData; }
bool TofParent::TimeoutOccured(void) { return timeoutOccured; }
ErrorCodes TofParent::Stop(void) { return ErrorCodes::ERROR; }
ErrorCodes TofParent::Continue(void) { return ErrorCodes::ERROR; }

#ifdef _MSC_VER
    #pragma endregion
    #pragma region VL6180X //----------------------------------------------------------------------------------------------------
#endif
TofVL6180X::TofVL6180X(uint8_t i2cAddress, uint8_t xshutPin)
	: TofParent(i2cAddress, xshutPin) {}

bool TofVL6180X::Init(void) {
	On();
	delay(100);
	sensor.setTimeout(500);
	sensor.init();
	sensor.configureDefault();
	sensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 15);
	sensor.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 20);
	sensor.setTimeout(500);
	delay(100);
	sensor.stopContinuous();
	sensor.setTimeout(500);
	delay(100);
	sensor.setAddress(i2cAddress);
	sensor.startRangeContinuous(RANGING_BUDGET_SHORT);
	return true;
}

ErrorCodes TofVL6180X::Read(void) {
	uint8_t update = sensor.readRangeContinuous();
	if (update != 0) {
		lastMeasurement = update;
		ts_lastMeasurement = millis();
		newData = true;
		measurementCount++;

		if (lastMeasurement < 255) {
			lastStatus = TofStatus::VALID;
			return ErrorCodes::NEW_DATA;
		}
		else {
			lastStatus = TofStatus::OUT_OF_RANGE;
			return ErrorCodes::OUT_OF_RANGE;
		}
	}
	if (sensor.timeoutOccurred()) return ErrorCodes::TIMEOUT;
	else return ErrorCodes::OK;
}

uint16_t TofVL6180X::GetRange(void) {
	return lastMeasurement;
}

ErrorCodes TofVL6180X::Stop(void) {
	sensor.stopContinuous();
	delay(5);
	newData = false;
	waitingForMeasurement = false;

	if (sensor.getTimeout()) {
		timeoutOccured = true;
		lastStatus = TofStatus::TIMEOUT;
		return ErrorCodes::TIMEOUT;
	}

	timeoutOccured = false;
	return ErrorCodes::OK;
}

ErrorCodes TofVL6180X::Continue(void) {
	sensor.startRangeContinuous(RANGING_BUDGET_SHORT);
	timeoutOccured = false;
	waitingForMeasurement = false;
	ts_lastReadCall = millis();
	return ErrorCodes::OK;
}

#ifdef _MSC_VER
    #pragma endregion
    #pragma region VL53L4CD //---------------------------------------------------------------------------------------------------
#endif
TofVL53L4CD::TofVL53L4CD(uint8_t i2cAddress, uint8_t xshutPin)
	: TofParent(i2cAddress, xshutPin), sensor(&Wire, xshutPin) {}

bool TofVL53L4CD::Init(void) {
	On();
	delay(10);
	sensor.begin();
	sensor.VL53L4CD_Off();
	sensor.InitSensor(i2cAddress);
	sensor.VL53L4CD_SetRangeTiming(RANGING_BUDGET_MID, 0);
	sensor.VL53L4CD_StartRanging();
	timeoutOccured = false;
	waitingForMeasurement = false;
	ts_lastReadCall = millis();
	return true;
}

// ErrorCodes TofVL53L4CD::Read(void) {
// 	uint8_t NewDataReady = 0;
// 	VL53L4CD_Result_t results;
// 	uint8_t status;

// 	do {
// 		status = sensor.VL53L4CD_CheckForDataReady(&NewDataReady);
// 	} while (!NewDataReady);

// 	if ((!status) && (NewDataReady != 0)) {
// 		//(Mandatory) Clear HW interrupt to restart measurements
// 		sensor.VL53L4CD_ClearInterrupt();

// 		//Read measured distance. RangeStatus = 0 means valid data
// 		sensor.VL53L4CD_GetResult(&results);
// 	}

//     // !!!----!!!!----calculateTrueRange_constant(&results.distance_mm, rangeCoeff_midFront_constant);

//     ts_lastMeasurement = millis();
//     newData = true;
//     measurementCount++;

// 	// Falls der Messwert ungültig ist (außer Reichweite)
// 	if (!results.range_status) {
//         lastMeasurement = results.distance_mm;
//         lastStatus = TofStatus::VALID;
//         return ErrorCodes::NEW_DATA;
//     }
// 	else {
//         lastMeasurement = 1023;
//         lastStatus = TofStatus::OUT_OF_RANGE;
//         return ErrorCodes::OUT_OF_RANGE;
//     }
// }

ErrorCodes TofVL53L4CD::Read(void) {
	uint8_t newDataReady = 0;
	uint8_t rangeStatus = 255;
	uint16_t distance = 0;
	uint8_t status = 0;

	const uint32_t now = millis();

	// If Read() hasn't been called for a while, discard the old wait state.
	// Otherwise after a 10s pause the first new call would immediately return TIMEOUT.
	if ((now - ts_lastReadCall) > TIMEOUT_TIME_MS) {
		waitingForMeasurement = false;
	}
	ts_lastReadCall = now;

	// A new wait cycle only begins when we start actively polling
	if (!waitingForMeasurement) {
		ts_waitStart = now;
		waitingForMeasurement = true;
	}

	status = sensor.VL53L4CD_CheckForDataReady(&newDataReady);
	if (status != 0) {
		waitingForMeasurement = false;
		timeoutOccured = false;
		lastStatus = TofStatus::ERROR;
		return ErrorCodes::ERROR;
	}

	if (newDataReady == 0) {
		if ((now - ts_waitStart) > TIMEOUT_TIME_MS) {
			waitingForMeasurement = false;
			timeoutOccured = true;
			lastMeasurement = INVALID_MEASUREMENT;
			lastStatus = TofStatus::TIMEOUT;
			return ErrorCodes::TIMEOUT;
		}

		return ErrorCodes::NO_NEW_DATA;
	}

	status = sensor.VL53L4CD_ClearInterrupt();
	if (status != 0) {
		waitingForMeasurement = false;
		timeoutOccured = false;
		lastStatus = TofStatus::ERROR;
		return ErrorCodes::ERROR;
	}

	status = sensor.VL53L4CD_GetMinimalResult(&rangeStatus, &distance);
	if (status != 0) {
		waitingForMeasurement = false;
		timeoutOccured = false;
		lastStatus = TofStatus::ERROR;
		return ErrorCodes::ERROR;
	}

	waitingForMeasurement = false;
	timeoutOccured = false;

	ts_lastMeasurement = now;
	newData = true;
	measurementCount++;

	// Optional:
	// calculateTrueRange_constant(&results.distance_mm, rangeCoeff_midFront_constant);

	if (rangeStatus == 0) {
		lastMeasurement = distance;
		lastStatus = TofStatus::VALID;
		return ErrorCodes::NEW_DATA;
	}
	else {
		lastMeasurement = INVALID_MEASUREMENT;
		lastStatus = TofStatus::OUT_OF_RANGE;
		return ErrorCodes::NEW_DATA;
	}
}

uint16_t TofVL53L4CD::GetRange(void) {
	newData = false;
	return lastMeasurement;
}

ErrorCodes TofVL53L4CD::Stop(void) {
	sensor.VL53L4CD_StopRanging();
	waitingForMeasurement = false;
	return ErrorCodes::OK;
}

ErrorCodes TofVL53L4CD::Continue(void) {
	sensor.VL53L4CD_StartRanging();
	waitingForMeasurement = false;
	timeoutOccured = false;
	ts_lastReadCall = millis();
	return ErrorCodes::OK;
}

#ifdef _MSC_VER
    #pragma endregion
    #pragma region VL53L5CX //---------------------------------------------------------------------------------------------------
#endif
TofVL53L5CX::TofVL53L5CX(uint8_t i2cAddress, uint8_t xshutPin)
	: TofParent(i2cAddress, xshutPin) {}

bool TofVL53L5CX::Init(void) {
	On();
	delay(10);
	if (!sensor.begin()) sensor.begin(i2cAddress);
	sensor.setResolution(8 * 8);
	sensor.setAddress(i2cAddress);
	sensor.setRangingFrequency(15);
	sensor.startRanging();
	return true;
}

bool TofVL53L5CX::IsRamp(void) {
	// Wait for new data with timeout
	const unsigned long timeout = 1000;
	unsigned long startTime = millis();

	while (!sensor.isDataReady()) {
		if (millis() - startTime > timeout) {
			#ifdef DEBUG_X64_VIEW
			Serial.println("Timeout while waiting for new data.");
			#endif
			return false;
		}
	}

	// Data is ready, read it
	if (sensor.getRangingData(&measurementData)) {
		#ifdef DEBUG_X64
		for (int y = 0; y <= imageWidth * (imageWidth - 1); y += imageWidth) {
			for (int x = imageWidth - 1; x >= 0; x--) {
				Serial.print(measurementData.distance_mm[x + y]);
				Serial.print("\t");
			}
			Serial.println();
		}
		Serial.println();
		#endif
		int upper = (measurementData.distance_mm[messurePointsUpper[0]] + measurementData.distance_mm[messurePointsUpper[1]]) / 2;
		int lower = (measurementData.distance_mm[messurePointsLower[0]] + measurementData.distance_mm[messurePointsLower[1]]) / 2;

		if (lower <= FRONT_MAX_DETECTION_DISTANCE && upper <= FRONT_MAX_DETECTION_DISTANCE) {
			#ifdef DEBUG_X64_VIEW
			Serial.print("Upper: ");
			Serial.print(upper);
			Serial.print(" \tLower: ");
			Serial.print(lower);
			Serial.print(" \tDiff: ");
			Serial.print(upper - lower);
			Serial.print("\n");
			#endif

			if ((upper - lower) >= FRONT_DIFF_DETECTION_VALUE) {
				#ifdef DEBUG_X64_VIEW
				Serial.println("Ramp detected!");
				#endif
				return true;	// Ramp detected
			}
			else return false;	// Ramp not detected
		}
		else return false;	// Ramp not detected
	}
	else return false;
}

#ifdef _MSC_VER
    #pragma endregion
    #pragma region TOF Class //--------------------------------------------------------------------------------------------------
#endif
// Private method to disable all ToF sensors via their XSHUT pins
void TofSensors::DisableAll(void) {
	leftBack.Off();
	leftFront.Off();
	rightFront.Off();
	rightBack.Off();
	front.Off();
	back.Off();
	frontWall.Off();
	backWall.Off();
	// front_x64.Off();
	// back_x64.Off();
}

ErrorCodes TofSensors::Init(void) {
	DisableAll();
	EnableUpdate();
	delay(50);

	if (leftBack.Init() == true && leftFront.Init() == true && rightFront.Init() == true
		&& rightBack.Init() == true && back.Init() == true && front.Init() == true
		&& backWall.Init() == true && frontWall.Init() == true)
		// && back_x64.Init() == true && front_x64.Init() == true)
		return ErrorCodes::OK;
	else
		return ErrorCodes::ERROR;
}

ErrorCodes TofSensors::Update(void) {
	if (!_UPDATE_ENABLED) return ErrorCodes::OK;

	ErrorCodes errLB = leftBack.Read();
	ErrorCodes errLF = leftFront.Read();
	ErrorCodes errRF = rightFront.Read();
	ErrorCodes errRB = rightBack.Read();
	ErrorCodes errBU = back.Read();
	ErrorCodes errFU = front.Read();
	ErrorCodes errBL = backWall.Read();
	ErrorCodes errFL = frontWall.Read();

	if (errLB == ErrorCodes::NEW_DATA || errLF == ErrorCodes::NEW_DATA ||
		errRF == ErrorCodes::NEW_DATA || errRB == ErrorCodes::NEW_DATA ||
		errBU == ErrorCodes::NEW_DATA || errFU == ErrorCodes::NEW_DATA ||
		errBL == ErrorCodes::NEW_DATA || errFL == ErrorCodes::NEW_DATA)
	{
		return ErrorCodes::NEW_DATA;
	}

	return ErrorCodes::NO_NEW_DATA;
}

void TofSensors::DisableUpdate(void) {
	_UPDATE_ENABLED = false;
}

void TofSensors::EnableUpdate(void) {
	_UPDATE_ENABLED = true;
}

uint16_t TofSensors::GetRange(TofType sensor) {
	EnableUpdate();

	switch (sensor) {
	case TofType::LEFT_BACK:
		return leftBack.GetRange();
		break;

	case TofType::LEFT_FRONT:
		return leftFront.GetRange();
		break;

	case TofType::RIGHT_FRONT:
		return rightFront.GetRange();
		break;

	case TofType::RIGHT_BACK:
		return rightBack.GetRange();
		break;

	case TofType::BACK:
		return back.GetRange();
		break;

	case TofType::FRONT:
		return front.GetRange();
		break;

	case TofType::BACK_WALL:
		return backWall.GetRange();
		break;

	case TofType::FRONT_WALL:
		return frontWall.GetRange();
		break;

	default:
		return 0;
		break;
	}
}

TofStatus TofSensors::GetStatus(TofType sensor) {
	EnableUpdate();

	switch (sensor) {
	case TofType::LEFT_BACK:
		return leftBack.GetStatus();
		break;

	case TofType::LEFT_FRONT:
		return leftFront.GetStatus();
		break;

	case TofType::RIGHT_FRONT:
		return rightFront.GetStatus();
		break;

	case TofType::RIGHT_BACK:
		return rightBack.GetStatus();
		break;

	case TofType::BACK:
		return back.GetStatus();
		break;

	case TofType::FRONT:
		return front.GetStatus();
		break;

	case TofType::BACK_WALL:
		return backWall.GetStatus();
		break;

	case TofType::FRONT_WALL:
		return frontWall.GetStatus();
		break;

	default:
		return TofStatus::TIMEOUT;
		break;
	}
}

bool TofSensors::IsDataNew(TofType sensor) {
	EnableUpdate();

	switch (sensor) {
	case TofType::LEFT_BACK:
		return leftBack.IsDataNew();
		break;

	case TofType::LEFT_FRONT:
		return leftFront.IsDataNew();
		break;

	case TofType::RIGHT_FRONT:
		return rightFront.IsDataNew();
		break;

	case TofType::RIGHT_BACK:
		return rightBack.IsDataNew();
		break;

	case TofType::BACK:
		return back.IsDataNew();
		break;

	case TofType::FRONT:
		return front.IsDataNew();
		break;

	case TofType::BACK_WALL:
		return backWall.IsDataNew();
		break;

	case TofType::FRONT_WALL:
		return frontWall.IsDataNew();
		break;

	default:
		return false;
		break;
	}
}

bool TofSensors::AnyTimeoutOccured(void) {
	if (leftBack.TimeoutOccured() || leftFront.TimeoutOccured() || rightFront.TimeoutOccured()
		|| rightBack.TimeoutOccured() || front.TimeoutOccured() || back.TimeoutOccured()
		|| frontWall.TimeoutOccured() || backWall.TimeoutOccured())
		return true;
	else
		return false;
}

int8_t TofSensors::CalculateLeftRightError(float angleError, uint8_t sideWallThreshold, uint8_t gapRobotWall) {
	if (leftBack.GetRange() < sideWallThreshold && leftFront.GetRange() < sideWallThreshold && rightFront.GetRange() < sideWallThreshold && rightBack.GetRange() < sideWallThreshold)
		return ((leftBack.GetRange() + leftFront.GetRange()) / 2 - (rightFront.GetRange() + rightBack.GetRange()) / 2) / 2;
	else if (leftBack.GetRange() < sideWallThreshold && leftFront.GetRange() < sideWallThreshold)
		return (leftBack.GetRange() + leftFront.GetRange()) / 2 - gapRobotWall;
	else if (rightFront.GetRange() < sideWallThreshold && rightBack.GetRange() < sideWallThreshold)
		return gapRobotWall - (rightFront.GetRange() + rightBack.GetRange()) / 2;
	// else if (leftFront.GetRange() < sideWallThreshold && angleError >= -2.5 && angleError <= 2.5)
	// 	return leftFront.GetRange() - gapRobotWall;
	// else if (rightFront.GetRange() < sideWallThreshold && angleError >= -2.5 && angleError <= 2.5)
	// 	return gapRobotWall - rightFront.GetRange();
	else return 0;
}

uint8_t TofSensors::GetWalls(void) {
	uint8_t wallInfo = 0;

	// Detect ramps directly so a ramp ahead/behind is not recorded as a wall.
	bool rampInfront = IsRampThere(false);
	bool rampBehind  = IsRampThere(true);

	if (leftBack.GetRange() < 150
		&& leftFront.GetRange() < 150) wallInfo |= (1 << 3);	// Left
	if (rightBack.GetRange() < 150
		&& rightFront.GetRange() < 150) wallInfo |= (1 << 1);	// Right
	if (back.GetRange() < 150 && !rampBehind) wallInfo |= (1 << 2);	// Back
	if (front.GetRange() < 150 && !rampInfront) wallInfo |= (1 << 0);	// Front

	#ifdef DEBUG_SCAN
		Serial.print("Wall Info: ");
		Serial.print(wallInfo, BIN);
		Serial.print("\tLB: ");
		Serial.print(leftBack.GetRange());
		Serial.print("\tLF: ");
		Serial.print(leftFront.GetRange());
		Serial.print("\tRF: ");
		Serial.print(rightFront.GetRange());
		Serial.print("\tRB: ");
		Serial.print(rightBack.GetRange());
		Serial.print("\tMF: ");
		Serial.print(front.GetRange());
		Serial.print("\tMB: ");
		Serial.println(back.GetRange());
	#endif

	return wallInfo;
}

bool TofSensors::IsRampThere(bool side) {
	uint16_t upper = side ? back.GetRange()     : front.GetRange();
	uint16_t lower = side ? backWall.GetRange() : frontWall.GetRange();
	// Per-side cap: only ranges where the drive would reference absolutely (front <=410, back <=250)
	// can be corrupted by a ramp; the back cap also stays under the 12 deg floor-intersection (~494 mm).
	uint16_t maxDist = side ? RAMP_MAX_DETECTION_DISTANCE_BACK : RAMP_MAX_DETECTION_DISTANCE_FRONT;
	// Both sensors must see a near surface; otherwise there is no wall/ramp to compare against.
	if(upper > maxDist || lower > maxDist)
		return false;
	// Cast to signed before subtracting to avoid uint16_t underflow when lower > upper.
	int16_t diff = static_cast<int16_t>(upper) - static_cast<int16_t>(lower);
	// Back needs a lower threshold: the 12 deg/height geometry yields a smaller ramp diff (~36)
	// than the front (~60), while flat walls stay negative on both sides.
	int16_t threshold = side ? RAMP_DIFF_THRESHOLD_BACK : RAMP_DIFF_THRESHOLD_FRONT;
	#ifdef DEBUG_RAMP
		Serial.print(side ? "BACK" : "FRONT");
		Serial.print(" ramp  upper="); Serial.print(upper);
		Serial.print("  lower="); Serial.print(lower);
		Serial.print("  diff="); Serial.println(diff);
	#endif
	return diff >= threshold;
}

#ifdef _MSC_VER
    #pragma endregion
#endif