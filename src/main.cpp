/**
* @authors: Vincent Rohkamm, Florian Wiesner
* @date: 18.11.2025
* @description: Main program file for the robot
*/

#ifdef _MSC_VER
  #pragma region Defines //------------------------------------------------------------------------
#endif

#define BAUD_RATE 115200
#define I2C_CLOCK 1000000UL
#define BUTTON_BLACK 49
#define BUTTON_GRAY	 51

#define RESET_HEIGHT_SPAN 90
#define UPPER_LEVEL_HEIGHT 230
#define LOWER_LEVEL_HEIGHT -100

// #define PID_TUNE_MODE        // Uncomment to enable drive-forever PID tuning harness
// #define TURN_TUNE_MODE       // Uncomment to enable alternating-90° turn PID tuning harness
// #define DEBUG_LOOP_TIMING    // Uncomment to print per-subsystem timing in cyclicMainTask/cyclicRunTask

#ifdef _MSC_VER
  #pragma endregion Defines
  #pragma region Includes //-----------------------------------------------------------------------
#endif

//----Includes----
#include <Arduino.h>
#include <Wire.h>

  //Custom Includes - General
#include <SerialSetup.h>
#include <CustomDatatypes.h>

  //Custom Includes - Modules
#include <Mapping.h>
#include <UserInterface.h>
#include <TofSensors.h>
#include <ColorSensing.h>
#include <Gyro.h>
#include <Motor.h>
#include <Ejector.h>
#include <Driving.h>
#include <Vcameras.h>
#include <BLE.h>

#ifdef _MSC_VER
  #pragma endregion Includes
  #pragma region Objects //------------------------------------------------------------------------
#endif

//----Objects----
UserInterface UI(100); // Update Interval: 50ms
EEPROM eeprom;
ColorSensing cs/*(&Serial)*/;
GyroBNO055 gyro;
Ejector ejector;
TofSensors tof;
Mapping mapper;
Drivetrain drivetrain;
Driving robot;
Vcameras cam(&Serial);
BLE_UART ble(false);


#ifdef _MSC_VER
#pragma endregion Objects
#pragma region Variables //----------------------------------------------------------------------
#endif

//----Variables----
RobotState currentMenuState;
RunState currentRunState;
uint32_t lastButtonPressGray;
uint32_t ts_lastCycle;

//----Flags----
bool _ROBOT_TURNING = false;
bool _RAMP_INFRONT = false;
bool _RAMP_BEHIND = false;
ErrorCodes _CHECKPOINT = ErrorCodes::OK;

#ifdef _MSC_VER
  #pragma endregion Variables
  #pragma region Prototypes //----------------------------------------------------------------------
#endif

//----Functions----
void cyclicMainTask();
void cyclicRunTask();

void ISR_BTN_BLACK();
void ISR_BTN_GRAY();

#ifdef _MSC_VER
  #pragma endregion Functions
  #pragma region Initialization //-----------------------------------------------------------------
#endif

int main(void) {
  serialSetup(BAUD_RATE);

  currentMenuState = RobotState::BOOT;

  Wire.begin();
  Wire.setClock(I2C_CLOCK);
  Wire1.begin();

  //Initialize Modules
    //User Interface
  UI.Initialize();

  Wire1.setClock(I2C_CLOCK);

  UI.ConnectPointer(&currentMenuState, &cs, &mapper, &cam, &ejector);
    //Buttons
  pinMode(BUTTON_BLACK, INPUT);
  pinMode(BUTTON_GRAY, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON_BLACK), ISR_BTN_BLACK, RISING);
	attachInterrupt(digitalPinToInterrupt(BUTTON_GRAY), ISR_BTN_GRAY, RISING);
  lastButtonPressGray = millis();
  UI.AddInfoMsg("Buttons", "OK", true);

  //----EEPROM----
  (eeprom.Init() != ErrorCodes::OK) ? UI.AddInfoMsg("EEPROM", "ERROR", false) : UI.AddInfoMsg("EEPROM", "OK", true); 
  
  //----Color sensor----
  if(cs.Init(&Wire,&UI,&eeprom)!=0) UI.AddInfoMsg("Color Sensor", "ERROR", false);
  else UI.AddInfoMsg("Color Sensor", "OK", true);
  cs.EnableRead(true);

  //----ToF----
  if (tof.Init() == ErrorCodes::OK)
    UI.AddInfoMsg("TOF", "OK", true);
  else
    UI.AddInfoMsg("TOF", "ERROR", true);

  //----Gyro----
  if (gyro.Init() == ErrorCodes::OK)
    UI.AddInfoMsg("Gyro", "OK", true);
  else
    UI.AddInfoMsg("Gyro", "ERROR", true);

  //----BLE----
  switch(ble.init(&UI)){
  case ErrorCodes::OK:
    UI.AddInfoMsg("BLE", "OK", true);
    break;
  case ErrorCodes::no_connection:
    UI.AddInfoMsg("BLE", "CONN ERROR", false);
    break;
  case ErrorCodes::disabled:
    UI.AddInfoMsg("BLE", "NOT ENABLED", false);
    break;
  default:
    UI.AddInfoMsg("BLE", "UNKNOWN", false);
    break;
  } 

  //----Camera----
  if(cam.Init(&ejector, &mapper, &robot, &UI, &drivetrain) != ErrorCodes::OK) UI.AddInfoMsg("Cameras", "CONN ERROR", false);
  else {UI.AddInfoMsg("Cameras", "OK", true);}

  //----Robot----
  robot.Init(&cs, &tof, &gyro, &mapper, &cam ,&drivetrain);
  UI.AddInfoMsg("Driving", "OK", true);
  UI.AddInfoMsg("Drivetrain", "OK", true);

  //----Ejector----
  ejector.Init(&robot);
  UI.AddInfoMsg("Ejectors", "OK", true);

  UI.AddInfoMsg("Finished STARTUP", "ACK", false);

#ifdef PID_TUNE_MODE
  while (currentMenuState != RobotState::RUN) {
    cyclicMainTask();
  }
  gyro.ResetAllAngles();
  robot.SetRobotTargetAngle(Orientations::North);
  currentRunState = RunState::DRIVE;
  robot.EnableBumpers();
  robot.StartDrive(false);
  while (true) {
    serialLoop();
    cyclicMainTask();
    cyclicRunTask();
    robot.ControlDrive(UI.GetDriveSpeed(), gyro.GetAngleFromOrientation(robot.GetRobotTargetAngle()));
  }
#endif

#ifdef TURN_TUNE_MODE
  while (currentMenuState != RobotState::RUN) {
    cyclicMainTask();
  }
  gyro.ResetAllAngles();
  bool _tuneRight = true;
  robot.StartTurn(90.0f);
  while (true) {
    serialLoop();
    cyclicMainTask();
    cyclicRunTask();
    if (robot.ControlTurn(_tuneRight ? 90.0f : 0.0f) == ErrorCodes::TURNED) {
      delay(300);
      _tuneRight = !_tuneRight;
      robot.StartTurn(_tuneRight ? 90.0f : 0.0f);
    }
  }
#endif

#ifdef _MSC_VER
  #pragma endregion Initialization
  #pragma region Cyclic //-------------------------------------------------------------------------
#endif
while (true) {
  serialLoop();
  cyclicMainTask();

  if (currentMenuState == RobotState::RUN) {
    if (currentRunState == RunState::INITIAL) { //Initial Run Logic
      mapper.Reset();
      cam.Enable(true);
      robot.EnableBumpers();	  //Enable Bumpers
			robot.StartAlign();	      //Start Aligning
			gyro.ResetAllAngles();	  //Gyro angle zero
			robot.SetMaxRampIncline(0);
			robot.SetCurrentRobotHeight(0);
      delay(250); 
      currentRunState = RunState::SETTILE;
    }

    cyclicRunTask();  //Cyclic Run Tasks

    if (currentRunState == RunState::SETTILE) {
      UI.BuzzerSignal(5, 0, 1);
			mapper.SetTile(tof.GetWalls(_RAMP_INFRONT, _RAMP_BEHIND), cs.GetFloor());

      //Checkpoint handling
      if(cs.GetFloor() == TileType::checkpoint) UI.ShowPopup("CHECKPOINT",ErrorCodes::info, 4);
			currentRunState = RunState::GET_INSTRUCTIONS;
      robot.SetLastSetTile(millis());
    } 

    else if (currentRunState == RunState::GET_INSTRUCTIONS) {
      UI.UpdateMap();
      //Get Instructions Logic
      switch (mapper.GetInstruction()) 
      {
      case Instructionset::T_North:
      //Turn North Logic
        robot.EndDrive();
				robot.StartAdjustment();
				currentRunState = RunState::TURN;
				robot.SetRobotTargetAngle(Orientations::North);
        robot.StartTurn(gyro.GetAngleFromOrientation(robot.GetRobotTargetAngle()));
				_ROBOT_TURNING = true;
        break;

      case Instructionset::T_East:
        //Turn East Logic
				robot.EndDrive();
				robot.StartAdjustment();
				currentRunState = RunState::TURN;
				robot.SetRobotTargetAngle(Orientations::East);
        robot.StartTurn(gyro.GetAngleFromOrientation(robot.GetRobotTargetAngle()));
				_ROBOT_TURNING = true;
        break;

      case Instructionset::T_South:
        //Turn South Logic
				robot.EndDrive();
				robot.StartAdjustment();
				currentRunState = RunState::TURN;
				robot.SetRobotTargetAngle(Orientations::South);
        robot.StartTurn(gyro.GetAngleFromOrientation(robot.GetRobotTargetAngle()));
				_ROBOT_TURNING = true;
        break;

      case Instructionset::T_West:
        //Turn West Logic
				robot.EndDrive();
				robot.StartAdjustment();
				currentRunState = RunState::TURN;
				robot.SetRobotTargetAngle(Orientations::West);
        robot.StartTurn(gyro.GetAngleFromOrientation(robot.GetRobotTargetAngle()));
				_ROBOT_TURNING = true;
        break;

      case Instructionset::D_Forward:
        //Drive Forward Logic
        //Blue Tile:
        if (cs.GetFloor() == TileType::blue) {
          //Stoppen
          robot.EndDrive();
          UI.ShowPopup("BLUE TILE", ErrorCodes::info);
          uint32_t time = millis();
          while(millis() <= time + 5000){
            UI.Update();
            delay(480);
          }
          //Weiterfahren
        }
        currentRunState = RunState::DRIVE;
        robot.StartDrive(false);
        break;

      case Instructionset::ramp:
        currentRunState = RunState::DRIVE;
        robot.StartDrive(true);
        break;

      case Instructionset::MazeFinished:
        //Maze Finished Logic
        currentRunState = RunState::END;
        break;
      
      default:
        break;
      }

    } 
    
    else if (currentRunState == RunState::CHECKPOINT_RESET) {
      //Checkpoint Reset Logic
      UI.ShowResetScreen();
      robot.EndDrive();
      robot.DisableBumpers();
      UI.UpdateResetProgress("disable Bump",1,4);
      cs.Freeze(true);
      cam.Enable(false);

      _CHECKPOINT = ErrorCodes::ready;
      UI.UpdateResetProgress("Start Ready ",2,4);
      while(_CHECKPOINT != ErrorCodes::start) {delay(5); cs.Update();}

      delay(10);
      robot.EnableBumpers();
      robot.StartAlign();
			gyro.ResetAllAngles(); //Reset Gyro => Robot has to look towards NORTH!
      UI.UpdateResetProgress("Reset Robot ",3,4);
      cs.Freeze(false);
      cs.Update();
      delay(200);
      mapper.RestartCheckpoint();
      robot.SetRobotTargetAngle(Orientations::North);
      UI.UpdateResetProgress("Reset Map   ",4,4);
      _CHECKPOINT = ErrorCodes::OK;
      currentRunState = RunState::SETTILE;
      UI.FinishReset(true);
    } 
    
    else if (currentRunState == RunState::TURN) {
      //Turn Logic
      if (robot.ControlTurn(gyro.GetAngleFromOrientation(robot.GetRobotTargetAngle())) == ErrorCodes::TURNED) {
        robot.EndTurn();
        _ROBOT_TURNING = false;
				currentRunState = RunState::SETTILE;
      }
    }
    
    else if (currentRunState == RunState::ALIGN) {
      //Align Logic
    } 
    
    // CHECK_DRIVE and RAMP states are now inlined into DRIVE so ControlDrive runs every iteration.
    // else if (currentRunState == RunState::CHECK_DRIVE) { ... }
    // else if (currentRunState == RunState::RAMP) { ... }

    else if (currentRunState == RunState::DRIVE) {
      ErrorCodes driveSave = robot.ControlDrive(UI.GetDriveSpeed(), gyro.GetAngleFromOrientation(robot.GetRobotTargetAngle()));
      if (driveSave == ErrorCodes::TIMEOUT) {
        robot.TimeoutDrive();
        currentRunState = RunState::SETTILE;
      }
      else if (robot.CheckDrive() == ErrorCodes::SCAN_DRIVE) {
        currentRunState = RunState::SCAN;
      }
      else if (robot.RampHandler() == ErrorCodes::RAMP_END) {
        currentRunState = RunState::SCAN;
      }
      // else: stay in DRIVE
    }

    else if (currentRunState == RunState::SCAN) {
			//Move the robot in the next tile and scan next field
			if (!robot.IsOnRamp())
        mapper.Move(true);	//Move robot forward

			//if Ramp detected during DRIVE give
			if(robot.IsOnRamp()){
				uint8_t rampLenght = 0;
				int8_t rampDirection = 0;

				//Calculate RAMP INFOS
				rampLenght = (robot.GetRampLength() / 300);	//Calculate num of Tiles
        if (rampLenght == 0 && robot.GetRampLength() >= 100)
          rampLenght = 1;
				//Determine RAMP Direction
				if(robot.GetCurrentRobotHeight() < robot.GetCurrentRobotHeight() + robot.GetRampHeight()){
					if(robot.GetCurrentRobotHeight() <= LOWER_LEVEL_HEIGHT && robot.GetCurrentRobotHeight() + robot.GetRampHeight() > LOWER_LEVEL_HEIGHT
						) rampDirection = 1;
					else if(robot.GetCurrentRobotHeight() < UPPER_LEVEL_HEIGHT && robot.GetCurrentRobotHeight() + robot.GetRampHeight() >= UPPER_LEVEL_HEIGHT
						) rampDirection = 1;
				}
				else if(robot.GetCurrentRobotHeight() > robot.GetCurrentRobotHeight() + robot.GetRampHeight()){
					if(robot.GetCurrentRobotHeight() >= UPPER_LEVEL_HEIGHT && robot.GetCurrentRobotHeight() + robot.GetRampHeight() < UPPER_LEVEL_HEIGHT
						) rampDirection = -1;
					else if(robot.GetCurrentRobotHeight() < UPPER_LEVEL_HEIGHT && robot.GetCurrentRobotHeight() + robot.GetRampHeight() <= LOWER_LEVEL_HEIGHT
						) rampDirection = -1;
				}
				robot.SetCurrentRobotHeight(robot.GetCurrentRobotHeight() + robot.GetRampHeight());
				if(robot.GetCurrentRobotHeight() <= RESET_HEIGHT_SPAN && robot.GetCurrentRobotHeight() >= -RESET_HEIGHT_SPAN) robot.SetCurrentRobotHeight(0);
				robot.ClearOnRamp();
				robot.SetMaxRampIncline(0);
				//pass RampInfos to Mapping
        // Serial.println("Ramp D: " + String(rampDirection));
        if (rampDirection == 1)
          mapper.Ramp(ErrorCodes::up, rampLenght);
        else if (rampDirection == -1)
          mapper.Ramp(ErrorCodes::down, rampLenght);
        else if (rampDirection == 0)
          mapper.Ramp(ErrorCodes::same, rampLenght);
			}
			currentRunState = RunState::SETTILE;
    }

    else if (currentRunState == RunState::END) {
      //End of Run Logic
      robot.EndDrive();
      robot.DisableBumpers();
      cam.Enable(false);
      currentMenuState = RobotState::ABOUT;
      UI.Update();
      UI.LED_BUZZER_Signal(1000,1000,5);
    }
  } 
  
  else if (currentMenuState == RobotState::SETTINGS) {
    //Settings Task
  } 
  
  else if (currentMenuState == RobotState::INFO_SENSOR || currentMenuState == RobotState::INFO_VISUAL) {
    UI.UpdateSensors(tof.GetRange(TofType::RIGHT_FRONT),tof.GetRange(TofType::RIGHT_BACK),
      tof.GetRange(TofType::LEFT_FRONT),tof.GetRange(TofType::LEFT_BACK),
      tof.GetRange(TofType::FRONT),tof.GetRange(TofType::FRONT_WALL),
      tof.GetRange(TofType::BACK),tof.GetRange(TofType::BACK_WALL),
      gyro.GetAngle(GyroAxles::Axis_X), gyro.GetAngle(GyroAxles::Axis_Y), gyro.GetAngle(GyroAxles::Axis_Z)
    );
    delay(20);
  }
  
  else if (currentMenuState == RobotState::BT) {
    ble.connect();
    currentMenuState = RobotState::SETTINGS;
  }

}
return 0;
}
#ifdef _MSC_VER
  #pragma endregion Cyclic
  #pragma region Functions //----------------------------------------------------------------------
#endif

void cyclicMainTask() {
  #ifdef DEBUG_LOOP_TIMING
  uint32_t _t = millis();
  tof.Update(); Serial.print("TOF:"); Serial.print(millis() - _t); _t = millis();
  UI.Update();  Serial.print("\tUI:");  Serial.print(millis() - _t); _t = millis();
  cs.Update();  Serial.print("\tCS:");  Serial.println(millis() - _t);
  #else
  tof.Update();
  UI.Update();
  cs.Update();
  #endif
}
void cyclicRunTask() {
  #ifdef DEBUG_LOOP_TIMING
  uint32_t _t = millis();
  uint8_t buffer = tof.GetWalls(_RAMP_INFRONT, _RAMP_BEHIND);
  Serial.print("GW:"); Serial.print(millis() - _t); _t = millis();
  cam.Update(cs.GetFloor() == TileType::dangerZone);
  Serial.print("\tCAM:"); Serial.println(millis() - _t);
  #else
  //uint8_t buffer = tof.GetWalls(_RAMP_INFRONT, _RAMP_BEHIND);
  cam.Update(cs.GetFloor() == TileType::dangerZone);
  #endif

  //Black Tile Handling
	if(cs.GetFloor() == TileType::black) {
		robot.EndDrive();	//Stop Robot
		//UI.signal.buzzer_pulse(5, 3);	//Signal BLACK
    mapper.Move(true);
		mapper.SetTile(0x0F, TileType::black);
		mapper.Move(false);	//Move robot Backwards

		//DRIVE BACKWARDS FUNCTION
		robot.ReverseBlackTile();
		for(uint8_t i = 0; i < 5; i++){
			cs.Update();	//Update ColorSensors
			delay(5);
		}
		currentRunState = RunState::SETTILE;	//SetTile again
	}

  //Drive Slower if FRONT detects change
	if(cs.GetAlert() && !cs.Freeze())
		robot.SetSlowSpeed(true);

  //Reset to std speed mod if nth is on ALERT
  if(!cs.GetAlert() && !cam.IsAlert()) robot.SetSlowSpeed(false);

  //Bumper Handling
	if(currentRunState != RunState::INITIAL){
		if(robot.BumperHandler() == ErrorCodes::BUMPER_WALL) currentRunState = RunState::SETTILE;
	}
}

  //Button for Starting and Checkpoint
void ISR_BTN_BLACK() {
  if(currentMenuState != RobotState::RUN){
    currentMenuState = RobotState::RUN;
    currentRunState = RunState::INITIAL;
  }
  else if(currentMenuState == RobotState::RUN && currentRunState != RunState::INITIAL){
    if(_CHECKPOINT == ErrorCodes::OK){
      _CHECKPOINT = ErrorCodes::stop;
      currentRunState = RunState::CHECKPOINT_RESET;
    } else if(_CHECKPOINT == ErrorCodes::ready){
      _CHECKPOINT = ErrorCodes::start;
    }
  }
}

void ISR_BTN_GRAY() {
  //Button for changing Drive Mode
	if (lastButtonPressGray + 300 < millis()) {
		UI.CycleDriveMode();
		lastButtonPressGray = millis();
	}
}

#ifdef VISUAL_STUDIO
  #pragma endregion Functions //-------------------------------------------------------------------
#endif