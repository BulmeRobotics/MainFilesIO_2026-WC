/**
* @authors: Vincent Rohkamm, Florian Wiesner
* @date: 18.11.2025
* @description: Main Program file for Robot für den Alten Roboter
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

#ifdef _MSC_VER
  #pragma endregion Defines
  #pragma region Includes //-----------------------------------------------------------------------
#endif

//Includes
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

#ifdef _MSC_VER
  #pragma endregion Includes
  #pragma region Objects //------------------------------------------------------------------------
#endif

//Objects
UserInterface UI(100); // Update Interval: 50ms
EEPROM eeprom;
ColorSensing cs/*(&Serial)*/;
GyroBNO055 gyro;
Ejector ejector;
TofSensors tof;
Mapping mapper;
Drivetrain drivetrain;
Driving robot;
Vcameras cam;

#ifdef _MSC_VER
#pragma endregion Objects
#pragma region Variables //----------------------------------------------------------------------
#endif

//Variables
RobotState currentMenuState;
RunState currentRunState;
uint32_t lastButtonPressGray;
uint32_t ts_lastCycle;

//Flags
bool _ROBOT_TURNING = false;
bool _RAMP_INFRONT = false;
bool _RAMP_BEHIND = false;
ErrorCodes _CHECKPOINT = ErrorCodes::OK;

#ifdef _MSC_VER
  #pragma endregion Variables
  #pragma region Prototypes //----------------------------------------------------------------------
#endif

  //FUNKTIONEN
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

  UI.ConnectPointer(&currentMenuState, &cs, &mapper, &cam);
    //Buttons
  pinMode(BUTTON_BLACK, INPUT);
  pinMode(BUTTON_GRAY, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON_BLACK), ISR_BTN_BLACK, RISING);
	attachInterrupt(digitalPinToInterrupt(BUTTON_GRAY), ISR_BTN_GRAY, RISING);
  lastButtonPressGray = millis();
  UI.AddInfoMsg("Buttons", "OK", true);


  //EEPROM
  eeprom.Init() != ErrorCodes::OK ? UI.AddInfoMsg("I2C", "ERROR", false) : UI.AddInfoMsg("I2C", "OK", true); 
  
  //Color sensor
  if(cs.Init(&Wire,&UI,&eeprom)!=0) UI.AddInfoMsg("Color Sensor", "ERROR", false);
  else UI.AddInfoMsg("Color Sensor", "OK", true);
  cs.EnableRead(true);

  //ToF
  if (tof.Init() == ErrorCodes::OK)
    UI.AddInfoMsg("TOF", "OK", true);
  else
    UI.AddInfoMsg("TOF", "ERROR", true);

  //Gyro
  if (gyro.Init() == ErrorCodes::OK)
    UI.AddInfoMsg("Gyro", "OK", true);
  else
    UI.AddInfoMsg("Gyro", "ERROR", true);

  UI.AddInfoMsg("Drivetrain", "OK", true);

  //Camera
  if(cam.Init(&ejector, &mapper, &robot, &UI, &drivetrain) != ErrorCodes::OK) UI.AddInfoMsg("Cameras", "CONN ERROR", false);
  else {UI.AddInfoMsg("Cameras", "OK", true);}

  //Robot
  robot.Init(&cs, &tof, &gyro, &mapper, &cam ,&drivetrain);
  UI.AddInfoMsg("Driving", "OK", true);

  //Ejector
  ejector.Init(&robot);
  UI.AddInfoMsg("Ejectors", "OK", true);

  UI.AddInfoMsg("Finished STARTUP", "ACK", false);
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
        currentRunState = RunState::CHECK_DRIVE;	//Start Drive
        robot.StartDrive(false);
        break;
      
      case Instructionset::ramp:
        currentRunState = RunState::CHECK_DRIVE;
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
    
    else if (currentRunState == RunState::CHECK_DRIVE) {
      //Drive Logic
      if(robot.CheckDrive() == ErrorCodes::CHECK_RAMP) currentRunState = RunState::RAMP;
			else currentRunState = RunState::SCAN;
    }

    else if (currentRunState == RunState::RAMP) {
      //Ramp Logic
      if(robot.RampHandler() == ErrorCodes::RAMP_END) currentRunState = RunState::SCAN;
			else currentRunState = RunState::DRIVE;
    }

    else if (currentRunState == RunState::DRIVE) {
      //Control Logic
      ErrorCodes driveSave = robot.ControlDrive((UI.GetDriveSpeed()), gyro.GetAngleFromOrientation(robot.GetRobotTargetAngle()));
			if(driveSave == ErrorCodes::CHECK_DRIVE) currentRunState = RunState::CHECK_DRIVE;
			else if (driveSave == ErrorCodes::TIMEOUT) {
        robot.TimeoutDrive();
				currentRunState = RunState::SETTILE;
      }
			else currentRunState = RunState::RAMP;
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
      UI.LED_BUZZER_Signal(1000,1000,5);
      currentMenuState = RobotState::ABOUT;
    }
  } 
  
  else if (currentMenuState == RobotState::SETTINGS) {
    //Settings Task
  } 
  
  else if (currentMenuState == RobotState::INFO_SENSOR) {

  } else if(currentMenuState == RobotState::INFO_VISUAL){

  } 
  
  else if (currentMenuState == RobotState::BT) {

  }

}
return 0;
}
#ifdef _MSC_VER
  #pragma endregion Cyclic
  #pragma region Functions //----------------------------------------------------------------------
#endif

void cyclicMainTask() {
  //Main cyclic tasks
  tof.Update();
  UI.Update();
  cs.Update();
}
void cyclicRunTask() {
  uint8_t buffer = tof.GetWalls(_RAMP_INFRONT, _RAMP_BEHIND);
  cam.Update(cs.GetFloor() == TileType::dangerZone);

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