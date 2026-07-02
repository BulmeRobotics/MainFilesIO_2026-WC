// author:  vincent rohkamm
// date:    05.12.2025
// description:
// *Mapping library for Robot

#ifndef _MAPPING_H_
#define _MAPPING_H_

#include <CustomDatatypes.h>

#define MAX_TILES 256
#define MAX_INSTRUCTIONS 256

#define COST_RAMP 5
#define COST_BLUE 7
#define COST_DANGER_ZONE 8
#define COST_OBSTACLE 10    //Cost of Obstacle when 5 times Bumper -> spread evenly 10 / 5 = 2 per Bump
#define BUMPER_RESET_COUNTER 5
#define COST_REGULAR 1

#define RAMP_LENGTH_MOD 3

#define OPTION_MAX_RESETS 2

// --------------------------------------------------------------------------------

enum class Instructionset : uint8_t {
    T_North, T_East, T_South, T_West,	//Desired Orientation
    D_Forward, ramp,					//Move
    unreachable, undefined, Overflow, solvMap, reset,//Errors
    FinishedInstructions, MazeFinished	//Finished! :)
};

struct Tile {
    int16_t x, y, z = 0;
    uint8_t weight = 1;
    uint8_t difficulty = 0;
    TileType type = TileType::inactive;
    int16_t north = -1, east = -1, south = -1, west = -1, up = -1, down = -1;
    bool victim = false;
};

struct OpenItem {
    uint16_t nodeIndex;
    uint16_t g;
    uint16_t f;
};


class Mapping {
private:    // --- PRIVATE ---

#ifdef _MSC_VER
#pragma region Panic Mode
#endif

    static constexpr uint8_t RELOCALIZE_REQUIRED_MATCHES = 3;

    // -- Variables --
    bool _PANIC_MODE_ACTIVE = false;
    uint16_t _panicCandidates[MAX_TILES];
    uint16_t _panicCandidateCount = 0;
    uint8_t _panicConfidence = 0;
    uint8_t _currentPanicWalls = 0;
    bool _panicHasMoved = true; // Verhindert, dass reines Drehen die Konfidenz erhöht

    // --- Panic Mode Methoden ---
    bool DoesTileMatchWalls(uint16_t tileIndex, uint8_t absWalls);
    void UpdateRelocalization(uint8_t absWalls);
    void MovePanicCandidates(Orientations dir);
    Instructionset GetPanicInstruction();


#ifdef _MSC_VER
#pragma region helpers
#endif

    // inline funktion um manhatten distanz zu errechnen
    inline uint16_t heuristic3D(Tile a, Tile b);

    uint16_t findNextTarget();

    /**
	 * @brief Searches for specific Tile by its coordinates
	 * @return Index of the Tile in tiles[] or -1 if not found
     */
    uint16_t findTileByCoordinate(int16_t x, int16_t y, int16_t z);

    void initLists(uint16_t target);

    /**
     * @brief finds next empty memory in tiles[]
     * @return index of next empty Memory for Tile
     */
    uint16_t findNextEmptyMemory();

    /**
     * @brief corrects wallInfo from relative to absolute Orientation
     * @param walls relative wall info as in SetTile
     * @param orientation Current Orientation of Robot
     * @return normalized wallInfo
     */
    uint8_t correctWallinfo(uint8_t walls, Orientations orientation);

    /**
     * @brief compares the saved Walls of the current tile with the input walls
     * @param walls wall info -> MUST BE ABSOLUTE!! -> call correctWallinfo before!
     * @return returns OK if ok or invalid if not matching
     */
    ErrorCodes compareWalls(uint8_t walls);

    /**
     * @brief Commits a staged checkpoint (updates lastCheckpointPosition and snapshots backupTiles).
     *        No-op if no checkpoint is pending.
     */
    void CommitPendingCheckpoint(void);

#ifdef _MSC_VER
#pragma region private vars
#endif

    // -- Map --
    Tile tiles[MAX_TILES];
    OpenItem open[MAX_TILES];
    bool closed[MAX_TILES];

    // -- Path --
    Instructionset path[MAX_INSTRUCTIONS];
    uint16_t pathIndex;

    // -- Position & Orientation --
    Orientations currentOrientation;
    uint16_t currentPosition, targetPosition;

    // -- Checkpoint --
    Tile backupTiles[MAX_TILES];
    uint16_t lastCheckpointPosition;
    uint8_t resetCounter = 0;
    // Deferred-commit guard: a detected checkpoint is staged here and only committed once the
    // robot reaches the next tile (proving it did not roll back off a decline at the lip).
    bool     _checkpointPending        = false;
    uint16_t pendingCheckpointPosition = 0;

    // -- Config --
    ErrorCodes pathPriority = ErrorCodes::straight;
    bool _RETURN_HOME = false;
    ErrorCodes _layerSetting = ErrorCodes::single;
    ErrorCodes _rampSetting = ErrorCodes::multi;

    // -- BUMPER --
    bool _BumperTriggered = false;

public: // --- PUBLIC ---

#ifdef _MSC_VER
#pragma region Information
#endif

    /**
     * @brief Set the priority of searching for next Tile
     * @param priority the new priority: -straight, -north, -east, -south, -west
     * @return returns if succesful or invalid
     */
    ErrorCodes SetPriority(ErrorCodes priority);

    /**
     * @brief gives next Instruction to Robot
     * @return returns the next Instruction
     */
    Instructionset GetInstruction();

    /**
     * @brief Informs Mapping of current Tile
     * @param walls walls around the Robot; relative -> b0...front, b1...right, b2...behind, b3...left
     * @param floor current floor Type -> Obstacle?, Color?, Regular?
     * @return Informs about Errors like not matching data
     * @todo
     */
    ErrorCodes SetTile(uint8_t walls, TileType floor);

    /**
     * @brief Called when victim is detected
     * @return already_found / OK / ERROR
     */
    ErrorCodes SetVictim();

    /**
     * @brief set ramp handling
     * @param set single / multi 
     * @return current setting
     */
    ErrorCodes SetSettings(ErrorCodes layer, ErrorCodes ramp){
        if(layer == ErrorCodes::single || layer == ErrorCodes::multi)
            _layerSetting = layer;
        else return ErrorCodes::invalid;

        if(ramp == ErrorCodes::single || ramp == ErrorCodes::multi || ramp == ErrorCodes::disabled)
            _rampSetting = ramp;
        else return ErrorCodes::invalid;

        return ErrorCodes::OK;
    }

    /**
     * @brief gets current ramp handling
     * @return current layer setting -> single / multi
     */
    ErrorCodes GetSetting(ErrorCodes set) {
        if(!(set == ErrorCodes::ramp || set == ErrorCodes::layer)) return ErrorCodes::invalid;
        ErrorCodes _setting = (set == ErrorCodes::ramp) ? _rampSetting : _layerSetting;
        return _setting; 
    };

    // /**
    //  * @brief Druckt die intern gespeicherte Karte des Roboters in die Konsole
    //  */
    // void PrintInternalMap();

#ifdef _MSC_VER
#pragma region Movement
#endif

    /**
     * @brief Moves Robot 1Tile forward or backwards
     * @param direction true...Forward, false...backward
     * @return Returns if move was possible or error
     */
    ErrorCodes Move(bool direction);

    /**
     * @brief Rotates Robot to desired direction
     * @param direction new direction the robot faces
     */
    ErrorCodes Turn(Orientations direction);

    /**
     * @brief registers a ramp instead of moving robot horizontally, is called instead of Move / ONLY TO BE CALLED WHEN changing level!!!
     * @param direction up / down
     * @param length length in tiles
     * @return returns if operation is valid
     */
    ErrorCodes Ramp(ErrorCodes direction, uint8_t length);

    /**
     * @brief Part of Ramp handling, needed because robot sometimes does 
     */
    void RollbackOne();

#ifdef _MSC_VER
#pragma region Error Handling
#endif

    /**
     * @brief Resets Mapping to start new Run
     */
    void Reset(void);

    /**
     * @brief  called to reset Mapping to last checkpoint, Resets mapping if no checkpoint registered
     * @return Returns Errors
     */
    ErrorCodes RestartCheckpoint(void);

    /**
     * @brief call when Bumper is triggered -> increases tile cost & locks the current route if desired
     * @param reset true...locks current route / false...increase cost of tile
     * @return returns if operation was succesful
     */
    ErrorCodes Bumper(bool reset = true);

#ifdef _MSC_VER
#pragma region UI
#endif
    Tile* GetTiles() { return tiles; }
    uint16_t GetCurrentPosition() { return currentPosition; }
    Orientations GetCurrentOrientation() { return currentOrientation; }
    bool IsPanicMode() { return _PANIC_MODE_ACTIVE; }
};

#endif