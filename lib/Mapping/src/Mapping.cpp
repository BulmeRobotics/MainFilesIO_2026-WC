// author: vincent rohkamm
// date: 05.12.2025

//#include <Arduino.h>
#include <cstdint>
#include <stdio.h>
#include <bitset>
#include <iostream>
#include <math.h>
#include <cstring>
#include "Mapping.h"

#include <Arduino.h>

#ifdef _MSC_VER
#pragma region helpers
#endif

inline uint16_t Mapping::heuristic3D(Tile a, Tile b) {
    return abs(a.x - b.x) + abs(a.y - b.y) + abs(a.z - b.z) * RAMP_LENGTH_MOD;
}

void Mapping::initLists(uint16_t target) {
    for (uint16_t i = 0; i < MAX_TILES; i++) {
        open[i].f = 0;
        open[i].g = 0;
        open[i].nodeIndex = 0;
        //open[i].g = heuristic3D(tiles[target], tiles[targetPosition]);
    }
    memset(closed, false, sizeof(closed));  //Closed List reset
}

uint16_t Mapping::findNextEmptyMemory() {
    for (uint16_t i = 0; i < MAX_TILES; i++)
    {
        if (tiles[i].type == TileType::inactive)
            return i;
    }
    return UINT16_MAX;  //kein freier Speicher mehr verfügbar
}

uint8_t Mapping::correctWallinfo(uint8_t walls, Orientations orientation) {
    uint8_t buff = 0;
    switch (orientation) {
    case Orientations::North:
        return walls;
        break;
    case Orientations::East:
        if (walls & 1 << 0) buff |= (1 << 1);	//Set East
        if (walls & 1 << 1) buff |= (1 << 2);	//Set South
        if (walls & 1 << 2) buff |= (1 << 3);	//Set West
        if (walls & 1 << 3) buff |= (1 << 0);	//Set North
        break;
    case Orientations::South:
        if (walls & 1 << 3) buff |= (1 << 1);	//Set East
        if (walls & 1 << 0) buff |= (1 << 2);	//Set South
        if (walls & 1 << 1) buff |= (1 << 3);	//Set West
        if (walls & 1 << 2) buff |= (1 << 0);	//Set North
        break;
    case Orientations::West:
        if (walls & 1 << 2) buff |= (1 << 1);	//Set East
        if (walls & 1 << 3) buff |= (1 << 2);	//Set South
        if (walls & 1 << 0) buff |= (1 << 3);	//Set West
        if (walls & 1 << 1) buff |= (1 << 0);	//Set North
        break;

    default:
        return 0b01000000;
        break;
    }
    return buff;
}

ErrorCodes Mapping::compareWalls(uint8_t walls) {
    if (currentPosition >= MAX_TILES) return ErrorCodes::invalid;

    auto bit = [&](uint8_t b)->bool { return (walls & (1U << b)) != 0U; };

    bool b_north = (tiles[currentPosition].north == -1);
    bool b_east = (tiles[currentPosition].east == -1);
    bool b_south = (tiles[currentPosition].south == -1);
    bool b_west = (tiles[currentPosition].west == -1);

    if (b_north != bit(0)) return ErrorCodes::invalid;
    if (b_east != bit(1)) return ErrorCodes::invalid;
    if (b_south != bit(2)) return ErrorCodes::invalid;
    if (b_west != bit(3)) return ErrorCodes::invalid;

    return ErrorCodes::OK;
}

uint16_t Mapping::findNextTarget() {
    if (currentPosition >= MAX_TILES) return UINT16_MAX;

    uint16_t bufferTargetIndex = UINT16_MAX;
    Orientations bufferPriority = Orientations::North;

    if (pathPriority == ErrorCodes::straight) bufferPriority = currentOrientation;
    else if (pathPriority == ErrorCodes::east) bufferPriority = Orientations::East;
    else if (pathPriority == ErrorCodes::south) bufferPriority = Orientations::South;
    else if (pathPriority == ErrorCodes::west) bufferPriority = Orientations::West;

	int16_t n = tiles[currentPosition].north,
        e = tiles[currentPosition].east,
        s = tiles[currentPosition].south,
        w = tiles[currentPosition].west;
    //Buffer for neighboring tile index

    switch (bufferPriority)
    {
    case Orientations::North:
        if      (n != -1 && tiles[n].type == TileType::unexplored) bufferTargetIndex = n;
        else if (e != -1 && tiles[e].type == TileType::unexplored) bufferTargetIndex = e;
        else if (s != -1 && tiles[s].type == TileType::unexplored) bufferTargetIndex = s;
        else if (w != -1 && tiles[w].type == TileType::unexplored) bufferTargetIndex = w;
        break;
    case Orientations::East:
        if      (e != -1 && tiles[e].type == TileType::unexplored) bufferTargetIndex = e;
        else if (s != -1 && tiles[s].type == TileType::unexplored) bufferTargetIndex = s;
        else if (w != -1 && tiles[w].type == TileType::unexplored) bufferTargetIndex = w;
        else if (n != -1 && tiles[n].type == TileType::unexplored) bufferTargetIndex = n;
        break;

    case Orientations::South:
        if      (s != -1 && tiles[s].type == TileType::unexplored) bufferTargetIndex = s;
        else if (w != -1 && tiles[w].type == TileType::unexplored) bufferTargetIndex = w;
        else if (n != -1 && tiles[n].type == TileType::unexplored) bufferTargetIndex = n;
        else if (e != -1 && tiles[e].type == TileType::unexplored) bufferTargetIndex = e;
        break;

    case Orientations::West:
        if      (w != -1 && tiles[w].type == TileType::unexplored) bufferTargetIndex = w;
        else if (n != -1 && tiles[n].type == TileType::unexplored) bufferTargetIndex = n;
        else if (e != -1 && tiles[e].type == TileType::unexplored) bufferTargetIndex = e;
        else if (s != -1 && tiles[s].type == TileType::unexplored) bufferTargetIndex = s;
        break;
    default:
        break;
    }

	if (bufferTargetIndex == UINT16_MAX) {  //no unexplored neighboring tile found, search for any unexplored tile in memory
        //BFS-Search for nearest target
		uint16_t queue[MAX_TILES];
        bool visited[MAX_TILES];        // Markiert bereits überprüfte Felder
        memset(visited, false, sizeof(visited)); // Alles auf false setzen

		uint16_t head = 0, tail = 0; // Zeiger für die Queue

		queue[tail++] = currentPosition; // Startpunkt in die Queue einfügen
		visited[currentPosition] = true; // Startpunkt als besucht markieren

        while (head < tail) {
            uint16_t current = queue[head++];

            // Haben wir ein unerforschtes Feld erreicht? -> Wir sind fertig!
            // Da sich die BFS wie eine Welle ausbreitet, ist das ERSTE gefundene
            // Feld garantiert das mit dem kürzesten echten Fahrweg.
            if (tiles[current].type == TileType::unexplored) {
                bufferTargetIndex = current;
                break; // Schleife abbrechen, Ziel gefunden!
            }

            // Wir dürfen die "Wasserwelle" nur von Feldern aus weiter ausbreiten,
            // die wir auch befahren können (also keine Wände, Löcher, etc.).
            // Unerforschte Felder selbst haben wir ja gerade oben schon abgefangen.
            if (tiles[current].type == TileType::visited ||
                tiles[current].type == TileType::checkpoint ||
                tiles[current].type == TileType::blue ||
                tiles[current].type == TileType::dangerZone ||
                tiles[current].type == TileType::obstacle ||
                current == currentPosition)
            {
                // Alle 6 möglichen Nachbarn (inkl. Rampen) prüfen
                int16_t neighbors[6] = {
                    tiles[current].north, tiles[current].east,
                    tiles[current].south, tiles[current].west,
                    tiles[current].up, tiles[current].down
                };

                for (uint8_t i = 0; i < 6; i++) {
                    int16_t n = neighbors[i];

                    // Wenn der Nachbar existiert, wir noch nicht dort waren
                    // und es kein schwarzes Feld oder statisches Hindernis ist:
                    if (n != -1 && !visited[n] &&
                        tiles[n].type != TileType::inactive &&
                        tiles[n].type != TileType::black)
                    {
                        visited[n] = true;     // Als besucht markieren
                        queue[tail++] = n;     // Hinten an die Queue anhängen
                    }
                }
            }
        }

        // Wenn die Queue leer ist und nichts gefunden wurde, ist das Labyrinth fertig
        if (bufferTargetIndex == UINT16_MAX) {
            _RETURN_HOME = true;
            bufferTargetIndex = 0; // Zurück zum Start (Index 0)
        }
    }
    //Serial.println("Target: x:" + String(tiles[bufferTargetIndex].x) + " y:" + String(tiles[bufferTargetIndex].y) +" z:" + String(tiles[bufferTargetIndex].z));
    return bufferTargetIndex;
}

uint16_t Mapping::findTileByCoordinate(int16_t x, int16_t y, int16_t z) {
    for (uint16_t i = 0; i < MAX_TILES; i++) {
        // Ignoriere inaktive Felder und prüfe die Koordinaten
        if (tiles[i].type != TileType::inactive &&
            tiles[i].x == x && tiles[i].y == y && tiles[i].z == z) {
            return i; // Feld existiert bereits, gib den Index zurück
        }
    }
    return UINT16_MAX; // Feld existiert noch nicht
}

#ifdef _MSC_VER
#pragma endregion helpers
#pragma region Movement
#endif

ErrorCodes Mapping::Turn(Orientations direction) {
    if ((uint8_t)direction > (uint8_t) Orientations::West) return ErrorCodes::invalid;  //Check for valid Orientation
    currentOrientation = direction; //Set Direction
    return ErrorCodes::OK;
}

ErrorCodes Mapping::Move(bool direction) {
    if (currentPosition >= MAX_TILES) return ErrorCodes::invalid;

    int16_t nextTile = currentPosition;
    Orientations helpOrientation = currentOrientation;

    if (!direction) { //transform according to direction (false...backward)
        uint8_t temp = static_cast<uint8_t>(helpOrientation);
        temp = (temp + 2) & 0x03;
        helpOrientation = static_cast<Orientations>(temp);
    }

    switch (helpOrientation) {
    case Orientations::North:
        nextTile = tiles[currentPosition].north;
        break;

    case Orientations::East:
        nextTile = tiles[currentPosition].east;
        break;

    case Orientations::South:
        nextTile = tiles[currentPosition].south;
        break;

    case Orientations::West:
        nextTile = tiles[currentPosition].west;
        break;

    default:
        return ErrorCodes::invalid;
        break;
    }

    //Check if tile is valid
    if (nextTile == -1) return ErrorCodes::wall;
    currentPosition = nextTile;
    
    //Serial.println("x: " + String(tiles[currentPosition].x) + " y: " + String(tiles[currentPosition].y) + " z: " + String(tiles[currentPosition].z));
    
    return ErrorCodes::OK;
}

ErrorCodes Mapping::Ramp(ErrorCodes direction, uint8_t length) {
    if(_layerSetting == ErrorCodes::single) direction = ErrorCodes::same;
    if(_rampSetting == ErrorCodes::single) length = 1;

    if(length <= 0) return ErrorCodes::invalid;
    
    //Use existing ramps:
    if(tiles[currentPosition]. down != -1){
        currentPosition = tiles[currentPosition].down;
        return Move(true);
    } else if(tiles[currentPosition].up != -1){
        currentPosition = tiles[currentPosition].up;
        return Move(true);
    }

    uint16_t nextPos = findNextEmptyMemory();
	if (nextPos == UINT16_MAX) return ErrorCodes::Overflow;   //No Memory left for new tile

    // Pointer to Member + dX Setup
    int16_t Tile::* forwardDir;
    int16_t Tile::* backDir;
    int16_t dx = 0, dy = 0;

    switch(currentOrientation){
        case Orientations::North: forwardDir = &Tile::north; backDir = &Tile::south; dy =  1; break;
        case Orientations::East:  forwardDir = &Tile::east;  backDir = &Tile::west;  dx =  1; break;
        case Orientations::South: forwardDir = &Tile::south; backDir = &Tile::north; dy = -1; break;
        case Orientations::West:  forwardDir = &Tile::west;  backDir = &Tile::east;  dx = -1; break;
        default: return ErrorCodes::invalid;
    }

    //Set Coords of next Pos
    tiles[nextPos].x = tiles[currentPosition].x;
    tiles[nextPos].y = tiles[currentPosition].y;

    //Length = 1 and dir == same
    // if (direction == ErrorCodes::same && length == 0) {
    //     tiles[nextPos].type = TileType::unexplored;
    //     tiles[currentPosition].weight = COST_RAMP;
    //     tiles[currentPosition].up = currentPosition;
    //     tiles[currentPosition].down = currentPosition;
    //     tiles[currentPosition].type = TileType::visited;
    //     tiles[nextPos].z = tiles[currentPosition].z;

    //     // Verbindung über Pointer
    //     tiles[currentPosition].*forwardDir = nextPos;
    //     tiles[nextPos].*backDir = currentPosition;
    //     currentPosition = nextPos;
    //     return ErrorCodes::OK;
    // }

    //Change Z level
    if (direction == ErrorCodes::up) {
        tiles[currentPosition].up = nextPos;
        tiles[nextPos].down = currentPosition;
        tiles[nextPos].z = tiles[currentPosition].z + 1;
    } else if(direction == ErrorCodes::down) {
        tiles[currentPosition].down = nextPos;
        tiles[nextPos].up = currentPosition;
        tiles[nextPos].z = tiles[currentPosition].z - 1;
    } else if(direction == ErrorCodes::same) {
        tiles[currentPosition].up = nextPos;
        tiles[nextPos].down = currentPosition;
        tiles[nextPos].z = tiles[currentPosition].z;
    } else {
        return ErrorCodes::invalid;
    }

    //Regular Ramp:
    tiles[currentPosition].type = TileType::visited;
    tiles[nextPos].type = TileType::visited;
    tiles[currentPosition].weight = COST_RAMP;
    tiles[nextPos].weight = COST_RAMP;

    uint16_t realPos = findNextEmptyMemory();
    if (realPos == UINT16_MAX) return ErrorCodes::Overflow; // Check if overflow

    tiles[realPos].type = TileType::unexplored;
    tiles[realPos].z = tiles[nextPos].z;

    // Verbindung über Pointer herstellen
    tiles[nextPos].*forwardDir = realPos;
    tiles[realPos].*backDir = nextPos;

    // Koordinaten über Deltas berechnen
    tiles[realPos].x = tiles[nextPos].x + (dx * length);
    tiles[realPos].y = tiles[nextPos].y + (dy * length);

    currentPosition = realPos;
    return ErrorCodes::OK;
}

void Mapping::RollbackOne(){
    if(pathIndex >= 1) pathIndex-= 1;
    uint16_t pos = currentPosition;
    Move(false);

    if(tiles[pos].weight == COST_RAMP) return;

    //Get Coordinates
    int16_t x = tiles[pos].x, y = tiles[pos].y, z = tiles[pos].z;

    // Pointer to back and front Tiles
    int16_t Tile::* forwardDir = 0;
    int16_t Tile::* backwardDir = 0;

    //get Position depending on Orientation
    switch (currentOrientation) {
        case Orientations::North: forwardDir = &Tile::north; backwardDir = &Tile::south; break;
        case Orientations::East:  forwardDir = &Tile::east;  backwardDir = &Tile::west;  break;
        case Orientations::South: forwardDir = &Tile::south; backwardDir = &Tile::north; break;
        case Orientations::West:  forwardDir = &Tile::west;  backwardDir = &Tile::east;  break;
        default: return; // return for safety
    }

    // remove wrong Tile 
    int16_t frontIdx = tiles[pos].*forwardDir;
    if (frontIdx != -1) {
        //Serial.println("Delete Tile");
        tiles[frontIdx] = Tile(); 
    }

    // Reset Tile
    tiles[pos] = Tile();
    tiles[pos].type = TileType::unexplored;
    tiles[pos].x = x;
    tiles[pos].y = y;
    tiles[pos].z = z;
    
    //Set Link to old position 
    tiles[pos].*backwardDir = currentPosition;
}

#ifdef _MSC_VER
#pragma endregion Movement
#pragma region Tile
#endif

ErrorCodes Mapping::SetTile(uint8_t walls, TileType floor) {
	if (currentPosition >= MAX_TILES) return ErrorCodes::invalid;
    //check if tile is activated:
    if (tiles[currentPosition].type == TileType::inactive) return ErrorCodes::invalid;

    uint8_t absWalls = correctWallinfo(walls, currentOrientation);
    if (absWalls & 1 << 6) return ErrorCodes::invalid; //Check for error in correctWallInfo

    //Check for valid floor
    if (floor < TileType::visited || floor > TileType::black) return ErrorCodes::invalid;

    //check checkpoint
    if (floor == TileType::checkpoint) {
        if (resetCounter > 0 || currentPosition != lastCheckpointPosition) {
            lastCheckpointPosition = currentPosition;
			memcpy(backupTiles, tiles, sizeof(tiles));
        }

    }

    //Check if tile was already visited:
    if (tiles[currentPosition].type != TileType::unexplored) {   //already visited
        if(floor == TileType::blue){
            tiles[currentPosition].type = TileType::blue;
            tiles[currentPosition].weight = COST_BLUE;
        }
        return compareWalls(absWalls);
    }

    //if not already visited: -------------------------------------------------

    //Floor + Cost according to floor
    tiles[currentPosition].type = floor;
    if (floor == TileType::black) {
        tiles[currentPosition].weight = 255;
    }
    else if (floor == TileType::blue) {
        tiles[currentPosition].weight = COST_BLUE;
    }
    else if (floor == TileType::dangerZone) {
        tiles[currentPosition].weight = COST_DANGER_ZONE;
    }
    else if (floor == TileType::obstacle) {
        tiles[currentPosition].weight = COST_OBSTACLE;
    }
    else tiles[currentPosition].weight = COST_REGULAR;

    uint16_t bufferIndex = 0;
    //check Walls -> activate neighboring tiles depending on walls and exploration status
    //start with Northern Tile:
    if ((absWalls & (1 << 0)) == 0 && tiles[currentPosition].north == -1) {
        uint16_t nx = tiles[currentPosition].x,
			ny = tiles[currentPosition].y + 1,
			nz = tiles[currentPosition].z;
		uint16_t existingNode = findTileByCoordinate(nx, ny, nz);

        if (existingNode != UINT16_MAX) {   //Tile exists, set Relation
            tiles[currentPosition].north = existingNode;
            tiles[existingNode].south = currentPosition;
        }
        else {
            bufferIndex = findNextEmptyMemory();

            tiles[currentPosition].north = bufferIndex;
            tiles[bufferIndex].south = currentPosition;
            tiles[bufferIndex].type = TileType::unexplored;

            tiles[bufferIndex].z = tiles[currentPosition].z;
            tiles[bufferIndex].x = tiles[currentPosition].x;
            tiles[bufferIndex].y = tiles[currentPosition].y + 1;
        }
    }
    //South
    if ((absWalls & (1 << 2)) == 0 && tiles[currentPosition].south == -1) {
        uint16_t nx = tiles[currentPosition].x,
            ny = tiles[currentPosition].y - 1,
            nz = tiles[currentPosition].z;
        uint16_t existingNode = findTileByCoordinate(nx, ny, nz);

        if(existingNode != UINT16_MAX) {   //Tile exists, set Relation
            tiles[currentPosition].south = existingNode;
            tiles[existingNode].north = currentPosition;
        }
        else {
            bufferIndex = findNextEmptyMemory();

            tiles[currentPosition].south = bufferIndex;
            tiles[bufferIndex].north = currentPosition;
            tiles[bufferIndex].type = TileType::unexplored;

            tiles[bufferIndex].z = tiles[currentPosition].z;
            tiles[bufferIndex].x = tiles[currentPosition].x;
            tiles[bufferIndex].y = tiles[currentPosition].y - 1;
        }
    }
    //East
    if ((absWalls & (1 << 1)) == 0 && tiles[currentPosition].east == -1) {
        uint16_t nx = tiles[currentPosition].x + 1,
            ny = tiles[currentPosition].y,
            nz = tiles[currentPosition].z;
        uint16_t existingNode = findTileByCoordinate(nx, ny, nz);

        if (existingNode != UINT16_MAX) {   //Tile exists, set Relation
            tiles[currentPosition].east = existingNode;
            tiles[existingNode].west = currentPosition;
        }
        else {
            bufferIndex = findNextEmptyMemory();

            tiles[currentPosition].east = bufferIndex;
            tiles[bufferIndex].west = currentPosition;
            tiles[bufferIndex].type = TileType::unexplored;

            tiles[bufferIndex].z = tiles[currentPosition].z;
            tiles[bufferIndex].x = tiles[currentPosition].x + 1;
            tiles[bufferIndex].y = tiles[currentPosition].y;
        }
    }
    //West
    if ((absWalls & (1 << 3)) == 0 && tiles[currentPosition].west == -1) {
        uint16_t nx = tiles[currentPosition].x - 1,
            ny = tiles[currentPosition].y,
            nz = tiles[currentPosition].z;
        uint16_t existingNode = findTileByCoordinate(nx, ny, nz);

        if (existingNode != UINT16_MAX) {   //Tile exists, set Relation
            tiles[currentPosition].west = existingNode;
            tiles[existingNode].east = currentPosition;
        }
        else {
            bufferIndex = findNextEmptyMemory();

            tiles[currentPosition].west = bufferIndex;
            tiles[bufferIndex].east = currentPosition;
            tiles[bufferIndex].type = TileType::unexplored;

            tiles[bufferIndex].z = tiles[currentPosition].z;
            tiles[bufferIndex].x = tiles[currentPosition].x - 1;
            tiles[bufferIndex].y = tiles[currentPosition].y;
        }
    }

    return ErrorCodes::OK;
}

#ifdef _MSC_VER
#pragma endregion Tile
#pragma region Options / Navigation
#endif

ErrorCodes Mapping::SetPriority(ErrorCodes priority) {
    if (priority == ErrorCodes::straight || priority == ErrorCodes::north || priority == ErrorCodes::east || priority == ErrorCodes::south || priority == ErrorCodes::west) {
        pathPriority = priority;
        return ErrorCodes::OK;
    }
    return ErrorCodes::invalid;
}

Instructionset Mapping::GetInstruction() {
    if (_BumperTriggered || path[pathIndex] == Instructionset::FinishedInstructions) {
        if (_RETURN_HOME && currentPosition == 0) {
            return Instructionset::MazeFinished;
        }

        _BumperTriggered = false;

        targetPosition = findNextTarget();

        std::cout << "currentPos: " << currentPosition << " Target: "<< targetPosition;

        //Pathfinding - A* Algorithm Implementation
        initLists(targetPosition);

        // Initialize start node
        uint16_t openCount = 1;
        open[0].nodeIndex = currentPosition;
        open[0].g = 0;
        open[0].f = heuristic3D(tiles[currentPosition], tiles[targetPosition]);

        // Parent tracking for path reconstruction (stores index of parent tile)
        int16_t parent[MAX_TILES];
        for (uint16_t i = 0; i < MAX_TILES; i++) parent[i] = -1;

        // A* Main Loop - continues until target is found or open list is empty
        while (closed[targetPosition] == false && openCount > 0) {

            // Find node with lowest F-cost in open list
            uint16_t lowestIndex = 0;
            uint16_t lowestF = open[0].f;
            for (uint16_t i = 1; i < openCount; i++) {
                if (open[i].f < lowestF) {
                    lowestF = open[i].f;
                    lowestIndex = i;
                }
            }

            // Get current node and move to closed list
            uint16_t currentNode = open[lowestIndex].nodeIndex;
            uint16_t currentG = open[lowestIndex].g;
            closed[currentNode] = true;

            // Remove from open list (shift array)
            for (uint16_t i = lowestIndex; i < openCount - 1; i++) {
                open[i] = open[i + 1];
            }
            openCount--;

            // Check if we reached the target
            if (currentNode == targetPosition) break;

            // Process all 6 possible neighbors (N, E, S, W, Up, Down)
            int16_t neighbors[6] = {
                tiles[currentNode].north,
                tiles[currentNode].east,
                tiles[currentNode].south,
                tiles[currentNode].west,
                tiles[currentNode].up,    // Ramp up
                tiles[currentNode].down   // Ramp down
            };

            for (uint8_t dir = 0; dir < 6; dir++) {
                int16_t neighborIndex = neighbors[dir];

                // Skip if no neighbor in this direction or already in closed list
                if (neighborIndex == -1 || closed[neighborIndex]) continue;

                // Skip inactive or black tiles (impassable)
                if (tiles[neighborIndex].type == TileType::inactive ||
                    tiles[neighborIndex].type == TileType::black) continue;

                // Calculate G-cost (movement cost from start to this neighbor)
                uint16_t edgeCost = tiles[neighborIndex].weight;

                // Apply higher cost for ramp transitions (dir 4=up, dir 5=down)
                if (dir == 4 || dir == 5) {
                    edgeCost += COST_RAMP;  // Add ramp penalty to discourage vertical movement
                }

                uint16_t tentativeG = currentG + edgeCost;

                // Check if neighbor is already in open list
                bool inOpenList = false;
                uint16_t openListIndex = 0;
                for (uint16_t i = 0; i < openCount; i++) {
                    if (open[i].nodeIndex == neighborIndex) {
                        inOpenList = true;
                        openListIndex = i;
                        break;
                    }
                }

                // If not in open list OR found a better path to this neighbor
                if (!inOpenList || tentativeG < open[openListIndex].g) {
                    // Calculate F-cost (F = G + H, where H is heuristic)
                    uint16_t h = heuristic3D(tiles[neighborIndex], tiles[targetPosition]);
                    uint16_t f = tentativeG + h;

                    if (inOpenList) {
                        // Update existing entry with better path
                        open[openListIndex].g = tentativeG;
                        open[openListIndex].f = f;
                        parent[neighborIndex] = currentNode;
                    }
                    else {
                        // Add new entry to open list
                        if (openCount < MAX_TILES) {
                            open[openCount].nodeIndex = neighborIndex;
                            open[openCount].g = tentativeG;
                            open[openCount].f = f;
                            parent[neighborIndex] = currentNode;
                            openCount++;
                        }
                    }
                }
            }
        }

        // Path Reconstruction - backtrack from target to start using parent array
        uint16_t pathLength = 0;
        uint16_t tempPath[MAX_INSTRUCTIONS];

        if (closed[targetPosition]) {  // Only reconstruct if target was reached
            int16_t current = targetPosition;

            // Build path backwards from target to start
            while (current != -1 && current != currentPosition && pathLength < MAX_INSTRUCTIONS) {
                tempPath[pathLength++] = current;
                current = parent[current];
            }

            // Convert path to movement instructions (reverse order)
            pathIndex = 0;
            
            bool inRamp = false;
            Orientations simOrientation = currentOrientation; 

            // IMPORTANT: -2 instead of -: Worst-Case 2 Instructions (Turn + Forward) for each Loop
            for (int16_t i = pathLength - 1; i >= 0 && pathIndex < MAX_INSTRUCTIONS - 2; i--) {
                uint16_t nextTile = tempPath[i];
                uint16_t fromTile = (i == pathLength - 1) ? currentPosition : tempPath[i + 1];

                // Überspringt den rein vertikalen Sprung im Graphen. Der nachfolgende 
                // horizontale Sprung generiert dann genau EINEN D_Forward Befehl.
                if (tiles[fromTile].up == nextTile || tiles[fromTile].down == nextTile) {
                    inRamp = true;
                    pathIndex--;
                    path[pathIndex++] = Instructionset::ramp;

                    // Generiere exakt EINEN Fahrbefehl für die gesamte Rampensequenz
                    //path[pathIndex++] = Instructionset::D_Forward;
                    continue; 
                }

                if(inRamp){
                    if(tiles[nextTile].weight != COST_RAMP){
                        inRamp = false;
                    }
                    continue;
                }

                // Determine direction from fromTile to nextTile
                if (tiles[fromTile].north == nextTile) {
                    if (simOrientation != Orientations::North) {
                        path[pathIndex++] = Instructionset::T_North;
                        simOrientation = Orientations::North; // Simulierte Richtung aktualisieren
                    }
                    path[pathIndex++] = Instructionset::D_Forward;
                }
                else if (tiles[fromTile].east == nextTile) {
                    if (simOrientation != Orientations::East) {
                        path[pathIndex++] = Instructionset::T_East;
                        simOrientation = Orientations::East;
                    }
                    path[pathIndex++] = Instructionset::D_Forward;
                }
                else if (tiles[fromTile].south == nextTile) {
                    if (simOrientation != Orientations::South) {
                        path[pathIndex++] = Instructionset::T_South;
                        simOrientation = Orientations::South;
                    }
                    path[pathIndex++] = Instructionset::D_Forward;
                }
                else if (tiles[fromTile].west == nextTile) {
                    if (simOrientation != Orientations::West) {
                        path[pathIndex++] = Instructionset::T_West;
                        simOrientation = Orientations::West;
                    }
                    path[pathIndex++] = Instructionset::D_Forward;
                }
            }

            // Mark end of instructions
            path[pathIndex] = Instructionset::FinishedInstructions;
            pathIndex = 1;  // Reset to start of path

            //for (int i = 0; path[i] != Instructionset::FinishedInstructions; i++)
            //{
            //    Serial.println("A: " + String((uint8_t)path[i]));
            //}
            
            return path[0];
        }
        else {
            // Target not reachable - return error
            return Instructionset::undefined;
        }
    }
    else {
        pathIndex += 1;
        return path[pathIndex - 1];
    }
    //In case of error
    return Instructionset::undefined;
}

void Mapping::Reset(void) {
    pathIndex = 0;
    path[0] = Instructionset::FinishedInstructions;

    //Reset Tiles:
    for (uint16_t i = 0; i < MAX_TILES; i++)
    {
		tiles[i] = Tile();   //Reset all tile data to default (inactive)
    }

    //set first tile as start
    tiles[0].type = TileType::unexplored;
    currentPosition = 0;

    resetCounter = 0;
    lastCheckpointPosition = currentPosition;
    memcpy(backupTiles, tiles, sizeof(tiles));
}

// void Mapping::PrintInternalMap() {
//     int minX = 0, maxX = 0, minY = 0, maxY = 0;

//     // 1. Finde die maximalen Grenzen der bisher entdeckten Karte heraus
//     for (uint16_t i = 0; i < MAX_TILES; i++) {
//         if (tiles[i].type != TileType::inactive) {
//             if (tiles[i].x < minX) minX = tiles[i].x;
//             if (tiles[i].x > maxX) maxX = tiles[i].x;
//             if (tiles[i].y < minY) minY = tiles[i].y;
//             if (tiles[i].y > maxY) maxY = tiles[i].y;
//         }
//     }

//     std::cout << "\n=== INTERNE ROBOTER KARTE ===\n";

//     // 2. Zeichne das Grid (Y rückwärts, da in der Konsole oben = y-max ist)
//     for (int y = maxY; y >= minY; y--) {
//         for (int x = minX; x <= maxX; x++) {
//             bool found = false;

//             for (uint16_t i = 0; i < MAX_TILES; i++) {
//                 if (tiles[i].type != TileType::inactive && tiles[i].x == x && tiles[i].y == y) {
//                     found = true;
//                     if (i == currentPosition) {
//                         std::cout << "R "; // Aktuelle Roboterposition
//                     }
//                     else if (tiles[i].type == TileType::unexplored) {
//                         std::cout << "? "; // Gesehen (Wand offen), aber noch nicht befahren
//                     }
//                     else if (tiles[i].type == TileType::visited) {
//                         std::cout << ". "; // Normales, besuchtes Feld
//                     }
//                     else if (tiles[i].type == TileType::black) {
//                         std::cout << "B "; // Schwarzes Feld
//                     }
//                     else if (tiles[i].type == TileType::obstacle) {
//                         std::cout << "O "; // Hindernis
//                     }
//                     else if (tiles[i].type == TileType::checkpoint) {
//                         std::cout << "C "; // Checkpoint
//                     }
//                     else {
//                         std::cout << "+ "; // Sonstiges (Blau etc.)
//                     }
//                     break; // Feld gefunden, breche innere Schleife ab
//                 }
//             }
//             if (!found) {
//                 std::cout << "  "; // Nichts an dieser Koordinate
//             }
//         }
//         std::cout << "\n";
//     }
//     std::cout << "=============================\n";
// }

ErrorCodes Mapping::Bumper(bool reset) {
    if (currentPosition >= MAX_TILES) return ErrorCodes::invalid;

    tiles[currentPosition].weight += COST_OBSTACLE / BUMPER_RESET_COUNTER;  //Increase Tile Traversing Cost

    if(!reset) return ErrorCodes::OK;

    _BumperTriggered = true;    //Set Bumper flag
    int16_t neighborIndex = -1;
    // Finde heraus, in welche Richtung wir gerade schauen und kappe die Verbindung
    switch (currentOrientation) {
    case Orientations::North:
        neighborIndex = tiles[currentPosition].north;
        tiles[currentPosition].north = -1; // Wand setzen
        if (neighborIndex != -1) tiles[neighborIndex].south = -1; // Gegenverbindung kappen
        break;
    case Orientations::East:
        neighborIndex = tiles[currentPosition].east;
        tiles[currentPosition].east = -1;
        if (neighborIndex != -1) tiles[neighborIndex].west = -1;
        break;
    case Orientations::South:
        neighborIndex = tiles[currentPosition].south;
        tiles[currentPosition].south = -1;
        if (neighborIndex != -1) tiles[neighborIndex].north = -1;
        break;
    case Orientations::West:
        neighborIndex = tiles[currentPosition].west;
        tiles[currentPosition].west = -1;
        if (neighborIndex != -1) tiles[neighborIndex].east = -1;
        break;
    }

    // Optionaler Memory-Clean-Up:
    // Wenn das abgetrennte Feld "unexplored" war und nun gar keine Verbindungen mehr hat,
    // können wir es wieder auf "inactive" setzen, um Speicher freizugeben.
    if (neighborIndex != -1 && tiles[neighborIndex].type == TileType::unexplored) {
        if (tiles[neighborIndex].north == -1 && tiles[neighborIndex].east == -1 &&
            tiles[neighborIndex].south == -1 && tiles[neighborIndex].west == -1) {
            tiles[neighborIndex] = Tile(); // Reset auf Standardwerte
        }
    }

    return ErrorCodes::OK; // A* wird im nächsten Loop automatisch einen neuen Weg berechnen!
}

ErrorCodes Mapping::RestartCheckpoint() {
    // 1. Saubere Karte und Position aus dem Backup wiederherstellen
    memcpy(tiles, backupTiles, sizeof(tiles));
    currentPosition = lastCheckpointPosition;

    // 2. Ausrichtung überschreiben (da der Schiedsrichter den Roboter neu hinstellt)
    currentOrientation = Orientations::North;

    // 3. Alten Pfad löschen, damit der A* neu rechnet
    pathIndex = 0;
    path[0] = Instructionset::FinishedInstructions;

    // 4. Prüfen, ob wir die maximalen Versuche erreicht haben
    if (resetCounter >= OPTION_MAX_RESETS) {
        _RETURN_HOME = true; // Zwingt den A*, den Weg zum Feld 0 zu suchen
    }
    resetCounter++;
    return ErrorCodes::OK;
}

ErrorCodes Mapping::SetVictim(){
    //Check if even valid tile Type
    if(!(tiles[currentPosition].type == TileType::visited || tiles[currentPosition].type == TileType::unexplored)) return ErrorCodes::invalid;

    //Check if already found
    if (tiles[currentPosition].victim) return ErrorCodes::already_found;
    tiles[currentPosition].victim = true;
    return ErrorCodes::OK;
}