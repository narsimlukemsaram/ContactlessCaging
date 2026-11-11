// Acoustic GS-PAT demonstration with keyboard-controlled focal point.
// TurtleBot3 motion control and PhaseSpace ROS 2 topics are provided
// in the companion ros2_ws/contactlesscaging_control package.
// Check calibration, PAT hardware and per-board phase packets before use.
// AcoustoBots_GSPAT_FourArray_Caging.cpp
// Build the same way you build your 2-board program.
// Keyboard: WASD (x/y), Q/E (z), F/B/L/R/H/D/b forward/back/left/right/hinge on/off/bead

#include <src/UDPDriver.h>
#include <AsierInhoUDP.h>
#include <GSPAT_SolverV2.h>
#include <Helper/HelperMethods.h>
#include <Helper/microTimer.h>
#include <vector>
#include <string.h>
#include <conio.h>
#include <math.h>
#include <stdio.h>
#ifdef _WIN32
#include <Windows.h>
#endif

UDPDriver udp;
int packetSize = 128;

void print(const char* s) { printf("%s\n", s); }

int main() {
    // ---------- CONFIG ----------
    const int numPatBoards = 4;                  // Left, Right, Front, Back
    const int numMonaRobots = 4;                  // legacy optional UDP-controlled robots; NOT TurtleBot3 ROS 2
    const float distance = 0.10f;              // separation between opposing boards (meters)

    // Board IDs in the order of the transforms below:
    // {Left, Right, Front, Back}
    int boardIDs[numPatBoards] = { 1, 4, 2, 3 };   // <<< EDIT to your IDs

    // PC (server) and client IPs
    int socketPort = 54007;
    char serverIP[] = "192.168.0.185";             // <<< EDIT if needed

    // PAT board IPs (same order as boardIDs & transforms)
    const char* PAT_addresses[numPatBoards] = {
        "192.168.0.178",   // Left  (e.g., Yellow Master)
        "192.168.0.138",   // Right
        "192.168.0.144",   // Front
        "192.168.0.124"    // Back
    }; // <<< EDIT to your four PAT boards

    // Mona/robot IPs (optional – used for motion/hinge/bead commands)
    const char* Mona_addresses[numMonaRobots] = {
        "192.168.0.195",   // Robot 1 (Left)
        "192.168.0.191",   // Robot 2 (Right)
        "192.168.0.181",   // Robot 3 (Front)
        "192.168.0.149"    // Robot 4 (Back)
    }; // <<< EDIT / trim as needed

    // --- Board-to-world transforms (row-major 4x4 for each board) ---
    // Convention: the original PAT local +Z is the board's normal.
    // We place four boards on a square around the origin, all facing the origin:
    // Left  @ x = -distance/2, facing +X
    // Right @ x = +distance/2, facing -X
    // Front @ y = +distance/2, facing -Y
    // Back  @ y = -distance/2, facing +Y
    //
    // Rotations chosen so that local +Z points along the facing direction.
    // If your physical mounting differs, tweak the signs below (only the 3x3 part and translation).
        float matBoardToWorld[16 * numPatBoards] = {
        // LEFT: local +Z -> world +X, board centre x = -distance/2
         0,  0,  1, -distance * 0.5f,
         0,  1,  0,  0,
        -1,  0,  0,  0,
         0,  0,  0,  1,

        // RIGHT: local +Z -> world -X, board centre x = +distance/2
         0,  0, -1,  distance * 0.5f,
         0,  1,  0,  0,
         1,  0,  0,  0,
         0,  0,  0,  1,

        // FRONT: local +Z -> world -Y, board centre y = +distance/2
         1,  0,  0,  0,
         0,  0, -1,  distance * 0.5f,
         0,  1,  0,  0,
         0,  0,  0,  1,

        // BACK: local +Z -> world +Y, board centre y = -distance/2
         1,  0,  0,  0,
         0,  0,  1, -distance * 0.5f,
         0, -1,  0,  0,
         0,  0,  0,  1
    };

    // ---------- INIT ----------
    AsierInhoUDP::RegisterPrintFuncs(print, print, print);
    AsierInhoUDP::AsierInhoUDPBoard* driver = AsierInhoUDP::createAsierInho();

    const int numTransducers = 64 * numPatBoards;

    udp.initialise(serverIP, socketPort);
    std::vector<int> PATs(numPatBoards), Monas(numMonaRobots);
    for (int i = 0; i < numPatBoards; i++) PATs[i] = udp.addClient(PAT_addresses[i]);
    for (int i = 0; i < numMonaRobots; i++) Monas[i] = udp.addClient(Mona_addresses[i]);

    // Solver
    GSPAT_V2::RegisterPrintFuncs(print, print, print);
    GSPAT::Solver* solver = GSPAT_V2::createSolver(numTransducers);

    if (!driver->connect(numPatBoards, boardIDs, matBoardToWorld)) {
        printf("Failed to connect to PAT boards.\n");
        return -1;
    }

    // Read board parameters for all connected boards
    std::vector<float> transducerPositions(numTransducers * 3);
    std::vector<float> transducerNormals(numTransducers * 3);
    std::vector<float> amplitudeAdjust(numTransducers);
    std::vector<int>   mappings(numTransducers);
    std::vector<int>   phaseDelays(numTransducers);
    int numDiscreteLevels = 0;

    driver->readParameters(transducerPositions.data(), transducerNormals.data(),
        mappings.data(), phaseDelays.data(),
        amplitudeAdjust.data(), &numDiscreteLevels);

    solver->setBoardConfig(transducerPositions.data(), transducerNormals.data(),
        mappings.data(), phaseDelays.data(),
        amplitudeAdjust.data(), numDiscreteLevels);

    // ---------- TARGET(S) ----------
    const size_t numPoints = 1; // single trap; increase if needed
    const size_t numGeometries = 1;

    float curPos[4 * numGeometries * numPoints];
    float amplitude[numGeometries * numPoints];

    // Start at the center of the cage (0,0,0.05)
    for (size_t g = 0; g < numGeometries; ++g) {
        for (size_t p = 0; p < numPoints; ++p) {
            curPos[4 * g * numPoints + 4 * p + 0] = 0.0f;   // x
            curPos[4 * g * numPoints + 4 * p + 1] = 0.0f;   // y
            curPos[4 * g * numPoints + 4 * p + 2] = 0.05f;  // z (height)
            curPos[4 * g * numPoints + 4 * p + 3] = 1.0f;
            amplitude[g * numPoints + p] = 40000.0f;
        }
    }

    unsigned char* msg = nullptr;
    float mI[16] = { 1,0,0,0,  0,1,0,0,  0,0,1,0,  0,0,0,1 };
    float matrix[16 * numPoints];
    for (size_t i = 0; i < numPoints; i++) memcpy(&matrix[16 * i], mI, sizeof(mI));

    // Compute & push first pattern to all boards
    {
        GSPAT::Solution* solution = solver->createSolution(numPoints, numGeometries, true, curPos, amplitude, matrix, matrix);
        solver->compute(solution);
        solution->finalMessages(&msg);
        for (int i = 0; i < numPatBoards; i++) udp.sendString(&msg[packetSize * i], packetSize, PATs[i]);
        solver->releaseSolution(solution);
        printf("Trap at (%0.4f,%0.4f,%0.4f)\n", curPos[0], curPos[1], curPos[2]);
    }

    // ---------- INTERACTIVE LOOP ----------
    bool finished = false, commit = false;
    while (!finished) {
        int k = _getch();
        switch (k) {
        case 'a': for (size_t g = 0; g < numGeometries; g++) for (size_t p = 0; p < numPoints; p++) curPos[4 * g * numPoints + 4 * p + 0] += 0.0005f; commit = true; break;
        case 'd': for (size_t g = 0; g < numGeometries; g++) for (size_t p = 0; p < numPoints; p++) curPos[4 * g * numPoints + 4 * p + 0] -= 0.0005f; commit = true; break;
        case 'w': for (size_t g = 0; g < numGeometries; g++) for (size_t p = 0; p < numPoints; p++) curPos[4 * g * numPoints + 4 * p + 1] += 0.0005f; commit = true; break;
        case 's': for (size_t g = 0; g < numGeometries; g++) for (size_t p = 0; p < numPoints; p++) curPos[4 * g * numPoints + 4 * p + 1] -= 0.0005f; commit = true; break;
        case 'q': for (size_t g = 0; g < numGeometries; g++) for (size_t p = 0; p < numPoints; p++) curPos[4 * g * numPoints + 4 * p + 2] += 0.00025f; commit = true; break;
        case 'e': for (size_t g = 0; g < numGeometries; g++) for (size_t p = 0; p < numPoints; p++) curPos[4 * g * numPoints + 4 * p + 2] -= 0.00025f; commit = true; break;

            // Robot side-commands (optional)
        case 'F': for (int i = 0; i < numMonaRobots; i++) udp.sendByte('F', Monas[i]); break;
        case 'B': for (int i = 0; i < numMonaRobots; i++) udp.sendByte('B', Monas[i]); break;
        case 'L': for (int i = 0; i < numMonaRobots; i++) udp.sendByte('L', Monas[i]); break;
        case 'R': for (int i = 0; i < numMonaRobots; i++) udp.sendByte('R', Monas[i]); break;
        case 'H': for (int i = 0; i < numMonaRobots; i++) udp.sendByte('H', Monas[i]); printf("Hinge ON\n"); break;
        case 'D': for (int i = 0; i < numMonaRobots; i++) udp.sendByte('D', Monas[i]); printf("Hinge OFF\n"); break;
        case 'b': for (int i = 0; i < numMonaRobots; i++) udp.sendByte('b', Monas[i]); printf("Bead ON\n"); break;

        case '1': commit = false; break; // ignore
        case 'x':
        case 'X': finished = true; break;
        default: break;
        }

        if (commit) {
            GSPAT::Solution* newSol = solver->createSolution(numPoints, numGeometries, true, curPos, amplitude, matrix, matrix);
            solver->compute(newSol);
            newSol->finalMessages(&msg);
            for (int i = 0; i < numPatBoards; i++) udp.sendString(&msg[packetSize * i], packetSize, PATs[i]);
            solver->releaseSolution(newSol);
            printf("Trap at (%0.4f,%0.4f,%0.4f)\n", curPos[0], curPos[1], curPos[2]);
            commit = false;
        }
    }

    // ---------- CLEANUP ----------
    driver->turnTransducersOff();
#ifdef _WIN32
    Sleep(100);
#endif
    driver->disconnect();
    delete driver;
    delete solver;
    return 0;
}