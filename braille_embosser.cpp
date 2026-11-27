#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;

// START IQ MACROS
#define waitUntil(condition)                                             \
  do {                                                                   \
    wait(5, msec);                                                       \
  } while (!(condition))

#define repeat(iterations)                                               \
  for (int iterator = 0; iterator < iterations; iterator++)
// END IQ MACROS

// Robot configuration code.
inertial BrainInertial = inertial();
motor MotorFeed = motor(PORT9, true);
motor MotorX = motor(PORT10, false);
motor MotorZ = motor(PORT7, true);
optical OpticalPaper = optical(PORT12);
distance DistanceBed = distance(PORT8);
touchled TouchStatus = touchled(PORT11);
controller Controller = controller();

// generating and setting random seed
void initializeRandomSeed() {
  wait(100, msec);
  double xAxis = BrainInertial.acceleration(xaxis) * 1000;
  double yAxis = BrainInertial.acceleration(yaxis) * 1000;
  double zAxis = BrainInertial.acceleration(zaxis) * 1000;
  int seed = int(xAxis + yAxis + zAxis);
  srand(seed);
}

// Converts a color to a string
const char* convertColorToString(color col) {
  if (col == colorType::red) return "red";
  else if (col == colorType::green) return "green";
  else if (col == colorType::blue) return "blue";
  else if (col == colorType::white) return "white";
  else if (col == colorType::yellow) return "yellow";
  else if (col == colorType::orange) return "orange";
  else if (col == colorType::purple) return "purple";
  else if (col == colorType::cyan) return "cyan";
  else if (col == colorType::black) return "black";
  else if (col == colorType::transparent) return "transparent";
  else if (col == colorType::red_violet) return "red_violet";
  else if (col == colorType::violet) return "violet";
  else if (col == colorType::blue_violet) return "blue_violet";
  else if (col == colorType::blue_green) return "blue_green";
  else if (col == colorType::yellow_green) return "yellow_green";
  else if (col == colorType::yellow_orange) return "yellow_orange";
  else if (col == colorType::red_orange) return "red_orange";
  else if (col == colorType::none) return "none";
  else return "unknown";
}

// Convert colorType to string
const char* convertColorToString(colorType col) {
  if (col == colorType::red) return "red";
  else if (col == colorType::green) return "green";
  else if (col == colorType::blue) return "blue";
  else if (col == colorType::white) return "white";
  else if (col == colorType::yellow) return "yellow";
  else if (col == colorType::orange) return "orange";
  else if (col == colorType::purple) return "purple";
  else if (col == colorType::cyan) return "cyan";
  else if (col == colorType::black) return "black";
  else if (col == colorType::transparent) return "transparent";
  else if (col == colorType::red_violet) return "red_violet";
  else if (col == colorType::violet) return "violet";
  else if (col == colorType::blue_violet) return "blue_violet";
  else if (col == colorType::blue_green) return "blue_green";
  else if (col == colorType::yellow_green) return "yellow_green";
  else if (col == colorType::yellow_orange) return "yellow_orange";
  else if (col == colorType::red_orange) return "red_orange";
  else if (col == colorType::none) return "none";
  else return "unknown";
}

void vexcodeInit() {
  initializeRandomSeed();
}

#pragma endregion VEXcode Generated Robot Configuration

/*
 * MTE 100 / MTE 121 Design Project - Group 28
 * Automated Desktop Braille Embosser Control Firmware (v21 - Auto Sensors)
 *
 * Team Members:
 *   - Joseph Lu         : homeAllAxes, checkSafetyAndPaper
 *   - Junwon Will Choi  : selectWordFromController, updateLiveTelemetry
 *   - Nicholas Chan     : moveCarriageToColumn, punchBrailleDot
 *   - Liam McCarthy     : getCharDotMatrix, advancePaperStrip,
 *                         executeCleanShutdown
 *
 * Mechanical Configuration (Final Revision - Tetrix Prime):
 *   - Row-by-row horizontal embossing across the word (Row 0 -> 1 -> 2)
 *   - Relative Z stroke: start at bottom, lift ~1 cm (+18 deg) before X
 *     homes to motor side, stall-punch down, lift strictly +18 deg up
 *   - Auto-detects Gen 1 (colorsensor/sonar) AND Gen 2 (optical/distance)
 *     across all 12 Brain ports so both sensor generations work reliably.
 *   - PORT9  (MotorFeed)   : Single Y-axis roller axle (2x 200 mm wheels)
 *   - PORT10 (MotorX)      : 16T VEX IQ sprocket + 6.35 mm chain
 *   - PORT7  (MotorZ)      : Spur gear + vertical rack stylus into
 *                            3D-printed BrailleBasePlate indents (dia 2.5)
 *   - PORT12 (Color/Opt)   : Paper presence / brightness sensor (higher)
 *   - PORT8  (Sonar/Dist)  : Obstruction safety interlock (lower)
 *   - PORT11 (TouchStatus) : User Start/Pause input & RGB status LED
 */

#include <cmath>

// =========================================================================
// Physical Geometry & Calibration Constants (read-only; no mutable globals)
// =========================================================================
// Measured BrailleBasePlate geometry (mm, dia 2.5 mm holes):
//   - Left edge to Letter 0 Col 0 center  : 2.5 mm
//   - Col 0 to Col 1 within one letter    : 3.5 mm
//   - Col 1 of letter i to Col 0 of i+1   : 5.5 mm (char pitch = 9.0 mm)
//   - Vertical row step (Row 0 -> 1 -> 2) : 3.5 mm
const double COL_ZERO_MM          = 2.5;   // X home stop -> Hole 0 (mm)
const double INTRA_COL_MM         = 3.5;   // Col 0 -> Col 1 in cell (mm)
const double INTER_CHAR_GAP_MM    = 5.5;   // Col 1 -> next Col 0 (mm)
const double CHAR_PITCH_MM        = INTRA_COL_MM + INTER_CHAR_GAP_MM;
const double ROW_PITCH_MM         = 3.5;   // Vertical Braille row step (mm)
const double LOAD_FEED_MM         = 0.0;   // Startup feed-in (mm)
const double EJECT_FEED_MM        = 320.0; // Full sheet ejection (mm)

// Exact VEX IQ drive kinematics from physical hardware:
//   X: 16-tooth VEX IQ sprocket (228-2500-233) * 6.35 mm chain = 101.6 mm
//   Y: VEX IQ 200 mm travel wheel (1:1 direct axle)            = 200.0 mm
const double X_MM_PER_REV         = 101.6;
const double Y_MM_PER_REV         = 200.0;

// Derived encoder angle targets (degrees):
//   X: COL_ZERO = 8.86 deg, INTRA_COL = 12.40 deg, CHAR_PITCH = 31.89 deg
//   Y: ROW_FEED = 6.30 deg (3.5 mm)
const double COL_ZERO_DEG         = (COL_ZERO_MM / X_MM_PER_REV) * 360.0;
const double INTRA_COL_DEG        = (INTRA_COL_MM / X_MM_PER_REV) * 360.0;
const double CHAR_PITCH_DEG       = (CHAR_PITCH_MM / X_MM_PER_REV) * 360.0;
const double LOAD_FEED_DEG        = (LOAD_FEED_MM / Y_MM_PER_REV) * 360.0;
const double ROW_FEED_DEG         = (ROW_PITCH_MM / Y_MM_PER_REV) * 360.0;
const double EJECT_FEED_DEG       = (EJECT_FEED_MM / Y_MM_PER_REV) * 360.0;

// Anti-backlash approach overshoot when returning left on VEX chain (deg).
const double X_BACKLASH_DEG       = 8.0;

// Z-axis relative-stroke parameters (start with stylus ALL THE WAY DOWN):
//   Every lift moves strictly +Z_LIFT_DEG (+18 deg = ~1 cm on 60T gear)
//   above the actual bottom position so the rack can NEVER creep higher.
const double Z_LIFT_DEG           = 18.0;  // Strictly ~1 cm lift (deg)
const double Z_MIN_CLEAR_DEG      = 8.0;   // Minimum lift to clear paper
const double Z_MAX_DOWN_TRAVEL    = 90.0;  // Safety cap during stall punch
const double POS_TOLERANCE_DEG    = 0.5;   // Tight X/Y precision window
const double STALL_CURRENT_AMP    = 1.25;  // Y/Z-lift stall current
const double X_JAM_CURRENT_AMP    = 1.65;  // Sustained X jam fault current
const double X_HOMING_STALL_AMP   = 1.40;  // Sustained X end-stop current
const double Z_PUNCH_STALL_AMP    = 1.85;  // Max downward stall current
const double OBSTRUCTION_MIN_MM   = 80.0;  // Default Gen2 obstruction (mm)
const double SONAR_MIN_MM         = 100.0; // Default Gen1 sonar range (mm)
const double WHITE_PAPER_MIN_PCT  = 15.0;  // Min white-paper brightness (%)
const int    PUNCH_SQUEEZE_MSEC   = 150;   // Active bottom squeeze (ms)
const int    PUNCH_DWELL_MSEC     = 250;   // Hold-at-bottom dwell (ms)
const int    Z_POST_LIFT_PAUSE_MS = 450;   // Pause at top before X (3x)
const int    MAX_WORD_LENGTH      = 16;    // Max queued chars per job
const bool   MIRROR_COLUMNS_FOR_DOWNWARD_PUNCH = false;

// Fault code constants for diagnostic display on shutdown.
const int FAULT_NONE              = 0;
const int FAULT_BED_BLOCKED       = 1;
const int FAULT_SAFETY_TIMEOUT    = 2;
const int FAULT_X_STALL           = 3;
const int FAULT_Z_LIFT            = 4;
const int FAULT_Y_JAM             = 5;

// Standard Grade 1 Unified English Braille lookup table (A-Z).
// Dimensions: [26 letters][3 dot rows][2 dot columns].
const bool BRAILLE_ALPHABET[26][3][2] = {
    {{true,  false}, {false, false}, {false, false}}, // A
    {{true,  false}, {true,  false}, {false, false}}, // B
    {{true,  true},  {false, false}, {false, false}}, // C
    {{true,  true},  {false, true},  {false, false}}, // D
    {{true,  false}, {false, true},  {false, false}}, // E
    {{true,  true},  {true,  false}, {false, false}}, // F
    {{true,  true},  {true,  true},  {false, false}}, // G
    {{true,  false}, {true,  true},  {false, false}}, // H
    {{false, true},  {true,  false}, {false, false}}, // I
    {{false, true},  {true,  true},  {false, false}}, // J
    {{true,  false}, {false, false}, {true,  false}}, // K
    {{true,  false}, {true,  false}, {true,  false}}, // L
    {{true,  true},  {false, false}, {true,  false}}, // M
    {{true,  true},  {false, true},  {true,  false}}, // N
    {{true,  false}, {false, true},  {true,  false}}, // O
    {{true,  true},  {true,  false}, {true,  false}}, // P
    {{true,  true},  {true,  true},  {true,  false}}, // Q
    {{true,  false}, {true,  true},  {true,  false}}, // R
    {{false, true},  {true,  false}, {true,  false}}, // S
    {{false, true},  {true,  true},  {true,  false}}, // T
    {{true,  false}, {false, false}, {true,  true}},  // U
    {{true,  false}, {true,  false}, {true,  true}},  // V
    {{false, true},  {true,  true},  {false, true}},  // W
    {{true,  true},  {false, false}, {true,  true}},  // X
    {{true,  true},  {false, true},  {true,  true}},  // Y
    {{true,  false}, {false, true},  {true,  true}}   // Z
};

// =========================================================================
// Section 1: System Homing, Calibration, and Safety Monitoring
// Author: Joseph Lu
// =========================================================================

// Helper: reads reflected brightness (0-100%) from either Gen 1
// colorsensor (colorGen == 1) or Gen 2 optical (colorGen == 2).
double readPaperBrightness(int colorGen, colorsensor &colorG1,
                           optical &opticalG2) {
    if (colorGen == 1) {
        return (double)colorG1.brightness();
    } else if (colorGen == 2) {
        return opticalG2.brightness();
    }
    return 0.0;
}

// Helper: reads bed distance (mm) and detected flag from either Gen 1
// ultrasonic sonar (distGen == 1) or Gen 2 laser distance (distGen == 2).
double readBedDistance(int distGen, sonar &sonarG1, distance &distG2,
                       bool &objectFound) {
    if (distGen == 1) {
        objectFound = sonarG1.foundObject();
        return sonarG1.distance(mm);
    } else if (distGen == 2) {
        objectFound = distG2.isObjectDetected();
        return distG2.objectDistance(mm);
    }
    objectFound = false;
    return 0.0;
}

// Function 1: Zeroes Z at the bottom start position, lifts Z strictly
// ~1 cm (+18 deg) and pauses 450 ms BEFORE homing X all the way to the
// motor-side stop via sustained-stall detection, enables the white LED on
// Gen 1 or Gen 2 color/optical sensor, auto-calibrates sensor thresholds,
// and prints the exact detected port + generation on the Brain LCD.
bool homeAllAxes(double &calibratedPressDeg, double &paperThreshold,
                 double &distThreshold, int colorPort, int colorGen,
                 colorsensor &colorG1, optical &opticalG2, int distPort,
                 int distGen, sonar &sonarG1, distance &distG2) {
    timer homingTimer;
    homingTimer.clear();

    MotorZ.setMaxTorque(100, percent);
    MotorX.setMaxTorque(100, percent);
    TouchStatus.on(yellow);

    // Enable integrated white LED on whichever generation is connected.
    if (colorGen == 1) {
        colorG1.setLight(ledState::on);
        colorG1.setLight(100, percent);
    } else if (colorGen == 2) {
        opticalG2.setLight(ledState::on);
        opticalG2.setLightPower(100, percent);
    }

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("v21 Lift Z ~1cm");

    // 1. Start with stylus resting at the bottom; zero encoder cleanly
    // without jamming downward, then lift strictly +18 deg (~1 cm) BEFORE
    // X calibrates so the stylus is raised clear of the plate.
    MotorZ.stop(hold);
    MotorZ.setPosition(0, degrees);
    wait(50, msec);
    calibratedPressDeg = Z_MAX_DOWN_TRAVEL;

    homingTimer.clear();
    MotorZ.setMaxTorque(70, percent);
    MotorZ.spin(forward, 20, percent);
    while (MotorZ.position(degrees) < Z_LIFT_DEG &&
           homingTimer.time(msec) < 1500 &&
           (homingTimer.time(msec) < 150 ||
            MotorZ.current(amp) < STALL_CURRENT_AMP)) {
        wait(5, msec);
    }
    MotorZ.stop(hold);
    MotorZ.setMaxTorque(100, percent);
    wait(Z_POST_LIFT_PAUSE_MS, msec); // Pause 450 ms AFTER reaching top.

    // 2. With Z raised ~1 cm, home X-axis carriage ALL THE WAY to the
    // motor-side physical stop using sustained stall (180 ms continuous).
    Brain.Screen.newLine();
    Brain.Screen.print("v21 Homing X...");
    homingTimer.clear();
    MotorX.spin(forward, 30, percent);
    int stallCount = 0;
    while (homingTimer.time(msec) < 8000 && stallCount < 12) {
        if (homingTimer.time(msec) > 400 &&
            (fabs(MotorX.velocity(rpm)) < 2.0 ||
             MotorX.current(amp) >= X_HOMING_STALL_AMP)) {
            stallCount++;
        } else {
            stallCount = 0;
        }
        wait(15, msec);
    }
    MotorX.stop(coast);
    wait(60, msec); // Relax chain preload against end stop before zeroing.
    MotorX.setPosition(0, degrees);
    wait(60, msec);

    // Leave single Y-axis roller axle (PORT9) in coast for easy loading.
    MotorFeed.stop(coast);
    MotorFeed.setPosition(0, degrees);

    // 3. Auto-calibrate Distance/Sonar threshold from clear-bed baseline.
    wait(150, msec);
    bool startDistDetected = false;
    double measuredBedDist =
        readBedDistance(distGen, sonarG1, distG2, startDistDetected);

    if (distGen == 1) {
        distThreshold = SONAR_MIN_MM;
        if (startDistDetected && measuredBedDist > 80.0 &&
            measuredBedDist < 900.0) {
            distThreshold = measuredBedDist * 0.65;
            if (distThreshold < 60.0) {
                distThreshold = 60.0;
            } else if (distThreshold > 140.0) {
                distThreshold = 140.0;
            }
        }
    } else {
        distThreshold = OBSTRUCTION_MIN_MM;
        if (startDistDetected && measuredBedDist > 50.0 &&
            measuredBedDist < 1800.0) {
            distThreshold = measuredBedDist * 0.65;
            if (distThreshold < 40.0) {
                distThreshold = 40.0;
            } else if (distThreshold > 100.0) {
                distThreshold = 100.0;
            }
        }
    }

    // 4. Calibrate white-paper brightness threshold (never disable to 0
    // when a color/optical sensor is connected).
    double startBright = readPaperBrightness(colorGen, colorG1, opticalG2);
    paperThreshold = WHITE_PAPER_MIN_PCT;
    if (startBright >= 30.0) {
        paperThreshold = startBright * 0.55;
    }

    TouchStatus.on(blue);
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("v21 C:P%dG%d D:P%dG%d",
                       colorPort + 1, colorGen, distPort + 1, distGen);
    Brain.Screen.newLine();
    Brain.Screen.print("B:%.0f%% T:%.0f%%", startBright, paperThreshold);
    Brain.Screen.newLine();
    Brain.Screen.print("D:%.0f T:%.0fmm", measuredBedDist, distThreshold);
    wait(2200, msec);
    return true;
}

// Function 2: Polls TouchStatus (manual pause), Color/Optical sensor
// (requires white paper >= paperThreshold when detected), and Sonar/
// Distance sensor (< distThreshold). Updates live B/D readings while
// paused and waits up to 25 s for recovery + Touch LED (or Check skip).
bool checkSafetyAndPaper(bool &isPaused, timer &jobTimer,
                         double paperThreshold, double distThreshold,
                         int colorGen, colorsensor &colorG1,
                         optical &opticalG2, int distGen, sonar &sonarG1,
                         distance &distG2) {
    if (TouchStatus.pressing()) {
        isPaused = true;
        while (TouchStatus.pressing()) {
            wait(15, msec);
        }
    }

    double optBright = readPaperBrightness(colorGen, colorG1, opticalG2);
    bool paperMissing = (colorGen > 0 && optBright < paperThreshold);

    bool distObj = false;
    double bedClearanceMm =
        readBedDistance(distGen, sonarG1, distG2, distObj);
    bool obstructionDetected =
        (distGen > 0 && distObj && bedClearanceMm < distThreshold);

    if (paperMissing || obstructionDetected) {
        isPaused = true;
    }

    if (!isPaused) {
        return true;
    }

    // Enter safe pause state: brake X, hold Z and feed roller.
    MotorX.stop(brake);
    MotorFeed.stop(hold);
    MotorZ.stop(hold);
    TouchStatus.on(yellow);

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    if (paperMissing) {
        Brain.Screen.print("PAUSED:No Paper");
    } else if (obstructionDetected) {
        Brain.Screen.print("PAUSED:Distance");
    } else {
        Brain.Screen.print("PAUSED:User");
    }
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("LED=Go Chk=Skip");

    timer pauseTimer;
    pauseTimer.clear();
    bool resumed = false;
    int uiTicks = 0;

    while (pauseTimer.time(seconds) < 25.0 && !resumed) {
        double nowBright =
            readPaperBrightness(colorGen, colorG1, opticalG2);
        bool paperPresentNow =
            (colorGen == 0 || nowBright >= paperThreshold);

        bool nowDistObj = false;
        double currentClearance =
            readBedDistance(distGen, sonarG1, distG2, nowDistObj);
        bool pathClearNow =
            (distGen == 0 || !nowDistObj ||
             currentClearance >= distThreshold);

        // Refresh live Color/Optical & Distance readings every ~200 ms.
        if (uiTicks % 8 == 0) {
            Brain.Screen.setCursor(2, 1);
            Brain.Screen.print("B:%.0f/%.0f D:%.0f  ",
                               nowBright, paperThreshold,
                               currentClearance);
        }
        uiTicks++;

        bool ledResume =
            TouchStatus.pressing() && paperPresentNow && pathClearNow;
        bool manualOverride = Brain.buttonCheck.pressing();

        if (ledResume || manualOverride) {
            while (TouchStatus.pressing() ||
                   Brain.buttonCheck.pressing()) {
                wait(15, msec);
            }
            isPaused = false;
            resumed = true;
            TouchStatus.on(green);
        }
        wait(25, msec);
    }

    return resumed;
}

// =========================================================================
// Section 2: User Interface, Word Buffering, and Live Telemetry
// Author: Junwon Will Choi
// =========================================================================

// Function 3: Interactive word queue allowing the user to select up to
// (maxLen - 1) characters ('A'-'Z' and ' ') using Brain or Controller
// buttons, then start embossing with TouchStatus (or Controller FDown).
// Defaults to "UW" if started with an empty queue.
int selectWordFromController(char wordBuffer[], int maxLen) {
    int charCount = 0;
    int currentLetterIdx = 0; // 0 = 'A' .. 25 = 'Z', 26 = ' '
    bool selectionComplete = false;

    TouchStatus.on(blue);
    timer inputTimeout;
    inputTimeout.clear();

    while (!selectionComplete && charCount < maxLen - 1 &&
           inputTimeout.time(seconds) < 60.0) {
        char previewChar = ' ';
        if (currentLetterIdx < 26) {
            previewChar = (char)('A' + currentLetterIdx);
        }

        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Char:[%c] Q:%d", previewChar, charCount);
        Brain.Screen.newLine();
        for (int i = 0; i < charCount; i++) {
            Brain.Screen.print("%c", wordBuffer[i]);
        }
        if (charCount == 0) {
            Brain.Screen.print("(Default: UW)");
        }
        Brain.Screen.newLine();
        Brain.Screen.print("Touch LED=Start");

        // Cycle character forward (Brain Right or Controller E Up).
        if (Brain.buttonRight.pressing() ||
            Controller.ButtonEUp.pressing()) {
            currentLetterIdx = (currentLetterIdx + 1) % 27;
            inputTimeout.clear();
            while (Brain.buttonRight.pressing() ||
                   Controller.ButtonEUp.pressing()) {
                wait(15, msec);
            }
        }

        // Cycle character backward (Brain Left or Controller E Down).
        if (Brain.buttonLeft.pressing() ||
            Controller.ButtonEDown.pressing()) {
            currentLetterIdx = (currentLetterIdx + 26) % 27;
            inputTimeout.clear();
            while (Brain.buttonLeft.pressing() ||
                   Controller.ButtonEDown.pressing()) {
                wait(15, msec);
            }
        }

        // Append preview character (Brain Check or Controller F Up).
        if (Brain.buttonCheck.pressing() ||
            Controller.ButtonFUp.pressing()) {
            wordBuffer[charCount] = previewChar;
            charCount++;
            inputTimeout.clear();
            while (Brain.buttonCheck.pressing() ||
                   Controller.ButtonFUp.pressing()) {
                wait(15, msec);
            }
        }

        // Confirm queued word via TouchStatus or Controller FDown
        // (avoids Brain.buttonDown/buttonLeft alias collision on IQ).
        bool startTriggered =
            TouchStatus.pressing() ||
            Controller.ButtonFDown.pressing();

        if (startTriggered) {
            while (TouchStatus.pressing() ||
                   Controller.ButtonFDown.pressing()) {
                wait(15, msec);
            }
            selectionComplete = true;
        }

        wait(40, msec);
    }

    // Default to "UW" if no characters were explicitly queued.
    if (charCount == 0) {
        wordBuffer[0] = 'U';
        wordBuffer[1] = 'W';
        charCount = 2;
    }
    wordBuffer[charCount] = '\0';
    return charCount;
}

// Function 4: Updates the VEX IQ Brain LCD with the active character,
// active Braille row (1-3), dots punched, live sensor readings (Color/
// Optical brightness % and Sonar/Distance mm), and elapsed time.
void updateLiveTelemetry(char currentChar, int rowIndex, int totalRows,
                         int dotsPunched, timer &jobTimer, int colorGen,
                         colorsensor &colorG1, optical &opticalG2,
                         int distGen, sonar &sonarG1, distance &distG2) {
    bool unusedFound = false;
    double liveBright = readPaperBrightness(colorGen, colorG1, opticalG2);
    double liveDist =
        readBedDistance(distGen, sonarG1, distG2, unusedFound);

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("R%d/%d '%c' D:%d",
                       rowIndex + 1, totalRows, currentChar, dotsPunched);
    Brain.Screen.newLine();
    Brain.Screen.print("B:%.0f%% D:%.0fmm", liveBright, liveDist);
    Brain.Screen.newLine();
    Brain.Screen.print("Time: %.1fs", jobTimer.time(seconds));
}

// =========================================================================
// Section 3: X-Axis Chain Carriage and Z-Axis Rack-and-Pinion Punch Control
// Author: Nicholas Chan
// =========================================================================

// Function 5: Positions the bearing-guided X-axis carriage over any
// (charIdx, dotCol) hole on BrailleBasePlate (2.5 mm offset, 3.5 mm intra-
// cell spacing, 5.5 mm inter-cell gap -> 9.0 mm char pitch) via the 16T
// sprocket. Pass charIdx < 0 to re-home cleanly to 0 deg between rows.
bool moveCarriageToColumn(int charIdx, int dotCol, double colZeroDeg,
                          double intraColDeg, double charPitchDeg) {
    double targetDeg = 0.0;
    if (charIdx >= 0) {
        targetDeg =
            colZeroDeg + (charIdx * charPitchDeg) + (dotCol * intraColDeg);
    }
    if (targetDeg < 0.0) {
        targetDeg = 0.0;
    }

    timer moveTimer;
    moveTimer.clear();
    bool stalled = false;
    double currentDeg = -MotorX.position(degrees);
    double maxMoveMs = 4000.0 + fabs(targetDeg - currentDeg) * 25.0;
    MotorX.setMaxTorque(100, percent);

    // Special case: returning to home (0 deg) on the motor side between
    // rows or on shutdown using sustained-stall detection.
    if (charIdx < 0 || targetDeg <= 0.5) {
        MotorX.spin(forward, 28, percent);
        int homeStallCount = 0;
        while (-MotorX.position(degrees) > POS_TOLERANCE_DEG &&
               moveTimer.time(msec) < maxMoveMs &&
               homeStallCount < 10) {
            if (moveTimer.time(msec) > 350 &&
                (fabs(MotorX.velocity(rpm)) < 2.0 ||
                 MotorX.current(amp) >= X_HOMING_STALL_AMP)) {
                homeStallCount++;
            } else {
                homeStallCount = 0;
            }
            wait(15, msec);
        }
        MotorX.stop(coast);
        wait(50, msec);
        MotorX.setPosition(0, degrees);
        wait(40, msec);
        return true;
    }

    // If moving left (decreasing angle), overshoot past target by
    // X_BACKLASH_DEG (clamped >= 0) so the final approach is always
    // from the same tensioned side of the VEX chain.
    if (targetDeg < currentDeg - POS_TOLERANCE_DEG) {
        double preTarget = targetDeg - X_BACKLASH_DEG;
        if (preTarget < 0.0) {
            preTarget = 0.0;
        }
        int leftSpeed = 28;
        if (currentDeg - preTarget > 30.0) {
            leftSpeed = 32;
        }
        int jamCount = 0;
        MotorX.spin(forward, leftSpeed, percent);
        while (-MotorX.position(degrees) > preTarget + POS_TOLERANCE_DEG &&
               !stalled) {
            if (moveTimer.time(msec) > 600 &&
                fabs(MotorX.velocity(rpm)) < 1.5 &&
                MotorX.current(amp) >= X_JAM_CURRENT_AMP) {
                jamCount++;
                if (jamCount >= 50) {
                    stalled = true;
                }
            } else {
                jamCount = 0;
            }
            if (moveTimer.time(msec) > maxMoveMs) {
                stalled = true;
            }
            wait(5, msec);
        }
        MotorX.stop(hold);
        wait(40, msec);
        currentDeg = -MotorX.position(degrees);
        moveTimer.clear();
    }

    // Final approach moving right (reverse spin) into the exact hole.
    if (!stalled && targetDeg > currentDeg + POS_TOLERANCE_DEG) {
        int rightSpeed = 28;
        if (targetDeg - currentDeg > 30.0) {
            rightSpeed = 32;
        }
        int jamCount = 0;
        MotorX.spin(reverse, rightSpeed, percent);
        while (-MotorX.position(degrees) < targetDeg - POS_TOLERANCE_DEG &&
               !stalled) {
            if (moveTimer.time(msec) > 600 &&
                fabs(MotorX.velocity(rpm)) < 1.5 &&
                MotorX.current(amp) >= X_JAM_CURRENT_AMP) {
                jamCount++;
                if (jamCount >= 50) {
                    stalled = true;
                }
            } else {
                jamCount = 0;
            }
            if (moveTimer.time(msec) > maxMoveMs) {
                stalled = true;
            }
            wait(5, msec);
        }
    }

    MotorX.stop(hold);
    return !stalled;
}

// Function 6: Pushes the Z-axis stylus firmly down at 100% max torque
// until strong stall (current >= 1.85 A or 160 ms sustained zero velocity,
// safety capped at maxDownTravelDeg), applies a 150 ms active bottom
// squeeze + 250 ms hold dwell, then lifts strictly +Z_LIFT_DEG (+18 deg)
// above actual bottomPos and pauses 450 ms AT THE TOP before X moves.
bool punchBrailleDot(double maxDownTravelDeg, int dwellTimeMsec) {
    timer strokeTimer;
    strokeTimer.clear();

    double topPos = MotorZ.position(degrees);

    // Firm, full-torque downward embossing stroke until strong stall.
    MotorZ.setMaxTorque(100, percent);
    MotorZ.spin(reverse, 35, percent);
    int zStallCount = 0;
    bool bottomStalled = false;
    while ((topPos - MotorZ.position(degrees)) < maxDownTravelDeg &&
           strokeTimer.time(msec) < 2000 && !bottomStalled) {
        if (strokeTimer.time(msec) > 150) {
            if (MotorZ.current(amp) >= Z_PUNCH_STALL_AMP) {
                bottomStalled = true;
            } else if (fabs(MotorZ.velocity(rpm)) < 2.0) {
                zStallCount++;
                if (zStallCount >= 32) {
                    bottomStalled = true;
                }
            } else {
                zStallCount = 0;
            }
        }
        wait(5, msec);
    }

    // Brief 150 ms active downward squeeze at full torque, then hold
    // firmly at the bottom for dwellTimeMsec to form a deeper tactile dot.
    MotorZ.spin(reverse, 20, percent);
    timer dwellTimer;
    dwellTimer.clear();
    while (dwellTimer.time(msec) < PUNCH_SQUEEZE_MSEC) {
        wait(5, msec);
    }
    MotorZ.stop(hold);

    dwellTimer.clear();
    while (dwellTimer.time(msec) < dwellTimeMsec) {
        wait(5, msec);
    }

    // Measure actual physical bottom angle AFTER squeeze/dwell and lift
    // STRICTLY +18 deg (~1 cm) above it so lift height never creeps up.
    double bottomPos = MotorZ.position(degrees);
    double liftTarget = bottomPos + Z_LIFT_DEG;

    strokeTimer.clear();
    MotorZ.setMaxTorque(70, percent);
    MotorZ.spin(forward, 20, percent);
    while (MotorZ.position(degrees) < liftTarget &&
           strokeTimer.time(msec) < 1500 &&
           (strokeTimer.time(msec) < 150 ||
            MotorZ.current(amp) < STALL_CURRENT_AMP)) {
        wait(5, msec);
    }
    MotorZ.stop(hold);
    MotorZ.setMaxTorque(100, percent);
    wait(Z_POST_LIFT_PAUSE_MS, msec); // Pause 450 ms AFTER reaching top.

    // Fail only if Z genuinely failed to lift at least 8 deg off bottom.
    return ((MotorZ.position(degrees) - bottomPos) >= Z_MIN_CLEAR_DEG);
}

// =========================================================================
// Section 4: Braille Lookup, Single-Roller Paper Feed, and Clean Shutdown
// Author: Liam McCarthy
// =========================================================================

// Function 7: Validates an ASCII character ('A'-'Z', 'a'-'z', or ' '),
// normalizes lowercase letters, and populates cellMatrix[3][2] from
// BRAILLE_ALPHABET. Returns false for unsupported characters.
bool getCharDotMatrix(char letter, bool cellMatrix[3][2]) {
    if (letter >= 'a' && letter <= 'z') {
        letter = (char)(letter - 'a' + 'A');
    }

    if (letter == ' ') {
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 2; c++) {
                cellMatrix[r][c] = false;
            }
        }
        return true;
    }

    if (letter < 'A' || letter > 'Z') {
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 2; c++) {
                cellMatrix[r][c] = false;
            }
        }
        return false;
    }

    int alphaIndex = (int)(letter - 'A');
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 2; c++) {
            cellMatrix[r][c] = BRAILLE_ALPHABET[alphaIndex][r][c];
        }
    }
    return true;
}

// Function 8: Advances the paper strip via the single Y-axis roller axle
// (PORT9, 2x 200 mm VEX travel wheels) by feedDegrees within 0.5 deg with
// 250 ms inrush-masked jam detection. Uses 18% speed on short row steps
// and 50% speed on long full-sheet ejection moves.
bool advancePaperStrip(double feedDegrees, bool holdAfterFeed) {
    if (feedDegrees > 0.0) {
        double targetY = MotorFeed.position(degrees) + feedDegrees;
        int feedSpeed = 18;
        if (feedDegrees > 45.0) {
            feedSpeed = 50;
        }
        double maxFeedMs = 1500.0 + feedDegrees * 12.0;

        timer feedTimer;
        feedTimer.clear();

        MotorFeed.spin(forward, feedSpeed, percent);
        while (MotorFeed.position(degrees) < targetY - POS_TOLERANCE_DEG &&
               feedTimer.time(msec) < maxFeedMs) {
            if (feedTimer.time(msec) > 250 &&
                MotorFeed.current(amp) > STALL_CURRENT_AMP) {
                MotorFeed.stop(brake);
                return false;
            }
            wait(5, msec);
        }
    }

    if (holdAfterFeed) {
        MotorFeed.stop(hold);
    } else {
        MotorFeed.stop(coast);
    }
    return true;
}

// Function 9: Executes a safe non-abort shutdown. Raises Z to 2x clearance
// (+36 deg total), ejects the completed strip via MotorFeed (unless
// faulted), parks X-axis at 0 deg, de-energizes motors and sensor LED,
// displays job stats (or specific fault cause), and calls programStop().
void executeCleanShutdown(int totalCharsEmbossed, int totalDotsPunched,
                          double totalTimeSec, int faultCode,
                          int colorGen, colorsensor &colorG1,
                          optical &opticalG2) {
    timer shutdownTimer;
    shutdownTimer.clear();

    // At job end, raise Z to 2x normal lift height (+36 deg total = ~2 cm)
    // before ejecting paper and parking X (already at 1x unless Z faulted).
    double extraLiftDeg = Z_LIFT_DEG;
    if (faultCode == FAULT_Z_LIFT) {
        extraLiftDeg = 2.0 * Z_LIFT_DEG;
    }
    double endLiftTarget = MotorZ.position(degrees) + extraLiftDeg;
    MotorZ.setMaxTorque(70, percent);
    MotorZ.spin(forward, 20, percent);
    while (MotorZ.position(degrees) < endLiftTarget &&
           shutdownTimer.time(msec) < 2000 &&
           (shutdownTimer.time(msec) < 150 ||
            MotorZ.current(amp) < STALL_CURRENT_AMP)) {
        wait(10, msec);
    }
    MotorZ.stop(hold);
    MotorZ.setMaxTorque(100, percent);
    wait(Z_POST_LIFT_PAUSE_MS, msec);

    // Eject finished Braille strip on normal completion; release on fault.
    if (faultCode == FAULT_NONE) {
        advancePaperStrip(EJECT_FEED_DEG, false);
    } else {
        advancePaperStrip(0.0, false);
    }

    // Park X-axis carriage at left home position.
    moveCarriageToColumn(-1, 0, COL_ZERO_DEG,
                         INTRA_COL_DEG, CHAR_PITCH_DEG);

    // De-energize all 3 motors and color/optical LED for safe handling.
    MotorFeed.stop(coast);
    MotorX.stop(coast);
    MotorZ.stop(coast);
    if (colorGen == 1) {
        colorG1.setLight(ledState::off);
    } else if (colorGen == 2) {
        opticalG2.setLight(ledState::off);
    }

    // Display final summary (<= 16 chars/line) and audio/visual cue.
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    if (faultCode != FAULT_NONE) {
        TouchStatus.on(red);
        Brain.playSound(siren);
        if (faultCode == FAULT_BED_BLOCKED) {
            Brain.Screen.print("FAULT:Bed<20mm");
        } else if (faultCode == FAULT_SAFETY_TIMEOUT) {
            Brain.Screen.print("FAULT:Safety");
        } else if (faultCode == FAULT_X_STALL) {
            Brain.Screen.print("FAULT:X Stall");
        } else if (faultCode == FAULT_Z_LIFT) {
            Brain.Screen.print("FAULT:Z Lift");
        } else {
            Brain.Screen.print("FAULT:Y Jam");
        }
    } else {
        TouchStatus.on(blue);
        Brain.playSound(tada);
        Brain.Screen.print("JOB COMPLETE!");
    }
    Brain.Screen.newLine();
    Brain.Screen.print("C:%d Dots:%d",
                       totalCharsEmbossed, totalDotsPunched);
    Brain.Screen.newLine();
    Brain.Screen.print("Time: %.1fs", totalTimeSec);

    wait(5000, msec);
    Brain.programStop();
}

// =========================================================================
// Main Control Loop (Row-by-Row Horizontal Word Embossing)
// =========================================================================

int main() {
    vexcodeInit();

    // Auto-detect exact physical port and generation (Gen 1 vs Gen 2) for
    // Color/Optical and Sonar/Distance across all 12 Brain smart ports.
    int colorPort = PORT12;
    int colorGen = 0; // 1 = Gen 1 colorsensor, 2 = Gen 2 optical
    int distPort = PORT8;
    int distGen = 0;  // 1 = Gen 1 sonar, 2 = Gen 2 distance
    for (int p = PORT1; p <= PORT12; p++) {
        IQ_DeviceType dt = vexDeviceTypeGetByIndex(p);
        if (dt == kDeviceTypeRgbSensor) {
            colorPort = p;
            colorGen = 1;
        } else if (dt == kDeviceTypeOpticalSensor) {
            colorPort = p;
            colorGen = 2;
        } else if (dt == kDeviceTypeSonarSensor) {
            distPort = p;
            distGen = 1;
        } else if (dt == kDeviceTypeDistanceSensor) {
            distPort = p;
            distGen = 2;
        }
    }

    // Bind only the detected sensor generation to its physical port;
    // unused drivers bind safely to empty ports 1-4 with zero conflict.
    colorsensor colorG1(colorGen == 1 ? colorPort : PORT1);
    optical opticalG2(colorGen == 2 ? colorPort : PORT2);
    sonar sonarG1(distGen == 1 ? distPort : PORT3);
    distance distG2(distGen == 2 ? distPort : PORT4);

    double calibratedPressDeg = Z_MAX_DOWN_TRAVEL;
    double paperThreshold = WHITE_PAPER_MIN_PCT;
    double distThreshold = OBSTRUCTION_MIN_MM;
    char wordBuffer[MAX_WORD_LENGTH];
    bool wordCells[MAX_WORD_LENGTH][3][2];
    bool charValid[MAX_WORD_LENGTH];
    bool isPaused = false;
    int faultCode = FAULT_NONE;
    int totalDotsPunched = 0;
    int validCharCount = 0;

    if (!homeAllAxes(calibratedPressDeg, paperThreshold, distThreshold,
                     colorPort, colorGen, colorG1, opticalG2, distPort,
                     distGen, sonarG1, distG2)) {
        executeCleanShutdown(0, 0, 0.0, FAULT_BED_BLOCKED,
                             colorGen, colorG1, opticalG2);
        return 0;
    }

    int wordLen = selectWordFromController(wordBuffer, MAX_WORD_LENGTH);

    // Pre-decode all queued characters into wordCells[i][row][col].
    for (int i = 0; i < wordLen; i++) {
        charValid[i] = getCharDotMatrix(wordBuffer[i], wordCells[i]);
        if (charValid[i]) {
            validCharCount++;
        }
    }

    timer jobTimer;
    jobTimer.clear();
    TouchStatus.on(green);

    // Lock single feed roller in hold mode (and feed lead-in if set).
    if (!advancePaperStrip(LOAD_FEED_DEG, true)) {
        faultCode = FAULT_Y_JAM;
    }

    // Sweep ROW BY ROW across the entire word (Row 0 -> Row 1 -> Row 2).
    // Moving monotonically left-to-right across each row eliminates X chain
    // reversal backlash between dots and matches BrailleBasePlate spacing.
    int row = 0;
    while (row < 3 && faultCode == FAULT_NONE) {
        if (!checkSafetyAndPaper(isPaused, jobTimer, paperThreshold,
                                 distThreshold, colorGen, colorG1,
                                 opticalG2, distGen, sonarG1, distG2)) {
            faultCode = FAULT_SAFETY_TIMEOUT;
        }

        int charIdx = 0;
        while (charIdx < wordLen && faultCode == FAULT_NONE) {
            if (charValid[charIdx] && wordBuffer[charIdx] != ' ') {
                int col = 0;
                while (col < 2 && faultCode == FAULT_NONE) {
                    if (!checkSafetyAndPaper(isPaused, jobTimer,
                                             paperThreshold, distThreshold,
                                             colorGen, colorG1, opticalG2,
                                             distGen, sonarG1, distG2)) {
                        faultCode = FAULT_SAFETY_TIMEOUT;
                    } else {
                        int effectiveCol = col;
                        if (MIRROR_COLUMNS_FOR_DOWNWARD_PUNCH) {
                            effectiveCol = 1 - col;
                        }

                        // Step sideways to this dot slot, then decide
                        // whether to punch down or skip to next slot.
                        if (!moveCarriageToColumn(charIdx, effectiveCol,
                                                  COL_ZERO_DEG,
                                                  INTRA_COL_DEG,
                                                  CHAR_PITCH_DEG)) {
                            faultCode = FAULT_X_STALL;
                        } else if (wordCells[charIdx][row][col]) {
                            if (!punchBrailleDot(calibratedPressDeg,
                                                 PUNCH_DWELL_MSEC)) {
                                faultCode = FAULT_Z_LIFT;
                            } else {
                                totalDotsPunched++;
                                updateLiveTelemetry(wordBuffer[charIdx],
                                                    row, 3,
                                                    totalDotsPunched,
                                                    jobTimer, colorGen,
                                                    colorG1, opticalG2,
                                                    distGen, sonarG1,
                                                    distG2);
                            }
                        } else {
                            wait(100, msec);
                        }
                    }
                    col++;
                }
            }
            charIdx++;
        }

        // After finishing a full horizontal row, return X left and step
        // paper forward by 1 Braille dot row (3.5 mm) for the next row.
        if (faultCode == FAULT_NONE && row < 2) {
            if (!checkSafetyAndPaper(isPaused, jobTimer, paperThreshold,
                                     distThreshold, colorGen, colorG1,
                                     opticalG2, distGen, sonarG1,
                                     distG2)) {
                faultCode = FAULT_SAFETY_TIMEOUT;
            } else if (!moveCarriageToColumn(-1, 0, COL_ZERO_DEG,
                                             INTRA_COL_DEG,
                                             CHAR_PITCH_DEG)) {
                faultCode = FAULT_X_STALL;
            } else if (!advancePaperStrip(ROW_FEED_DEG, true)) {
                faultCode = FAULT_Y_JAM;
            }
        }
        row++;
    }

    int charsCompleted = 0;
    if (faultCode == FAULT_NONE) {
        charsCompleted = validCharCount;
    }

    executeCleanShutdown(charsCompleted, totalDotsPunched,
                         jobTimer.time(seconds), faultCode,
                         colorGen, colorG1, opticalG2);
    return 0;
}
