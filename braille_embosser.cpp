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
motor MotorZ = motor(PORT7, false);
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
 * Automated Desktop Braille Embosser Control Firmware (v13 - Bottom-Zero Z)
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
 *   - Bottom-zero Z homing: start all the way DOWN (0 deg), lift ~1 cm
 *     (+20 deg), home X all the way to the motor side, then step dot by dot
 *   - PORT9  (MotorFeed)   : Single Y-axis roller axle (2x 200 mm wheels)
 *   - PORT10 (MotorX)      : 16T VEX IQ sprocket + 6.35 mm chain
 *   - PORT7  (MotorZ)      : Spur gear + vertical rack stylus into
 *                            3D-printed BrailleBasePlate indents (dia 2.5)
 *   - PORT12 (OpticalPaper): Paper presence / brightness sensor (higher)
 *   - PORT8  (DistanceBed) : Obstruction safety interlock (lower)
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

// Z-axis bottom-zero parameters (start with stylus ALL THE WAY DOWN):
//   0 deg         = bottom contact on paper / base plate indent
//   +Z_LIFT_DEG   = short ~1 cm upward lift (+20 deg on 60T gear)
//   -Z_PRESS_DEG  = slow, full-torque downward press past 0 into die
const double Z_LIFT_DEG           = 20.0;  // ~1 cm upward lift (deg)
const double Z_MIN_CLEAR_DEG      = 8.0;   // Minimum lift to clear paper
const double Z_PRESS_OVERDRIVE    = 18.0;  // Firm press past bottom 0 (deg)
const double POS_TOLERANCE_DEG    = 0.5;   // Tight X/Y precision window
const double STALL_CURRENT_AMP    = 1.25;  // X/Y jam detection current
const double X_HOMING_STALL_AMP   = 1.40;  // Sustained X end-stop current
const double Z_PUNCH_CURRENT_AMP  = 1.75;  // High Z embossing force current
const double HOMING_STALL_AMP     = 0.85;  // Gentle Z lift/snug current
const double OBSTRUCTION_MIN_MM   = 25.0;  // Minimum safe bed clearance
const int    PUNCH_DWELL_MSEC     = 400;   // Active-press dwell time (ms)
const int    Z_POST_LIFT_PAUSE_MS = 150;   // Wait 150 ms AFTER reaching top
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

// Function 1: Bottom-zero Z calibration (start with stylus ALL THE WAY
// DOWN), lifts Z gently ~1 cm (+20 deg), pauses 150 ms, homes X ALL THE
// WAY to the motor-side stop using sustained-stall detection, checks
// DistanceBed clearance, and calibrates OpticalPaper brightness.
bool homeAllAxes(double &calibratedPressDeg, double &paperThreshold) {
    timer homingTimer;
    homingTimer.clear();

    MotorZ.setMaxTorque(100, percent);
    MotorX.setMaxTorque(100, percent);
    TouchStatus.on(yellow);
    OpticalPaper.setLight(ledState::on);
    OpticalPaper.setLightPower(100, percent);

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("v13 Zeroing Z...");

    // 1. Gently snug Z downward (start with stylus all the way down) and
    // define 0.0 deg as exact paper / base plate bottom contact.
    MotorZ.spin(reverse, 15, percent);
    while (homingTimer.time(msec) < 200 &&
           (homingTimer.time(msec) < 100 ||
            MotorZ.current(amp) < HOMING_STALL_AMP)) {
        wait(10, msec);
    }
    MotorZ.stop(hold);
    MotorZ.setPosition(0, degrees);
    wait(60, msec); // Allow VEX IQ smart-port encoder reset to register.
    calibratedPressDeg = Z_PRESS_OVERDRIVE;

    // 2. Lift Z gently upward ~1 cm (+20 deg) and pause 150 ms AFTER it
    // reaches the top so the stylus clears paper cleanly before X moves.
    Brain.Screen.newLine();
    Brain.Screen.print("v13 Lift Z ~1cm");
    homingTimer.clear();
    MotorZ.setMaxTorque(45, percent);
    MotorZ.spin(forward, 20, percent);
    while (MotorZ.position(degrees) < Z_LIFT_DEG &&
           homingTimer.time(msec) < 1500 &&
           (homingTimer.time(msec) < 150 ||
            MotorZ.current(amp) < HOMING_STALL_AMP)) {
        wait(5, msec);
    }
    MotorZ.stop(hold);
    MotorZ.setMaxTorque(100, percent);
    wait(Z_POST_LIFT_PAUSE_MS, msec); // Wait 150 ms after reaching top.

    // 3. Home X-axis carriage ALL THE WAY to the motor-side physical stop
    // using sustained stall (180 ms continuous) so chain drag never stops
    // homing early.
    Brain.Screen.newLine();
    Brain.Screen.print("v13 Homing X...");
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
    MotorX.stop(brake);
    MotorX.setPosition(0, degrees);
    wait(60, msec);

    // Leave single Y-axis roller axle (PORT9) in coast for easy loading.
    MotorFeed.stop(coast);
    MotorFeed.setPosition(0, degrees);

    // 4. Check DistanceBed for startup obstruction (< 25 mm).
    double measuredBedDist = DistanceBed.objectDistance(mm);
    if (measuredBedDist > 0.0 && measuredBedDist < OBSTRUCTION_MIN_MM) {
        TouchStatus.on(red);
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Fault:Bed<25mm");
        wait(1000, msec);
        return false;
    }

    // 5. Auto-calibrate OpticalPaper brightness from loaded paper.
    wait(100, msec);
    double startBright = OpticalPaper.brightness();
    if (startBright >= 20.0) {
        paperThreshold = startBright * 0.5;
    } else {
        paperThreshold = 0.0;
    }

    TouchStatus.on(blue);
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("v13 Ready!");
    Brain.Screen.newLine();
    Brain.Screen.print("Z:+%.0f B:%.0f%%",
                       MotorZ.position(degrees), startBright);
    Brain.Screen.newLine();
    Brain.Screen.print("Dist: %.0fmm", measuredBedDist);
    wait(900, msec);
    return true;
}

// Function 2: Polls TouchStatus (manual pause), OpticalPaper (paper-out),
// and DistanceBed (obstruction < 25 mm). Halts all 3 motors safely on a
// trigger and waits up to 25 s for recovery and TouchStatus confirmation.
bool checkSafetyAndPaper(bool &isPaused, timer &jobTimer,
                         double paperThreshold) {
    if (TouchStatus.pressing()) {
        isPaused = true;
        while (TouchStatus.pressing()) {
            wait(15, msec);
        }
    }

    // Check calibrated brightness threshold; if sensor sits too far to
    // read brightness at startup (paperThreshold == 0), only pause when a
    // blue card (hue 180-260) is held right in front of OpticalPaper.
    double optBright = OpticalPaper.brightness();
    bool paperMissing = false;
    if (paperThreshold > 0.0) {
        paperMissing = (optBright < paperThreshold);
    } else if (OpticalPaper.isNearObject()) {
        double seenHue = OpticalPaper.hue();
        paperMissing = (seenHue >= 180.0 && seenHue <= 260.0);
    }

    double bedClearanceMm = DistanceBed.objectDistance(mm);
    bool obstructionDetected =
        (bedClearanceMm > 0.0 && bedClearanceMm < OBSTRUCTION_MIN_MM);

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
        Brain.Screen.print("PAUSED:Blocked");
    } else {
        Brain.Screen.print("PAUSED:User");
    }

    Brain.Screen.newLine();
    Brain.Screen.print("B:%.0f%% D:%.0fmm", optBright, bedClearanceMm);
    Brain.Screen.newLine();
    Brain.Screen.print("LED=Go Chk=Skip");

    timer pauseTimer;
    pauseTimer.clear();
    bool resumed = false;

    while (pauseTimer.time(seconds) < 25.0 && !resumed) {
        bool paperPresentNow = true;
        if (paperThreshold > 0.0) {
            paperPresentNow = (OpticalPaper.brightness() >= paperThreshold);
        } else if (OpticalPaper.isNearObject()) {
            double nowHue = OpticalPaper.hue();
            paperPresentNow = !(nowHue >= 180.0 && nowHue <= 260.0);
        }

        double currentClearance = DistanceBed.objectDistance(mm);
        bool pathClearNow =
            (currentClearance <= 0.0 ||
             currentClearance >= OBSTRUCTION_MIN_MM);

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
// active Braille row (1-3), completion percentage, cumulative punched
// dots, and elapsed job time (all strings <= 16 chars for Brain LCD).
void updateLiveTelemetry(char currentChar, int rowIndex, int totalRows,
                         int dotsPunched, timer &jobTimer) {
    int percentDone = 0;
    if (totalRows > 0) {
        percentDone = ((rowIndex + 1) * 100) / totalRows;
    }

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Row %d/%d '%c'",
                       rowIndex + 1, totalRows, currentChar);
    Brain.Screen.newLine();
    Brain.Screen.print("Done:%d%% D:%d", percentDone, dotsPunched);
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
    double maxMoveMs = 2500.0 + fabs(targetDeg - currentDeg) * 20.0;

    // Special case: returning to home (0 deg) on the motor side between
    // rows or on shutdown using sustained-stall detection.
    if (charIdx < 0 || targetDeg <= 0.5) {
        MotorX.spin(forward, 26, percent);
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
        MotorX.stop(brake);
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
        int leftSpeed = 20;
        if (currentDeg - preTarget > 30.0) {
            leftSpeed = 24;
        }
        MotorX.spin(forward, leftSpeed, percent);
        while (-MotorX.position(degrees) > preTarget + POS_TOLERANCE_DEG &&
               !stalled) {
            if ((moveTimer.time(msec) > 350 &&
                 fabs(MotorX.velocity(rpm)) < 1.5 &&
                 MotorX.current(amp) > STALL_CURRENT_AMP) ||
                moveTimer.time(msec) > maxMoveMs) {
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
        int rightSpeed = 20;
        if (targetDeg - currentDeg > 30.0) {
            rightSpeed = 24;
        }
        MotorX.spin(reverse, rightSpeed, percent);
        while (-MotorX.position(degrees) < targetDeg - POS_TOLERANCE_DEG &&
               !stalled) {
            if ((moveTimer.time(msec) > 350 &&
                 fabs(MotorX.velocity(rpm)) < 1.5 &&
                 MotorX.current(amp) > STALL_CURRENT_AMP) ||
                moveTimer.time(msec) > maxMoveMs) {
                stalled = true;
            }
            wait(5, msec);
        }
    }

    MotorX.stop(hold);
    return !stalled;
}

// Function 6: Pushes the Z-axis stylus SLOWLY down at full 100% max torque
// into the die indent, keeps steady downward pressure during dwellTimeMsec
// to form a crisp tactile dot, then lifts gently ~1 cm (+20 deg) and waits
// 150 ms AFTER reaching the top BEFORE X moves to the next column.
bool punchBrailleDot(double pressOverdriveDeg, int dwellTimeMsec) {
    timer strokeTimer;
    strokeTimer.clear();

    // Slow, high-force downward embossing stroke (25% speed, 100% torque).
    MotorZ.setMaxTorque(100, percent);
    MotorZ.spin(reverse, 25, percent);
    while (MotorZ.position(degrees) > -pressOverdriveDeg &&
           strokeTimer.time(msec) < 1500 &&
           (strokeTimer.time(msec) < 200 ||
            MotorZ.current(amp) < Z_PUNCH_CURRENT_AMP)) {
        wait(5, msec);
    }

    // Maintain slow, full-torque downward pressure during dwell so paper
    // fibers deform deeply into the indent instead of springing back.
    MotorZ.spin(reverse, 20, percent);
    timer dwellTimer;
    dwellTimer.clear();
    while (dwellTimer.time(msec) < dwellTimeMsec) {
        wait(5, msec);
    }
    MotorZ.stop(hold);
    wait(60, msec); // Release downward gear preload before lifting gently.

    // Lift stylus gently back up ~1 cm (+20 deg) at reduced upward torque,
    // then wait 150 ms AFTER reaching the top before X moves sideways.
    strokeTimer.clear();
    MotorZ.setMaxTorque(45, percent);
    MotorZ.spin(forward, 20, percent);
    while (MotorZ.position(degrees) < Z_LIFT_DEG &&
           strokeTimer.time(msec) < 1500 &&
           (strokeTimer.time(msec) < 150 ||
            MotorZ.current(amp) < HOMING_STALL_AMP)) {
        wait(5, msec);
    }
    MotorZ.stop(hold);
    MotorZ.setMaxTorque(100, percent);
    wait(Z_POST_LIFT_PAUSE_MS, msec); // Wait 150 ms AFTER reaching top.

    // Fail only if Z genuinely failed to clear the paper (< 8 deg lift).
    return (MotorZ.position(degrees) >= Z_MIN_CLEAR_DEG);
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

// Function 9: Executes a safe non-abort shutdown. Holds Z at +Z_LIFT_DEG
// clearance, ejects the completed strip via MotorFeed (unless faulted),
// parks X-axis at 0 deg, sets all 3 motors to coast, displays job stats
// (and specific fault cause if faultCode != 0), and calls programStop().
void executeCleanShutdown(int totalCharsEmbossed, int totalDotsPunched,
                          double totalTimeSec, int faultCode) {
    timer shutdownTimer;
    shutdownTimer.clear();

    // Ensure Z-axis stylus is lifted gently to +Z_LIFT_DEG before eject.
    if (MotorZ.position(degrees) < Z_LIFT_DEG) {
        MotorZ.setMaxTorque(45, percent);
        MotorZ.spin(forward, 20, percent);
        while (MotorZ.position(degrees) < Z_LIFT_DEG &&
               shutdownTimer.time(msec) < 2000 &&
               (shutdownTimer.time(msec) < 200 ||
                MotorZ.current(amp) < HOMING_STALL_AMP)) {
            wait(10, msec);
        }
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

    // De-energize all 3 motors and optical LED for safe handling.
    MotorFeed.stop(coast);
    MotorX.stop(coast);
    MotorZ.stop(coast);
    OpticalPaper.setLight(ledState::off);

    // Display final summary (<= 16 chars/line) and audio/visual cue.
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    if (faultCode != FAULT_NONE) {
        TouchStatus.on(red);
        Brain.playSound(siren);
        if (faultCode == FAULT_BED_BLOCKED) {
            Brain.Screen.print("FAULT:Bed<25mm");
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

    wait(3000, msec);
    Brain.programStop();
}

// =========================================================================
// Main Control Loop (Row-by-Row Horizontal Word Embossing)
// =========================================================================

int main() {
    vexcodeInit();

    double calibratedPressDeg = Z_PRESS_OVERDRIVE;
    double paperThreshold = 0.0;
    char wordBuffer[MAX_WORD_LENGTH];
    bool wordCells[MAX_WORD_LENGTH][3][2];
    bool charValid[MAX_WORD_LENGTH];
    bool isPaused = false;
    int faultCode = FAULT_NONE;
    int totalDotsPunched = 0;
    int validCharCount = 0;

    if (!homeAllAxes(calibratedPressDeg, paperThreshold)) {
        executeCleanShutdown(0, 0, 0.0, FAULT_BED_BLOCKED);
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
        if (!checkSafetyAndPaper(isPaused, jobTimer, paperThreshold)) {
            faultCode = FAULT_SAFETY_TIMEOUT;
        }

        int charIdx = 0;
        while (charIdx < wordLen && faultCode == FAULT_NONE) {
            if (charValid[charIdx] && wordBuffer[charIdx] != ' ') {
                int col = 0;
                while (col < 2 && faultCode == FAULT_NONE) {
                    int effectiveCol = col;
                    if (MIRROR_COLUMNS_FOR_DOWNWARD_PUNCH) {
                        effectiveCol = 1 - col;
                    }

                    // Step sideways to this dot slot, then decide whether
                    // to punch down or skip and move to the next slot.
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
                            updateLiveTelemetry(wordBuffer[charIdx], row,
                                                3, totalDotsPunched,
                                                jobTimer);
                        }
                    } else {
                        wait(100, msec);
                    }
                    col++;
                }
            }
            charIdx++;
        }

        // After finishing a full horizontal row, return X left and step
        // paper forward by 1 Braille dot row (3.5 mm) for the next row.
        if (faultCode == FAULT_NONE && row < 2) {
            if (!moveCarriageToColumn(-1, 0, COL_ZERO_DEG,
                                      INTRA_COL_DEG, CHAR_PITCH_DEG)) {
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
                         jobTimer.time(seconds), faultCode);
    return 0;
}
