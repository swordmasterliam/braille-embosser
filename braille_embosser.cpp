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
motor MotorFeed = motor(PORT9, false);
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
 * Automated Desktop Braille Embosser Control Firmware (Row-by-Row Mode)
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
 *   - PORT9  (MotorFeed)   : Single Y-axis roller axle (2x 200 mm VEX IQ
 *                            travel wheels); active hold secures paper
 *   - PORT10 (MotorX)      : 16T VEX IQ sprocket (6.35 mm pitch chain,
 *                            101.6 mm/rev) driving bearing carriage
 *   - PORT7  (MotorZ)      : 60T blue spur gear & vertical rack stylus into
 *                            3D-printed BrailleBasePlate (pre-made indents)
 *   - PORT12 (OpticalPaper): Paper presence / brightness sensor (higher)
 *   - PORT8  (DistanceBed) : Bed height calibration & obstruction (lower)
 *   - PORT11 (TouchStatus) : User Start/Pause input & RGB status indicator
 */

#include <cmath>

// =========================================================================
// Physical Geometry & Calibration Constants (read-only; no mutable globals)
// =========================================================================
// TUNE THESE VALUES FIRST ON THE PHYSICAL ROBOT:
//   1. COL_ZERO_MM     : Ruler distance (mm) from X home stop to Hole 0
//   2. DOT_PITCH_MM    : Center-to-center hole pitch (2.5 mm Braille grid;
//                        set to 25.0 if your plate is 2.5 cm spacing)
//   3. CHAR_STEP_HOLES : Grid holes from Letter i Col 0 to Letter i+1 Col 0
//                        (3 holes = 2 dot cols + 1 blank gap hole = 7.5 mm)
const double DOT_PITCH_MM         = 2.5;   // Center-to-center pitch (mm)
const double COL_ZERO_MM          = 10.0;  // X home stop -> Hole 0 (mm)
const int    CHAR_STEP_HOLES      = 3;     // Grid holes per character cell
const double LOAD_FEED_MM         = 0.0;   // Startup feed-in (mm)
const double EJECT_FEED_MM        = 120.0; // Final label ejection (mm)

// Exact VEX IQ drive kinematics from physical hardware:
//   X: 16-tooth VEX IQ sprocket (228-2500-233) * 6.35 mm chain = 101.6 mm
//   Y: VEX IQ 200 mm travel wheel (1:1 direct axle)            = 200.0 mm
const double X_MM_PER_REV         = 101.6;
const double Y_MM_PER_REV         = 200.0;

// Derived encoder angle targets (degrees):
//   At 2.5 mm pitch: X hole step = 8.86 deg, Y row step = 4.50 deg
const double COL_ZERO_DEG         = (COL_ZERO_MM / X_MM_PER_REV) * 360.0;
const double COL_STEP_DEG         = (DOT_PITCH_MM / X_MM_PER_REV) * 360.0;
const double LOAD_FEED_DEG        = (LOAD_FEED_MM / Y_MM_PER_REV) * 360.0;
const double ROW_FEED_DEG         = (DOT_PITCH_MM / Y_MM_PER_REV) * 360.0;
const double EJECT_FEED_DEG       = (EJECT_FEED_MM / Y_MM_PER_REV) * 360.0;

// Anti-backlash approach overshoot when returning left on VEX chain (deg).
const double X_BACKLASH_DEG       = 8.0;

// Z-axis punch depth and post-dot lift clearance (degrees from top home 0).
// 60T VEX gear travels ~0.665 mm/deg: 35 deg = ~23 mm downward stroke.
// Start with the dot pusher RAISED TO THE MAX (still meshed with gear).
const double DEFAULT_PUNCH_DEG    = 35.0;  // Nominal downward Z stroke
const double MIN_PUNCH_DEG        = 25.0;  // Minimum calibrated Z stroke
const double MAX_PUNCH_DEG        = 55.0;  // Maximum calibrated Z stroke
const double Z_LIFT_TARGET_DEG    = 0.0;   // Target angle after dot lift
const double POS_TOLERANCE_DEG    = 0.5;   // Tight X/Y precision window
const double Z_HOME_TOLERANCE_DEG = 2.0;   // Z lift/return window
const double STALL_CURRENT_AMP    = 1.15;  // Jam threshold during motion
const double HOMING_STALL_AMP     = 0.80;  // Hard-stop homing threshold
const double PAPER_BRIGHTNESS_MIN = 8.0;   // Optical brightness min (%)
const double OBSTRUCTION_MIN_MM   = 25.0;  // Minimum safe bed clearance
const int    PUNCH_DWELL_MSEC     = 300;   // Dwell time to form dot (ms)
const int    MAX_WORD_LENGTH      = 16;    // Max queued chars per job
const bool   MIRROR_COLUMNS_FOR_DOWNWARD_PUNCH = false;

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

// Function 1: Homes Z-axis and X-axis against mechanical end-stops using
// motor current stall polling (without a top bumper switch), zeroes all
// three motor encoders, leaves MotorFeed in coast for easy paper loading,
// checks bed clearance, and calibrates Z stroke depth.
bool homeAllAxes(double &calibratedPunchDeg, double &paperThreshold) {
    timer homingTimer;
    homingTimer.clear();

    TouchStatus.on(yellow);
    OpticalPaper.setLight(ledState::on);
    OpticalPaper.setLightPower(100, percent);

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("v6 Homing Z...");

    // Gently snug Z-axis stylus upward (start with pusher raised to max)
    // using a short 400 ms guard so the rack never runs off the 60T gear.
    MotorZ.spin(forward, 15, percent);
    while (homingTimer.time(msec) < 400 &&
           (homingTimer.time(msec) < 150 ||
            MotorZ.current(amp) < HOMING_STALL_AMP)) {
        wait(10, msec);
    }
    MotorZ.stop(hold);
    MotorZ.setPosition(0, degrees);

    // Home X-axis bearing carriage ALL THE WAY to the left end-stop.
    // Checks actual shaft velocity (< 2 rpm) + current (>= 1.15 A) after
    // 350 ms so brief chain friction mid-rail never stops homing early.
    Brain.Screen.newLine();
    Brain.Screen.print("v6 Homing X...");
    homingTimer.clear();
    MotorX.spin(forward, 22, percent);
    while (homingTimer.time(msec) < 6000 &&
           (homingTimer.time(msec) < 350 ||
            (fabs(MotorX.velocity(rpm)) > 2.0 &&
             MotorX.current(amp) < STALL_CURRENT_AMP))) {
        wait(15, msec);
    }
    MotorX.stop(brake);
    MotorX.setPosition(0, degrees);

    // Leave single Y-axis roller axle (PORT9) in coast for easy loading.
    MotorFeed.stop(coast);
    MotorFeed.setPosition(0, degrees);

    // Auto-calibrate OpticalPaper brightness threshold from loaded paper.
    // Only arm brightness threshold if sensor reads >= 20% reliably.
    wait(120, msec);
    double startBright = OpticalPaper.brightness();
    if (startBright >= 20.0) {
        paperThreshold = startBright * 0.5;
    } else {
        paperThreshold = 0.0;
    }

    // Verify embossing zone is unobstructed and calibrate Z punch stroke.
    calibratedPunchDeg = DEFAULT_PUNCH_DEG;
    double measuredBedDist = DistanceBed.objectDistance(mm);
    if (measuredBedDist > 0.0 && measuredBedDist < OBSTRUCTION_MIN_MM) {
        TouchStatus.on(red);
        Brain.Screen.newLine();
        Brain.Screen.print("Fault: Bed < 25mm");
        wait(1000, msec);
        return false;
    }

    if (measuredBedDist >= 35.0 && measuredBedDist <= 95.0) {
        calibratedPunchDeg =
            DEFAULT_PUNCH_DEG + (measuredBedDist - 60.0) * 0.4;
        if (calibratedPunchDeg < MIN_PUNCH_DEG) {
            calibratedPunchDeg = MIN_PUNCH_DEG;
        } else if (calibratedPunchDeg > MAX_PUNCH_DEG) {
            calibratedPunchDeg = MAX_PUNCH_DEG;
        }
    }

    TouchStatus.on(blue);
    Brain.Screen.newLine();
    Brain.Screen.print("B:%.0f%% H:%.0f D:%.0f",
                       startBright, OpticalPaper.hue(), measuredBedDist);
    wait(1000, msec);
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
    // bright blue card (hue 180-260) is held right in front of the sensor.
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
        Brain.Screen.print("PAUSED: Paper Out");
    } else if (obstructionDetected) {
        Brain.Screen.print("PAUSED: Obstruction");
    } else {
        Brain.Screen.print("PAUSED: User Request");
    }

    Brain.Screen.newLine();
    Brain.Screen.print("B:%.0f%% D:%.0fmm", optBright, bedClearanceMm);
    Brain.Screen.newLine();
    Brain.Screen.print("LED=Resume Check=Skip");

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
// buttons, then start embossing with TouchStatus (or Brain Down / FDown).
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
        Brain.Screen.print("Select Char: [%c]", previewChar);
        Brain.Screen.newLine();
        Brain.Screen.print("Queued (%d): ", charCount);

        for (int i = 0; i < charCount; i++) {
            Brain.Screen.print("%c", wordBuffer[i]);
        }

        Brain.Screen.newLine();
        Brain.Screen.print("Tap LED/Down to Start");

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

        // Confirm queued word via TouchStatus, Brain Down, or FDown.
        bool startTriggered =
            TouchStatus.pressing() ||
            Brain.buttonDown.pressing() ||
            Controller.ButtonFDown.pressing();

        if (startTriggered) {
            while (TouchStatus.pressing() ||
                   Brain.buttonDown.pressing() ||
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
// dots, and elapsed job time.
void updateLiveTelemetry(char currentChar, int rowIndex, int totalRows,
                         int dotsPunched, timer &jobTimer) {
    int percentDone = 0;
    if (totalRows > 0) {
        percentDone = ((rowIndex + 1) * 100) / totalRows;
    }

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Row %d/%d | Char: '%c'",
                       rowIndex + 1, totalRows, currentChar);
    Brain.Screen.newLine();
    Brain.Screen.print("Progress:  %d%%", percentDone);
    Brain.Screen.newLine();
    Brain.Screen.print("Dots: %d | Time: %.1fs",
                       dotsPunched, jobTimer.time(seconds));
}

// =========================================================================
// Section 3: X-Axis Chain Carriage and Z-Axis Rack-and-Pinion Punch Control
// Author: Nicholas Chan
// =========================================================================

// Function 5: Positions the bearing-guided X-axis carriage over any
// horizontal grid hole (gridCol = 0, 1, 3, 4, ...) along the 2.5 mm
// BrailleBasePlate via the 16T VEX sprocket within 0.5 deg. Uses
// single-direction anti-backlash approach when returning left.
bool moveCarriageToColumn(int gridCol, double colZeroDeg,
                          double colStepDeg) {
    double targetDeg = colZeroDeg + (gridCol * colStepDeg);
    if (targetDeg < 0.0) {
        targetDeg = 0.0;
    }

    timer moveTimer;
    moveTimer.clear();
    bool stalled = false;
    double currentDeg = -MotorX.position(degrees);
    double maxMoveMs = 2000.0 + fabs(targetDeg - currentDeg) * 20.0;

    // If moving left (decreasing angle), overshoot past target by
    // X_BACKLASH_DEG (clamped >= 0) so the final approach is always
    // from the same tensioned side of the VEX chain.
    if (targetDeg < currentDeg - POS_TOLERANCE_DEG) {
        double preTarget = targetDeg - X_BACKLASH_DEG;
        if (preTarget < 0.0) {
            preTarget = 0.0;
        }
        int leftSpeed = 8;
        if (currentDeg - preTarget > 25.0) {
            leftSpeed = 12;
        }
        MotorX.spin(forward, leftSpeed, percent);
        while (-MotorX.position(degrees) > preTarget + POS_TOLERANCE_DEG &&
               !stalled) {
            if ((moveTimer.time(msec) > 150 &&
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
        int rightSpeed = 8;
        if (targetDeg - currentDeg > 25.0) {
            rightSpeed = 12;
        }
        MotorX.spin(reverse, rightSpeed, percent);
        while (-MotorX.position(degrees) < targetDeg - POS_TOLERANCE_DEG &&
               !stalled) {
            if ((moveTimer.time(msec) > 150 &&
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

// Function 6: Drives the Z-axis 60T gear and vertical rack downward to
// targetDepthDeg, dwells for dwellTimeMsec to form a crisp Braille bump
// inside the 3D-printed base indent, and lifts cleanly back up to
// Z_LIFT_TARGET_DEG (0 deg top clearance) before X moves horizontally.
bool punchBrailleDot(double targetDepthDeg, int dwellTimeMsec) {
    timer strokeTimer;
    strokeTimer.clear();

    // Downward embossing stroke into the 3D-printed indented base plate
    // (stops cleanly at targetDepthDeg or if stylus seats firmly in die).
    MotorZ.spin(reverse, 22, percent);
    while (-MotorZ.position(degrees) < targetDepthDeg &&
           strokeTimer.time(msec) < 2000 &&
           (strokeTimer.time(msec) < 150 ||
            MotorZ.current(amp) < STALL_CURRENT_AMP)) {
        wait(5, msec);
    }
    MotorZ.stop(hold);

    // Dwell at full depth so paper fibers retain a raised tactile bump.
    timer dwellTimer;
    dwellTimer.clear();
    while (dwellTimer.time(msec) < dwellTimeMsec) {
        wait(5, msec);
    }

    // Lift stylus back up to Z_LIFT_TARGET_DEG (0 deg = full top lift)
    // so it completely clears the paper before lateral carriage motion.
    strokeTimer.clear();
    bool hitTopStop = false;
    MotorZ.spin(forward, 25, percent);
    while (-MotorZ.position(degrees) >
               Z_LIFT_TARGET_DEG + Z_HOME_TOLERANCE_DEG &&
           strokeTimer.time(msec) < 2500 && !hitTopStop) {
        if (strokeTimer.time(msec) > 150 &&
            MotorZ.current(amp) >= HOMING_STALL_AMP) {
            hitTopStop = true;
        }
        wait(5, msec);
    }
    MotorZ.stop(hold);

    if (hitTopStop) {
        MotorZ.setPosition(0, degrees);
        return true;
    }
    return (-MotorZ.position(degrees) <=
            Z_LIFT_TARGET_DEG + Z_HOME_TOLERANCE_DEG);
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
// (PORT1, 2x 200 mm VEX travel wheels) by feedDegrees within 0.5 deg with
// 150 ms inrush-masked jam detection. Uses slow speed (10%) on short row
// steps to avoid overshoot and 25% on long ejection moves. Locks in hold
// mode during punching when holdAfterFeed is true, or coasts when false.
bool advancePaperStrip(double feedDegrees, bool holdAfterFeed) {
    if (feedDegrees > 0.0) {
        double targetY = MotorFeed.position(degrees) + feedDegrees;
        int feedSpeed = 10;
        if (feedDegrees > 45.0) {
            feedSpeed = 25;
        }
        double maxFeedMs = 1500.0 + feedDegrees * 12.0;

        timer feedTimer;
        feedTimer.clear();

        MotorFeed.spin(forward, feedSpeed, percent);
        while (MotorFeed.position(degrees) < targetY - POS_TOLERANCE_DEG &&
               feedTimer.time(msec) < maxFeedMs) {
            if (feedTimer.time(msec) > 150 &&
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

// Function 9: Executes a safe non-abort shutdown. Retracts Z-axis punch,
// ejects the completed strip via MotorFeed (unless a fault occurred),
// parks X-axis at 0 deg, sets all 3 motors to coast, displays job stats,
// and terminates cleanly via Brain.programStop().
void executeCleanShutdown(int totalCharsEmbossed, int totalDotsPunched,
                          double totalTimeSec, bool faultOccurred) {
    timer shutdownTimer;
    shutdownTimer.clear();

    // Ensure Z-axis stylus is fully lifted before moving paper or X.
    MotorZ.spin(forward, 25, percent);
    while (-MotorZ.position(degrees) > Z_HOME_TOLERANCE_DEG &&
           shutdownTimer.time(msec) < 2000 &&
           (shutdownTimer.time(msec) < 150 ||
            MotorZ.current(amp) < HOMING_STALL_AMP)) {
        wait(10, msec);
    }
    MotorZ.stop(brake);

    // Eject finished Braille strip on normal completion; release on fault.
    if (!faultOccurred) {
        advancePaperStrip(EJECT_FEED_DEG, false);
    } else {
        advancePaperStrip(0.0, false);
    }

    // Park X-axis carriage at left home position.
    moveCarriageToColumn(0, 0.0, COL_STEP_DEG);

    // De-energize all 3 motors and optical LED for safe handling.
    MotorFeed.stop(coast);
    MotorX.stop(coast);
    MotorZ.stop(coast);
    OpticalPaper.setLight(ledState::off);

    // Display final summary and audio/visual completion cue.
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    if (faultOccurred) {
        TouchStatus.on(red);
        Brain.playSound(siren);
        Brain.Screen.print("SHUTDOWN: FAULT HALT");
    } else {
        TouchStatus.on(blue);
        Brain.playSound(tada);
        Brain.Screen.print("JOB COMPLETE - EJECTED");
    }
    Brain.Screen.newLine();
    Brain.Screen.print("Chars: %d | Dots: %d",
                       totalCharsEmbossed, totalDotsPunched);
    Brain.Screen.newLine();
    Brain.Screen.print("Runtime: %.1f sec", totalTimeSec);

    wait(3000, msec);
    Brain.programStop();
}

// =========================================================================
// Main Control Loop (Row-by-Row Horizontal Word Embossing)
// =========================================================================

int main() {
    vexcodeInit();

    double calibratedPunchDeg = DEFAULT_PUNCH_DEG;
    double paperThreshold = PAPER_BRIGHTNESS_MIN;
    char wordBuffer[MAX_WORD_LENGTH];
    bool wordCells[MAX_WORD_LENGTH][3][2];
    bool charValid[MAX_WORD_LENGTH];
    bool isPaused = false;
    bool systemFault = false;
    int totalDotsPunched = 0;
    int validCharCount = 0;

    if (!homeAllAxes(calibratedPunchDeg, paperThreshold)) {
        executeCleanShutdown(0, 0, 0.0, true);
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
        systemFault = true;
    }

    // Sweep ROW BY ROW across the entire word (Row 0 -> Row 1 -> Row 2).
    // Moving monotonically left-to-right across each row eliminates X chain
    // reversal backlash between dots and matches the linear hole plate.
    int row = 0;
    while (row < 3 && !systemFault) {
        if (!checkSafetyAndPaper(isPaused, jobTimer, paperThreshold)) {
            systemFault = true;
        }

        int charIdx = 0;
        while (charIdx < wordLen && !systemFault) {
            if (charValid[charIdx]) {
                int col = 0;
                while (col < 2 && !systemFault) {
                    if (wordCells[charIdx][row][col]) {
                        int effectiveCol = col;
                        if (MIRROR_COLUMNS_FOR_DOWNWARD_PUNCH) {
                            effectiveCol = 1 - col;
                        }
                        int gridCol =
                            charIdx * CHAR_STEP_HOLES + effectiveCol;

                        if (!moveCarriageToColumn(gridCol, COL_ZERO_DEG,
                                                  COL_STEP_DEG)) {
                            systemFault = true;
                        } else if (!punchBrailleDot(calibratedPunchDeg,
                                                    PUNCH_DWELL_MSEC)) {
                            systemFault = true;
                        } else {
                            totalDotsPunched++;
                            updateLiveTelemetry(wordBuffer[charIdx], row,
                                                3, totalDotsPunched,
                                                jobTimer);
                        }
                    }
                    col++;
                }
            }
            charIdx++;
        }

        // After finishing a full horizontal row, return X left and step
        // paper forward by 1 Braille dot row (2.5 mm) for the next row.
        if (!systemFault && row < 2) {
            if (!moveCarriageToColumn(0, 0.0, COL_STEP_DEG)) {
                systemFault = true;
            } else if (!advancePaperStrip(ROW_FEED_DEG, true)) {
                systemFault = true;
            }
        }
        row++;
    }

    int charsCompleted = 0;
    if (!systemFault) {
        charsCompleted = validCharCount;
    }

    executeCleanShutdown(charsCompleted, totalDotsPunched,
                         jobTimer.time(seconds), systemFault);
    return 0;
}
