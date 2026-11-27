#pragma region VEXcode Generated Robot Configuration
// Standard VEXcode IQ C++ headers.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "vex.h"

using namespace vex;

// Default VEX IQ Brain instance.
brain Brain;

// START IQ MACROS
#define waitUntil(condition)                                             \
  do {                                                                   \
    wait(5, msec);                                                       \
  } while (!(condition))

#define repeat(iterations)                                               \
  for (int iterator = 0; iterator < iterations; iterator++)
// END IQ MACROS

// Robot hardware port configuration (4 motors, 3 peripheral sensors +
// built-in encoders, Brain buttons, and VEX Controller).
// NOTE: Flip the second arg of MotorFeedOut to true if the exit roller
// motor is mounted facing the opposite side of the Tetrix frame.
inertial BrainInertial = inertial();
motor MotorFeedIn = motor(PORT1, false);
motor MotorX = motor(PORT2, false);
motor MotorZ = motor(PORT3, false);
motor MotorFeedOut = motor(PORT4, false);
optical OpticalPaper = optical(PORT5);
distance DistanceBed = distance(PORT6);
touchled TouchStatus = touchled(PORT8);
controller Controller = controller();

// Generates and sets the random seed using inertial accelerometer data.
void initializeRandomSeed() {
  wait(100, msec);
  double xAxis = BrainInertial.acceleration(xaxis) * 1000;
  double yAxis = BrainInertial.acceleration(yaxis) * 1000;
  double zAxis = BrainInertial.acceleration(zaxis) * 1000;
  int seed = int(xAxis + yAxis + zAxis);
  srand(seed);
}

// Converts a VEX color object to a readable C-string.
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

// Converts a VEX colorType enum value to a readable C-string.
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
 * Automated Desktop Braille Embosser Control Firmware
 *
 * Team Members:
 *   - Joseph Lu         : homeAllAxes, checkSafetyAndPaper
 *   - Junwon Will Choi  : selectWordFromController, updateLiveTelemetry
 *   - Nicholas Chan     : moveCarriageToColumn, punchBrailleDot
 *   - Liam McCarthy     : getCharDotMatrix, advancePaperStrip,
 *                         executeCleanShutdown
 *
 * Mechanical Configuration (Final Revision - Tetrix Prime):
 *   - Tetrix Prime aluminum frame with 3D-printed bearing-guided carriage
 *   - PORT1 (MotorFeedIn) : Entrance Y-axis roller (pulls strip in)
 *   - PORT2 (MotorX)      : Lowered VEX IQ belt drive with VEX sprocket
 *   - PORT3 (MotorZ)      : 60-tooth spur gear & vertical rack stylus into
 *                           3D-printed base plate with pre-made indents
 *                           (encoder + stall-stop homing replaces bumper)
 *   - PORT4 (MotorFeedOut): Exit Y-axis roller (tensions strip with PORT1
 *                           in active hold, replacing clamp, & ejects out)
 *   - PORT5 (OpticalPaper): Paper presence / brightness sensor
 *   - PORT6 (DistanceBed) : Bed height calibration & obstruction interlock
 *   - PORT8 (TouchStatus) : User Start/Pause input & RGB status indicator
 */

#include <cmath>

// Calibration and safety constants (read-only; no mutable globals).
const double COL_ZERO_DEG         = 35.0;  // Left Braille column (deg)
const double COL_ONE_DEG          = 80.0;  // Right Braille column (deg)
const double LOAD_FEED_DEG        = 90.0;  // Initial pull-in (0 if pre-set)
const double ROW_FEED_DEG         = 45.0;  // 2.5 mm dot-row paper feed
const double CHAR_GAP_FEED_DEG    = 90.0;  // 5.0 mm inter-character gap
const double EJECT_FEED_DEG       = 540.0; // Exit-roller ejection rotation
const double DEFAULT_PUNCH_DEG    = 115.0; // Nominal Z-axis punch stroke
const double MIN_PUNCH_DEG        = 100.0; // Minimum calibrated Z stroke
const double MAX_PUNCH_DEG        = 130.0; // Maximum calibrated Z stroke
const double POS_TOLERANCE_DEG    = 2.5;   // Closed-loop encoder window
const double STALL_CURRENT_AMP    = 1.15;  // Jam threshold during motion
const double HOMING_STALL_AMP     = 0.80;  // Hard-stop homing threshold
const double PAPER_BRIGHTNESS_MIN = 30.0;  // Optical paper threshold (%)
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
// four motor encoders, releases entrance/exit rollers to coast for easy
// paper loading, checks bed clearance, and calibrates Z stroke depth.
bool homeAllAxes(double &calibratedPunchDeg) {
    timer homingTimer;
    homingTimer.clear();

    if (TouchStatus.installed()) {
        TouchStatus.on(yellow);
    }
    if (OpticalPaper.installed()) {
        OpticalPaper.setLight(ledState::on);
    }

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Homing Axes...");

    // Retract Z-axis stylus upward to upper mechanical stop via current
    // stall sensing after a 150 ms motor inrush mask (2500 ms max).
    if (MotorZ.installed()) {
        MotorZ.spin(forward, 12, percent);
        while (homingTimer.time(msec) < 2500 &&
               (homingTimer.time(msec) < 150 ||
                MotorZ.current(amp) < HOMING_STALL_AMP)) {
            wait(10, msec);
        }
        MotorZ.stop(hold);
        MotorZ.setPosition(0, degrees);
    }

    // Home X-axis bearing carriage to left end-stop via VEX belt drive.
    if (MotorX.installed()) {
        homingTimer.clear();
        MotorX.spin(forward, 15, percent);
        while (homingTimer.time(msec) < 4000 &&
               (homingTimer.time(msec) < 150 ||
                MotorX.current(amp) < HOMING_STALL_AMP)) {
            wait(10, msec);
        }
        MotorX.stop(brake);
        MotorX.setPosition(0, degrees);
    }

    // Leave entrance (PORT1) & exit (PORT4) rollers in coast for loading.
    MotorFeedIn.stop(coast);
    MotorFeedIn.setPosition(0, degrees);
    if (MotorFeedOut.installed()) {
        MotorFeedOut.stop(coast);
        MotorFeedOut.setPosition(0, degrees);
    }

    // Verify embossing zone is unobstructed and calibrate Z punch stroke.
    calibratedPunchDeg = DEFAULT_PUNCH_DEG;
    if (DistanceBed.installed()) {
        double measuredBedDist = DistanceBed.objectDistance(mm);
        if (measuredBedDist > 0.0 &&
            measuredBedDist < OBSTRUCTION_MIN_MM) {
            if (TouchStatus.installed()) {
                TouchStatus.on(red);
            }
            Brain.Screen.newLine();
            Brain.Screen.print("Fault: Bed Obstructed");
            return false;
        }

        if (measuredBedDist >= 35.0 && measuredBedDist <= 95.0) {
            calibratedPunchDeg =
                DEFAULT_PUNCH_DEG + (measuredBedDist - 60.0) * 1.4;
            if (calibratedPunchDeg < MIN_PUNCH_DEG) {
                calibratedPunchDeg = MIN_PUNCH_DEG;
            } else if (calibratedPunchDeg > MAX_PUNCH_DEG) {
                calibratedPunchDeg = MAX_PUNCH_DEG;
            }
        }
    }

    if (TouchStatus.installed()) {
        TouchStatus.on(blue);
    }
    Brain.Screen.newLine();
    Brain.Screen.print("Homing Complete.");
    return true;
}

// Function 2: Polls TouchStatus (manual pause), OpticalPaper (paper-out),
// and DistanceBed (obstruction < 25 mm). Halts all 4 motors safely on a
// trigger and waits up to 25 s for recovery and TouchStatus confirmation.
bool checkSafetyAndPaper(bool &isPaused, timer &jobTimer) {
    if (TouchStatus.installed() && TouchStatus.pressing()) {
        isPaused = true;
        while (TouchStatus.pressing()) {
            wait(15, msec);
        }
    }

    bool paperMissing = false;
    if (OpticalPaper.installed()) {
        paperMissing =
            (!OpticalPaper.isNearObject() ||
             OpticalPaper.brightness() < PAPER_BRIGHTNESS_MIN);
    }

    bool obstructionDetected = false;
    if (DistanceBed.installed()) {
        double bedClearanceMm = DistanceBed.objectDistance(mm);
        obstructionDetected =
            (bedClearanceMm > 0.0 && bedClearanceMm < OBSTRUCTION_MIN_MM);
    }

    if (paperMissing || obstructionDetected) {
        isPaused = true;
    }

    if (!isPaused) {
        return true;
    }

    // Enter safe pause state: brake X, hold Z and both feed rollers.
    MotorX.stop(brake);
    MotorFeedIn.stop(hold);
    if (MotorFeedOut.installed()) {
        MotorFeedOut.stop(hold);
    }
    MotorZ.stop(hold);
    if (TouchStatus.installed()) {
        TouchStatus.on(yellow);
    }

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
    Brain.Screen.print("Load paper & tap LED");
    Brain.Screen.newLine();
    Brain.Screen.print("Job Time: %.1fs", jobTimer.time(seconds));

    timer pauseTimer;
    pauseTimer.clear();
    bool resumed = false;

    while (pauseTimer.time(seconds) < 25.0 && !resumed) {
        bool paperPresentNow = true;
        if (OpticalPaper.installed()) {
            paperPresentNow =
                (OpticalPaper.isNearObject() &&
                 OpticalPaper.brightness() >= PAPER_BRIGHTNESS_MIN);
        }

        bool pathClearNow = true;
        if (DistanceBed.installed()) {
            double currentClearance = DistanceBed.objectDistance(mm);
            pathClearNow =
                (currentClearance <= 0.0 ||
                 currentClearance >= OBSTRUCTION_MIN_MM);
        }

        bool resumePressed =
            (TouchStatus.installed() && TouchStatus.pressing()) ||
            Brain.buttonCheck.pressing();

        if (resumePressed && paperPresentNow && pathClearNow) {
            while ((TouchStatus.installed() && TouchStatus.pressing()) ||
                   Brain.buttonCheck.pressing()) {
                wait(15, msec);
            }
            isPaused = false;
            resumed = true;
            if (TouchStatus.installed()) {
                TouchStatus.on(green);
            }
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

    if (TouchStatus.installed()) {
        TouchStatus.on(blue);
    }
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
            (TouchStatus.installed() && TouchStatus.pressing()) ||
            Brain.buttonDown.pressing() ||
            Controller.ButtonFDown.pressing();

        if (startTriggered) {
            while ((TouchStatus.installed() && TouchStatus.pressing()) ||
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
// percentage completion, cumulative punched dots, and elapsed job time.
void updateLiveTelemetry(char currentChar, int charIndex, int totalChars,
                         int dotsPunched, timer &jobTimer) {
    int percentDone = 0;
    if (totalChars > 0) {
        percentDone = ((charIndex + 1) * 100) / totalChars;
    }

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Embossing: '%c' (%d/%d)",
                       currentChar, charIndex + 1, totalChars);
    Brain.Screen.newLine();
    Brain.Screen.print("Progress:  %d%%", percentDone);
    Brain.Screen.newLine();
    Brain.Screen.print("Dots: %d | Time: %.1fs",
                       dotsPunched, jobTimer.time(seconds));
}

// =========================================================================
// Section 3: X-Axis Belt Carriage and Z-Axis Rack-and-Pinion Punch Control
// Author: Nicholas Chan
// =========================================================================

// Function 5: Positions the bearing-guided X-axis carriage at Column 0 or
// Column 1 via the lowered VEX IQ belt drive, monitoring encoder target,
// 150 ms inrush-masked stall current, and a 2000 ms safety timeout.
bool moveCarriageToColumn(int colIndex, double colZeroDeg,
                          double colOneDeg) {
    int effectiveCol = colIndex;
    if (MIRROR_COLUMNS_FOR_DOWNWARD_PUNCH) {
        effectiveCol = 1 - colIndex;
    }

    double targetDeg = colZeroDeg;
    if (effectiveCol == 1) {
        targetDeg = colOneDeg;
    }

    timer moveTimer;
    moveTimer.clear();
    bool stalled = false;
    double currentDeg = -MotorX.position(degrees);

    if (targetDeg > currentDeg + POS_TOLERANCE_DEG) {
        MotorX.spin(reverse, 15, percent);
        while (-MotorX.position(degrees) < targetDeg - POS_TOLERANCE_DEG &&
               !stalled) {
            if ((moveTimer.time(msec) > 150 &&
                 MotorX.current(amp) > STALL_CURRENT_AMP) ||
                moveTimer.time(msec) > 2000) {
                stalled = true;
            }
            wait(5, msec);
        }
    } else if (targetDeg < currentDeg - POS_TOLERANCE_DEG) {
        MotorX.spin(forward, 15, percent);
        while (-MotorX.position(degrees) > targetDeg + POS_TOLERANCE_DEG &&
               !stalled) {
            if ((moveTimer.time(msec) > 150 &&
                 MotorX.current(amp) > STALL_CURRENT_AMP) ||
                moveTimer.time(msec) > 2000) {
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
// inside the 3D-printed base indent, and retracts to 0 deg via encoder.
bool punchBrailleDot(double targetDepthDeg, int dwellTimeMsec) {
    timer strokeTimer;
    strokeTimer.clear();

    // Downward embossing stroke into the 3D-printed indented base plate.
    MotorZ.spin(reverse, 12, percent);
    while (-MotorZ.position(degrees) < targetDepthDeg &&
           strokeTimer.time(msec) < 1500) {
        wait(5, msec);
    }
    MotorZ.stop(hold);

    // Dwell at full depth so paper fibers retain a raised tactile bump.
    timer dwellTimer;
    dwellTimer.clear();
    while (dwellTimer.time(msec) < dwellTimeMsec) {
        wait(5, msec);
    }

    // Retract upward to 0 deg via encoder, stopping early if upper stop
    // is reached (stall current re-zeroes MotorZ to prevent drift).
    strokeTimer.clear();
    bool hitTopStop = false;
    MotorZ.spin(forward, 12, percent);
    while (-MotorZ.position(degrees) > POS_TOLERANCE_DEG &&
           strokeTimer.time(msec) < 1800 && !hitTopStop) {
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
    return (-MotorZ.position(degrees) <= POS_TOLERANCE_DEG);
}

// =========================================================================
// Section 4: Braille Lookup, Entrance/Exit Roller Feed, and Clean Shutdown
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

// Function 8: Synchronously drives the entrance roller (PORT1) and exit
// roller (PORT4) by feedDegrees with independent encoder stops and 150 ms
// inrush-masked jam detection on both motors. When holdAfterFeed is true,
// both rollers lock in hold mode to tension the strip flat over the die
// plate without a mechanical clamp; when false, both rollers coast freely.
bool advancePaperStrip(double feedDegrees, bool holdAfterFeed) {
    bool useIn = MotorFeedIn.installed();
    bool useOut = MotorFeedOut.installed();

    if (feedDegrees > 0.0 && (useIn || useOut)) {
        double targetIn = MotorFeedIn.position(degrees) + feedDegrees;
        double targetOut = MotorFeedOut.position(degrees) + feedDegrees;
        bool inDone = !useIn;
        bool outDone = !useOut;
        double maxFeedMs = 1500.0 + feedDegrees * 8.0;

        timer feedTimer;
        feedTimer.clear();

        if (!inDone) {
            MotorFeedIn.spin(forward, 25, percent);
        }
        if (!outDone) {
            MotorFeedOut.spin(forward, 25, percent);
        }

        while ((!inDone || !outDone) &&
               feedTimer.time(msec) < maxFeedMs) {
            if (!inDone &&
                MotorFeedIn.position(degrees) >=
                    targetIn - POS_TOLERANCE_DEG) {
                MotorFeedIn.stop(hold);
                inDone = true;
            }
            if (!outDone &&
                MotorFeedOut.position(degrees) >=
                    targetOut - POS_TOLERANCE_DEG) {
                MotorFeedOut.stop(hold);
                outDone = true;
            }

            bool inJam =
                (!inDone && MotorFeedIn.current(amp) > STALL_CURRENT_AMP);
            bool outJam =
                (!outDone &&
                 MotorFeedOut.current(amp) > STALL_CURRENT_AMP);

            if (feedTimer.time(msec) > 150 && (inJam || outJam)) {
                MotorFeedIn.stop(brake);
                if (useOut) {
                    MotorFeedOut.stop(brake);
                }
                return false;
            }
            wait(5, msec);
        }
    }

    if (holdAfterFeed) {
        MotorFeedIn.stop(hold);
        if (useOut) {
            MotorFeedOut.stop(hold);
        }
    } else {
        MotorFeedIn.stop(coast);
        if (useOut) {
            MotorFeedOut.stop(coast);
        }
    }
    return true;
}

// Function 9: Executes a safe non-abort shutdown. Retracts Z-axis punch,
// drives the completed strip out via entrance & exit rollers (unless a
// fault occurred), parks X-axis at 0 deg, sets all 4 motors to coast,
// displays job stats, and terminates cleanly via Brain.programStop().
void executeCleanShutdown(int totalCharsEmbossed, int totalDotsPunched,
                          double totalTimeSec, bool faultOccurred) {
    timer shutdownTimer;
    shutdownTimer.clear();

    // Ensure Z-axis stylus is safely retracted before moving paper or X.
    if (MotorZ.installed()) {
        MotorZ.spin(forward, 12, percent);
        while (-MotorZ.position(degrees) > POS_TOLERANCE_DEG &&
               shutdownTimer.time(msec) < 1500 &&
               (shutdownTimer.time(msec) < 150 ||
                MotorZ.current(amp) < HOMING_STALL_AMP)) {
            wait(10, msec);
        }
        MotorZ.stop(brake);
    }

    // Eject finished Braille strip on normal completion; release on fault.
    if (!faultOccurred) {
        advancePaperStrip(EJECT_FEED_DEG, false);
    } else {
        advancePaperStrip(0.0, false);
    }

    // Park X-axis carriage at left home position.
    if (MotorX.installed()) {
        moveCarriageToColumn(0, 0.0, COL_ONE_DEG);
    }

    // De-energize all 4 motors and optical LED for safe handling.
    MotorFeedIn.stop(coast);
    if (MotorFeedOut.installed()) {
        MotorFeedOut.stop(coast);
    }
    MotorX.stop(coast);
    MotorZ.stop(coast);
    if (OpticalPaper.installed()) {
        OpticalPaper.setLight(ledState::off);
    }

    // Display final summary and audio/visual completion cue.
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    if (faultOccurred) {
        if (TouchStatus.installed()) {
            TouchStatus.on(red);
        }
        Brain.playSound(siren);
        Brain.Screen.print("SHUTDOWN: FAULT HALT");
    } else {
        if (TouchStatus.installed()) {
            TouchStatus.on(blue);
        }
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
// Main Control Loop
// =========================================================================

int main() {
    vexcodeInit();

    double calibratedPunchDeg = DEFAULT_PUNCH_DEG;
    char wordBuffer[MAX_WORD_LENGTH];
    bool isPaused = false;
    bool systemFault = false;
    int totalDotsPunched = 0;
    int charsCompleted = 0;

    if (!homeAllAxes(calibratedPunchDeg)) {
        executeCleanShutdown(0, 0, 0.0, true);
        return 0;
    }

    int wordLen = selectWordFromController(wordBuffer, MAX_WORD_LENGTH);

    timer jobTimer;
    jobTimer.clear();
    if (TouchStatus.installed()) {
        TouchStatus.on(green);
    }

    // Pull strip in from entrance roller and lock entrance & exit rollers.
    if (!advancePaperStrip(LOAD_FEED_DEG, true)) {
        systemFault = true;
    }

    int charIdx = 0;
    while (charIdx < wordLen && !systemFault) {
        char currentChar = wordBuffer[charIdx];
        bool cellMatrix[3][2];

        bool validChar = getCharDotMatrix(currentChar, cellMatrix);
        updateLiveTelemetry(currentChar, charIdx, wordLen,
                            totalDotsPunched, jobTimer);

        if (validChar) {
            int row = 0;
            while (row < 3 && !systemFault) {
                if (!checkSafetyAndPaper(isPaused, jobTimer)) {
                    systemFault = true;
                }

                int col = 0;
                while (col < 2 && !systemFault) {
                    if (cellMatrix[row][col]) {
                        if (!moveCarriageToColumn(col, COL_ZERO_DEG,
                                                  COL_ONE_DEG)) {
                            systemFault = true;
                        } else if (!punchBrailleDot(calibratedPunchDeg,
                                                    PUNCH_DWELL_MSEC)) {
                            systemFault = true;
                        } else {
                            totalDotsPunched++;
                            updateLiveTelemetry(currentChar, charIdx,
                                                wordLen, totalDotsPunched,
                                                jobTimer);
                        }
                    }
                    col++;
                }

                if (!systemFault) {
                    double feedStep = ROW_FEED_DEG;
                    if (row == 2) {
                        feedStep = CHAR_GAP_FEED_DEG;
                    }
                    if (!advancePaperStrip(feedStep, true)) {
                        systemFault = true;
                    }
                }
                row++;
            }
            if (!systemFault) {
                charsCompleted++;
            }
        }
        charIdx++;
    }

    executeCleanShutdown(charsCompleted, totalDotsPunched,
                         jobTimer.time(seconds), systemFault);
    return 0;
}
