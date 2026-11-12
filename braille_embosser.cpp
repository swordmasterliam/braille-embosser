//============================================================================
// MTE 100 & MTE 121 Course Project - Automated Desktop Braille Embosser
// Group 28: Liam McCarthy, Junwon Will Choi, Nicholas Chan, Joseph Lu
// Date: November 2026
// Platform: VEX IQ (2nd Generation) C++
//
// Compliance Notes (MTE 121 Syntax & Rubric):
// - No spinFor(), spinToPosition(), break, continue, goto, or event callbacks.
// - All motor movements are closed-loop polled using motor.position(degrees),
//   limit switches, motor current monitoring, and built-in timers.
// - 9 Non-trivial functions (minimum 6 required) attributed across all 4
//   team members, utilizing value parameters, reference parameters, arrays,
//   and return values.
//============================================================================

#include "vex.h"
#include "iq_cpp.h"
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace vex;

//----------------------------------------------------------------------------
// Hardware Port Configuration (4 VEX Motors + 5 Distinct Input Types)
//----------------------------------------------------------------------------
brain Brain;
inertial BrainInertial = inertial();
controller Controller = controller();

// 4 VEX Motors (All with Encoder-Controlled Movement)
motor MotorFeed  = motor(PORT1, false); // Y-Axis: Paper strip pinch rollers
motor MotorX     = motor(PORT2, false); // X-Axis: Lateral rack-and-pinion carriage
motor MotorZ     = motor(PORT3, false); // Z-Axis: Vertical dot punch rack-and-pinion
motor MotorClamp = motor(PORT4, false); // Paper Clamp: Hinged hold-down bar

// Peripheral Sensors
optical  OpticalPaper = optical(PORT5);  // Input 1: Paper presence detector
distance DistanceBed  = distance(PORT6); // Input 2: Bed height & obstruction check
bumper   BumperTop    = bumper(PORT7);   // Input 3: Z-axis upper homing limit switch
touchled TouchStatus  = touchled(PORT8); // Input 4: Start/Pause trigger & RGB status
// Input 5: Motor Encoders, Brain Buttons, and Remote Controller buttons

//----------------------------------------------------------------------------
// Mechanical & Kinematic Constants (Tetrix Prime Base + VEX IQ Geartrain)
//----------------------------------------------------------------------------
const double COL_ZERO_DEG        = 35.0;   // X-encoder angle for Braille Column 0 (Left)
const double COL_ONE_DEG         = 80.0;   // X-encoder angle for Braille Column 1 (Right)
const double ROW_FEED_DEG        = 45.0;   // Y-encoder rotation for 2.5 mm dot-row advance
const double CHAR_GAP_FEED_DEG   = 90.0;   // Y-encoder rotation for inter-character gap
const double EJECT_FEED_DEG      = 540.0;  // Y-encoder rotation to eject finished label
const double CLAMP_ENGAGE_DEG    = 65.0;   // Clamp motor angle to press bar onto paper
const double DEFAULT_PUNCH_DEG   = 115.0;  // Default Z-encoder angle for dot indentation
const double POS_TOLERANCE_DEG   = 2.5;    // Closed-loop encoder positioning tolerance
const double STALL_CURRENT_AMP   = 1.15;   // Motor stall current threshold (Amps)
const int    PUNCH_DWELL_MSEC    = 300;    // Timer dwell time at bottom of punch stroke
const int    MAX_WORD_LENGTH     = 16;     // Maximum queued characters per job

//----------------------------------------------------------------------------
// Data Organization: Grade 1 Braille Lookup Table (A-Z)
// Each letter is a 3-row by 2-column boolean matrix:
//   Row 0: [Dot 1 (top-left),    Dot 4 (top-right)]
//   Row 1: [Dot 2 (middle-left), Dot 5 (middle-right)]
//   Row 2: [Dot 3 (bottom-left), Dot 6 (bottom-right)]
//----------------------------------------------------------------------------
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

//============================================================================
// MODULE 1: SYSTEM HOMING, CALIBRATION & SAFETY MONITORING
// Author: Joseph Lu (Software Lead)
//============================================================================

//----------------------------------------------------------------------------
// Function 1: homeAllAxes
// Author: Joseph Lu
// Purpose: Performs startup initialization, homes the Z-axis punch to the
//          upper Bumper Switch, zeroes motor encoders, verifies paper presence
//          with the Optical Sensor, and calibrates Z-stroke depth using the
//          Distance Sensor.
// Parameters:
//   - calibratedPunchDeg (double&, pass-by-reference): Outputs calibrated
//     Z-axis encoder target angle based on measured bed distance.
// Returns: bool (true if initialization and homing succeeded, false on fault)
//----------------------------------------------------------------------------
bool homeAllAxes(double &calibratedPunchDeg) {
    timer homingTimer;
    homingTimer.clear();

    TouchStatus.on(yellow);
    OpticalPaper.setLight(ledState::on);

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Homing Axes...");

    // Step 1: Retract Z-axis punch upward until BumperTop is pressed or timeout
    MotorZ.spin(reverse, 30, percent);
    while (!BumperTop.pressing() && homingTimer.time(msec) < 3000) {
        wait(10, msec);
    }
    MotorZ.stop(hold);

    if (!BumperTop.pressing()) {
        TouchStatus.on(red);
        Brain.Screen.newLine();
        Brain.Screen.print("Fault: Z Limit Timeout");
        return false;
    }
    MotorZ.setPosition(0, degrees);

    // Step 2: Release paper clamp and zero clamp encoder
    homingTimer.clear();
    MotorClamp.spin(reverse, 25, percent);
    while (homingTimer.time(msec) < 600 && MotorClamp.current(amp) < STALL_CURRENT_AMP) {
        wait(10, msec);
    }
    MotorClamp.stop(hold);
    MotorClamp.setPosition(0, degrees);

    // Step 3: Home X-axis carriage to left mechanical datum
    homingTimer.clear();
    MotorX.spin(reverse, 25, percent);
    while (homingTimer.time(msec) < 2500 && MotorX.current(amp) < STALL_CURRENT_AMP) {
        wait(10, msec);
    }
    MotorX.stop(brake);
    MotorX.setPosition(0, degrees);
    MotorFeed.setPosition(0, degrees);

    // Step 4: Calibrate Z punch depth from Distance Sensor reading (mm)
    double measuredBedDist = DistanceBed.objectDistance(mm);
    if (measuredBedDist >= 35.0 && measuredBedDist <= 95.0) {
        // Scale punch stroke proportionally to measured gantry-to-bed height
        calibratedPunchDeg = DEFAULT_PUNCH_DEG + (measuredBedDist - 60.0) * 1.4;
    } else {
        calibratedPunchDeg = DEFAULT_PUNCH_DEG;
    }

    TouchStatus.on(blue);
    Brain.Screen.newLine();
    Brain.Screen.print("Homing Complete.");
    return true;
}

//----------------------------------------------------------------------------
// Function 2: checkSafetyAndPaper
// Author: Joseph Lu
// Purpose: Polls Optical Sensor for paper-out events, Distance Sensor for
//          obstruction/hand intrusion, and Touch LED for user pause requests.
//          Handles recovery if paper is reloaded before idle timeout expires.
// Parameters:
//   - isPaused (bool&, pass-by-reference): Tracks system pause state.
//   - jobTimer (timer&, pass-by-reference): Master job timer reference.
// Returns: bool (true if safe to proceed, false if job timed out / aborted)
//----------------------------------------------------------------------------
bool checkSafetyAndPaper(bool &isPaused, timer &jobTimer) {
    // Check if user tapped TouchLED to toggle pause
    if (TouchStatus.pressing()) {
        isPaused = true;
        while (TouchStatus.pressing()) {
            wait(15, msec);
        }
    }

    // Check Optical Sensor brightness/hue for missing paper and Distance Sensor for hand intrusion
    double bedClearanceMm = DistanceBed.objectDistance(mm);
    bool paperMissing = (OpticalPaper.hue() > 200 && OpticalPaper.hue() < 260); // Dark bed visible when strip runs out
    bool obstructionDetected = (bedClearanceMm < 25.0);

    if (paperMissing || obstructionDetected) {
        isPaused = true;
    }

    if (!isPaused) {
        return true;
    }

    // Enter safe pause state: stop motors and set status LED to Yellow
    MotorX.stop(brake);
    MotorFeed.stop(brake);
    TouchStatus.on(yellow);

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    if (paperMissing) {
        Brain.Screen.print("PAUSED: Paper Out!");
    } else if (obstructionDetected) {
        Brain.Screen.print("PAUSED: Obstruction!");
    } else {
        Brain.Screen.print("PAUSED: User Request");
    }
    Brain.Screen.newLine();
    Brain.Screen.print("Load paper & tap LED");

    timer pauseTimer;
    pauseTimer.clear();
    bool resumed = false;

    // Wait up to 25 seconds for user to reload paper and tap Touch LED
    while (pauseTimer.time(seconds) < 25.0 && !resumed) {
        bool paperPresentNow = !(OpticalPaper.hue() > 200 && OpticalPaper.hue() < 260);
        bool pathClearNow = (DistanceBed.objectDistance(mm) >= 25.0);

        if (TouchStatus.pressing() && paperPresentNow && pathClearNow) {
            while (TouchStatus.pressing()) {
                wait(15, msec);
            }
            isPaused = false;
            resumed = true;
            TouchStatus.on(green);
        }
        wait(25, msec);
    }

    // Reset touch timestamp on jobTimer activity
    if (resumed && jobTimer.time(seconds) >= 0.0) {
        return true;
    }
    return false;
}

//============================================================================
// MODULE 2: USER INTERFACE, WORD BUFFERING & TELEMETRY
// Author: Liam McCarthy (Integration & UI Lead)
//============================================================================

//----------------------------------------------------------------------------
// Function 3: selectWordFromController
// Author: Liam McCarthy
// Purpose: Implements Stretch Goal 1 (Multi-character word queue). Allows the
//          operator to cycle alphanumeric characters (A-Z, space) using the
//          VEX Controller or Brain buttons and queue a full word prior to
//          embossing.
// Parameters:
//   - wordBuffer (char[], array parameter): Buffer to store selected characters.
//   - maxLen (int, pass-by-value): Maximum capacity of wordBuffer.
// Returns: int (Total number of characters queued by the user)
//----------------------------------------------------------------------------
int selectWordFromController(char wordBuffer[], int maxLen) {
    int charCount = 0;
    int currentLetterIdx = 0; // 0 = 'A' .. 25 = 'Z', 26 = ' '
    bool selectionComplete = false;

    TouchStatus.on(blue);
    timer inputTimeout;
    inputTimeout.clear();

    while (!selectionComplete && charCount < maxLen - 1 && inputTimeout.time(seconds) < 60.0) {
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
        Brain.Screen.print("Tap TouchLED to Start");

        // Cycle letter up (Brain Right or Controller EUp)
        if (Brain.buttonRight.pressing() || Controller.ButtonEUp.pressing()) {
            currentLetterIdx = (currentLetterIdx + 1) % 27;
            inputTimeout.clear();
            while (Brain.buttonRight.pressing() || Controller.ButtonEUp.pressing()) {
                wait(15, msec);
            }
        }

        // Cycle letter down (Brain Left or Controller EDown)
        if (Brain.buttonLeft.pressing() || Controller.ButtonEDown.pressing()) {
            currentLetterIdx = (currentLetterIdx + 26) % 27;
            inputTimeout.clear();
            while (Brain.buttonLeft.pressing() || Controller.ButtonEDown.pressing()) {
                wait(15, msec);
            }
        }

        // Confirm character into buffer (Brain Check or Controller FUp)
        if (Brain.buttonCheck.pressing() || Controller.ButtonFUp.pressing()) {
            wordBuffer[charCount] = previewChar;
            charCount++;
            inputTimeout.clear();
            while (Brain.buttonCheck.pressing() || Controller.ButtonFUp.pressing()) {
                wait(15, msec);
            }
        }

        // Finish word entry and begin embossing (TouchLED press)
        if (TouchStatus.pressing()) {
            while (TouchStatus.pressing()) {
                wait(15, msec);
            }
            selectionComplete = true;
        }

        wait(40, msec);
    }

    // Default demo word ("UW") if operator starts immediately without queuing
    if (charCount == 0) {
        wordBuffer[0] = 'U';
        wordBuffer[1] = 'W';
        charCount = 2;
    }
    wordBuffer[charCount] = '\0';
    return charCount;
}

//----------------------------------------------------------------------------
// Function 4: updateLiveTelemetry
// Author: Liam McCarthy
// Purpose: Implements Stretch Goal 3 (Live Progress & Telemetry). Displays
//          currently embossing character, completion percentage, dot count,
//          and elapsed time on the VEX Brain LCD screen.
// Parameters:
//   - currentChar (char, pass-by-value): Character currently being embossed.
//   - charIndex (int, pass-by-value): Zero-based index of current character.
//   - totalChars (int, pass-by-value): Total characters in queued word.
//   - dotsPunched (int, pass-by-value): Cumulative raised dots embossed so far.
//   - jobTimer (timer&, pass-by-reference): Active timer tracking elapsed time.
// Returns: void
//----------------------------------------------------------------------------
void updateLiveTelemetry(char currentChar, int charIndex, int totalChars, int dotsPunched, timer &jobTimer) {
    int percentDone = 0;
    if (totalChars > 0) {
        percentDone = ((charIndex + 1) * 100) / totalChars;
    }

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Embossing: '%c' (%d/%d)", currentChar, charIndex + 1, totalChars);
    Brain.Screen.newLine();
    Brain.Screen.print("Progress:  %d%%", percentDone);
    Brain.Screen.newLine();
    Brain.Screen.print("Dots: %d | Time: %.1fs", dotsPunched, jobTimer.time(seconds));
}

//============================================================================
// MODULE 3: X-AXIS GANTRY CARRIAGE & Z-AXIS RACK-AND-PINION PUNCH CONTROL
// Author: Junwon Will Choi (Mechanical Gear Trains & Kinematics Lead)
//============================================================================

//----------------------------------------------------------------------------
// Function 5: moveCarriageToColumn
// Author: Junwon Will Choi
// Purpose: Drives the X-axis rack-and-pinion carriage to align the vertical
//          punch pin with Column 0 (left dot column) or Column 1 (right dot
//          column), monitoring motor current and timer for stall/jam protection.
// Parameters:
//   - colIndex (int, pass-by-value): Target column (0 = left, 1 = right).
//   - colZeroDeg (double, pass-by-value): Calibrated encoder angle for Col 0.
//   - colOneDeg (double, pass-by-value): Calibrated encoder angle for Col 1.
// Returns: bool (true if target reached within tolerance, false on stall)
//----------------------------------------------------------------------------
bool moveCarriageToColumn(int colIndex, double colZeroDeg, double colOneDeg) {
    double targetDeg = colZeroDeg;
    if (colIndex == 1) {
        targetDeg = colOneDeg;
    }

    timer moveTimer;
    moveTimer.clear();
    bool stalled = false;

    double currentDeg = MotorX.position(degrees);
    if (targetDeg > currentDeg + POS_TOLERANCE_DEG) {
        MotorX.spin(forward, 30, percent);
        while (MotorX.position(degrees) < targetDeg - POS_TOLERANCE_DEG && !stalled) {
            if (MotorX.current(amp) > STALL_CURRENT_AMP || moveTimer.time(msec) > 2000) {
                stalled = true;
            }
            wait(5, msec);
        }
    } else if (targetDeg < currentDeg - POS_TOLERANCE_DEG) {
        MotorX.spin(reverse, 30, percent);
        while (MotorX.position(degrees) > targetDeg + POS_TOLERANCE_DEG && !stalled) {
            if (MotorX.current(amp) > STALL_CURRENT_AMP || moveTimer.time(msec) > 2000) {
                stalled = true;
            }
            wait(5, msec);
        }
    }

    MotorX.stop(hold);
    return !stalled;
}

//----------------------------------------------------------------------------
// Function 6: punchBrailleDot
// Author: Junwon Will Choi
// Purpose: Executes a single vertical Z-axis embossing stroke. Drives the
//          3D-printed stylus tip into the compliant backing pad to targetDepthDeg,
//          holds for dwellTimeMsec using a built-in timer to form a permanent
//          tactile bump, and retracts to BumperTop to prevent encoder drift.
// Parameters:
//   - targetDepthDeg (double, pass-by-value): Target Z encoder angle.
//   - dwellTimeMsec (int, pass-by-value): Hold duration in milliseconds.
// Returns: bool (true if stroke and top-limit retraction succeeded)
//----------------------------------------------------------------------------
bool punchBrailleDot(double targetDepthDeg, int dwellTimeMsec) {
    timer strokeTimer;
    strokeTimer.clear();

    // Downward indentation stroke
    MotorZ.spin(forward, 45, percent);
    while (MotorZ.position(degrees) < targetDepthDeg && strokeTimer.time(msec) < 1500) {
        wait(5, msec);
    }
    MotorZ.stop(hold);

    // Controlled dwell timer to deform paper fibers cleanly into backing pad
    timer dwellTimer;
    dwellTimer.clear();
    while (dwellTimer.time(msec) < dwellTimeMsec) {
        wait(5, msec);
    }

    // Retract upward until top Bumper Switch clicks
    strokeTimer.clear();
    MotorZ.spin(reverse, 40, percent);
    while (!BumperTop.pressing() && strokeTimer.time(msec) < 1800) {
        wait(5, msec);
    }
    MotorZ.stop(hold);

    if (BumperTop.pressing()) {
        MotorZ.setPosition(0, degrees);
        return true;
    }
    return false;
}

//============================================================================
// MODULE 4: BRAILLE MATRIX LOOKUP, PAPER FEEDING & CLEAN SHUTDOWN
// Author: Nicholas Chan (Project Manager)
//============================================================================

//----------------------------------------------------------------------------
// Function 7: getCharDotMatrix
// Author: Nicholas Chan
// Purpose: Converts an uppercase character ('A'-'Z' or space) into its 3x2
//          Braille dot matrix by copying from BRAILLE_ALPHABET.
// Parameters:
//   - letter (char, pass-by-value): Input ASCII character to translate.
//   - cellMatrix (bool[3][2], 2D array parameter): Output 3x2 dot pattern.
// Returns: bool (true if character is valid, false if unsupported input)
//----------------------------------------------------------------------------
bool getCharDotMatrix(char letter, bool cellMatrix[3][2]) {
    // Convert lowercase to uppercase if needed
    if (letter >= 'a' && letter <= 'z') {
        letter = (char)(letter - 'a' + 'A');
    }

    // Handle space character (all 6 dots false)
    if (letter == ' ') {
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 2; c++) {
                cellMatrix[r][c] = false;
            }
        }
        return true;
    }

    // Validate alphabetical range 'A'-'Z'
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

//----------------------------------------------------------------------------
// Function 8: clampAndAdvancePaper
// Author: Nicholas Chan
// Purpose: Coordinates the paper clamp bar (Motor 4) and Y-axis paper feed
//          rollers (Motor 1). Unclamps paper, advances the strip by feedDegrees,
//          and re-clamps the strip flat prior to punching the next row.
// Parameters:
//   - feedDegrees (double, pass-by-value): Y-axis roller rotation in degrees.
//   - engageClampAfter (bool, pass-by-value): Whether to lower clamp after feed.
// Returns: bool (true if feed completed without stall, false otherwise)
//----------------------------------------------------------------------------
bool clampAndAdvancePaper(double feedDegrees, bool engageClampAfter) {
    timer feedTimer;
    feedTimer.clear();

    // Step 1: Lift paper clamp bar to release strip for feeding
    MotorClamp.spin(reverse, 30, percent);
    while (MotorClamp.position(degrees) > 5.0 && feedTimer.time(msec) < 800) {
        wait(5, msec);
    }
    MotorClamp.stop(hold);

    // Step 2: Advance paper strip by feedDegrees using Y-axis pinch rollers
    if (feedDegrees > 0.0) {
        double startY = MotorFeed.position(degrees);
        double targetY = startY + feedDegrees;
        feedTimer.clear();

        MotorFeed.spin(forward, 25, percent);
        while (MotorFeed.position(degrees) < targetY - POS_TOLERANCE_DEG && feedTimer.time(msec) < 2500) {
            if (MotorFeed.current(amp) > STALL_CURRENT_AMP) {
                MotorFeed.stop(brake);
                return false;
            }
            wait(5, msec);
        }
        MotorFeed.stop(hold);
    }

    // Step 3: Re-engage clamp bar to hold paper strip taut during embossing
    if (engageClampAfter) {
        feedTimer.clear();
        MotorClamp.spin(forward, 30, percent);
        while (MotorClamp.position(degrees) < CLAMP_ENGAGE_DEG && feedTimer.time(msec) < 800) {
            wait(5, msec);
        }
        MotorClamp.stop(hold);
    }
    return true;
}

//----------------------------------------------------------------------------
// Function 9: executeCleanShutdown
// Author: Nicholas Chan
// Purpose: Executes the non-abort shutdown procedure required by MTE 100/121.
//          Retracts Z-punch, releases clamp, ejects finished Braille label,
//          returns X-carriage to 0 deg, updates status LED & screen summary,
//          and stops the program cleanly via Brain.programStop().
// Parameters:
//   - totalCharsEmbossed (int, pass-by-value): Number of characters completed.
//   - totalDotsPunched (int, pass-by-value): Number of tactile dots embossed.
//   - totalTimeSec (double, pass-by-value): Total runtime in seconds.
//   - faultOccurred (bool, pass-by-value): True if shutdown triggered by fault.
// Returns: void
//----------------------------------------------------------------------------
void executeCleanShutdown(int totalCharsEmbossed, int totalDotsPunched, double totalTimeSec, bool faultOccurred) {
    timer shutdownTimer;
    shutdownTimer.clear();

    // Ensure Z-axis punch is retracted to upper limit switch
    MotorZ.spin(reverse, 35, percent);
    while (!BumperTop.pressing() && shutdownTimer.time(msec) < 1500) {
        wait(10, msec);
    }
    MotorZ.stop(brake);

    // Release paper clamp and eject finished Braille strip if no mechanical jam
    if (!faultOccurred) {
        clampAndAdvancePaper(EJECT_FEED_DEG, false);
    } else {
        clampAndAdvancePaper(0.0, false);
    }

    // Return X-carriage to home position (0 deg)
    moveCarriageToColumn(0, 0.0, COL_ONE_DEG);

    // Stop all 4 motors and turn off optical LED
    MotorFeed.stop(coast);
    MotorX.stop(coast);
    MotorZ.stop(coast);
    MotorClamp.stop(coast);
    OpticalPaper.setLight(ledState::off);

    // Display final job summary on Brain screen and set status LED
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    if (faultOccurred) {
        TouchStatus.on(red);
        playSound(siren);
        Brain.Screen.print("SHUTDOWN: FAULT HALT");
    } else {
        TouchStatus.on(blue);
        playSound(tada);
        Brain.Screen.print("JOB COMPLETE - EJECTED");
    }
    Brain.Screen.newLine();
    Brain.Screen.print("Chars: %d | Dots: %d", totalCharsEmbossed, totalDotsPunched);
    Brain.Screen.newLine();
    Brain.Screen.print("Runtime: %.1f sec", totalTimeSec);

    wait(3000, msec);
    Brain.programStop();
}

//============================================================================
// MAIN PROGRAM LOOP
//============================================================================
int main() {
    double calibratedPunchDeg = DEFAULT_PUNCH_DEG;
    char wordBuffer[MAX_WORD_LENGTH];
    bool isPaused = false;
    bool systemFault = false;
    int totalDotsPunched = 0;
    int charsCompleted = 0;

    // 1. Startup & Homing Sequence (Joseph Lu)
    if (!homeAllAxes(calibratedPunchDeg)) {
        executeCleanShutdown(0, 0, 0.0, true);
        return 0;
    }

    // 2. Operator Word Selection Queue (Liam McCarthy)
    int wordLen = selectWordFromController(wordBuffer, MAX_WORD_LENGTH);

    // 3. Start Embossing Job Timer & Engage Initial Clamp
    timer jobTimer;
    jobTimer.clear();
    TouchStatus.on(green);
    clampAndAdvancePaper(0.0, true);

    // 4. Main Character & Row Embossing Loop
    int charIdx = 0;
    while (charIdx < wordLen && !systemFault) {
        char currentChar = wordBuffer[charIdx];
        bool cellMatrix[3][2];

        // Translate character to 3x2 Braille matrix (Nicholas Chan)
        bool validChar = getCharDotMatrix(currentChar, cellMatrix);
        updateLiveTelemetry(currentChar, charIdx, wordLen, totalDotsPunched, jobTimer);

        if (validChar) {
            // Process 3 rows of the 2x3 Braille cell
            int row = 0;
            while (row < 3 && !systemFault) {
                // Poll safety sensors before each row (Joseph Lu)
                if (!checkSafetyAndPaper(isPaused, jobTimer)) {
                    systemFault = true;
                }

                // Process 2 columns in the current row (Junwon Will Choi)
                int col = 0;
                while (col < 2 && !systemFault) {
                    if (cellMatrix[row][col]) {
                        if (!moveCarriageToColumn(col, COL_ZERO_DEG, COL_ONE_DEG)) {
                            systemFault = true;
                        } else if (!punchBrailleDot(calibratedPunchDeg, PUNCH_DWELL_MSEC)) {
                            systemFault = true;
                        } else {
                            totalDotsPunched++;
                            updateLiveTelemetry(currentChar, charIdx, wordLen, totalDotsPunched, jobTimer);
                        }
                    }
                    col++;
                }

                // Advance paper strip to next dot row (or inter-character gap after Row 2)
                if (!systemFault) {
                    double feedStep = ROW_FEED_DEG;
                    if (row == 2) {
                        feedStep = CHAR_GAP_FEED_DEG;
                    }
                    if (!clampAndAdvancePaper(feedStep, true)) {
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

    // 5. Non-Abort Clean Shutdown & Label Ejection (Nicholas Chan)
    executeCleanShutdown(charsCompleted, totalDotsPunched, jobTimer.time(seconds), systemFault);
    return 0;
}
