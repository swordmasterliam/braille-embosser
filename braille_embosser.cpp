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

// MTE 100 / MTE 121 Group 28 - Braille Embosser Firmware (v21)
#include <cmath>

// Geometry, drive kinematics (16T sprocket = 101.6mm, 200mm wheel), limits
const double COL_ZERO_MM          = 2.5;
const double INTRA_COL_MM         = 3.5;
const double INTER_CHAR_GAP_MM    = 5.5;
const double CHAR_PITCH_MM        = INTRA_COL_MM + INTER_CHAR_GAP_MM;
const double ROW_PITCH_MM         = 3.5;
const double LOAD_FEED_MM         = 0.0;
const double EJECT_FEED_MM        = 320.0;

const double X_MM_PER_REV         = 101.6;
const double Y_MM_PER_REV         = 200.0;

const double COL_ZERO_DEG         = (COL_ZERO_MM / X_MM_PER_REV) * 360.0;
const double INTRA_COL_DEG        = (INTRA_COL_MM / X_MM_PER_REV) * 360.0;
const double CHAR_PITCH_DEG       = (CHAR_PITCH_MM / X_MM_PER_REV) * 360.0;
const double LOAD_FEED_DEG        = (LOAD_FEED_MM / Y_MM_PER_REV) * 360.0;
const double ROW_FEED_DEG         = (ROW_PITCH_MM / Y_MM_PER_REV) * 360.0;
const double EJECT_FEED_DEG       = (EJECT_FEED_MM / Y_MM_PER_REV) * 360.0;

const double X_BACKLASH_DEG       = 8.0;
const double Z_LIFT_DEG           = 18.0;
const double Z_MIN_CLEAR_DEG      = 8.0;
const double Z_MAX_DOWN_TRAVEL    = 90.0;
const double POS_TOLERANCE_DEG    = 0.5;
const double STALL_CURRENT_AMP    = 1.25;
const double X_JAM_CURRENT_AMP    = 1.65;
const double X_HOMING_STALL_AMP   = 1.40;
const double Z_PUNCH_STALL_AMP    = 1.85;
const double OBSTRUCTION_MIN_MM   = 80.0;
const double SONAR_MIN_MM         = 100.0;
const double WHITE_PAPER_MIN_PCT  = 15.0;
const int    PUNCH_SQUEEZE_MSEC   = 150;
const int    PUNCH_DWELL_MSEC     = 250;
const int    Z_POST_LIFT_PAUSE_MS = 450;
const int    MAX_WORD_LENGTH      = 16;
const bool   MIRROR_COLUMNS_FOR_DOWNWARD_PUNCH = false;

const int FAULT_NONE              = 0;
const int FAULT_BED_BLOCKED       = 1;
const int FAULT_SAFETY_TIMEOUT    = 2;
const int FAULT_X_STALL           = 3;
const int FAULT_Z_LIFT            = 4;
const int FAULT_Y_JAM             = 5;

// Grade 1 Braille dot lookup table (A-Z, 3 rows x 2 cols)
const bool BRAILLE_ALPHABET[26][3][2] =
{
    {{true,  false}, {false, false}, {false, false}},
    {{true,  false}, {true,  false}, {false, false}},
    {{true,  true},  {false, false}, {false, false}},
    {{true,  true},  {false, true},  {false, false}},
    {{true,  false}, {false, true},  {false, false}},
    {{true,  true},  {true,  false}, {false, false}},
    {{true,  true},  {true,  true},  {false, false}},
    {{true,  false}, {true,  true},  {false, false}},
    {{false, true},  {true,  false}, {false, false}},
    {{false, true},  {true,  true},  {false, false}},
    {{true,  false}, {false, false}, {true,  false}},
    {{true,  false}, {true,  false}, {true,  false}},
    {{true,  true},  {false, false}, {true,  false}},
    {{true,  true},  {false, true},  {true,  false}},
    {{true,  false}, {false, true},  {true,  false}},
    {{true,  true},  {true,  false}, {true,  false}},
    {{true,  true},  {true,  true},  {true,  false}},
    {{true,  false}, {true,  true},  {true,  false}},
    {{false, true},  {true,  false}, {true,  false}},
    {{false, true},  {true,  true},  {true,  false}},
    {{true,  false}, {false, false}, {true,  true}},
    {{true,  false}, {true,  false}, {true,  true}},
    {{false, true},  {true,  true},  {false, true}},
    {{true,  true},  {false, false}, {true,  true}},
    {{true,  true},  {false, true},  {true,  true}},
    {{true,  false}, {false, true},  {true,  true}}
};

// Section 1: Homing, Calibration & Safety - Joseph Lu

double readPaperBrightness(int colorGen, colorsensor &colorG1,
                           optical &opticalG2)
{
    if (colorGen == 1)
    {
        return (double)colorG1.brightness();
    }
    else if (colorGen == 2)
    {
        return opticalG2.brightness();
    }
    return 0.0;
}

double readBedDistance(int distGen, sonar &sonarG1, distance &distG2,
                       bool &objectFound)
{
    if (distGen == 1)
    {
        objectFound = sonarG1.foundObject();
        return sonarG1.distance(mm);
    }
    else if (distGen == 2)
    {
        objectFound = distG2.isObjectDetected();
        return distG2.objectDistance(mm);
    }
    objectFound = false;
    return 0.0;
}

// Lift Z ~1cm from bottom, home X to motor stop, calibrate sensors
bool homeAllAxes(double &calibratedPressDeg, double &paperThreshold,
                 double &distThreshold, int colorPort, int colorGen,
                 colorsensor &colorG1, optical &opticalG2, int distPort,
                 int distGen, sonar &sonarG1, distance &distG2)
{
    timer homingTimer;
    homingTimer.clear();

    MotorZ.setMaxTorque(100, percent);
    MotorX.setMaxTorque(100, percent);
    TouchStatus.on(yellow);

    if (colorGen == 1)
    {
        colorG1.setLight(ledState::on);
        colorG1.setLight(100, percent);
    }
    else if (colorGen == 2)
    {
        opticalG2.setLight(ledState::on);
        opticalG2.setLightPower(100, percent);
    }

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("v21 Lift Z ~1cm");

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
            MotorZ.current(amp) < STALL_CURRENT_AMP))
    {
        wait(5, msec);
    }
    MotorZ.stop(hold);
    MotorZ.setMaxTorque(100, percent);
    wait(Z_POST_LIFT_PAUSE_MS, msec);

    Brain.Screen.newLine();
    Brain.Screen.print("v21 Homing X...");
    homingTimer.clear();
    MotorX.spin(forward, 30, percent);
    int stallCount = 0;
    while (homingTimer.time(msec) < 8000 && stallCount < 12)
    {
        if (homingTimer.time(msec) > 400 &&
            (fabs(MotorX.velocity(rpm)) < 2.0 ||
             MotorX.current(amp) >= X_HOMING_STALL_AMP))
        {
            stallCount++;
        }
        else
        {
            stallCount = 0;
        }
        wait(15, msec);
    }
    MotorX.stop(coast);
    wait(60, msec);
    MotorX.setPosition(0, degrees);
    wait(60, msec);

    MotorFeed.stop(coast);
    MotorFeed.setPosition(0, degrees);

    wait(150, msec);
    bool startDistDetected = false;
    double measuredBedDist =
        readBedDistance(distGen, sonarG1, distG2, startDistDetected);

    if (distGen == 1)
    {
        distThreshold = SONAR_MIN_MM;
        if (startDistDetected && measuredBedDist > 80.0 &&
            measuredBedDist < 900.0)
        {
            distThreshold = measuredBedDist * 0.65;
            if (distThreshold < 60.0)
            {
                distThreshold = 60.0;
            }
            else if (distThreshold > 140.0)
            {
                distThreshold = 140.0;
            }
        }
    }
    else
    {
        distThreshold = OBSTRUCTION_MIN_MM;
        if (startDistDetected && measuredBedDist > 50.0 &&
            measuredBedDist < 1800.0)
        {
            distThreshold = measuredBedDist * 0.65;
            if (distThreshold < 40.0)
            {
                distThreshold = 40.0;
            }
            else if (distThreshold > 100.0)
            {
                distThreshold = 100.0;
            }
        }
    }

    double startBright = readPaperBrightness(colorGen, colorG1, opticalG2);
    paperThreshold = WHITE_PAPER_MIN_PCT;
    if (startBright >= 30.0)
    {
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

// Poll Touch LED pause, paper presence, and distance obstruction
bool checkSafetyAndPaper(bool &isPaused, timer &jobTimer,
                         double paperThreshold, double distThreshold,
                         int colorGen, colorsensor &colorG1,
                         optical &opticalG2, int distGen, sonar &sonarG1,
                         distance &distG2)
{
    if (TouchStatus.pressing())
    {
        isPaused = true;
        while (TouchStatus.pressing())
        {
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

    if (paperMissing || obstructionDetected)
    {
        isPaused = true;
    }

    if (!isPaused)
    {
        return true;
    }

    MotorX.stop(brake);
    MotorFeed.stop(hold);
    MotorZ.stop(hold);
    TouchStatus.on(yellow);

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    if (paperMissing)
    {
        Brain.Screen.print("PAUSED:No Paper");
    }
    else if (obstructionDetected)
    {
        Brain.Screen.print("PAUSED:Distance");
    }
    else
    {
        Brain.Screen.print("PAUSED:User");
    }
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("LED=Go Chk=Skip");

    timer pauseTimer;
    pauseTimer.clear();
    bool resumed = false;
    int uiTicks = 0;

    while (pauseTimer.time(seconds) < 25.0 && !resumed)
    {
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

        if (uiTicks % 8 == 0)
        {
            Brain.Screen.setCursor(2, 1);
            Brain.Screen.print("B:%.0f/%.0f D:%.0f  ",
                               nowBright, paperThreshold,
                               currentClearance);
        }
        uiTicks++;

        bool ledResume =
            TouchStatus.pressing() && paperPresentNow && pathClearNow;
        bool manualOverride = Brain.buttonCheck.pressing();

        if (ledResume || manualOverride)
        {
            while (TouchStatus.pressing() ||
                   Brain.buttonCheck.pressing())
            {
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

// Section 2: UI & Live Telemetry - Junwon Will Choi

// Queue characters from Brain or Controller; defaults to "UW"
int selectWordFromController(char wordBuffer[], int maxLen)
{
    int charCount = 0;
    int currentLetterIdx = 0;
    bool selectionComplete = false;

    TouchStatus.on(blue);
    timer inputTimeout;
    inputTimeout.clear();

    while (!selectionComplete && charCount < maxLen - 1 &&
           inputTimeout.time(seconds) < 60.0)
    {
        char previewChar = ' ';
        if (currentLetterIdx < 26)
        {
            previewChar = (char)('A' + currentLetterIdx);
        }

        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Char:[%c] Q:%d", previewChar, charCount);
        Brain.Screen.newLine();
        for (int i = 0; i < charCount; i++)
        {
            Brain.Screen.print("%c", wordBuffer[i]);
        }
        if (charCount == 0)
        {
            Brain.Screen.print("(Default: UW)");
        }
        Brain.Screen.newLine();
        Brain.Screen.print("Touch LED=Start");

        if (Brain.buttonRight.pressing() ||
            Controller.ButtonEUp.pressing())
        {
            currentLetterIdx = (currentLetterIdx + 1) % 27;
            inputTimeout.clear();
            while (Brain.buttonRight.pressing() ||
                   Controller.ButtonEUp.pressing())
            {
                wait(15, msec);
            }
        }

        if (Brain.buttonLeft.pressing() ||
            Controller.ButtonEDown.pressing())
        {
            currentLetterIdx = (currentLetterIdx + 26) % 27;
            inputTimeout.clear();
            while (Brain.buttonLeft.pressing() ||
                   Controller.ButtonEDown.pressing())
            {
                wait(15, msec);
            }
        }

        if (Brain.buttonCheck.pressing() ||
            Controller.ButtonFUp.pressing())
        {
            wordBuffer[charCount] = previewChar;
            charCount++;
            inputTimeout.clear();
            while (Brain.buttonCheck.pressing() ||
                   Controller.ButtonFUp.pressing())
            {
                wait(15, msec);
            }
        }

        bool startTriggered =
            TouchStatus.pressing() ||
            Controller.ButtonFDown.pressing();

        if (startTriggered)
        {
            while (TouchStatus.pressing() ||
                   Controller.ButtonFDown.pressing())
            {
                wait(15, msec);
            }
            selectionComplete = true;
        }

        wait(40, msec);
    }

    if (charCount == 0)
    {
        wordBuffer[0] = 'U';
        wordBuffer[1] = 'W';
        charCount = 2;
    }
    wordBuffer[charCount] = '\0';
    return charCount;
}

// Display row progress, live sensor readings, and elapsed job time
void updateLiveTelemetry(char currentChar, int rowIndex, int totalRows,
                         int dotsPunched, timer &jobTimer, int colorGen,
                         colorsensor &colorG1, optical &opticalG2,
                         int distGen, sonar &sonarG1, distance &distG2)
{
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

// Section 3: X Carriage & Z Punch Control - Nicholas Chan

// Move X carriage to (charIdx, dotCol) or re-home when charIdx < 0
bool moveCarriageToColumn(int charIdx, int dotCol, double colZeroDeg,
                          double intraColDeg, double charPitchDeg)
{
    double targetDeg = 0.0;
    if (charIdx >= 0)
    {
        targetDeg =
            colZeroDeg + (charIdx * charPitchDeg) + (dotCol * intraColDeg);
    }
    if (targetDeg < 0.0)
    {
        targetDeg = 0.0;
    }

    timer moveTimer;
    moveTimer.clear();
    bool stalled = false;
    double currentDeg = -MotorX.position(degrees);
    double maxMoveMs = 4000.0 + fabs(targetDeg - currentDeg) * 25.0;
    MotorX.setMaxTorque(100, percent);

    if (charIdx < 0 || targetDeg <= 0.5)
    {
        MotorX.spin(forward, 28, percent);
        int homeStallCount = 0;
        while (-MotorX.position(degrees) > POS_TOLERANCE_DEG &&
               moveTimer.time(msec) < maxMoveMs &&
               homeStallCount < 10)
        {
            if (moveTimer.time(msec) > 350 &&
                (fabs(MotorX.velocity(rpm)) < 2.0 ||
                 MotorX.current(amp) >= X_HOMING_STALL_AMP))
            {
                homeStallCount++;
            }
            else
            {
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

    if (targetDeg < currentDeg - POS_TOLERANCE_DEG)
    {
        double preTarget = targetDeg - X_BACKLASH_DEG;
        if (preTarget < 0.0)
        {
            preTarget = 0.0;
        }
        int leftSpeed = 28;
        if (currentDeg - preTarget > 30.0)
        {
            leftSpeed = 32;
        }
        int jamCount = 0;
        MotorX.spin(forward, leftSpeed, percent);
        while (-MotorX.position(degrees) > preTarget + POS_TOLERANCE_DEG &&
               !stalled)
        {
            if (moveTimer.time(msec) > 600 &&
                fabs(MotorX.velocity(rpm)) < 1.5 &&
                MotorX.current(amp) >= X_JAM_CURRENT_AMP)
            {
                jamCount++;
                if (jamCount >= 50)
                {
                    stalled = true;
                }
            }
            else
            {
                jamCount = 0;
            }
            if (moveTimer.time(msec) > maxMoveMs)
            {
                stalled = true;
            }
            wait(5, msec);
        }
        MotorX.stop(hold);
        wait(40, msec);
        currentDeg = -MotorX.position(degrees);
        moveTimer.clear();
    }

    if (!stalled && targetDeg > currentDeg + POS_TOLERANCE_DEG)
    {
        int rightSpeed = 28;
        if (targetDeg - currentDeg > 30.0)
        {
            rightSpeed = 32;
        }
        int jamCount = 0;
        MotorX.spin(reverse, rightSpeed, percent);
        while (-MotorX.position(degrees) < targetDeg - POS_TOLERANCE_DEG &&
               !stalled)
        {
            if (moveTimer.time(msec) > 600 &&
                fabs(MotorX.velocity(rpm)) < 1.5 &&
                MotorX.current(amp) >= X_JAM_CURRENT_AMP)
            {
                jamCount++;
                if (jamCount >= 50)
                {
                    stalled = true;
                }
            }
            else
            {
                jamCount = 0;
            }
            if (moveTimer.time(msec) > maxMoveMs)
            {
                stalled = true;
            }
            wait(5, msec);
        }
    }

    MotorX.stop(hold);
    return !stalled;
}

// Press Z down until stall, dwell, lift +18 deg relative, pause at top
bool punchBrailleDot(double maxDownTravelDeg, int dwellTimeMsec)
{
    timer strokeTimer;
    strokeTimer.clear();

    double topPos = MotorZ.position(degrees);

    MotorZ.setMaxTorque(100, percent);
    MotorZ.spin(reverse, 35, percent);
    int zStallCount = 0;
    bool bottomStalled = false;
    while ((topPos - MotorZ.position(degrees)) < maxDownTravelDeg &&
           strokeTimer.time(msec) < 2000 && !bottomStalled)
    {
        if (strokeTimer.time(msec) > 150)
        {
            if (MotorZ.current(amp) >= Z_PUNCH_STALL_AMP)
            {
                bottomStalled = true;
            }
            else if (fabs(MotorZ.velocity(rpm)) < 2.0)
            {
                zStallCount++;
                if (zStallCount >= 32)
                {
                    bottomStalled = true;
                }
            }
            else
            {
                zStallCount = 0;
            }
        }
        wait(5, msec);
    }

    MotorZ.spin(reverse, 20, percent);
    timer dwellTimer;
    dwellTimer.clear();
    while (dwellTimer.time(msec) < PUNCH_SQUEEZE_MSEC)
    {
        wait(5, msec);
    }
    MotorZ.stop(hold);

    dwellTimer.clear();
    while (dwellTimer.time(msec) < dwellTimeMsec)
    {
        wait(5, msec);
    }

    double bottomPos = MotorZ.position(degrees);
    double liftTarget = bottomPos + Z_LIFT_DEG;

    strokeTimer.clear();
    MotorZ.setMaxTorque(70, percent);
    MotorZ.spin(forward, 20, percent);
    while (MotorZ.position(degrees) < liftTarget &&
           strokeTimer.time(msec) < 1500 &&
           (strokeTimer.time(msec) < 150 ||
            MotorZ.current(amp) < STALL_CURRENT_AMP))
    {
        wait(5, msec);
    }
    MotorZ.stop(hold);
    MotorZ.setMaxTorque(100, percent);
    wait(Z_POST_LIFT_PAUSE_MS, msec);

    return ((MotorZ.position(degrees) - bottomPos) >= Z_MIN_CLEAR_DEG);
}

// Section 4: Braille Lookup, Paper Feed & Shutdown - Liam McCarthy

// Decode ASCII letter or space into 3x2 Braille matrix
bool getCharDotMatrix(char letter, bool cellMatrix[3][2])
{
    if (letter >= 'a' && letter <= 'z')
    {
        letter = (char)(letter - 'a' + 'A');
    }

    if (letter == ' ')
    {
        for (int r = 0; r < 3; r++)
        {
            for (int c = 0; c < 2; c++)
            {
                cellMatrix[r][c] = false;
            }
        }
        return true;
    }

    if (letter < 'A' || letter > 'Z')
    {
        for (int r = 0; r < 3; r++)
        {
            for (int c = 0; c < 2; c++)
            {
                cellMatrix[r][c] = false;
            }
        }
        return false;
    }

    int alphaIndex = (int)(letter - 'A');
    for (int r = 0; r < 3; r++)
    {
        for (int c = 0; c < 2; c++)
        {
            cellMatrix[r][c] = BRAILLE_ALPHABET[alphaIndex][r][c];
        }
    }
    return true;
}

// Advance paper strip by feedDegrees using single roller axle (PORT9)
bool advancePaperStrip(double feedDegrees, bool holdAfterFeed)
{
    if (feedDegrees > 0.0)
    {
        double targetY = MotorFeed.position(degrees) + feedDegrees;
        int feedSpeed = 18;
        if (feedDegrees > 45.0)
        {
            feedSpeed = 50;
        }
        double maxFeedMs = 1500.0 + feedDegrees * 12.0;

        timer feedTimer;
        feedTimer.clear();

        MotorFeed.spin(forward, feedSpeed, percent);
        while (MotorFeed.position(degrees) < targetY - POS_TOLERANCE_DEG &&
               feedTimer.time(msec) < maxFeedMs)
        {
            if (feedTimer.time(msec) > 250 &&
                MotorFeed.current(amp) > STALL_CURRENT_AMP)
            {
                MotorFeed.stop(brake);
                return false;
            }
            wait(5, msec);
        }
    }

    if (holdAfterFeed)
    {
        MotorFeed.stop(hold);
    }
    else
    {
        MotorFeed.stop(coast);
    }
    return true;
}

// Raise Z 2x, eject sheet, park X, coast motors, display summary
void executeCleanShutdown(int totalCharsEmbossed, int totalDotsPunched,
                          double totalTimeSec, int faultCode,
                          int colorGen, colorsensor &colorG1,
                          optical &opticalG2)
{
    timer shutdownTimer;
    shutdownTimer.clear();

    double extraLiftDeg = Z_LIFT_DEG;
    if (faultCode == FAULT_Z_LIFT)
    {
        extraLiftDeg = 2.0 * Z_LIFT_DEG;
    }
    double endLiftTarget = MotorZ.position(degrees) + extraLiftDeg;
    MotorZ.setMaxTorque(70, percent);
    MotorZ.spin(forward, 20, percent);
    while (MotorZ.position(degrees) < endLiftTarget &&
           shutdownTimer.time(msec) < 2000 &&
           (shutdownTimer.time(msec) < 150 ||
            MotorZ.current(amp) < STALL_CURRENT_AMP))
    {
        wait(10, msec);
    }
    MotorZ.stop(hold);
    MotorZ.setMaxTorque(100, percent);
    wait(Z_POST_LIFT_PAUSE_MS, msec);

    if (faultCode == FAULT_NONE)
    {
        advancePaperStrip(EJECT_FEED_DEG, false);
    }
    else
    {
        advancePaperStrip(0.0, false);
    }

    moveCarriageToColumn(-1, 0, COL_ZERO_DEG,
                         INTRA_COL_DEG, CHAR_PITCH_DEG);

    MotorFeed.stop(coast);
    MotorX.stop(coast);
    MotorZ.stop(coast);
    if (colorGen == 1)
    {
        colorG1.setLight(ledState::off);
    }
    else if (colorGen == 2)
    {
        opticalG2.setLight(ledState::off);
    }

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    if (faultCode != FAULT_NONE)
    {
        TouchStatus.on(red);
        Brain.playSound(siren);
        if (faultCode == FAULT_BED_BLOCKED)
        {
            Brain.Screen.print("FAULT:Bed<20mm");
        }
        else if (faultCode == FAULT_SAFETY_TIMEOUT)
        {
            Brain.Screen.print("FAULT:Safety");
        }
        else if (faultCode == FAULT_X_STALL)
        {
            Brain.Screen.print("FAULT:X Stall");
        }
        else if (faultCode == FAULT_Z_LIFT)
        {
            Brain.Screen.print("FAULT:Z Lift");
        }
        else
        {
            Brain.Screen.print("FAULT:Y Jam");
        }
    }
    else
    {
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

// Main row-by-row horizontal word embossing loop
int main()
{
    vexcodeInit();

    // Auto-detect Gen 1 / Gen 2 color and distance sensors on ports 1-12
    int colorPort = PORT12;
    int colorGen = 0;
    int distPort = PORT8;
    int distGen = 0;
    for (int p = PORT1; p <= PORT12; p++)
    {
        IQ_DeviceType dt = vexDeviceTypeGetByIndex(p);
        if (dt == kDeviceTypeRgbSensor)
        {
            colorPort = p;
            colorGen = 1;
        }
        else if (dt == kDeviceTypeOpticalSensor)
        {
            colorPort = p;
            colorGen = 2;
        }
        else if (dt == kDeviceTypeSonarSensor)
        {
            distPort = p;
            distGen = 1;
        }
        else if (dt == kDeviceTypeDistanceSensor)
        {
            distPort = p;
            distGen = 2;
        }
    }

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
                     distGen, sonarG1, distG2))
    {
        executeCleanShutdown(0, 0, 0.0, FAULT_BED_BLOCKED,
                             colorGen, colorG1, opticalG2);
        return 0;
    }

    int wordLen = selectWordFromController(wordBuffer, MAX_WORD_LENGTH);

    for (int i = 0; i < wordLen; i++)
    {
        charValid[i] = getCharDotMatrix(wordBuffer[i], wordCells[i]);
        if (charValid[i])
        {
            validCharCount++;
        }
    }

    timer jobTimer;
    jobTimer.clear();
    TouchStatus.on(green);

    if (!advancePaperStrip(LOAD_FEED_DEG, true))
    {
        faultCode = FAULT_Y_JAM;
    }

    int row = 0;
    while (row < 3 && faultCode == FAULT_NONE)
    {
        if (!checkSafetyAndPaper(isPaused, jobTimer, paperThreshold,
                                 distThreshold, colorGen, colorG1,
                                 opticalG2, distGen, sonarG1, distG2))
        {
            faultCode = FAULT_SAFETY_TIMEOUT;
        }

        int charIdx = 0;
        while (charIdx < wordLen && faultCode == FAULT_NONE)
        {
            if (charValid[charIdx] && wordBuffer[charIdx] != ' ')
            {
                int col = 0;
                while (col < 2 && faultCode == FAULT_NONE)
                {
                    if (!checkSafetyAndPaper(isPaused, jobTimer,
                                             paperThreshold, distThreshold,
                                             colorGen, colorG1, opticalG2,
                                             distGen, sonarG1, distG2))
                    {
                        faultCode = FAULT_SAFETY_TIMEOUT;
                    }
                    else
                    {
                        int effectiveCol = col;
                        if (MIRROR_COLUMNS_FOR_DOWNWARD_PUNCH)
                        {
                            effectiveCol = 1 - col;
                        }

                        if (!moveCarriageToColumn(charIdx, effectiveCol,
                                                  COL_ZERO_DEG,
                                                  INTRA_COL_DEG,
                                                  CHAR_PITCH_DEG))
                        {
                            faultCode = FAULT_X_STALL;
                        }
                        else if (wordCells[charIdx][row][col])
                        {
                            if (!punchBrailleDot(calibratedPressDeg,
                                                 PUNCH_DWELL_MSEC))
                            {
                                faultCode = FAULT_Z_LIFT;
                            }
                            else
                            {
                                totalDotsPunched++;
                                updateLiveTelemetry(wordBuffer[charIdx],
                                                    row, 3,
                                                    totalDotsPunched,
                                                    jobTimer, colorGen,
                                                    colorG1, opticalG2,
                                                    distGen, sonarG1,
                                                    distG2);
                            }
                        }
                        else
                        {
                            wait(100, msec);
                        }
                    }
                    col++;
                }
            }
            charIdx++;
        }

        if (faultCode == FAULT_NONE && row < 2)
        {
            if (!checkSafetyAndPaper(isPaused, jobTimer, paperThreshold,
                                     distThreshold, colorGen, colorG1,
                                     opticalG2, distGen, sonarG1,
                                     distG2))
            {
                faultCode = FAULT_SAFETY_TIMEOUT;
            }
            else if (!moveCarriageToColumn(-1, 0, COL_ZERO_DEG,
                                             INTRA_COL_DEG,
                                             CHAR_PITCH_DEG))
            {
                faultCode = FAULT_X_STALL;
            }
            else if (!advancePaperStrip(ROW_FEED_DEG, true))
            {
                faultCode = FAULT_Y_JAM;
            }
        }
        row++;
    }

    int charsCompleted = 0;
    if (faultCode == FAULT_NONE)
    {
        charsCompleted = validCharCount;
    }

    executeCleanShutdown(charsCompleted, totalDotsPunched,
                         jobTimer.time(seconds), faultCode,
                         colorGen, colorG1, opticalG2);
    return 0;
}
