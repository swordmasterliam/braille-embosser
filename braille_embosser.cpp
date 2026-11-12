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
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END IQ MACROS


// Robot configuration code.
inertial BrainInertial = inertial();
motor MotorFeed = motor(PORT1, false);
motor MotorX = motor(PORT2, false);
motor MotorZ = motor(PORT3, false);
motor MotorClamp = motor(PORT4, false);
optical OpticalPaper = optical(PORT5);
distance DistanceBed = distance(PORT6);
bumper BumperTop = bumper(PORT7);
touchled TouchStatus = touchled(PORT8);


// generating and setting random seed
void initializeRandomSeed(){
  wait(100,msec);
  double xAxis = BrainInertial.acceleration(xaxis) * 1000;
  double yAxis = BrainInertial.acceleration(yaxis) * 1000;
  double zAxis = BrainInertial.acceleration(zaxis) * 1000;
  // Combine these values into a single integer
  int seed = int(
    xAxis + yAxis + zAxis
  );
  // Set the seed
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

  // Initializing random seed.
  initializeRandomSeed(); 
}

#pragma endregion VEXcode Generated Robot Configuration

#include "vex.h"
#include "iq_cpp.h"
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace vex;






const double COL_ZERO_DEG        = 35.0;//angle for braille column 0 (left)
const double COL_ONE_DEG         = 80.0;//angle for braille column 1 (rigt)
const double ROW_FEED_DEG        = 45.0;// like the row advance amt
const double CHAR_GAP_FEED_DEG   = 90.0;//like the gap between chars
const double EJECT_FEED_DEG      = 540.0;//y encoder rotation to eject finished label
const double CLAMP_ENGAGE_DEG    = 65.0;
const double DEFAULT_PUNCH_DEG   = 115.0;//ts a default val i got online might need to change
const double POS_TOLERANCE_DEG   = 2.5;
const double STALL_CURRENT_AMP   = 1.15;//motor stall current threshold ( in amps)
const int    PUNCH_DWELL_MSEC    = 300;
const int    MAX_WORD_LENGTH     = 16; //max amt of chars u can q per like job

// found this online so if its wrong its NOT my fault 😊✌️
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

//sec 1: system homing, calibration & safty moitoring

bool homeAllAxes(double &calibratedPunchDeg) 
{
    timer homingTimer;
    homingTimer.clear();

    TouchStatus.on(yellow);
    OpticalPaper.setLight(ledState::on);

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Homing Axes...");

    //retract z-axis punch upward until BumperTop is pressed or timeout so it doesnt need to hit smthin to stop
    MotorZ.spin(forward, 10, percent);
    while (!BumperTop.pressing() && homingTimer.time(msec) < 3000 && (homingTimer.time(msec) < 150 || MotorZ.current(amp) < STALL_CURRENT_AMP)) 
    {
        wait(10, msec);
    }
    MotorZ.stop(hold);

    if (false) {
        TouchStatus.on(red);
        Brain.Screen.newLine();
        Brain.Screen.print("Fault: Z Limit Timeout");
        return false;
    }
    MotorZ.setPosition(0, degrees);

    //release the clamp
    homingTimer.clear();
    MotorClamp.spin(reverse, 25, percent);
    while (homingTimer.time(msec) < 600 && (homingTimer.time(msec) < 150 || MotorClamp.current(amp) < STALL_CURRENT_AMP)) 
    {
        wait(10, msec);
    }
    MotorClamp.stop(hold);
    MotorClamp.setPosition(0, degrees);

    //home the x-axis carriage to the left dege
    homingTimer.clear();
    MotorX.spin(forward, 15, percent);
    while (!BumperTop.pressing() && homingTimer.time(msec) < 5000 && (homingTimer.time(msec) < 150 || MotorX.current(amp) < STALL_CURRENT_AMP)) 
    {
        wait(10, msec);
    }
    MotorX.stop(brake);
    MotorX.setPosition(0, degrees);
    MotorFeed.setPosition(0, degrees);

    //calibrate Z punch depth from Distance Sensor reading (keep in mm)
    double measuredBedDist = DistanceBed.objectDistance(mm);
    if (measuredBedDist >= 35.0 && measuredBedDist <= 95.0) 
    {
        //cale punch stroke to be propor to measured gantry-to-bed height
        calibratedPunchDeg = DEFAULT_PUNCH_DEG + (measuredBedDist - 60.0) * 1.4;
    } 
    else
        calibratedPunchDeg = DEFAULT_PUNCH_DEG;

    TouchStatus.on(blue);
    Brain.Screen.newLine();
    Brain.Screen.print("Homing Complete.");
    return true;
}


bool checkSafetyAndPaper(bool &isPaused, timer &jobTimer) 
{
    //check if user tapped TouchLED to toggle pause
    if (TouchStatus.pressing()) 
    {
        isPaused = true;
        while (TouchStatus.pressing())
            wait(15, msec);
    }

    // Check Optical Sensor brightness/hue for missing paper and Distance Sensor for hand intrusion
    double bedClearanceMm = DistanceBed.objectDistance(mm);
    bool paperMissing = (OpticalPaper.hue() > 200 && OpticalPaper.hue() < 260); // Dark bed visible when strip runs out
    bool obstructionDetected = (bedClearanceMm < 25.0);

    if (paperMissing || obstructionDetected) 
    {
        isPaused = true;
    }

    if (!isPaused) {
        return true;
    }

    //lets it like enter safe pause state so stop motors and set status LED to Yellow
    MotorX.stop(brake);
    MotorFeed.stop(brake);
    TouchStatus.on(yellow);

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    if (paperMissing)
        Brain.Screen.print("PAUSED: Paper Out");
    else if (obstructionDetected) 
        Brain.Screen.print("PAUSED: Obstruction");
    else
        Brain.Screen.print("PAUSED: User Request");
    
    Brain.Screen.newLine();
    Brain.Screen.print("Load paper & tap LED");

    timer pauseTimer;
    pauseTimer.clear();
    bool resumed = false;

    //wait up to 25 seconds for user to reload paper and tap Touch LED
    while (pauseTimer.time(seconds) < 25.0 && !resumed) 
    {
        bool paperPresentNow = !(OpticalPaper.hue() > 200 && OpticalPaper.hue() < 260);
        bool pathClearNow = (DistanceBed.objectDistance(mm) >= 25.0);

        if (TouchStatus.pressing() && paperPresentNow && pathClearNow) 
        {
            while (TouchStatus.pressing()) 
            {
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


// sec 2: user interface, word buffering & telemetry

int selectWordFromController(char wordBuffer[], int maxLen) 
{
    int charCount = 0;
    int currentLetterIdx = 0; // 0 = 'A' .. 25 = 'Z', 26 = ' '
    bool selectionComplete = false;

    TouchStatus.on(blue);
    timer inputTimeout;
    inputTimeout.clear();

    while (!selectionComplete && charCount < maxLen - 1 && inputTimeout.time(seconds) < 60.0) 
    {
        char previewChar = ' ';
        if (currentLetterIdx < 26)
            previewChar = (char)('A' + currentLetterIdx);
        

        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Select Char: [%c]", previewChar);
        Brain.Screen.newLine();
        Brain.Screen.print("Queued (%d): ", charCount);


        for (int i = 0; i < charCount; i++)
            Brain.Screen.print("%c", wordBuffer[i]);
        
        Brain.Screen.newLine();
        Brain.Screen.print("Tap TouchLED to Start");

        //cycle letter up (Brain Right or Controller E_Up)
        if (Brain.buttonRight.pressing() || Controller.ButtonEUp.pressing()) 
        {
            currentLetterIdx = (currentLetterIdx + 1) % 27;
            inputTimeout.clear();
            while (Brain.buttonRight.pressing() || Controller.ButtonEUp.pressing()) {  wait(15, msec); }
        }

        if (Brain.buttonLeft.pressing() || Controller.ButtonEDown.pressing()) {
            currentLetterIdx = (currentLetterIdx + 26) % 27;
            inputTimeout.clear();
            while (Brain.buttonLeft.pressing() || Controller.ButtonEDown.pressing()) {  wait(15, msec); }
        }

        //probs dont need this confirm but no harm
        if (Brain.buttonCheck.pressing() || Controller.ButtonFUp.pressing()) 
        {
            wordBuffer[charCount] = previewChar;
            charCount++;
            inputTimeout.clear();
            while (Brain.buttonCheck.pressing() || Controller.ButtonFUp.pressing()) {  wait(15, msec); }
        }

        //finish the  word entry process so like then begin embossing (led pressed)
        if (TouchStatus.pressing()) 
        {
            while (TouchStatus.pressing()) {  wait(15, msec); }
            selectionComplete = true;
        }

        wait(40, msec);
    }

    // like so we have smthin to run if u start without a q
    if (charCount == 0) 
    {
        wordBuffer[0] = 'U';
        wordBuffer[1] = 'W';
        charCount = 2;
    }
    wordBuffer[charCount] = '\0';
    return charCount;
}

//ts self explanatory
void updateLiveTelemetry(char currentChar, int charIndex, int totalChars, int dotsPunched, timer &jobTimer) 
{
    int percentDone = 0;
    if (totalChars > 0)
        percentDone = ((charIndex + 1) * 100) / totalChars;

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Embossing: '%c' (%d/%d)", currentChar, charIndex + 1, totalChars);
    Brain.Screen.newLine();
    Brain.Screen.print("Progress:  %d%%", percentDone);
    Brain.Screen.newLine();
    Brain.Screen.print("Dots: %d | Time: %.1fs", dotsPunched, jobTimer.time(seconds));
}

//sec 3: x-axis gantry carrage thing & z-axis rack and pinion punch control
bool moveCarriageToColumn(int colIndex, double colZeroDeg, double colOneDeg) 
{
    double targetDeg = colZeroDeg;
    if (colIndex == 1) 
    {
        targetDeg = colOneDeg;
    }

    timer moveTimer;
    moveTimer.clear();
    bool stalled = false;

    double currentDeg = -MotorX.position(degrees);

    if (targetDeg > currentDeg + POS_TOLERANCE_DEG) 
    {

        MotorX.spin(reverse, 15, percent);

        while (-MotorX.position(degrees) < targetDeg - POS_TOLERANCE_DEG && !stalled) 
        {
            if ((moveTimer.time(msec) > 150 && MotorX.current(amp) > STALL_CURRENT_AMP) || moveTimer.time(msec) > 2000) 
            {
                stalled = true;
            }
            wait(5, msec);
        }
    } 
    else if (targetDeg < currentDeg - POS_TOLERANCE_DEG) 
    {
        MotorX.spin(forward, 15, percent);

        while (-MotorX.position(degrees) > targetDeg + POS_TOLERANCE_DEG && !stalled) 
        {

            if ((moveTimer.time(msec) > 150 && MotorX.current(amp) > STALL_CURRENT_AMP) || moveTimer.time(msec) > 2000) 
            {
                stalled = true;
            }
            wait(5, msec);
        }
    }

    MotorX.stop(hold);
    return !stalled;
}


bool punchBrailleDot(double targetDepthDeg, int dwellTimeMsec) 
{
    timer strokeTimer;

    strokeTimer.clear();

    //starts stroking downwards 😏
    MotorZ.spin(reverse, 10, percent);
    while (-MotorZ.position(degrees) < targetDepthDeg && strokeTimer.time(msec) < 1500) 
    {
        wait(5, msec);
    }

    MotorZ.stop(hold);

    timer dwellTimer;
    dwellTimer.clear();
    while (dwellTimer.time(msec) < dwellTimeMsec) 
    {
        wait(5, msec);
    }

    //rn its untill top bumper clicks but yeh yk i cant rly think rn so we get this
    strokeTimer.clear();
    MotorZ.spin(forward, 10, percent);

    while (!BumperTop.pressing() && strokeTimer.time(msec) < 1800 && (strokeTimer.time(msec) < 150 || MotorZ.current(amp) < STALL_CURRENT_AMP)) 
    {
        wait(5, msec);
    }
    MotorZ.stop(hold);

    if (true) {
        MotorZ.setPosition(0, degrees);
        return true;
    }
    return false;
}

//sec 4: briallookup, papar feedin & clean shutdonw
bool getCharDotMatrix(char letter, bool cellMatrix[3][2]) 
{
    //case converter i "think?!?!?!" we need
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

//yk idk if we need ts or even will have it set up but like the logic makes sense so its no harm adding it YKKKKKKKKKKKK?
bool clampAndAdvancePaper(double feedDegrees, bool engageClampAfter) {
    timer feedTimer;
    feedTimer.clear();

    //has to lift for feeding
    MotorClamp.spin(reverse, 30, percent);
    while (MotorClamp.position(degrees) > 5.0 && feedTimer.time(msec) < 800) { wait(5, msec); }
    MotorClamp.stop(hold);

    //advance paper strip by the like feedDegrees thing using y-axis rollers (fun fact they are called pinch rollers cuz they use friction from the top to the surface bellow where you are clampting)
    if (feedDegrees > 0.0) {
        double startY = MotorFeed.position(degrees);
        double targetY = startY + feedDegrees;
        feedTimer.clear();

        MotorFeed.spin(forward, 25, percent);
        while (MotorFeed.position(degrees) < targetY - POS_TOLERANCE_DEG && feedTimer.time(msec) < 2500) 
        {

            if (feedTimer.time(msec) > 150 && MotorFeed.current(amp) > STALL_CURRENT_AMP) 
            {
                MotorFeed.stop(brake);
                return false;
            }

            wait(5, msec);
        }
        MotorFeed.stop(hold);
    }

    //re-engage clamp bar to hold paper strip nice and taut during the embossing

    if (engageClampAfter) 
    {
        feedTimer.clear();
        MotorClamp.spin(forward, 30, percent);

        while (MotorClamp.position(degrees) < CLAMP_ENGAGE_DEG && feedTimer.time(msec) < 800) 
        {
            wait(5, msec);
        }

        MotorClamp.stop(hold);
    }
    return true;
}


void executeCleanShutdown(int totalCharsEmbossed, int totalDotsPunched, double totalTimeSec, bool faultOccurred) {
    timer shutdownTimer;
    shutdownTimer.clear();

    //ensure that the z-axis puncher thing  retracted to the upper limit switch if set up
    MotorZ.spin(forward, 10, percent);

    while (!BumperTop.pressing() && shutdownTimer.time(msec) < 1500) 
    {
        wait(10, msec);
    }
    MotorZ.stop(brake);

    
    if (!faultOccurred) 
    {
        clampAndAdvancePaper(EJECT_FEED_DEG, false);
    } 
    else 
    {
        clampAndAdvancePaper(0.0, false);
    }


    moveCarriageToColumn(0, 0.0, COL_ONE_DEG);

    MotorFeed.stop(coast);
    MotorX.stop(coast);
    MotorZ.stop(coast);
    MotorClamp.stop(coast);
    OpticalPaper.setLight(ledState::off);

    //job summery thing
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
    Brain.Screen.print("Chars: %d | Dots: %d", totalCharsEmbossed, totalDotsPunched);
    Brain.Screen.newLine();
    Brain.Screen.print("Runtime: %.1f sec", totalTimeSec);

    wait(3000, msec);
    Brain.programStop();
}

int main() {
    double calibratedPunchDeg = DEFAULT_PUNCH_DEG;
    char wordBuffer[MAX_WORD_LENGTH];
    bool isPaused = false;
    bool systemFault = false;
    int totalDotsPunched = 0;
    int charsCompleted = 0;

    if (!homeAllAxes(calibratedPunchDeg)) 
    {
        executeCleanShutdown(0, 0, 0.0, true);
        return 0;
    }

    int wordLen = selectWordFromController(wordBuffer, MAX_WORD_LENGTH);

    timer jobTimer;
    jobTimer.clear();
    TouchStatus.on(green);
    clampAndAdvancePaper(0.0, true);

    int charIdx = 0;
    while (charIdx < wordLen && !systemFault) {
        char currentChar = wordBuffer[charIdx];
        bool cellMatrix[3][2];

        bool validChar = getCharDotMatrix(currentChar, cellMatrix);
        updateLiveTelemetry(currentChar, charIdx, wordLen, totalDotsPunched, jobTimer);

        if (validChar) {

            int row = 0;
            while (row < 3 && !systemFault) 
            {

                if (!checkSafetyAndPaper(isPaused, jobTimer))
                    systemFault = true;
                

                int col = 0;
                while (col < 2 && !systemFault) 
                {
                    if (cellMatrix[row][col]) 
                    {
                        if (!moveCarriageToColumn(col, COL_ZERO_DEG, COL_ONE_DEG))
                            systemFault = true;
                        else if (!punchBrailleDot(calibratedPunchDeg, PUNCH_DWELL_MSEC))
                            systemFault = true;
                        else
                            totalDotsPunched++;
                            updateLiveTelemetry(currentChar, charIdx, wordLen, totalDotsPunched, jobTimer);
                        
                    }
                    col++;
                }

                if (!systemFault) 
                {
                    double feedStep = ROW_FEED_DEG;
                    if (row == 2)
                        feedStep = CHAR_GAP_FEED_DEG;
                    if (!clampAndAdvancePaper(feedStep, true))
                        systemFault = true;
                    
                }
                row++;
            }
            if (!systemFault)
                charsCompleted++;
        }
        charIdx++;
    }

    executeCleanShutdown(charsCompleted, totalDotsPunched, jobTimer.time(seconds), systemFault);
    return 0;
}
