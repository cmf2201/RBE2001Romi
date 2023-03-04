#include <QTRSensors.h>
#include "Timer.h"
#include <Romi32U4.h>

// QTR Sensors
QTRSensors qtr;

#define lineFollowCaliLow 550
#define lineFollowCaliHigh 900

const uint8_t SensorCount = 2;
uint16_t sensorValues[SensorCount];

bool BOTTOM_OUT_GRIPPER = true;

// Linear Servo declarations
int servoPin = 0;
int linearPotPin = A0;
int servoStop = 1490;
int servoJawDown = 500;
int servoJawUp = 2500;
int printDelay = 500;
int linearPotVoltageADC = 500;
int jawOpenPotVoltageADC = 1000;
int jawClosedPotVoltageADC = 500;
int previous1PotVoltage = 0;
int previous2PotVoltage = 0;
Timer printTimer(printDelay);
bool servoToggle = true;

// Gripper Servo Definitions
// int servoPin = 12;
int servoEnc = A0;

bool servoActive = false;

int servoClosedPositionMS = 2000;
int servoClosedPositionAR = 400;
int servoOpenedPositionMS = 1000;
int servoOpenedPositionAR = 205;

int servoStuckTolerance = 4;
int servoCloseTolerance = 7;

int servoReadingDelay = 50;
int servoPrevReading, servoStillCount;
long servoPrevMS;

long servoDebounce;
long servoDelayTime = 1000;

bool servostop = true;

// motor positions

long lastSampleTime = 0;

bool deadBandTesting = false;
long timeToPrint = 5;
long now = 0;
long newPosition = 0;
long oldPosition = 0;
long sampleTime = 100;
int speedInRPM = 0;
float CPR = 270;
int motorEffort = 400;
int deadBandCurrentEffort = 0;

// motor positions for bottom out gripper robot
int bottomOutSpot45 = 3472;
int bottomOutSpot60 = 8400;

int previousMotorPosition = 0;

// Ultrasonic Sensor Variables
const byte ultrasonicTriggerPin = 3;
const byte ultrasonicEchoPin = 30;
int frontDistance = 0;
long ultrasonicSignalDuration = 0;
float towerDistanceOne = 10.10;
float towerDistanceTwo = 3.5;
float blockDistanceOne = 2.80;
float blockDistanceTwo = 2.80;

//back up variables
float wallToLineDist = 27.00;
float blockBackUpDist = 15.00;

// Remote Testing Variables

// Remote Testing Variables

bool paused = false;

void moveUp(void);
void moveDown(void);

// Variables for FSM
int currentState = 0;
int PrevState = 0;
bool GripperBool = true;
int startPositionLeft = 0;
int startPositionRight = 0;
bool driveMotionIncomplete;
bool turnMotionIncomplete;

int currentTarget;
int currentSpeed;

//constants for line following constants
#define lineFoundThresholdHigh 900



//function definitions (so they can be called anywhere)
void stopIt();
float ultrasonicDistance();
void ultrasonicDistanceTest();
void lineFollowCalibration();
void lineFollowing();
void driveFor();
void turnFor();
void linefollowTesting();
void moveUp();
void moveDown();
void openBottomGripper();
void closeBottomGripper();
void openGripper();
void closeGripper();
void turnToPausableResume();
void turnToPausableResume(float target, float speed);
void turnToPausable(float target, float speed);
void driveToPausableResume();
void driveToPausableResume(float target, float speed);
void driveToPausable(float target, float speed);
void takeOffHighPlate();
void takeOffLowPlate();
void raiseSlightly();
void lowerSlightly();
void goToBottomEncoder();
void fourBarLow();
void resetEncoder();
void fourtyFiveDegreeSide();
void sixtyDegreeSide();
void updateCurrentState();
void calebFunction();
void rotateUntilLine();
// void rotateRightUntilLine();
// void rotateLeftUntilLine();
void forwardUntilLine();
void returnMotorPosition();
void driveToPausableTest();
void driveToPausableTest2();
void turnToPausableTest();
void turnToPausableTest2();
void unPause();