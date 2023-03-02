#include <QTRSensors.h>
#include "Timer.h"
#include <Romi32U4.h>

// QTR Sensors
QTRSensors qtr;

#define lineFollowCaliLow 550
#define lineFollowCaliHigh 900
bool BOTTOM_OUT_GRIPPER = true;

const uint8_t SensorCount = 2;
uint16_t sensorValues[SensorCount];

// Linear Servo declarations
int servoPin = 0;
int linearPotPin = A0;
int servoStop = 1490;
int servoJawDown = 500;
int servoJawUp = 2500;
int printDelay = 500;
int linearPotVoltageADC = 500;
int jawOpenPotVoltageADC = 1000;
int jawClosedPotVoltageADC = 450;
int previous1PotVoltage = 0;
int previous2PotVoltage = 0;
Timer printTimer(printDelay);
bool servoToggle = true;

// Gripper Servo Definitions
// int servoPin = 12;
int servoEnc = A0;

bool servoActive = false;

int servoClosedPositionMS = 1250;
int servoClosedPositionAR = 246;
int servoOpenedPositionMS = 10;
int servoOpenedPositionAR = 137;

int servoStuckTolerance = 4;
int servoCloseTolerance = 5;

int servoReadingDelay = 50;
int servoPrevReading, servoStillCount;
long servoPrevMS;

long servoDebounce;
long servoDelayTime = 1000;

bool servostop = true;

// motor positions
int firstSpot45deg = 3350;
int firstSpot60deg = 7284;

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


int previousMotorPosition = 0;

// Ultrasonic Sensor Variables
const byte ultrasonicTriggerPin = 3;
const byte ultrasonicEchoPin = 30;
int frontDistance = 0;
long ultrasonicSignalDuration = 0;
int towerDistanceOne = 8;
int towerDistanceTwo = 1;
int blockDistanceOne = 3;
int blockDistanceTwo = 3;

// Remote Testing Variables

bool paused = false;

void moveUp(void);
void moveDown(void);

// Variables for FSM
int currentState = 0;
int PrevState = 0;
bool GripperBool = true;