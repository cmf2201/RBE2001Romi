// Written and executed by Team 17 ( --THE DREAM TEAM-- )

#include <Arduino.h>
#include <Romi32U4.h>
#include "BlueMotor/BlueMotor.h"
#include <servo32u4.h>
#include "RemoteControl/RemoteConstants.h"
#include "RemoteControl/RemoteControl.h"
#include <QTRSensors.h>
#include <Chassis.h>
#include "constants.h"
#include <HCSR04.h>

byte triggerPin = 3;
byte echoPin = 30;


//BlueMotor Declarations
BlueMotor motor;

//ROMI buttons
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;

//Remote Control Declarations
RemoteControl remoteControl(2);

//Servo Declarations 
Servo32U4Pin5 gripperServo;

const byte servoAnalogSensor = A0;

boolean bottomOutGripper = true;

//Chassis Declarations
Chassis chassis;

int assistedLineFollowingTarget = 0;

//QTR Sensors Declarations
QTRSensors qtr;

const uint8_t SensorCount = 2;
uint16_t sensorValues[SensorCount];

//Servo Declarations
#define SERVO_STOP 0
#define SERVO_JAW_DOWN 500
#define SERVO_JAW_UP 2500


//Linear Servo declarations
int linearPotPin = A0;


int linearPotVoltageADC = 500;
int jawOpenPotVoltageADC = 490;
int jawClosedPotVoltageADC = 1022;
int previous1PotVoltage = 0;
int previous2PotVoltage = 0;
<<<<<<< HEAD
=======
Timer printTimer(printDelay);
bool servoToggle = true;

>>>>>>> 9063c0f4141d76fcbc4d25bf29ebac4c1455fdc1

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

// 

//motor positions
<<<<<<< HEAD
int firstSpot = 8114;
int secondSpot = 3970;
=======
int firstSpot45deg = 3236;
int secondSpot45deg = 4476;
int firstSpot60deg = 8150;
int secondSpot60deg = 7298;
>>>>>>> 9063c0f4141d76fcbc4d25bf29ebac4c1455fdc1

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

// Remote Testing Variables
// IRDecoder decoder(13);

bool paused = false;

void moveUp();
void moveDown();
void closeServo();
void openServo();
void lineFollowCalibration();
void lineFollowToPosition();
void lineFollowToPositionBegin();
void stopIt();
void qtrReadings();
void getChassisPosition();
void lineFollowBack();
void rotateLeft();
void lineFollow(int eff);
void lineFollowForward();

float getUltrasonicDistance();





void setup()
{
  //Start the serial monitor
  Serial.begin(9600);

  motor.setup();
  motor.reset();

  chassis.init();

  HCSR04.begin(triggerPin, echoPin);

  gripperServo.setMinMaxMicroseconds(0,2000);
  gripperServo.writeMicroseconds(0);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A4, A3}, SensorCount);

  remoteControl.setup();

  remoteControl.toggleFunc(moveUp,remoteUp);
  remoteControl.toggleFunc(moveDown,remoteDown);
  remoteControl.onPress(lineFollowToPositionBegin,remote0);
  remoteControl.toggleFunc(lineFollowToPosition,assistedLineFollowing);

  remoteControl.toggleFunc(lineFollowBack,remote2);

  remoteControl.onPress(getChassisPosition,remote8);
  remoteControl.onPress(rotateLeft,turnLeft);

  remoteControl.toggleFunc(lineFollowForward,remote3);
  remoteControl.onPress(lineFollowCalibration,remote4);


  remoteControl.eStop(stopIt,remoteEnterSave);

  delay(4000);
  Serial.println("READY!");

  // remoteControl.runCurrentFunctions();
}



void moveUp() {
  motor.setEffort(400);
  Serial.println("UP");
}

void moveDown() {
  motor.setEffort(-400);
  Serial.println("DOWN");
}

void stopIt() {
  Serial.println("ESTOP");
  Serial.println(motor.getPosition());
  motor.setEffort(0);
  gripperServo.writeMicroseconds(0);
  chassis.setMotorEfforts(0,0);
}


void closeServo() {
  gripperServo.writeMicroseconds(2000);
}

void openServo() {
  gripperServo.writeMicroseconds(10);
}

void lineFollowCalibration() {
  qtr.resetCalibration();
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }

  // print the calibration minimum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
}

//line follow until the given position is reached (in inches)
void lineFollowToPositionBegin() {
  chassis.driveFor(70.0,5.0);
  // remoteControl.startFunction(assistedLineFollowing);
}

//line follow until the given position is reach (in inches)
void lineFollowToPosition() {
  // Serial.println("TEST ON");
  // lineFollowing();
}


void qtrReadings() {
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print("sensor:");
    Serial.print(i);
    Serial.print('\t');
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.print("POSITION:");
  Serial.println(position);

}

//function for line following forward or backward
void lineFollow(int eff) {
  uint16_t position = qtr.readLineBlack(sensorValues);
  if(position > 700) {
    chassis.setMotorEfforts(eff,eff*.8);
  } else if (position < 300) {
    chassis.setMotorEfforts(eff*.8,eff);
  } else {
    chassis.setMotorEfforts(eff,eff);
  }
}

// Use the QTR sensors to have the ROMI follow a line
void lineFollowForward() {
  lineFollow(-80);
}

void lineFollowBack() {
  float distanceUlt = getUltrasonicDistance();
  Serial.println(distanceUlt);
  if(distanceUlt < 32.00) {

    chassis.setWheelSpeeds(-5.00,-5.00);
  } else {
    chassis.idle();
    remoteControl.stopFunction(remote2);
    remoteControl.startFunction(turnLeft);
  }
}

void rotateLeft() {
  chassis.turnFor(90,90.0);
}



void getChassisPosition() {
  chassis.printEncoderCounts();
}

float getUltrasonicDistance() {
  double* distances = HCSR04.measureDistanceCm();

  return distances[0];
}

//  Make sure the Battery Voltage has not dropped to low (below 6000)
void batteryCheck() {
  //  Make sure the Battery Voltage is correct
  if (readBatteryMillivolts() <= 6000) {
    Serial.print("BATTERY LOW: ");
    Serial.println(readBatteryMillivolts());
    buttonC.waitForButton();
  }
}


<<<<<<< HEAD
// void checkRemote(){
//   int16_t code = decoder.getKeyCode();
//   switch (code)
//   {
//   case remotePlayPause:
//     // resetMode();
//    paused = true;
//     break;
  
//   case remoteVolPlus:
//     paused = false;
//     break;
//   }
//   Serial.println(paused);
// }
=======
//closes the bottom out gripper, but stops if the gripper cannot close
void closeBottomGripper() {
  if(servoActive) {
    int servoCurrentPosition = analogRead(servoEnc);
    Serial.println(servoCurrentPosition);

    // every (servoReadingDelay) ms, try to move the gripper
    if(millis() > servoPrevMS + servoReadingDelay) {
      gripperServo.writeMicroseconds(servoClosedPositionMS);

      //check if the servo is getting stuck. If so, open the Bottom Gripper.
      if(abs(servoPrevReading - servoCurrentPosition) < servoStuckTolerance) {
        //once servoCount has gotten to 15 or greater, run the bottom out gripper code instead.
        if(servoStillCount >= 15) {
          Serial.println("STUCK!");
          openBottomGripper();
        }
        servoStillCount++;
        
      } else {
        servoStillCount = 0;
      }

      //if Gripper has reached target position, finish movement
      if(abs(servoCurrentPosition - servoClosedPositionAR) <= servoCloseTolerance) {
        Serial.println("FINISHED!");
        servoActive = false;
      }

      servoPrevReading = servoCurrentPosition;
      servoPrevMS = millis();
    }
  }

}

//runs through the bottom out gripper functions 
void bottomOutGripper() {
  if(buttonA.isPressed()) {
    servoActive = true;
    delay(1000);
  }
  if(buttonB.isPressed()) {
    closeBottomGripper();
  }
  if(buttonC.isPressed()){
    openBottomGripper();
  }
}

// Allows testing of Linear Gripper functionality, stopping and opening if servo is stuck, and closing onto plate on B press.
void openGripper() {
  // Get Pot Value
  linearPotVoltageADC = analogRead(linearPotPin);
  Serial.print("Initial linearPotVoltageADC:   ");
  Serial.println(linearPotVoltageADC);  
    // Move Jaw Down
    jawServo.writeMicroseconds(servoJawDown);

    if (linearPotVoltageADC < jawOpenPotVoltageADC)
    {
      linearPotVoltageADC = analogRead(linearPotPin);
      if (printTimer.isExpired()){
        Serial.print("linearPotVoltageADC:    ");
        Serial.println(linearPotVoltageADC);
      }
    } else {
    // Stop servo onced jaw is opened
    jawServo.writeMicroseconds(servoStop);
    remoteControl.stopFunction(remote2);
    }
}

void closeGripper() {
    linearPotVoltageADC = analogRead(linearPotPin);
  Serial.print("Initial linearPotVoltageADC:   ");
  Serial.println(linearPotVoltageADC); 
      // // Move Jaw Up
     jawServo.writeMicroseconds(servoJawUp);

    if (linearPotVoltageADC > jawClosedPotVoltageADC)
    {    
      linearPotVoltageADC = analogRead(linearPotPin);
      
      if (printTimer.isExpired()){
        Serial.print("linearPotVoltageADC:     ");
        Serial.println(linearPotVoltageADC);
       }
    
    } else {
    // Stop servo onced jaw is vlosed
    jawServo.writeMicroseconds(servoStop);
    remoteControl.stopFunction(remote1);
    }
}

// Moves motor until 4-bar linkage and gripper are positioned to grab the 60 degree plate
void takeOffHighPlate() {
  if(motor.getToggleOff()) {
    motor.moveTo(firstSpot60deg);
  } else {
    remoteControl.stopFunction(remote7);
    motor.setToggleOff(true);
  }
}

// Moves motor until 4-bar linkage and gripper are positioned to grab the 60 degree plate
void takeOffLowPlate() {
  if(motor.getToggleOff()) {
    motor.moveTo(firstSpot45deg);
  } else {
    remoteControl.stopFunction(remote8);
    motor.setToggleOff(true);
  }
}

void goToPositions() {
  // if A button is pressed, go to encoded 0 position, also the position at which the plate is grabbed off the block
  if(buttonA.isPressed()) {

    motor.moveTo(0);
  }
  //if B is presssed, go to hight plate position
  if(buttonB.isPressed()) {

    takeOffHighPlate();

    
  }

  //if C is pressed, go to low plate position
  if(buttonC.isPressed()) {
    takeOffLowPlate();
  }
}


// reset mode should be used to manually lower or change the linkage position to a zero value for the encoder
void resetMode() {
  // if A button is pressed, go up
  if(buttonA.isPressed()) {
    motor.setEffort(400);
  }
  //if B is presssed, go down
  if(buttonB.isPressed()) {
    motor.setEffort(-400);

    
  }

  // if neither button is being pressed, stop motor
  if (!(buttonA.isPressed() || buttonB.isPressed() || buttonC.isPressed())) {
    motor.setEffort(0);
  }
}

void resetEncoder() {
  motor.reset();
}

void setup()
{
  //Start the serial monitor
  Serial.begin(9600);

  motor.setup();
  motor.reset();

  chassis.init();

  gripperServo.setMinMaxMicroseconds(0,2000);
  gripperServo.writeMicroseconds(0);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A4, A3}, SensorCount);

  pinMode(ultrasonicEchoPin,INPUT); 
  pinMode(ultrasonicTriggerPin, OUTPUT);

  remoteControl.setup();

  remoteControl.onPress(moveUp,remoteUp);
  remoteControl.onPress(moveDown,remoteDown);

  remoteControl.onPress(stopIt,remoteEnterSave);
  remoteControl.onPress(printTest,remote0);

  remoteControl.eStop(stopIt,remote9);

  remoteControl.toggleFunc(closeGripper,remote1); //original: closeServo and openServo
  remoteControl.toggleFunc(openGripper,remote2);

  remoteControl.onPress(resetEncoder, remote3);

  remoteControl.toggleFunc(takeOffHighPlate, remote7);
  remoteControl.toggleFunc(takeOffLowPlate, remote8);

  remoteControl.onPress(lineFollowCalibration,remote4);
  remoteControl.toggleFunc(qtrReadings,remote6);
  remoteControl.toggleFunc(lineFollowing,remote5);

  delay(4000);
  Serial.println("READY!");

  // remoteControl.runCurrentFunctions();
}

>>>>>>> 9063c0f4141d76fcbc4d25bf29ebac4c1455fdc1

void loop()
{
  //check the battery
  batteryCheck();

  remoteControl.checkRemoteButtons();

}





