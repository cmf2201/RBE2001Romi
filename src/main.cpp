#include <Arduino.h>
#include <Romi32U4.h>
#include "BlueMotor/BlueMotor.h"
#include <servo32u4.h>
#include "Timer.h"
// #include "IRdecoder.h"
#include "RemoteControl/RemoteConstants.h"
#include "RemoteControl/RemoteControl.h"
#include <QTRSensors.h>
#include <Chassis.h>
#include "stateconstants.h"

BlueMotor motor;
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;

RemoteControl remoteControl(2);

#define BOTTOM_OUT_GRIPPER true

Servo32U4Pin5 gripperServo;
Servo32U4 jawServo;


Chassis chassis;


//QTR Sensors
QTRSensors qtr;

const uint8_t SensorCount = 2;
uint16_t sensorValues[SensorCount];


//Linear Servo declarations
int servoPin = 0;
int linearPotPin = A0;
int servoStop = 1490;  
int servoJawDown = 500;  
int servoJawUp = 2500;  
int printDelay = 500;
int linearPotVoltageADC = 500;
int jawOpenPotVoltageADC = 975;
int jawClosedPotVoltageADC = 530;
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


//motor positions
int firstSpot45deg = 3236;
int secondSpot45deg = 4476;
int firstSpot60deg = 7000;
int secondSpot60deg = 7298;

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

// Ultrasonic Sensor Variables 
const byte ultrasonicTriggerPin = 3;
const byte ultrasonicEchoPin = 30;
int frontDistance = 0;
long ultrasonicSignalDuration = 0;

// Remote Testing Variables
// IRDecoder decoder(13);

bool paused = false;

void moveUp(void);
void moveDown(void);

//Variables for FSM
int towerDistanceOne = 10;
int towerDistanceTwo = 10;
int currentState = 0;
int blockDistanceOne = 10;
int blockDistanceTwo = 10;
int PrevState = 0;

void stopIt() {
  Serial.println("ESTOP");
  Serial.println(motor.getPosition());
  motor.setEffort(0);
  gripperServo.writeMicroseconds(0);
  chassis.setMotorEfforts(0,0);
  currentState = 0;
}

void printTest() {
  Serial.print("AO: ");
  Serial.println(analogRead(A0));
  Serial.print("A2: ");
  Serial.println(analogRead(A2));
  Serial.print("A3: ");
  Serial.println(analogRead(A3));
  Serial.print("A4: ");
  Serial.println(analogRead(A4));
  // Serial.print("Current State: ");
  // Serial.println(currentState);
}

void closeServo() {
  Serial.println(analogRead(linearPotPin));
  gripperServo.writeMicroseconds(2000);
}

void openServo() {
  Serial.println(analogRead(linearPotPin));
  gripperServo.writeMicroseconds(10);
}


int ultrasonicDistance() {
  digitalWrite(ultrasonicTriggerPin, LOW); // Clear ultrasonic trigger pin
  delay(10);
  digitalWrite(ultrasonicTriggerPin, HIGH); // Outputs signal 
  delay(10);
  digitalWrite(ultrasonicTriggerPin, LOW); // Stops outputting signals
  ultrasonicSignalDuration = pulseIn(ultrasonicEchoPin, HIGH); 
  frontDistance = ultrasonicSignalDuration*0.034/2; // (time) * (speed of sound) / 2 (sending then receiving)
  Serial.print("Distance: ");
  Serial.println(frontDistance);
  return frontDistance;
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


void lineFollowing() {
  // Serial.print("CURRENTSTATE: ");
  // Serial.print(currentState);
  // Serial.print("\t");
  uint16_t position = qtr.readLineBlack(sensorValues);
  if(position > 700) {
 //   Serial.println("FAR RIGHT");
    chassis.setMotorEfforts(100,80);
  } else if (position < 300) {
 //   Serial.println("FAR LEFT");
    chassis.setMotorEfforts(80,100);
  } else {
    chassis.setMotorEfforts(100,100);
  }
  switch (currentState)
  {
  case lineFollowFourtyFiveTower:
    if (ultrasonicDistance() <= towerDistanceOne)
    {
      remoteControl.stopFunction(lineFollowingConst);
      chassis.setMotorEfforts(0,0);
      currentState = closeGripperFourtyFiveTowerUnconfirmed;
      remoteControl.startFunction(closeGripperConst);
    }
    
    break;
  
  case lineFollowSixtyTower:

  if (ultrasonicDistance() <= towerDistanceTwo)
  {
    chassis.setMotorEfforts(0,0);
    remoteControl.stopFunction(lineFollowingConst);
    currentState = closeGripperSixtyTowerUnconfirmed;
    remoteControl.startFunction(closeGripperConst);
  }
  break;
  case lineFollowBlockFourtyFive:
  if (ultrasonicDistance() <= blockDistanceOne)
  {
    remoteControl.stopFunction(lineFollowingConst);
    chassis.setMotorEfforts(0,0);
    currentState = lowerMaxFourtyFive;
    remoteControl.startFunction(fourBarLowConst);
  }
  break;
  case lineFollowBlockSixty:
    if (ultrasonicDistance() <= blockDistanceTwo)
  {
    remoteControl.stopFunction(lineFollowingConst);
    chassis.setMotorEfforts(0,0);
    currentState = lowerMaxSixty;
    remoteControl.startFunction(fourBarLowConst);
  }
  break;
  case lineFollowBlockSixtyLast:
  if (ultrasonicDistance() <= blockDistanceOne)
  {
    remoteControl.stopFunction(lineFollowingConst);
    chassis.setMotorEfforts(0,0);
    currentState = grabNewPlateSixty;
    remoteControl.startFunction(closeGripperConst);
  }
  break;
  case lineFollowBlockFourtyFiveLast:
  if (ultrasonicDistance() <= blockDistanceOne)
  {
    remoteControl.stopFunction(lineFollowingConst);
    chassis.setMotorEfforts(0,0);
    currentState = grabNewPlateFourtyFive;
    remoteControl.startFunction(closeGripperConst);
  }
  break;
  case lineFollowFourtyFiveTowerAgain:
    if (ultrasonicDistance() <= towerDistanceOne)
  {
    remoteControl.stopFunction(lineFollowingConst);
    chassis.setMotorEfforts(0,0);
    currentState = lowerSlightlyFourtyFive;
    remoteControl.startFunction(lowerSlightlyConst);
  }
  break;
  case lineFollowSixtyTowerAgain:
    if (ultrasonicDistance() <= towerDistanceTwo)
  {
    remoteControl.stopFunction(lineFollowingConst);
    chassis.setMotorEfforts(0,0);
    currentState = lowerSlightlySixty;
    remoteControl.startFunction(lowerSlightlyConst);
  }
  }
}



void moveUp() {
  motor.setEffort(400);
  Serial.println("UP");
  Serial.println(motor.getPosition());
}

void moveDown() {
  motor.setEffort(-400);
  Serial.println("DOWN");
  Serial.println(motor.getPosition());
}




// return the current RPM of the motor.
float getRPM() {
  now = millis();
  newPosition = motor.getPosition();
  speedInRPM = ((newPosition - oldPosition) / CPR) / ((float)(now - lastSampleTime) / 60000.0);
  lastSampleTime = millis();
  oldPosition = motor.getPosition();
  return speedInRPM;
}


//contains all functions that have to do with deadband for Lab 4
void deadbandFunctions(bool SerialReadings) {
 //while Button A is held, run the deadband Testing cycle. stop the motor otherwise.
  if(buttonA.isPressed()) {
    //while currentEffort is below max, print current effort + position
    if(deadBandCurrentEffort <= motorEffort) {
      if(SerialReadings) {
      Serial.print(millis());
      Serial.print(",");
      Serial.print(deadBandCurrentEffort);
      Serial.print(",");
      if(deadBandCurrentEffort < 0) {
        Serial.print(map(deadBandCurrentEffort,0,-400,-300,-400));
      } else {
        Serial.print(map(deadBandCurrentEffort,0,400,300,400));
      }
      Serial.print(",");
      Serial.println(getRPM()); 
      }
      motor.setEffortWithoutDB(deadBandCurrentEffort);
      delay(15);
      deadBandCurrentEffort++;

      //if cycle finishes, stop motor and print finish to Monitor
    } else if (deadBandCurrentEffort == motorEffort+1) {
      if(SerialReadings) {Serial.print("Finish!");}
      // motor.setEffort(0);
      deadBandCurrentEffort++;
    }

  } else {
    deadBandCurrentEffort = 0;
  }

  // when button B is pressed, move the motor backwards
  if (buttonC.isPressed()) {
    motor.setEffort(-motorEffort);
    Serial.println("TRYING");
  }

  // if neither button is being pressed, stop motor
  if (!(buttonA.isPressed() || buttonC.isPressed())) {
    motor.setEffort(0);
  }
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

//opens the bottom out gripper
void openBottomGripper() {
  if(servoActive) {
    int servoCurrentPosition = analogRead(servoEnc);
    Serial.println(servoCurrentPosition);
    gripperServo.writeMicroseconds(servoOpenedPositionMS);
    servoActive = false;
    Serial.println("OPENING");
  }
}

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
  // Serial.print("Initial linearPotVoltageADC:   ");
  // Serial.println(linearPotVoltageADC);  
    // Move Jaw Down
    jawServo.writeMicroseconds(servoJawDown);

    if (linearPotVoltageADC < jawOpenPotVoltageADC)
    {
      linearPotVoltageADC = analogRead(linearPotPin);
      if (printTimer.isExpired()){
        // Serial.print("linearPotVoltageADC:    ");
        // Serial.println(linearPotVoltageADC);
      }
    } else {
    // Stop servo onced jaw is opened
    jawServo.writeMicroseconds(servoStop);
    remoteControl.stopFunction(openGripperConst);
    remoteControl.stopFunction(remoteLeft);
    switch (currentState)
    {
    case openGripperFourtyFiveBlockConfirmed:
      currentState = backUpFourtyFive;
      remoteControl.startFunction(calebFunctionConst);
      break;
    
    case openGripperSixtyBlockConfirmed:
      currentState = backUpSixty;
      remoteControl.startFunction(calebFunctionConst);
      break;
    case openGripperFourtyFiveTowerConfirmed:
      Serial.println("Finished Fr Fr On God");
      break;
    case openGripperSixtyTowerConfirmed:
      Serial.println("Finished Fr Fr On God");
      break;
    }
    }
}

void closeGripper() {
    linearPotVoltageADC = analogRead(linearPotPin);
 // Serial.print("Initial linearPotVoltageADC:   ");
 // Serial.println(linearPotVoltageADC); 
      // // Move Jaw Up
     jawServo.writeMicroseconds(servoJawUp);

    if (linearPotVoltageADC > jawClosedPotVoltageADC)
    {    
      linearPotVoltageADC = analogRead(linearPotPin);
      
      if (printTimer.isExpired()){
   //     Serial.print("linearPotVoltageADC:     ");
  //      Serial.println(linearPotVoltageADC);
       }
    
    } else {
    // Stop servo onced jaw is vlosed
    jawServo.writeMicroseconds(servoStop);
    remoteControl.stopFunction(closeGripperConst);
    remoteControl.stopFunction(remoteRight);
    switch (currentState)
    {
    case confirmedFourtyFiveTower:    
      currentState = raiseSlightlyFourtyFiveTower;
      remoteControl.startFunction(raiseSlightlyConst);
      break;
    
    case confirmedSixtyTower:
      currentState = raiseSlightlySixtyTower;
      remoteControl.startFunction(raiseSlightlyConst);
      break;
    case grabNewPlateFourtyFive:
      currentState = backUpFourtyFiveAgain;
      remoteControl.startFunction(raiseSlightlyConst);
      break;
    case grabNewPlateSixty:
      currentState = backUpSixtyAgain;
      remoteControl.startFunction(calebFunctionConst);
      break;
    default:
    break;
    }
    }
}

// Moves motor until 4-bar linkage and gripper are positioned to grab the 60 degree plate
void takeOffHighPlate() {
  if(motor.getToggleOff()) {
    motor.moveTo(firstSpot60deg);
  } else {
    switch (currentState)
    {
    case raiseSixty:
      remoteControl.stopFunction(takeOffHighPlateConst);
      motor.setToggleOff(true);
      currentState = lineFollowSixtyTower;
      remoteControl.startFunction(lineFollowingConst);
      break;
    
    case raiseToPlaceSixty:
          remoteControl.stopFunction(takeOffHighPlateConst);
          motor.setToggleOff(true);
          currentState = raiseSlightlySixtyTowerAgain;
          remoteControl.startFunction(raiseSlightlyConst);
      break;
    }
  }
}

// Moves motor until 4-bar linkage and gripper are positioned to grab the 45 degree plate
void takeOffLowPlate() {
  if(motor.getToggleOff()) {
    motor.moveTo(firstSpot45deg);
  } else {
    remoteControl.stopFunction(takeOffLowPlateConst);
    motor.setToggleOff(true);
    switch (currentState)
    {
    case raiseFourtyFive:
      currentState = lineFollowFourtyFiveTower;
      remoteControl.startFunction(lineFollowingConst);
      break;
    
    case raiseToPlaceFourtyFive:
      currentState = raiseSlightlyFourtyFiveTowerAgain;
      remoteControl.startFunction(raiseSlightlyConst);
      break;
    }
  }
}

void raiseSlightly() {
  switch (currentState)
  {
  case raiseSlightlyFourtyFiveTower:

  //Serial.println("RAISING SLIGHTLY");
    if(motor.getToggleOff()) {
      motor.moveTo(firstSpot45deg + 300);
  } else {
    remoteControl.stopFunction(raiseSlightlyConst);
    motor.setToggleOff(true);
    currentState = CalebFunctionForty;
    remoteControl.startFunction(calebFunctionConst);    
    break;
  }

  case raiseSlightlySixtyTower:
    if(motor.getToggleOff()) {
      Serial.println("RAISING SLIGHTLY");
      motor.moveTo(firstSpot60deg - 300);
  } else {
    motor.setToggleOff(true);
    remoteControl.stopFunction(raiseSlightlyConst);
    currentState = CalebFunctionSixty;
    remoteControl.startFunction(calebFunctionConst);    
  }
  break;
  case raiseSlightlyFourtyFiveTowerAgain:
    if (motor.getToggleOff())
    {
      motor.moveTo(firstSpot45deg + 300);
    } else {
      motor.setToggleOff(true);
    remoteControl.stopFunction(raiseSlightlyConst);
    currentState = lineFollowFourtyFiveTowerAgain;
    remoteControl.startFunction(lineFollowingConst);
    break;
    }
  case raiseSlightlySixtyTowerAgain:
  if (motor.getToggleOff())
  {
    motor.moveTo(firstSpot60deg - 300);
  } else {
    motor.setToggleOff(true);
    remoteControl.stopFunction(raiseSlightlyConst);
    currentState = lineFollowSixtyTowerAgain;
    remoteControl.startFunction(lineFollowingConst);
    break;
  }
  }
  }

void lowerSlightly() {

switch (currentState)
{
case lowerSlightlyFourtyFive:
  if(motor.getToggleOff()) {
      motor.moveTo(firstSpot45deg - 300);
  } else {
    remoteControl.stopFunction(raiseSlightlyConst);
    motor.setToggleOff(true);
    currentState = placePlateFourtyFiveUnconfirmed;
  }
  break;

case lowerSlightlySixty:
if(motor.getToggleOff()) {
      motor.moveTo(firstSpot60deg + 300);
  } else {
    remoteControl.stopFunction(raiseSlightlyConst);
    motor.setToggleOff(true);
  currentState = placePlateSixtyUnconfirmed;
  break;
}
}
}

// Moves motor until 4-bar linkage and gripper are positioned to grab the lowest point
void fourBarLow() {
  if(motor.getToggleOff()) {
    motor.moveTo(0);
  } else {
    motor.setToggleOff(true);
    remoteControl.stopFunction(fourBarLowConst);
    remoteControl.stopFunction(remoteVolMinus);
    switch (currentState)
    {
    case lowerMaxFourtyFive:
      currentState = openGripperFourtyFiveBlockUnconfirmed;
      break;
    
    case lowerMaxSixty:
      currentState = openGripperSixtyBlockUnconfirmed;
      break;
    }
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



void fourtyFiveDegreeSide() {
  currentState = raiseFourtyFive;
  remoteControl.stopFunction(remote1);
  remoteControl.startFunction(takeOffLowPlateConst);
}

void sixtyDegreeSide() {
  currentState = raiseSixty;
  remoteControl.stopFunction(remote2);
  remoteControl.startFunction(takeOffHighPlateConst);
}

void updateCurrentState() {
   Serial.println(currentState);
  switch (currentState)
  {
  case closeGripperFourtyFiveTowerUnconfirmed:
    currentState = confirmedFourtyFiveTower;
   Serial.println(currentState);
    remoteControl.startFunction(closeGripperConst);
    break;
  
  case closeGripperSixtyTowerUnconfirmed:
    currentState = confirmedSixtyTower;
    Serial.println(currentState);
    remoteControl.startFunction(closeGripperConst);
    break;
  case openGripperFourtyFiveBlockUnconfirmed:
    currentState = openGripperFourtyFiveBlockConfirmed;
    remoteControl.startFunction(openGripperConst);
    break;
  case openGripperSixtyBlockUnconfirmed:
    currentState = openGripperSixtyBlockConfirmed;
    remoteControl.startFunction(openGripperConst);
    break;
  case placePlateFourtyFiveUnconfirmed:
    currentState = openGripperFourtyFiveTowerConfirmed;
    remoteControl.startFunction(openGripperConst);
    break;
  case placePlateSixtyUnconfirmed:
    currentState = openGripperSixtyTowerConfirmed;
    remoteControl.startFunction(openGripperConst);
    break;
  case waitForRefereeSixty:
  currentState = lineFollowBlockSixtyLast;
  remoteControl.startFunction(lineFollowingConst);
  break;
  case waitForRefereeForty:
  currentState = lineFollowBlockFourtyFiveLast;
  remoteControl.startFunction(lineFollowingConst);
  break;
  }
}

void calebFunction(){
  switch (currentState)
  {
  case CalebFunctionForty:
    currentState = rotateLeftUntilLine;
    remoteControl.stopFunction(calebFunctionConst);
    remoteControl.startFunction(rotateLeftUntilLineConst);
    break;
  
  case CalebFunctionSixty:
    currentState = rotateRightUntilLine;
    remoteControl.stopFunction(calebFunctionConst);
    remoteControl.startFunction(rotateRightUntilLineConst);
break;
  case backUpFourtyFive:
    currentState = waitForRefereeForty;
    remoteControl.stopFunction(calebFunctionConst);
    break;
  case backUpSixty:
    currentState = waitForRefereeSixty;
    remoteControl.stopFunction(calebFunctionConst);
    break;
  case backUpFourtyFiveAgain:
    currentState = rotateRightUntilLineAgain;
    remoteControl.stopFunction(calebFunctionConst);
    remoteControl.startFunction(rotateRightUntilLineConst);
    break;
  case backUpSixtyAgain:
    currentState = rotateLeftUntilLineAgain;
    remoteControl.stopFunction(calebFunctionConst);
    remoteControl.startFunction(rotateLeftUntilLineConst);
    break;
  }
}

void rotateRightUntilLineFunct(){

  switch (currentState)
  {
  case rotateRightUntilLine:
    currentState = lineFollowBlockSixty;
    remoteControl.stopFunction(rotateRightUntilLineConst);
    remoteControl.startFunction(lineFollowingConst);
    break;
  
  case rotateRightUntilLineAgain:
    currentState = raiseToPlaceFourtyFive;
    remoteControl.stopFunction(rotateRightUntilLineConst);
    remoteControl.startFunction(takeOffLowPlateConst);
    break;
  }
}

void rotateLeftUntilLineFunct(){
  switch (currentState)
  {
  case rotateLeftUntilLine:
    currentState = lineFollowBlockFourtyFive;
    remoteControl.stopFunction(rotateLeftUntilLineConst);
    remoteControl.startFunction(lineFollowingConst);
    break;
  
  case rotateLeftUntilLineAgain:
    currentState = raiseToPlaceSixty;
    remoteControl.stopFunction(rotateLeftUntilLineConst);
    remoteControl.startFunction(takeOffHighPlateConst);
    break;
  }
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

  remoteControl.toggleFunc(openGripper,remoteLeft);
  remoteControl.toggleFunc(closeGripper,remoteRight);

  remoteControl.toggleFunc(fourBarLow, remoteVolMinus);


  remoteControl.onPress(updateCurrentState, remoteEnterSave);
  remoteControl.onPress(printTest,remote0);

  remoteControl.eStop(stopIt,remote9);

  remoteControl.toggleFunc(fourtyFiveDegreeSide,remote1);
  remoteControl.toggleFunc(sixtyDegreeSide,remote2);

  remoteControl.onPress(resetEncoder, remote3);

  remoteControl.toggleFunc(takeOffHighPlate, takeOffHighPlateConst);
  remoteControl.toggleFunc(takeOffLowPlate, takeOffLowPlateConst);

  remoteControl.onPress(lineFollowCalibration,remote4);
  remoteControl.toggleFunc(qtrReadings,remote6);
  remoteControl.toggleFunc(lineFollowing,lineFollowingConst);

  remoteControl.toggleFunc(openGripper,openGripperConst);
  remoteControl.toggleFunc(closeGripper,closeGripperConst);

  remoteControl.toggleFunc(raiseSlightly, raiseSlightlyConst);
  remoteControl.toggleFunc(lowerSlightly, lowerSlightlyConst);

  remoteControl.toggleFunc(fourBarLow, fourBarLowConst);

  remoteControl.toggleFunc(calebFunction, calebFunctionConst);

  remoteControl.toggleFunc(rotateRightUntilLineFunct, rotateRightUntilLineConst);
  remoteControl.toggleFunc(rotateLeftUntilLineFunct, rotateLeftUntilLineConst);

  delay(4000);
  Serial.println("READY!");

  // remoteControl.runCurrentFunctions();
}


void loop()
{
  // check the battery
    // batteryCheck();

  // Code to test Remote
    // checkRemote();  

  // Reset mode for troubleshooting
    // resetMode();

  // Enters linear gripper mode to test and use linear gripper functionality
  // linearGripper();

  // run through deadBandFunctions
  // deadbandFunctions(true);

  // positional code
  // goToPositions();
  if (currentState != PrevState)
  {
    Serial.println(currentState);
    PrevState = currentState;
  }
  
  remoteControl.checkRemoteButtons();


  // motor.setEffort(200);
//
}





