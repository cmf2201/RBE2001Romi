#include <Arduino.h>
#include <Romi32U4.h>
#include "BlueMotor/BlueMotor.h"
#include <servo32u4.h>
#include "Timer.h"
#include "RemoteControl/RemoteConstants.h"
#include "RemoteControl/RemoteControl.h"
#include <QTRSensors.h>
#include <Chassis.h>
#include "stateconstants.h"
#include "mainconstants.h"
#include <HCSR04.h>

BlueMotor motor;
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;

RemoteControl remoteControl(2);

Servo32U4Pin5 gripperServo;
Servo32U4 jawServo;

Chassis chassis;

void stopIt()
{
  Serial.println("ESTOP");
  Serial.println(motor.getPosition());
  motor.setEffort(0);
  gripperServo.writeMicroseconds(0);
  chassis.setMotorEfforts(0, 0);
 // currentState = 0;
  Serial.println(currentState);
}

//call functions when unpaused
void unPause() {
  Serial.println("UNPAUSED");
  //resume forward motion if incomplete
  if(driveMotionIncomplete) {
    driveToPausableResume();
    Serial.println("RESUMEING DRIVE");
  }
  if(turnMotionIncomplete) {
    turnToPausableResume();
    Serial.println("RESUMEING TURN");
  }
}


//returns the distance in cm of the ultrasonic sensor
float ultrasonicDistance()
{
  double* distances = HCSR04.measureDistanceCm();
  return distances[0];
}

void ultrasonicDistanceTest()
{
  Serial.println(ultrasonicDistance());
}

void lineFollowCalibration()
{
  qtr.resetCalibration();
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
   // qtr.calibrationOn.maximum
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

void lineFollowing()
{
  int cutoff;
  float curDistance = ultrasonicDistance();
  //check if the target has been reached based on current state
  if( currentState == lineFollowFourtyFiveTower || currentState == lineFollowFourtyFiveTowerAgain){
    cutoff = towerDistanceOne;
  } else if (currentState == lineFollowSixtyTower || currentState == lineFollowSixtyTowerAgain) {
    cutoff = towerDistanceTwo;
  } else if (currentState == lineFollowBlockFourtyFive || currentState == lineFollowBlockFourtyFiveLast || lineFollowUntilBlockSixtyLast) {
    cutoff = blockDistanceTwo;
  } else {
    cutoff = blockDistanceOne;
  }
  bool targetReached = (curDistance <= cutoff || curDistance >= 1000);
  uint16_t position = qtr.readLineBlack(sensorValues);
  if (position > 700)
  {
    chassis.setMotorEfforts(80, 40);
  }
  else if (position < 300)
  {
    chassis.setMotorEfforts(40, 80);
  }
  else
  {
    chassis.setMotorEfforts(75, 80);
  }
  if(targetReached) {
    Serial.println("FINISHED TARGET");
    remoteControl.stopFunction(lineFollowingConst);
    chassis.idle();
    Serial.print(currentState);

    switch (currentState)
    {
      case lineFollowFourtyFiveTower:

        remoteControl.startFunction(closeGripperConst,
        currentState = closeGripperFourtyFiveTowerUnconfirmed);

        break;

      case lineFollowSixtyTower:

        remoteControl.startFunction(driveForConst);
        break;

      case lineFollowBlockFourtyFive:

        remoteControl.startFunction(fourBarLowConst,
          currentState = lowerMaxFourtyFive);
        break;

      case lineFollowBlockSixty:

        remoteControl.startFunction(fourBarLowConst,
          currentState = lowerMaxSixty);
        break;

      case lineFollowBlockSixtySecond:

        remoteControl.startFunction(closeGripperConst, 
          currentState = grabNewPlateSixty);
        break;

      case lineFollowBlockFourtyFiveLast:

        remoteControl.startFunction(closeGripperConst,
          currentState = grabNewPlateFourtyFive);
        break;

      case lineFollowFourtyFiveTowerAgain:

        remoteControl.startFunction(lowerSlightlyConst, 
          currentState = lowerSlightlyFourtyFive);
        break;
      case lineFollowSixtyTowerAgain:
        remoteControl.startFunction(driveForConst);
        break;
      case lineFollowUntilBlockFourtyLast:
        remoteControl.startFunction(turnForConst, 
        currentState = rotateRightToExit);
        break;
      case lineFollowUntilBlockSixtyLast: 
        remoteControl.startFunction(turnForConst, 
        currentState = rotateLeftToExit);
      break;

    }
  }
}


//function that will drive the robot forward a given amount
void driveFor() {
  if(!driveMotionIncomplete) {
    float distance = 0;
    float speed = 0;
    switch(currentState) {
      case lineFollowSixtyTower:
        distance = 5.0;
        speed = 5.0;
        break;

      case lineFollowSixtyTowerAgain:
        distance = 5.0;
        speed = 10.0;
        break;

      case raiseSlightlyFourtyFiveTower:
        distance = -20.0;
        speed = -10.0;
        break;

      case raiseSlightlySixtyTower:
        distance = -20.0;
        speed = -10.0;
        break;

      case backUpFourtyFive:
        distance = 10.0;
        speed = 10.0;
        break;

      case backUpSixty:
        distance = 10.0;
        speed = 10.0;
        break;
      
      case openGripperFourtyFiveTowerConfirmed:
        distance = -5.0;
        speed = -10.0;
        break;

      case openGripperSixtyTowerConfirmed:
        distance = -15.0;
        speed = -15.0;
        break;

      case goToOtherSideOfFieldFourty: 
        distance = 81;
        speed = 30;
      break; 

      case goToOtherSideOfFieldSixty: 
        distance = 81;
        speed = 30;
      break; 
      case driveBeforeFinishFourty:
        distance = 15;
        speed = 15;
      break;
      case driveBeforeFinishSixty:
        distance = 15;
        speed = 15;
      break;
    }
    driveToPausable(distance,speed);
    driveMotionIncomplete = true;
  }
  else 
  {
    if(chassis.checkMotionComplete()) {
      Serial.println("MOTION COMPLETE");

      remoteControl.stopFunction(driveForConst);
      chassis.idle();
      driveMotionIncomplete = false;

      switch(currentState) {
        case lineFollowSixtyTower:
          remoteControl.startFunction(closeGripperConst,
            currentState = closeGripperSixtyTowerUnconfirmed);
          break;

        case lineFollowSixtyTowerAgain:
          remoteControl.startFunction(lowerSlightlyConst, 
          currentState = lowerSlightlySixty);
          break;

        case raiseSlightlyFourtyFiveTower:
            remoteControl.startFunction(calebFunctionConst,
            currentState = CalebFunctionFourty);
          break;

        case raiseSlightlySixtyTower:
          remoteControl.startFunction(calebFunctionConst,
            currentState = CalebFunctionSixty);
          break;

        case backUpFourtyFive:
          currentState = waitForRefereeFourty;
          Serial.println(currentState);
          break;
        case openGripperFourtyFiveTowerConfirmed:
          remoteControl.startFunction(calebFunctionConst,
            currentState = leaveAfterPlacementFourtyTower);
          break;

        case openGripperSixtyTowerConfirmed:
          remoteControl.startFunction(calebFunctionConst,
            currentState = leaveAfterPlacementSixtyTower);
          break;
      case goToOtherSideOfFieldFourty: 
        remoteControl.startFunction(rotateUntilLineConst,
        currentState = rotateRightOppositeSideFourty);
      break;
      case goToOtherSideOfFieldSixty: 
        remoteControl.startFunction(rotateUntilLineConst,
        currentState = rotateLeftOppositeSideSixty);
      break;
      case driveBeforeFinishFourty:
        remoteControl.startFunction(rotateUntilLineConst,
          currentState = rotateUntilFinishedFourty);
      break;
      case driveBeforeFinishSixty:
        remoteControl.startFunction(rotateUntilLineConst,
          currentState = rotateUntilFinishedSixty);
      break;
      }
    }
  }
}

//function that will turn the robot forward a given amount
void turnFor() {
  if(!turnMotionIncomplete) {
    float angle = 0;
    float speed = 0;
    switch(currentState) {
      case CalebFunctionFourty:
        angle = 90.0;
        speed = 40.0;
        break;

      case CalebFunctionSixty:
        angle = -45.0;
        speed = 40.0;
        break;

      case backUpFourtyFiveAgain:
        angle = -90.0;
        speed = 40.0;
        break;

      case backUpSixtyAgain:
        angle = 45.0;
        speed = 40.0;
        break;
      
      case leaveAfterPlacementFourtyTower:
        angle = 45;
        speed = 40;
      break;

      case leaveAfterPlacementSixtyTower:
        angle = -45;
        speed = 40;
      break;

      case rotateLeftToExit: 
        angle = 90;
        speed = 30;
      break;

      case rotateRightToExit: 
        angle = -90;
        speed = 30;
      break;

    }
    turnToPausable(angle,speed);
    turnMotionIncomplete = true;
  }
  else 
  {
    if(chassis.checkMotionComplete()) {

      remoteControl.stopFunction(turnForConst);
      chassis.setMotorEfforts(0,0);
      turnMotionIncomplete = false;

      switch(currentState) {
        case CalebFunctionFourty:
          remoteControl.startFunction(rotateUntilLineConst,
            currentState = rotateLeftUntilLineState);
          break;

        case CalebFunctionSixty:
          remoteControl.startFunction(rotateUntilLineConst,
            currentState = rotateRightUntilLineState);
          break;

        case backUpFourtyFiveAgain:
          remoteControl.startFunction(rotateUntilLineConst,
            currentState = rotateRightUntilLineAgain);
          break;

        case backUpSixtyAgain:
          remoteControl.startFunction(rotateUntilLineConst,
            currentState = rotateLeftUntilLineAgain);
          break;
      case rotateRightToExit:
        remoteControl.startFunction(driveForConst,
          currentState = goToOtherSideOfFieldFourty);
      break;
      case rotateLeftToExit:
      remoteControl.startFunction(driveForConst,
          currentState = goToOtherSideOfFieldSixty);
      break;
      case leaveAfterPlacementFourtyTower:
      remoteControl.startFunction(rotateUntilLineConst,
          currentState = rotateLeftAfterPlacementFourtyTower);
        break;
      case leaveAfterPlacementSixtyTower:
      remoteControl.startFunction(rotateUntilLineConst,
          currentState = rotateRightAfterPlacementSixtyTower);
        break;
      }
    }
  }
}



void moveUp()
{
  motor.setEffort(400);
  Serial.println("UP");
  Serial.println(motor.getPosition());
}

void moveDown()
{
  motor.setEffort(-400);
  Serial.println("DOWN");
  Serial.println(motor.getPosition());
}

// return the current RPM of the motor.
float getRPM()
{
  now = millis();
  newPosition = motor.getPosition();
  speedInRPM = ((newPosition - oldPosition) / CPR) / ((float)(now - lastSampleTime) / 60000.0);
  lastSampleTime = millis();
  oldPosition = motor.getPosition();
  return speedInRPM;
}

// contains all functions that have to do with deadband for Lab 4
void deadbandFunctions(bool SerialReadings)
{
  // while Button A is held, run the deadband Testing cycle. stop the motor otherwise.
  if (buttonA.isPressed())
  {
    // while currentEffort is below max, print current effort + position
    if (deadBandCurrentEffort <= motorEffort)
    {
      if (SerialReadings)
      {
        Serial.print(millis());
        Serial.print(",");
        Serial.print(deadBandCurrentEffort);
        Serial.print(",");
        if (deadBandCurrentEffort < 0)
        {
          Serial.print(map(deadBandCurrentEffort, 0, -400, -300, -400));
        }
        else
        {
          Serial.print(map(deadBandCurrentEffort, 0, 400, 300, 400));
        }
        Serial.print(",");
        Serial.println(getRPM());
      }
      motor.setEffortWithoutDB(deadBandCurrentEffort);
      delay(15);
      deadBandCurrentEffort++;

      // if cycle finishes, stop motor and print finish to Monitor
    }
    else if (deadBandCurrentEffort == motorEffort + 1)
    {
      if (SerialReadings)
      {
        Serial.print("Finish!");
      }
      // motor.setEffort(0);
      deadBandCurrentEffort++;
    }
  }
  else
  {
    deadBandCurrentEffort = 0;
  }

  // when button B is pressed, move the motor backwards
  if (buttonC.isPressed())
  {
    motor.setEffort(-motorEffort);
    Serial.println("TRYING");
  }

  // if neither button is being pressed, stop motor
  if (!(buttonA.isPressed() || buttonC.isPressed()))
  {
    motor.setEffort(0);
  }
}

//opens the bottom out gripper
void openBottomGripper() {
  if(servoActive) {
    int servoCurrentPosition = analogRead(servoEnc);
   // Serial.println(servoCurrentPosition);
    if(servoCurrentPosition > servoOpenedPositionAR) {
      Serial.println(servoCurrentPosition);
      gripperServo.writeMicroseconds(servoOpenedPositionMS);
      Serial.println("OPENING");
      servoCurrentPosition = analogRead(servoEnc);
    } else {
      servoActive = false;
      Serial.println("FINISHED!");
      remoteControl.stopFunction(remote2);
    }
  }
}

//closes the bottom out gripper, but stops if the gripper cannot close
void closeBottomGripper() {
    int servoCurrentPosition = analogRead(servoEnc);
 //   Serial.println(servoCurrentPosition);

    // every (servoReadingDelay) ms, try to move the gripper
    if(millis() > servoPrevMS + servoReadingDelay) {
      gripperServo.writeMicroseconds(servoClosedPositionMS);

      //check if the servo is getting stuck. If so, open the Bottom Gripper.
      if(abs(servoPrevReading - servoCurrentPosition) < servoStuckTolerance) {
        //once servoCount has gotten to 15 or greater, run the bottom out gripper code instead.
        if(servoStillCount >= 15) {
          Serial.println("STUCK!");
          openBottomGripper();
          remoteControl.stopFunction(remote1);
        }
        servoStillCount++;
        
      } else {
        servoStillCount = 0;
      }

      //if Gripper has reached target position, finish movement
      if(abs(servoCurrentPosition - servoClosedPositionAR) <= servoCloseTolerance) {
        Serial.println("FINISHED!");
        servoActive = false;
        remoteControl.stopFunction(remote1);
      }

      servoPrevReading = servoCurrentPosition;
      servoPrevMS = millis();
    }
  

}

void openGripper()
{
  int servoCurrentPosition = analogRead(servoEnc);
 // Serial.println(servoCurrentPosition);
  if(servoCurrentPosition > servoOpenedPositionAR) {
  //  Serial.println(servoCurrentPosition);
    gripperServo.writeMicroseconds(servoOpenedPositionMS);
  //  Serial.println("OPENING");
  } else {
    Serial.println("FINISHED!");
    // Stop servo onced jaw is opened
    remoteControl.stopFunction(openGripperConst);
    remoteControl.stopFunction(remoteLeft);

    switch (currentState)
    {
    case openGripperFourtyFiveBlockConfirmed:
      remoteControl.startFunction(calebFunctionConst,
        currentState = backUpFourtyFive);
      break;

    case openGripperSixtyBlockConfirmed:
      remoteControl.startFunction(calebFunctionConst,
        currentState = backUpSixty);
      break;

    case openGripperFourtyFiveTowerConfirmed:
      remoteControl.startFunction(driveForConst);
      break;

    case openGripperSixtyTowerConfirmed:
      remoteControl.startFunction(driveForConst);
      break;
    }
  }
}

void closeGripper()
{
  int servoCurrentPosition = analogRead(servoEnc);
  //Serial.println(servoCurrentPosition);
  if(!(abs(servoCurrentPosition - servoClosedPositionAR) <= servoCloseTolerance)) {
    // every (servoReadingDelay) ms, try to move the gripper
    if(millis() > servoPrevMS + servoReadingDelay) {
      gripperServo.writeMicroseconds(servoClosedPositionMS);

      //check if the servo is getting stuck. If so, open the Bottom Gripper.
      if(abs(servoPrevReading - servoCurrentPosition) < servoStuckTolerance) {
        //once servoCount has gotten to 15 or greater, run the bottom out gripper code instead.
        if(servoStillCount >= 15) {
          Serial.println("STUCK!");
          openGripper();
          remoteControl.stopFunction(closeGripperConst);
          remoteControl.stopFunction(remoteRight);
        }
        servoStillCount++;
        
      } else {
        servoStillCount = 0;
      }

      //if Gripper has reached target position, finish movement
      

      servoPrevReading = servoCurrentPosition;
      servoPrevMS = millis();
    } 

  }else {
      // Stop servo onced jaw is vlosed
      Serial.println("FINISHED");
      remoteControl.stopFunction(closeGripperConst);
      remoteControl.stopFunction(remoteRight);

      switch (currentState)
      {
      case confirmedFourtyFiveTower:
          remoteControl.startFunction(raiseSlightlyConst,
          currentState = raiseSlightlyFourtyFiveTower);
        break;

      case confirmedSixtyTower:
        remoteControl.startFunction(raiseSlightlyConst,
          currentState = raiseSlightlySixtyTower);
        break;

      case grabNewPlateFourtyFive:
        remoteControl.startFunction(calebFunctionConst,
          currentState = backUpFourtyFiveAgain);
        break;

      case grabNewPlateSixty:
        remoteControl.startFunction(calebFunctionConst,
          currentState = backUpSixtyAgain);
        break;
      }
    }

}

// Moves motor until 4-bar linkage and gripper are positioned to grab the 60 degree plate
void takeOffHighPlate()
{
  if (motor.getToggleOff())
  {
    motor.moveTo(bottomOutSpot60);
  } 
  else
  { 
    motor.setToggleOff(true);
    remoteControl.stopFunction(takeOffHighPlateConst);
    chassis.driveFor(0,0);

    switch (currentState)
    {
    case raiseSixty:
      remoteControl.startFunction(lineFollowingConst,
        currentState = lineFollowSixtyTower);
      break;

    case raiseToPlaceSixty:
      remoteControl.startFunction(raiseSlightlyConst,
        currentState = raiseSlightlySixtyTowerAgain);
      break;
    }
  }
}

// Moves motor until 4-bar linkage and gripper are positioned to grab the 45 degree plate
void takeOffLowPlate()
{
  if (motor.getToggleOff())
  {
    motor.moveTo(bottomOutSpot45);
  }
  else
  {
    remoteControl.stopFunction(takeOffLowPlateConst);
    motor.setToggleOff(true);

    switch (currentState)
    {
    case raiseFourtyFive:
      remoteControl.startFunction(lineFollowingConst,
        currentState = lineFollowFourtyFiveTower);
      break;

    case raiseToPlaceFourtyFive:
      remoteControl.startFunction(raiseSlightlyConst, 
        currentState = raiseSlightlyFourtyFiveTowerAgain);
      break;
    }
  }
}

//raises the motor slightly
void raiseSlightly()
{
  //decide what position to move to
  if (motor.getToggleOff()){
    int moveToPos;
    switch(currentState) {
      case (raiseSlightlyFourtyFiveTower):
        moveToPos = bottomOutSpot45 + 800;
        break;

      case (raiseSlightlyFourtyFiveTowerAgain):
        moveToPos = bottomOutSpot45 + 800;
        break;
      
      case (raiseSlightlySixtyTower):
        moveToPos = bottomOutSpot60 - 1200;
        break;

      case (raiseSlightlySixtyTowerAgain):
        moveToPos = bottomOutSpot60 - 1200;
        break;
    }
    motor.moveTo(moveToPos);
  }
  else
  {
    motor.setToggleOff(true);
    remoteControl.stopFunction(raiseSlightlyConst);

    switch (currentState)
    {
      case raiseSlightlyFourtyFiveTower:
        remoteControl.startFunction(driveForConst);
        break;

      case raiseSlightlySixtyTower:
        remoteControl.startFunction(driveForConst);
        break;

      case raiseSlightlyFourtyFiveTowerAgain:
        remoteControl.startFunction(lineFollowingConst,
          currentState = lineFollowFourtyFiveTowerAgain);
        break;

      case raiseSlightlySixtyTowerAgain:
        remoteControl.startFunction(lineFollowingConst,
          currentState = lineFollowSixtyTowerAgain);
        break;
    }
  }  
}


void lowerSlightly()
{
  if (motor.getToggleOff())
  {
    int moveToPos;
    switch(currentState) {
      case (lowerSlightlyFourtyFive):
        moveToPos = bottomOutSpot45 - 100;
        break;

      case (lowerSlightlySixty):
        moveToPos = bottomOutSpot60 + 250;
        break;
    }
    motor.moveTo(moveToPos);
  } 
  else 
  {
    motor.setToggleOff(true);
    remoteControl.stopFunction(lowerSlightlyConst);

    switch (currentState)
    {
      case lowerSlightlyFourtyFive:
        currentState = placePlateFourtyFiveUnconfirmed;
        Serial.println(currentState);
        break;

      case lowerSlightlySixty:
        currentState = placePlateSixtyUnconfirmed;
        Serial.println(currentState);
        break;
    }
  }
}

void goToBottomEncoder() 
{
  if (motor.getToggleOff())
  {
    motor.moveTo(0);
  }
  else
  {
    motor.setToggleOff(true);
    remoteControl.stopFunction(remoteVolMinus);
  }
}


// Moves motor until 4-bar linkage and gripper are positioned to grab the lowest point
void fourBarLow()
{
  if (motor.getToggleOff())
  {
    motor.moveTo(400);
  }
  else
  {
    motor.setToggleOff(true);
    remoteControl.stopFunction(fourBarLowConst);
    switch (currentState)
    {
    case lowerMaxFourtyFive:
      currentState = openGripperFourtyFiveBlockUnconfirmed;
      Serial.println(currentState);
      break;

    case lowerMaxSixty:
      currentState = openGripperSixtyBlockUnconfirmed;
      Serial.println(currentState);
      break;
    }
  }
}

void resetEncoder()
{
  motor.reset();
  Serial.println("Motor reset :)");
}

void fourtyFiveDegreeSide()
{
  remoteControl.stopFunction(remote1);
  remoteControl.startFunction(takeOffLowPlateConst,
    currentState = raiseFourtyFive);
}

void sixtyDegreeSide()
{
  remoteControl.stopFunction(remote2);
  remoteControl.startFunction(takeOffHighPlateConst,
    currentState = raiseSixty);
}

//update the current state when a button is pressed so that the bot can wait for user input
void updateCurrentState()
{
  switch (currentState)
  {
  case closeGripperFourtyFiveTowerUnconfirmed:
    remoteControl.startFunction(closeGripperConst,
      currentState = confirmedFourtyFiveTower);
    break;

  case closeGripperSixtyTowerUnconfirmed:
    remoteControl.startFunction(closeGripperConst,
      currentState = confirmedSixtyTower);
    break;

  case openGripperFourtyFiveBlockUnconfirmed:
    remoteControl.startFunction(openGripperConst,
      currentState = openGripperFourtyFiveBlockConfirmed);
    break;

  case openGripperSixtyBlockUnconfirmed:
    remoteControl.startFunction(openGripperConst,
      currentState = openGripperSixtyBlockConfirmed);
    break;
  
  case placePlateFourtyFiveUnconfirmed:
    remoteControl.startFunction(openGripperConst,
      currentState = openGripperFourtyFiveTowerConfirmed);
    break;

  case placePlateSixtyUnconfirmed:
    remoteControl.startFunction(openGripperConst,
      currentState = openGripperSixtyTowerConfirmed);
    break;
  
  case waitForRefereeSixty:
      remoteControl.startFunction(lineFollowingConst,
      currentState = lineFollowBlockSixtySecond);
    break;

  case waitForRefereeFourty:
    remoteControl.startFunction(lineFollowingConst,
      currentState = lineFollowBlockFourtyFiveLast);
    break;
  }
}

void calebFunction()
{
  float distanceUlt = ultrasonicDistance();
  Serial.print("DISTANCE:   ");
  Serial.println(distanceUlt);
  bool targetReached;
  if(currentState == backUpFourtyFive || currentState == backUpSixty)
  {
    targetReached = !(distanceUlt < blockBackUpDist || distanceUlt > 100);
  } 
  else
  {
    targetReached = !(distanceUlt < wallToLineDist || distanceUlt > 100);
  }
  if(!targetReached) {
    chassis.setWheelSpeeds(-5.00, -5.00);
  }
  else
  {
    chassis.idle();
    remoteControl.stopFunction(calebFunctionConst);

    switch (currentState)
    {
      case CalebFunctionFourty:
        remoteControl.startFunction(turnForConst);
        break;

      case CalebFunctionSixty:
        remoteControl.startFunction(turnForConst);
        break;

      case backUpFourtyFive:
        remoteControl.startFunction(driveForConst);
        break;

      case backUpSixty:
          currentState = waitForRefereeSixty;
          Serial.println(currentState);
        //remoteControl.startFunction(driveForConst);
        break;

      case backUpFourtyFiveAgain:
        remoteControl.startFunction(turnForConst);
        break;

      case backUpSixtyAgain:
        remoteControl.startFunction(turnForConst);
        break;

      case leaveAfterPlacementFourtyTower:
            remoteControl.startFunction(turnForConst);
        break;

      case leaveAfterPlacementSixtyTower:
        remoteControl.startFunction(turnForConst);
        break;
    }
  }
}

//rotate the bot a given amount until it reaches a line
void rotateUntilLine() {
  qtr.read(sensorValues);
  Serial.print("FIRST:");
  Serial.print(sensorValues[0]);
  Serial.print(" SECOND:");
  Serial.println(sensorValues[1]);

  if(!(lineFoundThresholdHigh < sensorValues[0] || lineFoundThresholdHigh < sensorValues[1])) {
    if( currentState ==  rotateRightUntilLineState|| currentState == rotateRightUntilLineAgain || currentState == rotateRightOppositeSideFourty || currentState == rotateUntilFinishedFourty || currentState == rotateRightAfterPlacementSixtyTower) {
      chassis.setMotorEfforts(100, -50);
    } else if( currentState == rotateLeftUntilLineState || currentState == rotateLeftUntilLineAgain || currentState == rotateLeftOppositeSideSixty || currentState == rotateUntilFinishedSixty || currentState == rotateLeftAfterPlacementFourtyTower) {
      chassis.setMotorEfforts(-50, 100);
    }
  } 
  else 
  {
    remoteControl.stopFunction(rotateUntilLineConst);
    chassis.idle();

    switch (currentState) {
      case rotateRightUntilLineState:
        remoteControl.startFunction(lineFollowingConst,
          currentState = lineFollowBlockSixty);
        break;

      case rotateRightUntilLineAgain:
        remoteControl.startFunction(takeOffLowPlateConst,
          currentState = raiseToPlaceFourtyFive);
        break;

      case rotateLeftUntilLineState:
        remoteControl.startFunction(lineFollowingConst,
          currentState = lineFollowBlockFourtyFive);
        break;

      case rotateLeftUntilLineAgain:
        remoteControl.startFunction(takeOffHighPlateConst,
          currentState = raiseToPlaceSixty);
        break;
      case rotateLeftAfterPlacementFourtyTower:
        remoteControl.startFunction(lineFollowingConst,
          currentState = lineFollowUntilBlockFourtyLast);
      break;
      case rotateRightAfterPlacementSixtyTower:
        remoteControl.startFunction(lineFollowingConst,
          currentState = lineFollowUntilBlockSixtyLast);
      break;
      case rotateRightOppositeSideFourty:
        remoteControl.startFunction(driveForConst, 
          currentState = driveBeforeFinishFourty);
      break;
      case rotateLeftOppositeSideSixty:
        remoteControl.startFunction(driveForConst,
          currentState = driveBeforeFinishSixty);
      break;
      case rotateUntilFinishedFourty: 
        chassis.idle();
        currentState = 0;
        Serial.println("You're done! :) <3 ");
      break; 
      case rotateUntilFinishedSixty: 
        chassis.idle();
        currentState = 0;
        Serial.println("You're done! :) <3 ");
      break; 
    }
  }
}

// void forwardUntilLine() {
//   if(!(lineFoundThresholdHigh < sensorValues[0] || lineFoundThresholdHigh < sensorValues[1])) {

//   }
// }

void returnMotorPosition()
{
  Serial.println(motor.getPosition());
}

void turnToPausableResume() {
  //calculates how far the bot has already gone
  float deltaAngle = (chassis.getRightEncoderCount() - startPositionRight) / ((chassis.robotRadius * 3.14 / 180.0) / chassis.cmPerEncoderTick);
  float angleLeft = currentTarget - deltaAngle;
  
  // Serial.print("DELTA LEFT: ");    
  // Serial.println(angleLeft);

  //set the bot to drive the remainder
  chassis.turnFor(angleLeft,currentSpeed);
}

void turnToPausableResume(float target, float speed) {
  currentTarget = target;
  currentSpeed = speed;
  turnToPausableResume();
}

//used to reset the pausable turnFor
void turnToPausable(float target, float speed) {
  startPositionLeft = chassis.getLeftEncoderCount();
  startPositionRight = chassis.getRightEncoderCount();

  turnToPausableResume(target,speed);
}

void driveToPausableResume() {
  //call once
  float newDeltaL = (currentTarget/chassis.cmPerEncoderTick - (chassis.getLeftEncoderCount() - startPositionLeft)) * chassis.cmPerEncoderTick;
  float newDeltaR = (currentTarget/chassis.cmPerEncoderTick - (chassis.getRightEncoderCount() - startPositionRight)) * chassis.cmPerEncoderTick;
  float newDelta = (newDeltaL + newDeltaR)/2;

  // Serial.print("DELTA: ");    
  // Serial.println(newDelta);

  //continously call
  chassis.driveFor(newDelta,currentSpeed);
}

// driveFor being pausable
void driveToPausableResume(float target, float speed) {
  currentTarget = target;
  currentSpeed = speed;
  driveToPausableResume();
}

//used to reset the pausable driveFor
void driveToPausable(float target, float speed) {
  // calculate the total motion in encoder ticks
  startPositionLeft = chassis.getLeftEncoderCount();
  startPositionRight = chassis.getRightEncoderCount();

  driveToPausableResume(target,speed);
}

void setup()
{
  // Start the serial monitor
  Serial.begin(9600);

  motor.setup();
  motor.reset();

  chassis.init();

  gripperServo.setMinMaxMicroseconds(0, 2000);
  gripperServo.writeMicroseconds(0);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A4, A3}, SensorCount);

  //ultrasonic setup
  HCSR04.begin(ultrasonicTriggerPin, ultrasonicEchoPin);

  remoteControl.setup();

  remoteControl.onPress(moveUp, remoteUp);
  remoteControl.onPress(moveDown, remoteDown);

  remoteControl.toggleFunc(openGripper, remoteLeft);
  remoteControl.toggleFunc(closeGripper, remoteRight);

  // remoteControl.toggleFunc(fourBarLow, remoteVolMinus);
  remoteControl.onPress(stopIt, remoteVolPlus);

  remoteControl.onPress(updateCurrentState, remoteEnterSave);

  remoteControl.onPress(ultrasonicDistanceTest, remote0);
  remoteControl.onPress(returnMotorPosition, remoteBack);

 // remoteControl.toggleFunc(fourtyFiveDegreeSide, remote1);
  remoteControl.toggleFunc(sixtyDegreeSide, remote2);
  remoteControl.onPress(resetEncoder, remote3);

  remoteControl.onPress(lineFollowCalibration, remote4);
  remoteControl.toggleFunc(lineFollowing, remote5);

  remoteControl.eStop(stopIt, remote9);

  remoteControl.toggleFunc(lineFollowing, lineFollowingConst);

  remoteControl.toggleFunc(takeOffHighPlate, takeOffHighPlateConst);
  remoteControl.toggleFunc(takeOffLowPlate, takeOffLowPlateConst);

  remoteControl.toggleFunc(openGripper, openGripperConst);
  remoteControl.toggleFunc(closeGripper, closeGripperConst);

  remoteControl.toggleFunc(raiseSlightly, raiseSlightlyConst);
  remoteControl.toggleFunc(lowerSlightly, lowerSlightlyConst);

  remoteControl.toggleFunc(fourBarLow, fourBarLowConst);

  remoteControl.toggleFunc(calebFunction, calebFunctionConst);

  remoteControl.toggleFunc(rotateUntilLine, rotateUntilLineConst);

  remoteControl.toggleFunc(goToBottomEncoder, remoteVolMinus);

  remoteControl.toggleFunc(driveFor,driveForConst);
  remoteControl.toggleFunc(turnFor,turnForConst);

  remoteControl.ePause(stopIt,unPause,remote8);

  delay(4000);
  Serial.println("READY!");
}

void loop()
{
  // Code to test Remote
  // checkRemote();

  //  Make sure the Battery Voltage is correct
  //  if (readBatteryMillivolts() <= 7000)
  //  {
  //    Serial.print("BATTERY LOW: ");
  //    Serial.println(readBatteryMillivolts());
  //    buttonC.waitForButton();
  //  }

 
  remoteControl.checkRemoteButtons();
}
