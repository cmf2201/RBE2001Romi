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

// stop the bot
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

float ultrasonicDistance()
{
  double *distances = HCSR04.measureDistanceCm();
  return distances[0];
}

void ultrasonicDistanceTest()
{
  Serial.println(ultrasonicDistance());
}

void lineFollowCalibration()
{
  Serial.println("RESETING LINEFOLLOW...");
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
  // check if the target has been reached based on current state
  if (currentState == lineFollowFourtyFiveTower || currentState == lineFollowFourtyFiveTowerAgain)
  {
    cutoff = towerDistanceOne;
  }
  else if (currentState == lineFollowSixtyTower || currentState == lineFollowSixtyTowerAgain)
  {
    cutoff = towerDistanceTwo;
  }
  else if (currentState == lineFollowBlockFourtyFive || currentState == lineFollowBlockFourtyFiveLast)
  {
    cutoff = blockDistanceTwo;
  }
  else if (currentState == lineFollowBlockSixty || currentState == lineFollowBlockSixtyLast)
  {
    cutoff = blockDistanceOne;
  }
  bool targetReached = (curDistance <= cutoff || curDistance >= 1000);
  uint16_t position = qtr.readLineBlack(sensorValues);
  if (position > 700)
  {
    chassis.setMotorEfforts(60, 30);
  }
  else if (position < 300)
  {
    chassis.setMotorEfforts(30, 60);
  }
  else
  {
    chassis.setMotorEfforts(60, 60);
  }
  if (targetReached)
  {
    remoteControl.stopFunction(lineFollowingConst);
    chassis.setMotorEfforts(0, 0);

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
      chassis.setMotorEfforts(0, 0);

      remoteControl.startFunction(fourBarLowConst,
                                  currentState = lowerMaxFourtyFive);
      break;

    case lineFollowBlockSixty:
      chassis.setMotorEfforts(0, 0);

      remoteControl.startFunction(fourBarLowConst,
                                  currentState = lowerMaxSixty);
      break;

    case lineFollowBlockSixtyLast:
      chassis.setMotorEfforts(0, 0);

      remoteControl.startFunction(closeGripperConst,
                                  currentState = grabNewPlateSixty);
      break;

    case lineFollowBlockFourtyFiveLast:
      chassis.setMotorEfforts(0, 0);

      remoteControl.startFunction(closeGripperConst,
                                  currentState = grabNewPlateFourtyFive);
      break;

    case lineFollowFourtyFiveTowerAgain:
      chassis.setMotorEfforts(0, 0);

      remoteControl.startFunction(lowerSlightlyConst,
                                  currentState = lowerSlightlyFourtyFive);
      break;

    case lineFollowSixtyTowerAgain:

      remoteControl.startFunction(driveForConst);
      break;
    }
  }
}

// function that will drive the robot forward a given amount
void driveFor()
{
  if (!driveMotionIncomplete)
  {
    float distance = 0;
    float speed = 0;
    switch (currentState)
    {
    case lineFollowSixtyTower:
      distance = 10.0;
      speed = 5.0;
      break;

    case lineFollowSixtyTowerAgain:
      distance = 10.0;
      speed = 5.0;
      break;

    case raiseSlightlyFourtyFiveTower:
      distance = -20.0;
      speed = -5.0;
      break;

    case raiseSlightlySixtyTower:
      distance = -20.0;
      speed = -5.0;
      break;

    case backUpFourtyFive:
      distance = 10.0;
      speed = 5.0;
      break;

    case backUpSixty:
      distance = 10.0;
      speed = 5.0;
      break;

    case openGripperFourtyFiveTowerConfirmed:
      distance = -5;
      speed = -5;
      break;

    case openGripperSixtyTowerConfirmed:
      distance = -15;
      speed = -15;
      break;
    }
    driveToPausable(distance, speed);
    driveMotionIncomplete = true;
  }
  else
  {
    if (chassis.checkMotionComplete())
    {

      remoteControl.stopFunction(driveForConst);
      chassis.setMotorEfforts(0, 0);
      driveMotionIncomplete = false;
      Serial.println(currentState);

      switch (currentState)
      {
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
                                    currentState = CalebFunctionForty);
        break;

      case raiseSlightlySixtyTower:
        remoteControl.startFunction(calebFunctionConst,
                                    currentState = CalebFunctionSixty);
        break;

      case backUpFourtyFive:
        currentState = waitForRefereeForty;
        Serial.println(currentState);
        break;

      case backUpSixty:
        currentState = waitForRefereeSixty;
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
        
      }
    }
  }
}

// function that will turn the robot forward a given amount
void turnFor()
{
  if (!turnMotionIncomplete)
  {
    float angle = 0;
    float speed = 0;
    switch (currentState)
    {
    case CalebFunctionForty:
      angle = 90.0;
      speed = 30.0;
      break;

    case CalebFunctionSixty:
      angle = -20.0;
      speed = 10.0;
      break;

    case backUpFourtyFiveAgain:
      angle = -90.0;
      speed = 10.0;
      break;

    case backUpSixtyAgain:
      angle = 30.0;
      speed = 30.0;
      break;

    case leaveAfterPlacementFourtyTower:
      angle = 90.0;
      speed = 30.0;
      break;

    case replaceStepOneFourty:
      angle = 90.0;
      speed = 30.0;
    }
    
    turnToPausable(angle, speed);
    turnMotionIncomplete = true;
  }
  else
  {
    if (chassis.checkMotionComplete())
    {

      remoteControl.stopFunction(turnForConst);
      chassis.setMotorEfforts(0, 0);
      turnMotionIncomplete = false;

      switch (currentState)
      {
      case CalebFunctionForty:
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

      case replaceStepOneFourty:
        remoteControl.startFunction(lineFollowingConst);
        Serial.println("Replace me with a new robot plz master");
        break;
      }
    }
  }
}

void linefollowTesting()
{
  // Serial.print("CURRENTSTATE: ");
  // Serial.print(currentState);
  // Serial.print("\t");
  uint16_t position = qtr.readLineBlack(sensorValues);
  if (position > 700)
  {
    //   Serial.println("FAR RIGHT");
    chassis.setMotorEfforts(80, 64);
  }
  else if (position < 300)
  {
    //   Serial.println("FAR LEFT");
    chassis.setMotorEfforts(64, 80);
  }
  else
  {
    chassis.setMotorEfforts(80, 80);
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

// opens the bottom out gripper
void openBottomGripper()
{
  if (servoActive)
  {
    int servoCurrentPosition = analogRead(servoEnc);
    // Serial.println(servoCurrentPosition);
    if (servoCurrentPosition > servoOpenedPositionAR)
    {
      Serial.println(servoCurrentPosition);
      gripperServo.writeMicroseconds(servoOpenedPositionMS);
      Serial.println("OPENING");
      servoCurrentPosition = analogRead(servoEnc);
    }
    else
    {
      servoActive = false;
      Serial.println("FINISHED!");
      remoteControl.stopFunction(remote2);
    }
  }
}

// function opens the Gripper
void openGripper()
{
  int servoCurrentPosition = analogRead(servoEnc);
  // Serial.println(servoCurrentPosition);
  if (servoCurrentPosition < servoOpenedPositionAR)
  {
    gripperServo.writeMicroseconds(servoOpenedPositionMS);
  }
  else
  {
    Serial.println("FINISHED!");
    // Stop servo onced jaw is opened
    gripperServo.writeMicroseconds(0);
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

// closes gripper
void closeGripper()
{
  int servoCurrentPosition = analogRead(servoEnc);
  // Serial.println(servoCurrentPosition);

  if (servoCurrentPosition > servoClosedPositionAR)
  {
    gripperServo.writeMicroseconds(servoClosedPositionMS);
  }
  else
  {
    // Stop servo onced jaw is closed
    gripperServo.writeMicroseconds(0);
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

void turnToPausableResume()
{
  // calculates how far the bot has already gone
  float deltaAngle = (chassis.getRightEncoderCount() - startPositionRight) / ((chassis.robotRadius * 3.14 / 180.0) / chassis.cmPerEncoderTick);
  float angleLeft = currentTarget - deltaAngle;

  // Serial.print("DELTA LEFT: ");
  // Serial.println(angleLeft);

  // set the bot to drive the remainder
  chassis.turnFor(angleLeft, currentSpeed);
}

void turnToPausableResume(float target, float speed)
{
  currentTarget = target;
  currentSpeed = speed;
  turnToPausableResume();
}

// used to reset the pausable turnFor
void turnToPausable(float target, float speed)
{
  startPositionLeft = chassis.getLeftEncoderCount();
  startPositionRight = chassis.getRightEncoderCount();

  turnToPausableResume(target, speed);
}

void driveToPausableResume()
{
  // call once
  float newDeltaL = (currentTarget / chassis.cmPerEncoderTick - (chassis.getLeftEncoderCount() - startPositionLeft)) * chassis.cmPerEncoderTick;
  float newDeltaR = (currentTarget / chassis.cmPerEncoderTick - (chassis.getRightEncoderCount() - startPositionRight)) * chassis.cmPerEncoderTick;
  float newDelta = (newDeltaL + newDeltaR) / 2;

  // Serial.print("DELTA: ");
  // Serial.println(newDelta);

  // continously call
  chassis.driveFor(newDelta, currentSpeed);
}

// driveFor being pausable
void driveToPausableResume(float target, float speed)
{
  currentTarget = target;
  currentSpeed = speed;
  driveToPausableResume();
}

// used to reset the pausable driveFor
void driveToPausable(float target, float speed)
{
  // calculate the total motion in encoder ticks
  startPositionLeft = chassis.getLeftEncoderCount();
  startPositionRight = chassis.getRightEncoderCount();

  driveToPausableResume(target, speed);
}

// Moves motor until 4-bar linkage and gripper are positioned to grab the 60 degree plate
void takeOffHighPlate()
{
  if (motor.getToggleOff())
  {
    motor.moveTo(firstSpot60deg);
  }
  else
  {
    motor.setToggleOff(true);
    remoteControl.stopFunction(takeOffHighPlateConst);
    chassis.driveFor(0, 0);

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
    motor.moveTo(firstSpot45deg);
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

// raises the motor slightly
void raiseSlightly()
{
  // decide what position to move to
  if (motor.getToggleOff())
  {
    int moveToPos;
    switch (currentState)
    {
    case (raiseSlightlyFourtyFiveTower):
      moveToPos = firstSpot45deg + 800;
      break;

    case (raiseSlightlyFourtyFiveTowerAgain):
      moveToPos = firstSpot45deg + 800;
      break;

    case (raiseSlightlySixtyTower):
      moveToPos = firstSpot60deg - 1200;
      break;

    case (raiseSlightlySixtyTowerAgain):
      moveToPos = firstSpot60deg - 1200;
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
  if(motor.getToggleOff())
  {
    int moveToPosition;
    switch(currentState) {
      case lowerSlightlyFourtyFive:
        moveToPosition = firstSpot45deg - 100;
        break;

      case lowerSlightlySixty:
        moveToPosition = firstSpot60deg + 300;
        break;
    }
    motor.moveTo(moveToPosition);
  }
  else
  {
    remoteControl.stopFunction(raiseSlightlyConst);
    motor.setToggleOff(true);

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
    motor.moveTo(500);
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
  Serial.println("encoder reset >:)");
  motor.reset();
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

// update the current state when a button is pressed so that the bot can wait for user input
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
                                currentState = lineFollowBlockSixtyLast);
    break;

  case waitForRefereeForty:
    remoteControl.startFunction(lineFollowingConst,
                                currentState = lineFollowBlockFourtyFiveLast);
    break;
  }
}

// backs up using the ultrasonic to a particular distance
void calebFunction()
{
  float distanceUlt = ultrasonicDistance();
  Serial.print("DISTANCE:   ");
  Serial.println(distanceUlt);
  bool targetReached;
  if (currentState == backUpFourtyFive || currentState == backUpSixty)
  {
    targetReached = !(distanceUlt < blockBackUpDist || distanceUlt > 100);
  }
  else
  {
    targetReached = !(distanceUlt < wallToLineDist || distanceUlt > 100);
  }
  if (!targetReached)
  {
    chassis.setWheelSpeeds(-5.00, -5.00);
  }
  else
  {
    chassis.idle();
    remoteControl.stopFunction(calebFunctionConst);

    switch (currentState)
    {
    case CalebFunctionForty:
      remoteControl.startFunction(turnForConst);
      break;

    case CalebFunctionSixty:
      remoteControl.startFunction(turnForConst);
      break;

    case backUpFourtyFive:
      remoteControl.startFunction(driveForConst);
      break;

    case backUpSixty:
      remoteControl.startFunction(driveForConst);
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
      remoteControl.startFunction(rotateRightUntilLineConst,
                                  currentState = rotateRightAfterPlacementSixtyTower);
      break;
    }
  }
}

// rotate the bot a given amount until it reaches a line
void rotateUntilLine()
{
  qtr.read(sensorValues);
  if (!(lineFoundThresholdHigh > sensorValues[0] || lineFoundThresholdHigh > sensorValues[1]))
  {
    if (currentState == rotateRightUntilLineConst)
    {
      chassis.setMotorEfforts(50, -35);
    }
    else if (currentState == rotateLeftUntilLineConst || currentState == rotateLeftUntilLineAgain)
    {
      chassis.setMotorEfforts(-35, 50);
    }
  }
  else
  {
    remoteControl.stopFunction(rotateUntilLineConst);
    chassis.idle();

    switch (currentState)
    {
    case rotateRightUntilLineConst:
      remoteControl.startFunction(lineFollowingConst,
                                  currentState = lineFollowBlockSixty);
      break;

    case rotateRightUntilLineAgain:
      remoteControl.startFunction(takeOffLowPlateConst,
                                  currentState = raiseToPlaceFourtyFive);
      break;

    case rotateLeftUntilLineConst:
      remoteControl.startFunction(lineFollowingConst,
                                  currentState = lineFollowBlockFourtyFive);
      break;

    case rotateLeftUntilLineAgain:
      remoteControl.startFunction(takeOffHighPlateConst,
                                  currentState = raiseToPlaceSixty);
      break;
    }
  }
}

// void rotateRightUntilLine()
// {
//   if (chassis.checkMotionComplete())
//   {

//   switch (currentState)
//   {
//   case rotateRightUntilLineConst:
//       chassis.setMotorEfforts(50, -35);
//       if (qtr.readLineBlack(sensorValues) > 100 && qtr.readLineBlack(sensorValues) < 900)
//       {
//         Serial.println(qtr.readLineBlack(sensorValues));
//         currentState = lineFollowBlockSixty;
//         Serial.println(currentState);
//         remoteControl.stopFunction(rotateRightUntilLineConst);
//         remoteControl.startFunction(lineFollowingConst);
//       }
//     break;

//   case rotateRightUntilLineAgain:
//   chassis.setMotorEfforts(50, -35);
//       if (qtr.readLineBlack(sensorValues) > 100 && qtr.readLineBlack(sensorValues) < 900)
//       {
//         Serial.println(qtr.readLineBlack(sensorValues));
//     currentState = raiseToPlaceFourtyFive;
//     Serial.println(currentState);
//     remoteControl.stopFunction(rotateRightUntilLineConst);
//     remoteControl.startFunction(takeOffLowPlateConst);
//       }
//     break;

//     /// !!!!!!!! THIS SHOULD BE MOVED TO THE ROTATE FUNCTION? OR NEEDS TO FIXED CAUSE DIDN'T WORK? !!!!!!!!!
//   case rotateRightAfterPlacementSixtyTower:

//     remoteControl.stopFunction(rotateRightUntilLineConst);
//     currentState = forwardUntilTurnToLeaveSixty;
//     Serial.println(currentState);
//     remoteControl.startFunction(forwardFunctConst);
//     break;
//   case rotateUntilLineAndReplaceSixty:
//     remoteControl.stopFunction(rotateRightUntilLineConst);
//     Serial.println("Replace Me with a New Robot Plz Master");
//     break;
//   }
//   }
// }

// void rotateLeftUntilLine()
// {
//   Serial.println(qtr.readLineBlack(sensorValues));
//   if (chassis.checkMotionComplete())
//   {
//   switch (currentState)
//   {
//   case rotateLeftUntilLineConst:
//   chassis.setMotorEfforts(-35, 50);
//   if (qtr.readLineBlack(sensorValues) > 100 && qtr.readLineBlack(sensorValues) < 900)
//       {
//     currentState = lineFollowBlockFourtyFive;
//     Serial.println(currentState);
//     remoteControl.stopFunction(rotateLeftUntilLineConst);
//     remoteControl.startFunction(lineFollowingConst);
//       }
//     break;

//   case rotateLeftUntilLineAgain:
//     chassis.setMotorEfforts(-35, 50);
//   if (qtr.readLineBlack(sensorValues) > 100 && qtr.readLineBlack(sensorValues) < 900)
//       {
//     Serial.println(qtr.readLineBlack(sensorValues));
//     chassis.setMotorEfforts(0, 0);
//     currentState = raiseToPlaceSixty;
//     Serial.println(currentState);
//     remoteControl.stopFunction(rotateLeftUntilLineConst);
//     remoteControl.startFunction(takeOffHighPlateConst);
//       }
//     break;
//   case rotateLeftAfterPlacementFourtyTower:
//     remoteControl.stopFunction(rotateLeftUntilLineConst);
//     currentState = forwardUntilTurnToLeaveFourty;
//     Serial.println(currentState);
//     remoteControl.startFunction(forwardFunctConst);
//     break;
//   case rotateUntilLineAndReplaceFourty:
//     remoteControl.stopFunction(rotateLeftUntilLineConst);
//     Serial.println("Replace Me with a New Robot Plz Master");
//     break;
//   }
//   }
// }

void forwardUntilLine()
{

  qtr.read(sensorValues);
  if (!(lineFoundThresholdHigh > sensorValues[0] || lineFoundThresholdHigh > sensorValues[1]))
  {
    if (currentState == forwardUntilLineFourty)
    {
      chassis.setMotorEfforts(50, 50);
    }
    else if (currentState == rotateLeftUntilLineConst || currentState == rotateLeftUntilLineAgain)
    {
      chassis.setMotorEfforts(-35, 50);
    }
  }
  else
  {
    remoteControl.stopFunction(forwardUntilLineConst);
    chassis.idle();

    switch (currentState)
    {
      case forwardUntilLineFourty:
        remoteControl.startFunction(turnForConst,
          currentState = replaceStepOneFourty);
        break;
    }
  }
}

void returnMotorPosition()
{
  Serial.println(motor.getPosition());
}

void driveToPausableTest()
{
  remoteControl.stopFunction(remote3);
  remoteControl.startFunction(driveToPausableTest2Const);
}

void driveToPausableTest2()
{
  if (!driveMotionIncomplete)
  {
    driveToPausable(20.0, 5.0);
    Serial.println("DRIVING");
    driveMotionIncomplete = true;
  }
  else
  {
    if (chassis.checkMotionComplete())
    {
      Serial.println("COMPLETE");
      driveMotionIncomplete = false;
      remoteControl.stopFunction(driveToPausableTest2Const);
    }
  }
}

void turnToPausableTest()
{
  remoteControl.stopFunction(remote3);
  remoteControl.startFunction(turnToPausableTest2Const);
}

void turnToPausableTest2()
{
  if (!turnMotionIncomplete)
  {
    turnToPausable(-90.0, -15.0);
    Serial.println("TURNING");
    turnMotionIncomplete = true;
  }
  else
  {
    if (chassis.checkMotionComplete())
    {
      Serial.println("COMPLETE");
      turnMotionIncomplete = false;
      remoteControl.stopFunction(turnToPausableTest2Const);
    }
  }
}

// call functions when unpaused
void unPause()
{
  Serial.println("UNPAUSED");
  // resume forward motion if incomplete
  if (driveMotionIncomplete)
  {
    driveToPausableResume();
    Serial.println("RESUMEING DRIVE");
  }
  if (turnMotionIncomplete)
  {
    turnToPausableResume();
    Serial.println("RESUMEING TURN");
  }
}

void setup()
{
  // Start the serial monitor
  Serial.begin(9600);

  // blue motor setup
  motor.setup();
  motor.reset();

  // chassis setup
  chassis.init();

  // gripper Setup
  gripperServo.setMinMaxMicroseconds(0, 2000);
  gripperServo.writeMicroseconds(0);

  // line following setup
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A4, A3}, SensorCount);

  // ultrasonic setup
  HCSR04.begin(ultrasonicTriggerPin, ultrasonicEchoPin);

  // remote Control setup
  remoteControl.setup();

  remoteControl.onPress(moveUp, remoteUp);
  remoteControl.onPress(moveDown, remoteDown);

  remoteControl.toggleFunc(openGripper, remoteLeft);
  remoteControl.toggleFunc(closeGripper, remoteRight);

  // remoteControl.toggleFunc(fourBarLow, remoteVolMinus);
  remoteControl.onPress(stopIt, remoteVolPlus);

  remoteControl.onPress(updateCurrentState, remoteEnterSave);

  remoteControl.toggleFunc(ultrasonicDistanceTest, remote0);
  remoteControl.onPress(returnMotorPosition, remoteBack);

  remoteControl.toggleFunc(fourtyFiveDegreeSide, remote1);
  remoteControl.toggleFunc(sixtyDegreeSide, remote2);
  remoteControl.onPress(resetEncoder, remote7);

  remoteControl.onPress(lineFollowCalibration, remote4);
  remoteControl.toggleFunc(linefollowTesting, remote5);

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

  remoteControl.toggleFunc(forwardUntilLine, forwardUntilLineConst);

  remoteControl.toggleFunc(goToBottomEncoder, remoteVolMinus);

  // remoteControl.onPress(turnToPausableTest, remote3);
  // remoteControl.toggleFunc(driveToPausableTest2, driveToPausableTest2Const);
  // remoteControl.toggleFunc(turnToPausableTest2, turnToPausableTest2Const);

  remoteControl.toggleFunc(driveFor, driveForConst);
  remoteControl.toggleFunc(turnFor, turnForConst);

  remoteControl.eStop(stopIt, remote9);
  remoteControl.ePause(stopIt, unPause, remote8);

  delay(4000);
  Serial.println("READY!");
}

void loop()
{
  // Code to test Remote
  // checkRemote();

  //  Make sure the Battery Voltage is correct
  // if (readBatteryMillivolts() <= 6000)
  // {
  //   Serial.print("BATTERY LOW: ");
  //   Serial.println(readBatteryMillivolts());
  //   buttonC.waitForButton();
  // }

  remoteControl.checkRemoteButtons();
}
