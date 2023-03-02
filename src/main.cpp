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
  Serial.println(currentState);
}

void printTest()
{
  Serial.print("AO: ");
  Serial.println(analogRead(A0));
  Serial.print("A2: ");
  Serial.println(analogRead(A2));
  Serial.print("A3: ");
  Serial.println(analogRead(A3));
  Serial.print("A4: ");
  Serial.println(analogRead(A4));
  Serial.print("Current State: ");
  Serial.println(currentState);
}

int ultrasonicDistance()
{
  digitalWrite(ultrasonicTriggerPin, LOW); // Clear ultrasonic trigger pin
  delayMicroseconds(10);
  digitalWrite(ultrasonicTriggerPin, HIGH); // Outputs signal
  delayMicroseconds(10);
  digitalWrite(ultrasonicTriggerPin, LOW); // Stops outputting signals
  ultrasonicSignalDuration = pulseIn(ultrasonicEchoPin, HIGH);
  frontDistance = ultrasonicSignalDuration * 0.034 / 2; // (time) * (speed of sound) / 2 (sending then receiving)
   Serial.print("Distance: ");
   Serial.println(frontDistance);
  return frontDistance;
}

void ultrasonicDistanceTest()
{
  digitalWrite(ultrasonicTriggerPin, LOW); // Clear ultrasonic trigger pin
  delayMicroseconds(10);
  digitalWrite(ultrasonicTriggerPin, HIGH); // Outputs signal
  delayMicroseconds(10);
  digitalWrite(ultrasonicTriggerPin, LOW); // Stops outputting signals
  ultrasonicSignalDuration = pulseIn(ultrasonicEchoPin, HIGH);
  frontDistance = ultrasonicSignalDuration * 0.034 / 2; // (time) * (speed of sound) / 2 (sending then receiving)
  Serial.print("Distance: ");
  Serial.println(frontDistance);
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

void qtrReadings()
{
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

void lineFollowing()
{
  // Serial.print("CURRENTSTATE: ");
  // Serial.print(currentState);
  // Serial.print("\t");
  uint16_t position = qtr.readLineBlack(sensorValues);
  if (position > 700)
  {
    //   Serial.println("FAR RIGHT");
    chassis.setMotorEfforts(60, 20);
  }
  else if (position < 300)
  {
    //   Serial.println("FAR LEFT");
    chassis.setMotorEfforts(20, 60);
  }
  else
  {
    chassis.setMotorEfforts(100, 100);
  }
  switch (currentState)
  {
  case lineFollowFourtyFiveTower:
    // Serial.println("Test");
    if (ultrasonicDistance() <= towerDistanceOne || ultrasonicDistance() >= 1000)
    {
      remoteControl.stopFunction(lineFollowingConst);
      chassis.setMotorEfforts(0, 0);
      currentState = closeGripperFourtyFiveTowerUnconfirmed;
      Serial.println(currentState);
      remoteControl.startFunction(closeGripperConst);
    }

    break;

  case lineFollowSixtyTower:
    if (ultrasonicDistance() <= towerDistanceTwo || ultrasonicDistance() >= 1000)
    {
      delay(100);
      chassis.setMotorEfforts(0, 0);
      remoteControl.stopFunction(lineFollowingConst);
      currentState = closeGripperSixtyTowerUnconfirmed;
      Serial.println(currentState);
      remoteControl.startFunction(closeGripperConst);
    }
    break;
  case lineFollowBlockFourtyFive:
    if (ultrasonicDistance() <= blockDistanceOne || ultrasonicDistance() >= 1000)
    {
      remoteControl.stopFunction(lineFollowingConst);
      chassis.setMotorEfforts(0, 0);
      Serial.println(currentState);
      currentState = lowerMaxFourtyFive;
      Serial.println(currentState);
      remoteControl.startFunction(fourBarLowConst);
    }
    break;
  case lineFollowBlockSixty:
    if (ultrasonicDistance() <= blockDistanceTwo || ultrasonicDistance() >= 2000)
    {
      remoteControl.stopFunction(lineFollowingConst);
      chassis.setMotorEfforts(0, 0);
      currentState = lowerMaxSixty;
      Serial.println(currentState);
      remoteControl.startFunction(fourBarLowConst);
    }
    break;
  case lineFollowBlockSixtyLast:
    if (ultrasonicDistance() <= blockDistanceTwo || ultrasonicDistance() >= 2000)
    {
      remoteControl.stopFunction(lineFollowingConst);
      chassis.setMotorEfforts(0, 0);
      currentState = grabNewPlateSixty;
      Serial.println(currentState);
      remoteControl.startFunction(closeGripperConst);
    }
    break;
  case lineFollowBlockFourtyFiveLast:
    if (ultrasonicDistance() <= blockDistanceOne || ultrasonicDistance() >= 1000)
    {
      remoteControl.stopFunction(lineFollowingConst);
      chassis.setMotorEfforts(0, 0);
      currentState = grabNewPlateFourtyFive;
      Serial.println(currentState);
      remoteControl.startFunction(closeGripperConst);
    }
    break;
  case lineFollowFourtyFiveTowerAgain:
    if (ultrasonicDistance() <= towerDistanceOne || ultrasonicDistance() >= 1000)
    {
      remoteControl.stopFunction(lineFollowingConst);
      chassis.setMotorEfforts(0, 0);
      currentState = lowerSlightlyFourtyFive;
      Serial.println(currentState);
      remoteControl.startFunction(lowerSlightlyConst);
    }
    break;
  case lineFollowSixtyTowerAgain:
    if (ultrasonicDistance() <= towerDistanceTwo || ultrasonicDistance() >= 1000)
    {
      chassis.setMotorEfforts(-100, -100);
      delay(100);
      remoteControl.stopFunction(lineFollowingConst);
      chassis.setMotorEfforts(0, 0);
      currentState = lowerSlightlySixty;
      remoteControl.startFunction(lowerSlightlyConst);
    }
    break;
  case forwardUntilTurnToLeaveFourty:

    if (ultrasonicDistance() <= 6 || ultrasonicDistance() >= 1000)
    {

      remoteControl.stopFunction(lineFollowingConst);
      chassis.setMotorEfforts(0, 0);
      currentState = forwardUntilLineFourty;
      Serial.println(currentState);
      chassis.turnFor(-90, 90, true);
      chassis.driveFor(82, 10, false);
      remoteControl.startFunction(forwardUntilLineConst);
    }

    break;

  case forwardUntilTurnToLeaveSixty:

    if (ultrasonicDistance() <= 6 || ultrasonicDistance() >= 1000)
    {

      remoteControl.stopFunction(lineFollowingConst);
      chassis.setMotorEfforts(0, 0);
      currentState = forwardUntilLineSixty;
      Serial.println(currentState);
      chassis.turnFor(90, 90, true);
      chassis.driveFor(82, 10, false);
      remoteControl.startFunction(forwardUntilLineConst);
    }
    break;
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

void openGripper()
{
  int servoCurrentPosition = analogRead(servoEnc);
  Serial.println(servoCurrentPosition);
  if (servoCurrentPosition > servoOpenedPositionAR)
  {
    Serial.println(servoCurrentPosition);
    gripperServo.writeMicroseconds(servoOpenedPositionMS);
    Serial.println("OPENING");
  }
  else
  {
    Serial.println("FINISHED!");
    // Stop servo onced jaw is opened
    remoteControl.stopFunction(openGripperConst);
    remoteControl.stopFunction(remoteLeft);
    switch (currentState)
    {
    case openGripperFourtyFiveBlockConfirmed:
      currentState = backUpFourtyFive;
      Serial.println(currentState);
      remoteControl.startFunction(calebFunctionConst);
      break;

    case openGripperSixtyBlockConfirmed:
      currentState = backUpSixty;
      Serial.println(currentState);
      remoteControl.startFunction(calebFunctionConst);
      break;
    case openGripperFourtyFiveTowerConfirmed:
      currentState = leaveAfterPlacementFourtyTower;
      chassis.driveFor(-15, -15, true);
      Serial.println(currentState);
      remoteControl.startFunction(calebFunctionConst);
      break;
    case openGripperSixtyTowerConfirmed:
      currentState = leaveAfterPlacementSixtyTower;
      chassis.driveFor(-15, -15, true);
      Serial.println(currentState);
      remoteControl.startFunction(calebFunctionConst);
      break;
    }
  }
}

void closeGripper()
{
  int servoCurrentPosition = analogRead(servoEnc);
//  Serial.println(servoCurrentPosition);
  if (!(abs(servoCurrentPosition - servoClosedPositionAR) <= servoCloseTolerance))
  {
    // every (servoReadingDelay) ms, try to move the gripper
    if (millis() > servoPrevMS + servoReadingDelay)
    {
      gripperServo.writeMicroseconds(servoClosedPositionMS);

      // check if the servo is getting stuck. If so, open the Bottom Gripper.
      if (abs(servoPrevReading - servoCurrentPosition) < servoStuckTolerance)
      {
        // once servoCount has gotten to 15 or greater, run the bottom out gripper code instead.
        if (servoStillCount >= 15)
        {
          Serial.println("STUCK!");
          openGripper();
          remoteControl.stopFunction(closeGripperConst);
          remoteControl.stopFunction(remoteRight);
        }
        servoStillCount++;
      }
      else
      {
        servoStillCount = 0;
      }

      // if Gripper has reached target position, finish movement

      servoPrevReading = servoCurrentPosition;
      servoPrevMS = millis();
    }
  }
  else
  {
    // Stop servo onced jaw is vlosed
    Serial.println("FINISHED");
    remoteControl.stopFunction(closeGripperConst);
    remoteControl.stopFunction(remoteRight);
    switch (currentState)
    {
    case confirmedFourtyFiveTower:
      currentState = raiseSlightlyFourtyFiveTower;
      Serial.println(currentState);
      remoteControl.startFunction(raiseSlightlyConst);
      break;

    case confirmedSixtyTower:
      currentState = raiseSlightlySixtyTower;
      Serial.println(currentState);
      remoteControl.startFunction(raiseSlightlyConst);
      break;
    case grabNewPlateFourtyFive:
      currentState = backUpFourtyFiveAgain;
      Serial.println(currentState);
      remoteControl.startFunction(calebFunctionConst);
      break;
    case grabNewPlateSixty:
      currentState = backUpSixtyAgain;
      Serial.println(currentState);
      remoteControl.startFunction(calebFunctionConst);
      break;
    default:
      break;
    }
  }
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
    switch (currentState)
    {
    case raiseSixty:
      chassis.setMotorEfforts(0, 0);
      remoteControl.stopFunction(takeOffHighPlateConst);
      motor.setToggleOff(true);
      currentState = lineFollowSixtyTower;
      Serial.println(currentState);
      remoteControl.startFunction(lineFollowingConst);
      break;

    case raiseToPlaceSixty:
      chassis.setMotorEfforts(0, 0);
      remoteControl.stopFunction(takeOffHighPlateConst);
      motor.setToggleOff(true);
      currentState = raiseSlightlySixtyTowerAgain;
      Serial.println(currentState);
      remoteControl.startFunction(raiseSlightlyConst);
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
      currentState = lineFollowFourtyFiveTower;
      Serial.println(currentState);
      remoteControl.startFunction(lineFollowingConst);
      break;

    case raiseToPlaceFourtyFive:
      currentState = raiseSlightlyFourtyFiveTowerAgain;
      Serial.println(currentState);
      remoteControl.startFunction(raiseSlightlyConst);
      break;
    }
  }
}

void raiseSlightly()
{
  //  Serial.println(currentState);
  switch (currentState)
  {
  case raiseSlightlyFourtyFiveTower:

    /// Serial.println("FourtyFive");
    if (motor.getToggleOff())
    {
      motor.moveTo(firstSpot45deg + 1000);
    }
    else
    {
      remoteControl.stopFunction(raiseSlightlyConst);
      motor.setToggleOff(true);
      currentState = CalebFunctionForty;
      chassis.driveFor(-5, -10, true);
      Serial.println(currentState);
      remoteControl.startFunction(calebFunctionConst);
    }
    break;
  case raiseSlightlySixtyTower:
    if (motor.getToggleOff())
    {
      //  Serial.println("RAISING SLIGHTLY");
      motor.moveTo(firstSpot60deg - 1500);
    }
    else
    {
      motor.setToggleOff(true);
      remoteControl.stopFunction(raiseSlightlyConst);
      currentState = CalebFunctionSixty;
      Serial.println(currentState);
      chassis.driveFor(-5, -10, true);
      remoteControl.startFunction(calebFunctionConst);
    }
    break;
  case raiseSlightlyFourtyFiveTowerAgain:
    if (motor.getToggleOff())
    {
      motor.moveTo(firstSpot45deg + 1000);
    }
    else
    {
      motor.setToggleOff(true);
      remoteControl.stopFunction(raiseSlightlyConst);
      currentState = lineFollowFourtyFiveTowerAgain;
      Serial.println(currentState);
      remoteControl.startFunction(lineFollowingConst);
    }
    break;
  case raiseSlightlySixtyTowerAgain:
    if (motor.getToggleOff())
    {
      motor.moveTo(firstSpot60deg - 1900);
    }
    else
    {
      motor.setToggleOff(true);
      remoteControl.stopFunction(raiseSlightlyConst);
      currentState = lineFollowSixtyTowerAgain;
      Serial.println(currentState);
      remoteControl.startFunction(lineFollowingConst);
      chassis.getLeftEncoderCount();
    }
    break;
  }
}

void lowerSlightly()
{

  switch (currentState)
  {
  case lowerSlightlyFourtyFive:
    if (motor.getToggleOff())
    {
      motor.moveTo(firstSpot45deg - 100);
    }
    else
    {
      remoteControl.stopFunction(raiseSlightlyConst);
      motor.setToggleOff(true);
      currentState = placePlateFourtyFiveUnconfirmed;
      Serial.println(currentState);
    }
    break;

  case lowerSlightlySixty:
    if (motor.getToggleOff())
    {
      motor.moveTo(firstSpot60deg - 200);
    }
    else
    {
      remoteControl.stopFunction(raiseSlightlyConst);
      motor.setToggleOff(true);
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
}

void fourtyFiveDegreeSide()
{
  currentState = raiseFourtyFive;
  Serial.println(currentState);
  remoteControl.stopFunction(remote1);
  remoteControl.startFunction(takeOffLowPlateConst);
}

void sixtyDegreeSide()
{
  currentState = raiseSixty;
  Serial.println(currentState);
  remoteControl.stopFunction(remote2);
  remoteControl.startFunction(takeOffHighPlateConst);
}

void updateCurrentState()
{
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
    Serial.println(currentState);
    remoteControl.startFunction(openGripperConst);
    break;
  case openGripperSixtyBlockUnconfirmed:
    currentState = openGripperSixtyBlockConfirmed;
    Serial.println(currentState);
    remoteControl.startFunction(openGripperConst);
    break;
  case placePlateFourtyFiveUnconfirmed:
    currentState = openGripperFourtyFiveTowerConfirmed;
    Serial.println(currentState);
    remoteControl.startFunction(openGripperConst);
    break;
  case placePlateSixtyUnconfirmed:
    currentState = openGripperSixtyTowerConfirmed;
    Serial.println(currentState);
    remoteControl.startFunction(openGripperConst);
    break;
  case waitForRefereeSixty:
    currentState = lineFollowBlockSixtyLast;
    Serial.println(currentState);
    remoteControl.startFunction(lineFollowingConst);
    break;
  case waitForRefereeForty:
    currentState = lineFollowBlockFourtyFiveLast;
    Serial.println(currentState);
    remoteControl.startFunction(lineFollowingConst);
    break;
  }
}

void calebFunction()
{
  float distanceUlt = ultrasonicDistance();
  Serial.print("DISTANCE:   ");
  Serial.println(distanceUlt);
  switch (currentState)
  {
  case CalebFunctionForty:
    if (distanceUlt <= 29 || distanceUlt > 100)
    {
      chassis.setWheelSpeeds(-3.00, -3.00);
    }
    else
    {
      chassis.idle();
      currentState = rotateLeftUntilLine;
      Serial.println(currentState);
      remoteControl.stopFunction(calebFunctionConst);
      chassis.turnFor(80, 90.0);
      remoteControl.startFunction(rotateLeftUntilLineConst);
    }
    break;
  case CalebFunctionSixty:
    Serial.println("DISTANCE    ");
    Serial.print(distanceUlt);
    if (distanceUlt <= 29 || distanceUlt > 100)
    {
      chassis.setWheelSpeeds(-3.00, -3.00);
    }
    else
    {
      chassis.idle();
      currentState = rotateRightUntilLine;
      Serial.println(currentState);
      remoteControl.stopFunction(calebFunctionConst);
      chassis.turnFor(-80, 90.0);
      remoteControl.startFunction(rotateRightUntilLineConst);
    }
    break;
  case backUpFourtyFive:
    if (distanceUlt < 15 || distanceUlt > 100)
    {
      // Serial.println(ultrasonicDistance());
      chassis.setWheelSpeeds(-5.00, -5.00);
    }
    else
    {
      chassis.idle();
      motor.setEffort(400);
      delay(50);
      motor.setEffort(0);
      currentState = waitForRefereeForty;
      Serial.println(currentState);
      remoteControl.stopFunction(calebFunctionConst);
    }
    break;
  case backUpSixty:
    if (distanceUlt < 15 || distanceUlt > 100)
    {
      //  Serial.println(ultrasonicDistance());
      chassis.setWheelSpeeds(-5.00, -5.00);
    }
    else
    {
      chassis.idle();
      motor.setEffort(400);
      delay(50);
      motor.setEffort(0);
      currentState = waitForRefereeSixty;
      Serial.println(currentState);
      remoteControl.stopFunction(calebFunctionConst);
    }
    break;
  case backUpFourtyFiveAgain:
    if (distanceUlt < 27 || distanceUlt > 100)
    {
      //  Serial.println(ultrasonicDistance());
      chassis.setWheelSpeeds(-5.00, -5.00);
    }
    else
    {
      chassis.idle();
      currentState = rotateRightUntilLineAgain;
      Serial.println(currentState);
      remoteControl.stopFunction(calebFunctionConst);
      chassis.turnFor(-90, 90.0);
      remoteControl.startFunction(rotateRightUntilLineConst);
    }
    break;
  case backUpSixtyAgain:
    if (distanceUlt < 27 || distanceUlt > 100)
    {
      //  Serial.println(ultrasonicDistance());
      chassis.setWheelSpeeds(-5.00, -5.00);
    }
    else
    {
      chassis.idle();
      currentState = rotateLeftUntilLineAgain;
      Serial.println(currentState);
      remoteControl.stopFunction(calebFunctionConst);
      chassis.turnFor(90, 90.0);
      remoteControl.startFunction(rotateLeftUntilLineConst);
    }
    break;
  case leaveAfterPlacementFourtyTower:
    if (distanceUlt < 27 || distanceUlt > 100)
    {
      //  Serial.println(ultrasonicDistance());
      chassis.setWheelSpeeds(-5.00, -5.00);
    }
    else
    {
      chassis.idle();
      currentState = rotateLeftAfterPlacementFourtyTower;
      Serial.println(currentState);
      remoteControl.stopFunction(calebFunctionConst);
      chassis.turnFor(90, 90.0);
      remoteControl.startFunction(rotateLeftUntilLineConst);
    }
    break;
  case leaveAfterPlacementSixtyTower:
    if (distanceUlt < 27 || distanceUlt > 100)
    {
      //  Serial.println(ultrasonicDistance());
      chassis.setWheelSpeeds(-5.00, -5.00);
    }
    else
    {
      chassis.idle();
      currentState = rotateRightAfterPlacementSixtyTower;
      Serial.println(currentState);
      remoteControl.stopFunction(calebFunctionConst);
      chassis.turnFor(-90, 90.0);
      remoteControl.startFunction(rotateRightUntilLineConst);
    }
    break;
  }
}
void rotateRightUntilLineFunct()
{
  if (chassis.checkMotionComplete())
  {

    switch (currentState)
    {
    case rotateRightUntilLine:
      currentState = lineFollowBlockSixty;
      Serial.println(currentState);
      remoteControl.stopFunction(rotateRightUntilLineConst);
      remoteControl.startFunction(lineFollowingConst);
      break;

    case rotateRightUntilLineAgain:
      currentState = raiseToPlaceFourtyFive;
      Serial.println(currentState);
      remoteControl.stopFunction(rotateRightUntilLineConst);
      remoteControl.startFunction(takeOffLowPlateConst);
      break;
    case rotateRightAfterPlacementSixtyTower:
      remoteControl.stopFunction(rotateRightUntilLineConst);
      currentState = forwardUntilTurnToLeaveSixty;
      Serial.println(currentState);
      remoteControl.startFunction(lineFollowingConst);
      break;
    }
  }
}

void rotateLeftUntilLineFunct()
{
  if (chassis.checkMotionComplete())
  {
    switch (currentState)
    {
    case rotateLeftUntilLine:
      currentState = lineFollowBlockFourtyFive;
      Serial.println(currentState);
      remoteControl.stopFunction(rotateLeftUntilLineConst);
      remoteControl.startFunction(lineFollowingConst);
      break;

    case rotateLeftUntilLineAgain:
      currentState = raiseToPlaceSixty;
      Serial.println(currentState);
      remoteControl.stopFunction(rotateLeftUntilLineConst);
      remoteControl.startFunction(takeOffHighPlateConst);
      break;
    case rotateLeftAfterPlacementFourtyTower:
      remoteControl.stopFunction(rotateLeftUntilLineConst);
      currentState = forwardUntilTurnToLeaveFourty;
      Serial.println(currentState);
      remoteControl.startFunction(lineFollowingConst);
      break;
    }
  }
}

void forwardUntilLine()
{
  if (chassis.checkMotionComplete()) {
  switch (currentState)
  {
  case forwardUntilLineFourty:
    currentState = replaceStepOneFourty;
    Serial.println(currentState);
    chassis.idle();
    chassis.turnFor(-90, 90.0, false);
    break;
  case replaceStepOneFourty:
    currentState = replaceStepTwoFourty;
    Serial.println(currentState);
    chassis.driveFor(20, 10, false);
  break;
  case replaceStepTwoFourty:
    currentState = replaceStepThreeFourty;
    Serial.println(currentState);
    chassis.turnFor(-90, 90.0, false);
  break;
  case replaceStepThreeFourty:
    remoteControl.stopFunction(forwardUntilLineConst);
    remoteControl.startFunction(remote9);
    remoteControl.stopFunction(remote9);
    Serial.println("Replace Me with a New Robot Plz Master");
  break;
  case forwardUntilLineSixty:
    currentState = replaceStepOneSixty;
    Serial.println(currentState);
    chassis.idle();
    chassis.turnFor(90, 90.0, false);
    break;
  case replaceStepOneSixty:
    currentState = replaceStepTwoSixty;
    Serial.println(currentState);
    chassis.driveFor(23, 10, false);
  break;
  case replaceStepTwoSixty:
    currentState = replaceStepThreeSixty;
    Serial.println(currentState);
    chassis.turnFor(90, 90.0, false);
  break;
  case replaceStepThreeSixty:
    remoteControl.stopFunction(forwardUntilLineConst);
    remoteControl.startFunction(remote9);
    remoteControl.stopFunction(remote9);
    Serial.println("Replace Me with a New Robot Plz Master");
  break;
  }
  }
}

void replaceRobotFourty()
{
  currentState = leaveAfterPlacementFourtyTower;
  Serial.println(currentState);
  remoteControl.startFunction(calebFunctionConst);
}

void replaceRobotSixty()
{
  currentState = leaveAfterPlacementSixtyTower;
  Serial.println(currentState);
  remoteControl.startFunction(calebFunctionConst);
}

void returnMotorPosition()
{
  Serial.println(motor.getPosition());
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

  // auto calibraiton
  //  qtr.calibrationOn.minimum[0] = lineFollowCaliLow;
  //  qtr.calibrationOn.maximum[0] = lineFollowCaliHigh;

  pinMode(ultrasonicEchoPin, INPUT);
  pinMode(ultrasonicTriggerPin, OUTPUT);

  remoteControl.setup();

  remoteControl.onPress(moveUp, remoteUp);
  remoteControl.onPress(moveDown, remoteDown);

  remoteControl.toggleFunc(openGripper, remoteLeft);
  remoteControl.toggleFunc(closeGripper, remoteRight);

  // remoteControl.toggleFunc(fourBarLow, remoteVolMinus);
  remoteControl.onPress(stopIt, remoteVolPlus);

  remoteControl.onPress(replaceRobotFourty, remoteSetup);
  remoteControl.onPress(replaceRobotSixty, remoteStopMode);

  remoteControl.onPress(updateCurrentState, remoteEnterSave);

  remoteControl.onPress(ultrasonicDistanceTest, remote0);
  remoteControl.onPress(returnMotorPosition, remoteBack);

  remoteControl.toggleFunc(fourtyFiveDegreeSide, remote1);
  remoteControl.toggleFunc(sixtyDegreeSide, remote2);
  remoteControl.onPress(resetEncoder, remote3);

  remoteControl.onPress(lineFollowCalibration, remote4);
  remoteControl.toggleFunc(linefollowTesting, remote5);
  remoteControl.toggleFunc(qtrReadings, remote6);

  remoteControl.toggleFunc(lineFollowing, lineFollowingConst);

  remoteControl.toggleFunc(takeOffHighPlate, takeOffHighPlateConst);
  remoteControl.toggleFunc(takeOffLowPlate, takeOffLowPlateConst);

  remoteControl.toggleFunc(openGripper, openGripperConst);
  remoteControl.toggleFunc(closeGripper, closeGripperConst);

  remoteControl.toggleFunc(raiseSlightly, raiseSlightlyConst);
  remoteControl.toggleFunc(lowerSlightly, lowerSlightlyConst);

  remoteControl.toggleFunc(fourBarLow, fourBarLowConst);

  remoteControl.toggleFunc(calebFunction, calebFunctionConst);

  remoteControl.toggleFunc(rotateRightUntilLineFunct, rotateRightUntilLineConst);
  remoteControl.toggleFunc(rotateLeftUntilLineFunct, rotateLeftUntilLineConst);

  remoteControl.toggleFunc(forwardUntilLine, forwardUntilLineConst);

  remoteControl.toggleFunc(goToBottomEncoder, remoteVolMinus);

  remoteControl.eStop(stopIt, remote9);
  remoteControl.ePause(stopIt, remote8);

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
