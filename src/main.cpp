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

  float curDistance = ultrasonicDistance();
  switch (currentState)
  {
  case lineFollowFourtyFiveTower:
    if (curDistance < towerDistanceOne || curDistance >= 1000)
    {
      remoteControl.stopFunction(lineFollowingConst);
      chassis.setMotorEfforts(0, 0);
      
      remoteControl.startFunction(closeGripperConst,
        currentState = closeGripperFourtyFiveTowerUnconfirmed);
    }

    break;

  case lineFollowSixtyTower:
    if (curDistance <= towerDistanceTwo || curDistance >= 1000)
    {
      delay(100); //!!!!!!!!!!!!!!!!!!
      chassis.setMotorEfforts(0, 0);
      remoteControl.stopFunction(lineFollowingConst);

      remoteControl.startFunction(closeGripperConst,
        currentState = closeGripperSixtyTowerUnconfirmed);
    }
    break;

  case lineFollowBlockFourtyFive:
    if (curDistance <= blockDistanceOne || curDistance >= 1000)
    {
      remoteControl.stopFunction(lineFollowingConst);
      chassis.setMotorEfforts(0, 0);

      remoteControl.startFunction(fourBarLowConst,
        currentState = lowerMaxFourtyFive);
    }
    break;

  case lineFollowBlockSixty:
    if (curDistance <= blockDistanceTwo || curDistance >= 1000)
    {
      remoteControl.stopFunction(lineFollowingConst);
      chassis.setMotorEfforts(0, 0);

      remoteControl.startFunction(fourBarLowConst,
        currentState = lowerMaxSixty);
    }
    break;

  case lineFollowBlockSixtyLast:
    if (curDistance <= blockDistanceTwo || curDistance >= 1000)
    {
      remoteControl.stopFunction(lineFollowingConst);
      chassis.setMotorEfforts(0, 0);

      remoteControl.startFunction(closeGripperConst, 
        currentState = grabNewPlateSixty);
    }
    break;

  case lineFollowBlockFourtyFiveLast:
    if (curDistance <= blockDistanceOne || curDistance >= 1000)
    {
      remoteControl.stopFunction(lineFollowingConst);
      chassis.setMotorEfforts(0, 0);

      remoteControl.startFunction(closeGripperConst,
        currentState = grabNewPlateFourtyFive);
    }
    break;

  case lineFollowFourtyFiveTowerAgain:
    if (curDistance <= towerDistanceOne || curDistance >= 1000)
    {
      remoteControl.stopFunction(lineFollowingConst);
      chassis.setMotorEfforts(0, 0);

      remoteControl.startFunction(lowerSlightlyConst, 
        currentState = lowerSlightlyFourtyFive);
    }
    break;

  case lineFollowSixtyTowerAgain:
    if (curDistance <= towerDistanceTwo || curDistance >= 1000)
    {
      chassis.setMotorEfforts(-100, -100); //!!!!!!!!!!!!
      delay(50); //!!!!!!!!!!!!!
      remoteControl.stopFunction(lineFollowingConst);
      chassis.setMotorEfforts(0, 0);

      remoteControl.startFunction(lowerSlightlyConst, 
        currentState = lowerSlightlySixty);
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

//function opens the Gripper
void openGripper()
{
  int servoCurrentPosition = analogRead(servoEnc);
  if(servoCurrentPosition < servoOpenedPositionAR) {
    gripperServo.writeMicroseconds(servoOpenedPositionMS);
  } else {
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
      chassis.driveFor(-5, -5, true); //!!!!!!!!!!!!!!!!
      remoteControl.startFunction(calebFunctionConst,
        currentState = leaveAfterPlacementFourtyTower);
      break;

    case openGripperSixtyTowerConfirmed:
      chassis.driveFor(-15, -15, true); //!!!!!!!!!!!!!!!!!!!
      remoteControl.startFunction(calebFunctionConst,
        currentState = leaveAfterPlacementSixtyTower);
      break;

    }
  }
}

void closeGripper()
{
    int servoCurrentPosition = analogRead(servoEnc);
    if(servoCurrentPosition > servoClosedPositionAR) {
      gripperServo.writeMicroseconds(servoClosedPositionMS);
    }else {
      // Stop servo onced jaw is vlosed
      Serial.println("FINISHED");
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
    chassis.setMotorEfforts(0,0);

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

//raises the motor slightly
void raiseSlightly()
{
  //decide what position to move to
  if (motor.getToggleOff()){
    int moveToPos;
    switch(currentState) {
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
        chassis.driveFor(-20, -10, true); //NOT ESTOPAABLE!!!!!!!!!!!!!

        remoteControl.startFunction(calebFunctionConst,
          currentState = CalebFunctionForty);
        break;

      case raiseSlightlySixtyTower:
        chassis.driveFor(-20, -5, true); //NOT ESTOPABLE!!!!!!!!!!!!!!!

        remoteControl.startFunction(calebFunctionConst,
          currentState = CalebFunctionSixty);
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
      motor.moveTo(firstSpot60deg + 300);
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

void goToBottomEncoder() {
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
    if (distanceUlt < wallToLineDist || distanceUlt > 100)
    {
      chassis.setWheelSpeeds(-5.00, -5.00);
    }
    else
    {
      chassis.idle();
      currentState = rotateLeftUntilLine;
      Serial.println(currentState);
      remoteControl.stopFunction(calebFunctionConst);
      chassis.turnFor(90, 90.0);
      remoteControl.startFunction(rotateLeftUntilLineConst);
    }
    break;
  case CalebFunctionSixty:
    if (distanceUlt < wallToLineDist || distanceUlt > 100)
    {
      chassis.setWheelSpeeds(-5.00, -5.00);
    }
    else
    {
      chassis.idle();
      currentState = rotateRightUntilLine;
      Serial.println(currentState);
      remoteControl.stopFunction(calebFunctionConst);
      chassis.turnFor(-20, 10, true);
     remoteControl.startFunction(rotateRightUntilLineConst);
    }
    break;
  case backUpFourtyFive:
    if (distanceUlt < blockBackUpDist || distanceUlt > 100)
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
    if (distanceUlt < blockBackUpDist || distanceUlt > 100)
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
      chassis.turnFor(30, 90, true);
      delay(500);
      currentState = rotateLeftUntilLineAgain;
      Serial.println(currentState);
      remoteControl.stopFunction(calebFunctionConst);
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
      chassis.setMotorEfforts(50, -35);
      if (qtr.readLineBlack(sensorValues) > 100 && qtr.readLineBlack(sensorValues) < 900)
      {
        Serial.println(qtr.readLineBlack(sensorValues));
        currentState = lineFollowBlockSixty;
        Serial.println(currentState);
        remoteControl.stopFunction(rotateRightUntilLineConst);
        remoteControl.startFunction(lineFollowingConst);
      }
    break;

  case rotateRightUntilLineAgain:
  chassis.setMotorEfforts(50, -35);
      if (qtr.readLineBlack(sensorValues) > 100 && qtr.readLineBlack(sensorValues) < 900)
      {
        Serial.println(qtr.readLineBlack(sensorValues));
    currentState = raiseToPlaceFourtyFive;
    Serial.println(currentState);
    remoteControl.stopFunction(rotateRightUntilLineConst);
    remoteControl.startFunction(takeOffLowPlateConst);
      }
    break;
  case rotateRightAfterPlacementSixtyTower:

    remoteControl.stopFunction(rotateRightUntilLineConst);
    currentState = forwardUntilTurnToLeaveSixty;
    Serial.println(currentState);
    remoteControl.startFunction(forwardFunctConst);
    break;
  case rotateUntilLineAndReplaceSixty:
    remoteControl.stopFunction(rotateRightUntilLineConst);
    Serial.println("Replace Me with a New Robot Plz Master");
    break;
  }
  }
}

void rotateLeftUntilLineFunct()
{
  Serial.println(qtr.readLineBlack(sensorValues));
  if (chassis.checkMotionComplete())
  {
  switch (currentState)
  {
  case rotateLeftUntilLine:
  chassis.setMotorEfforts(-35, 50);
  if (qtr.readLineBlack(sensorValues) > 100 && qtr.readLineBlack(sensorValues) < 900)
      {
    currentState = lineFollowBlockFourtyFive;
    Serial.println(currentState);
    remoteControl.stopFunction(rotateLeftUntilLineConst);
    remoteControl.startFunction(lineFollowingConst);
      }
    break;

  case rotateLeftUntilLineAgain:
    chassis.setMotorEfforts(-35, 50);
  if (qtr.readLineBlack(sensorValues) > 100 && qtr.readLineBlack(sensorValues) < 900)
      {
    Serial.println(qtr.readLineBlack(sensorValues));
    chassis.setMotorEfforts(0, 0);
    currentState = raiseToPlaceSixty;
    Serial.println(currentState);
    remoteControl.stopFunction(rotateLeftUntilLineConst);
    remoteControl.startFunction(takeOffHighPlateConst);
      }
    break;
  case rotateLeftAfterPlacementFourtyTower:
    remoteControl.stopFunction(rotateLeftUntilLineConst);
    currentState = forwardUntilTurnToLeaveFourty;
    Serial.println(currentState);
    remoteControl.startFunction(forwardFunctConst);
    break;
  case rotateUntilLineAndReplaceFourty:
    remoteControl.stopFunction(rotateLeftUntilLineConst);
    Serial.println("Replace Me with a New Robot Plz Master");
    break;
  }
  }
}

void forwardFunct()
{
  switch (currentState)
  {
  case forwardUntilTurnToLeaveFourty:
    remoteControl.stopFunction(forwardUntilLineConst);
    currentState = forwardUntilLineFourty;
    Serial.println(currentState);
    remoteControl.startFunction(forwardUntilLineConst);
    break;

  case forwardUntilTurnToLeaveSixty:
    remoteControl.stopFunction(forwardUntilLineConst);
    currentState = forwardUntilLineSixty;
    Serial.println(currentState);
    remoteControl.startFunction(forwardUntilLineConst);
    break;
  }
}

void forwardUntilLine()
{
  if (chassis.checkMotionComplete()) {
  switch (currentState)
  {
  case forwardUntilLineFourty:
     chassis.setMotorEfforts(50, 50);
     Serial.println(qtr.readLineBlack(sensorValues));
     if (qtr.readLineBlack(sensorValues) < 700 && qtr.readLineBlack(sensorValues) > 300)
    {
    currentState = replaceStepOneFourty;
    Serial.println(currentState);
    chassis.idle();
    }
    //chassis.turnFor(-90, 90.0, false);
    break;
  case replaceStepOneFourty:
    chassis.setMotorEfforts(70, 0);
    Serial.println(qtr.readLineBlack(sensorValues));
    if (qtr.readLineBlack(sensorValues) < 700 && qtr.readLineBlack(sensorValues) > 300){
      chassis.setMotorEfforts(0,0);
      currentState = replaceStepTwoFourty;
      Serial.println(currentState);
    }
  break;
  case replaceStepTwoFourty:
     chassis.setMotorEfforts(50, 50);
        if (qtr.readLineBlack(sensorValues) < 700 && qtr.readLineBlack(sensorValues) > 300)
  {
    currentState = replaceStepThreeFourty;
    Serial.println(currentState);
    chassis.turnFor(-90, 90.0, true);
  }
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
    chassis.driveFor(82, 10, true);
    chassis.turnFor(90, 90.0, false);
    break;
  case replaceStepOneSixty:
    currentState = replaceStepTwoSixty;
    Serial.println(currentState);
    chassis.driveFor(20, 10, false);
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

void returnMotorPosition()
{
  Serial.println(motor.getPosition());
}

void setup()
{
  // Start the serial monitor
  Serial.begin(9600);

  //blue motor setup
  motor.setup();
  motor.reset();

  //chassis setup
  chassis.init();

  //gripper Setup
  gripperServo.setMinMaxMicroseconds(0, 2000);
  gripperServo.writeMicroseconds(0);

  //line following setup
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A4, A3}, SensorCount);

  //ultrasonic setup
  HCSR04.begin(ultrasonicTriggerPin, ultrasonicEchoPin);

  //remote Control setup
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

  remoteControl.toggleFunc(forwardFunct, forwardFunctConst);
  remoteControl.toggleFunc(forwardUntilLine, forwardUntilLineConst);

  remoteControl.toggleFunc(goToBottomEncoder, remoteVolMinus);

  remoteControl.eStop(stopIt, remote9);
  remoteControl.ePause(stopIt,remote8);

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
