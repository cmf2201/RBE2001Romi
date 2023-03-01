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

BlueMotor motor;
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;

RemoteControl remoteControl(2);

Servo32U4Pin5 gripperServo;
Servo32U4 jawServo;


Chassis chassis;


//QTR Sensors
QTRSensors qtr;

const uint8_t SensorCount = 2;
uint16_t sensorValues[SensorCount];

bool BOTTOM_OUT_GRIPPER = true;

//Linear Servo declarations
int servoPin = 0;
int linearPotPin = A0;
int servoStop = 1490;  
int servoJawDown = 500;  
int servoJawUp = 2500;  
int printDelay = 500;
int linearPotVoltageADC = 500;
int jawOpenPotVoltageADC = 750;
int jawClosedPotVoltageADC = 530;
int previous1PotVoltage = 0;
int previous2PotVoltage = 0;
Timer printTimer(printDelay);
bool servoToggle = true;


// Gripper Servo Definitions
// int servoPin = 12;
int servoEnc = A0;

bool servoActive = false;

int servoClosedPositionMS = 2000; // og = 1250
int servoClosedPositionAR = 390;
int servoOpenedPositionMS = 1000; // 275? og =10
int servoOpenedPositionAR = 205; // og 137

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
int firstSpot60deg = 8150;
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

// motor positions for bottom out gripper robot
int bottomOutSpot45 = 3472;
int bottomOutSpot60 = 8500;

// Ultrasonic Sensor Variables 
const byte ultrasonicTriggerPin = 3;
const byte ultrasonicEchoPin = 30;
int frontDistance = 0;
long ultrasonicSignalDuration = 0;

// Remote Testing Variables
// IRDecoder decoder(13);

bool paused = false;

// void moveUp(void);
// void moveDown(void);

void stopIt() {
  Serial.println("ESTOP");
  Serial.println(motor.getPosition());
  motor.setEffort(0);
  gripperServo.writeMicroseconds(0);
  chassis.setMotorEfforts(0,0);
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
}

// void closeServo() {
//   Serial.println(analogRead(linearPotPin));
//   gripperServo.writeMicroseconds(2000);
// }

// void openServo() {
//   Serial.println(analogRead(linearPotPin));
//   gripperServo.writeMicroseconds(10);
// }

void changeGripperType() {
  BOTTOM_OUT_GRIPPER = !BOTTOM_OUT_GRIPPER;
  if (BOTTOM_OUT_GRIPPER) {
    Serial.println("Bottom Out Gripper is now active");
  } else {
    Serial.println("Linear Gripper is now active");
  }
}

void ultrasonicDistance() {
  digitalWrite(ultrasonicTriggerPin, LOW); // Clear ultrasonic trigger pin
  delay(10);
  digitalWrite(ultrasonicTriggerPin, HIGH); // Outputs signal 
  delay(10);
  digitalWrite(ultrasonicTriggerPin, LOW); // Stops outputting signals
  ultrasonicSignalDuration = pulseIn(ultrasonicEchoPin, HIGH); 
  frontDistance = ultrasonicSignalDuration*0.034/2; // (time) * (speed of sound) / 2 (sending then receiving)
  Serial.print("Distance: ");
  Serial.println(frontDistance);
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
  uint16_t position = qtr.readLineBlack(sensorValues);
  if(position > 700) {
    Serial.println("FAR RIGHT");
    chassis.setMotorEfforts(100,80);
  } else if (position < 300) {
    Serial.println("FAR LEFT");
    chassis.setMotorEfforts(80,100);
  } else {
    chassis.setMotorEfforts(100,100);
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
    // buttonC.waitForButton();
  }
}

//opens the bottom out gripper
void openBottomGripper() {
  Serial.println("open going");
  if(servoActive) {
    int servoCurrentPosition = analogRead(servoEnc);
    Serial.println(servoCurrentPosition);
    if(servoCurrentPosition > servoOpenedPositionAR) {
      Serial.println(servoCurrentPosition);
      gripperServo.writeMicroseconds(servoOpenedPositionMS);
      Serial.println("OPENING");
      servoCurrentPosition = analogRead(servoEnc);
    } else {
      servoActive = false;
      Serial.println("OVER");
      remoteControl.stopFunction(remote2);
    }
  }
}

//closes the bottom out gripper, but stops if the gripper cannot close
void closeBottomGripper() {
  // Serial.println("close going");
  // if(servoActive) {
  //   int servoCurrentPosition = analogRead(servoEnc);
  //   Serial.println(servoCurrentPosition);
  //   if (servoCurrentPosition < servoClosedPositionAR) {
  //     Serial.println(servoCurrentPosition);
  //     gripperServo.writeMicroseconds(servoClosedPositionMS);
  //     Serial.println("CLOSING");
  //     servoCurrentPosition = analogRead(servoEnc);
  //   } else {
  //     servoActive = false;
  //     Serial.println("OVER");
  //     remoteControl.stopFunction(remote1);
  //   }
  // }

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

  if(BOTTOM_OUT_GRIPPER) {
    servoActive = true;
    openBottomGripper();
  } else {
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
}

void closeGripper() {
  Serial.println("close button pressed");  
  if (BOTTOM_OUT_GRIPPER) {
    servoActive = true;
    Serial.println("close gripper going");
    closeBottomGripper(); 
  } else {
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
}

// Moves motor until 4-bar linkage and gripper are positioned to grab the 60 degree plate
void takeOffHighPlate() {
  if(BOTTOM_OUT_GRIPPER) {
    firstSpot60deg = bottomOutSpot60;
  }
  if(motor.getToggleOff()) {
    motor.moveTo(firstSpot60deg);
  } else {
    remoteControl.stopFunction(remote7);
    motor.setToggleOff(true);
  }
}

// Moves motor until 4-bar linkage and gripper are positioned to grab the 60 degree plate
void takeOffLowPlate() {
  if(BOTTOM_OUT_GRIPPER) {
    firstSpot45deg = bottomOutSpot45;
  }
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
  Serial.println("Blue motor reset");
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

  pinMode(A0,INPUT);

  pinMode(ultrasonicEchoPin,INPUT); 
  pinMode(ultrasonicTriggerPin, OUTPUT);

  remoteControl.setup();

  remoteControl.onPress(changeGripperType, remoteSetup);
  remoteControl.onPress(batteryCheck, remotePlayPause);

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

  remoteControl.checkRemoteButtons();

  // gripperServo.writeMicroseconds(servoClosedPositionMS);
  // delay(500);
  // Serial.println("1");
  //  Serial.println(analogRead(A0));
  // delay(500);
  // gripperServo.writeMicroseconds(servoOpenedPositionMS);
  // delay(500);
  // Serial.println("2");
  // Serial.println(analogRead(A0));
  // delay(500);
  // Serial.println("3");
  // gripperServo.writeMicroseconds(1000);


  // motor.setEffort(200);
//
}





