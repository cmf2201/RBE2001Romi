#include <Arduino.h>
#include <Romi32U4.h>
#include "BlueMotor/BlueMotor.h"
#include <servo32u4.h>
#include "Timer.h"
#include "IRdecoder.h"
#include "RemoteControl/RemoteConstants.h"

BlueMotor motor;
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;

RemoteControl remoteControl(14);

#define BOTTOM_OUT_GRIPPER true

Servo32U4Pin5 gripperServo;
Servo32U4 jawServo;


//Linear Servo declarations
int servoPin = 0;
int linearPotPin = A0;
int servoStop = 1490;  
int servoJawDown = 500;  
int servoJawUp = 2500;  
int printDelay = 500;
int linearPotVoltageADC = 500;
int jawOpenPotVoltageADC = 490;
int jawClosedPotVoltageADC = 1022;
int previous1PotVoltage = 0;
int previous2PotVoltage = 0;
Timer printTimer(printDelay);


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
int firstSpot = 8114;
int secondSpot = 3970;

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
IRDecoder decoder(13);

bool paused = false;

void setup()
{
  //Start the serial monitor
  Serial.begin(9600);

  //setup the blue motor
  motor.setup();
  // motor.reset();
  //setup the Servo / encoder
  if(BOTTOM_OUT_GRIPPER) {
    gripperServo.setMinMaxMicroseconds(0,1400);
  } else {
    gripperServo.setMinMaxMicroseconds(0,2000);
  }
  gripperServo.attach();
  jawServo.attach();
  decoder.init();

  // gripperServo.writeMicroseconds(10);

  //ensure motor has had time to reset/setup
  delay(3000);
  Serial.println("READY!");
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
void linearGripper() {
  // Stop servo
  jawServo.writeMicroseconds(servoStop);
  delay(2000);
  // Get Pot Value
  linearPotVoltageADC = analogRead(linearPotPin);
  Serial.print("Initial linearPotVoltageADC:   ");
  Serial.println(linearPotVoltageADC);

  while (buttonB.isPressed())
  {
  
    // Move Jaw Down
    jawServo.writeMicroseconds(servoJawDown);
    previous1PotVoltage = 0;

    while (linearPotVoltageADC > jawOpenPotVoltageADC)
    {
      linearPotVoltageADC = analogRead(linearPotPin);
      if (printTimer.isExpired()){
        Serial.print("linearPotVoltageADC:    ");
        Serial.println(linearPotVoltageADC);
      }
    }

    // Stop servo onced jaw is opened
    jawServo.writeMicroseconds(servoStop);

    linearPotVoltageADC = analogRead(linearPotPin);
    Serial.print("Bottom linearPotVoltageADC Before Delay:    ");
    Serial.println(linearPotVoltageADC);
    delay(5000);
    linearPotVoltageADC = analogRead(linearPotPin);
    Serial.print("Bottom linearPotVoltageADC After Delay:     ");
    Serial.println(linearPotVoltageADC);
    delay(5000);


    // Move Jaw Up
    jawServo.writeMicroseconds(servoJawUp);

    while (linearPotVoltageADC < jawClosedPotVoltageADC)
    {    
      linearPotVoltageADC = analogRead(linearPotPin);
      
      if (printTimer.isExpired()){
        Serial.print("linearPotVoltageADC:     ");
        Serial.println(linearPotVoltageADC);

        //if servo is stopped (potentiometer reads same voltage as previous value), jaw returns to open position
        if (linearPotVoltageADC < previous1PotVoltage - 3)
        {
          jawServo.writeMicroseconds(servoJawDown);

          while (linearPotVoltageADC > jawOpenPotVoltageADC)
          {
            linearPotVoltageADC = analogRead(linearPotPin);
            if (printTimer.isExpired()){
              Serial.print("linearPotVoltageADC:    ");
              Serial.println(linearPotVoltageADC);
            }
          }
          break;
        }
        previous1PotVoltage = analogRead(linearPotPin);
      }
    
    }
  
    // Stop servo onced jaw is closed
    jawServo.writeMicroseconds(servoStop);

    linearPotVoltageADC = analogRead(linearPotPin);
    Serial.print("Final linearPotVoltageADC Before Delay:      ");
    Serial.println(linearPotVoltageADC);
    delay(5000);
    linearPotVoltageADC = analogRead(linearPotPin);
    Serial.print("Final linearPotVoltageADC After Delay:      ");
    Serial.println(linearPotVoltageADC);
    delay(5000);
  
  }
  
  // Stop servo
  jawServo.writeMicroseconds(servoStop);
}

// Moves motor until 4-bar linkage and gripper are positioned to grab the 45 degree plate
void goToHighPlate() {
  gripperServo.writeMicroseconds(servoOpenedPositionMS);
  Serial.println("TEST");
  delay(1000);
  motor.moveTo(firstSpot);
  delay(3000);
  gripperServo.writeMicroseconds(servoClosedPositionMS);
  delay(1000);
  motor.moveTo(firstSpot-300);
  delay(3000);
  motor.moveTo(firstSpot);
  delay(2000);
  gripperServo.writeMicroseconds(servoOpenedPositionMS);
  // motor.moveTo(0);
}

// Moves motor until 4-bar linkage and gripper are positioned to grab the 60 degree plate
void goToLowPlate() {
  gripperServo.writeMicroseconds(servoOpenedPositionMS);
  delay(1000);
  motor.moveTo(secondSpot);
  delay(3000);
  gripperServo.writeMicroseconds(servoClosedPositionMS);
  delay(1000);
  motor.moveTo(secondSpot+1000);
  delay(3000);
  motor.moveTo(secondSpot);
  delay(2000);
  gripperServo.writeMicroseconds(servoOpenedPositionMS);
}

void goToPositions() {
  // if A button is pressed, go to encoded 0 position, also the position at which the plate is grabbed off the block
  if(buttonA.isPressed()) {

    motor.moveTo(0);
  }
  //if B is presssed, go to hight plate position
  if(buttonB.isPressed()) {

    goToHighPlate();

    
  }

  //if C is pressed, go to low plate position
  if(buttonC.isPressed()) {
    goToLowPlate();
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

void checkRemote(){
  int16_t code = decoder.getKeyCode();
  switch (code)
  {
  case remotePlayPause:
    // resetMode();
   paused = true;
    break;
  
  case remoteVolPlus:
    paused = false;
    break;
  }
  Serial.println(paused);
}

void loop()
{
  // check the battery
    batteryCheck();

  // Code to test Remote
    checkRemote();  

  // Reset mode for troubleshooting
  //   resetMode();

  // Enters linear gripper mode to test and use linear gripper functionality
  // linearGripper();

  // run through deadBandFunctions
  // deadbandFunctions(true);

  // positional code
  // goToPositions();

}





