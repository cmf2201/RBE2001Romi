#include <Arduino.h>
#include "RemoteControl.h"
#include "IRdecoder.h"


// #define maxNumberOfFunctions 20

IRDecoder decoder(-1);


// setup the remote Control 
void RemoteControl::setup() {
    decoder = IRDecoder(pin);
    decoder.init();
    return;
}

void RemoteControl::test() {
    Serial.println(pin);
}

// Defines a function that should continuously run when the given button is pressed
void RemoteControl::toggleFunc(Func func, uint8_t remoteButton) {
    if(currentFunctionCount >= maxNumberOfFunctions) { 
        Serial.println("ERROR: TOO MANY FUNCTIONS DELCARED");
        return;
    }
    functions[currentFunctionCount] = func;
    remoteButtons[currentFunctionCount] = remoteButton;

    currentFunctionCount++;
}

// Defines a function that should run once when a given button is pressed
void RemoteControl::onPress(Func func, uint8_t remoteButton) {
    if(currentFunctionCount >= maxNumberOfFunctions) { 
        Serial.println("ERROR: TOO MANY FUNCTIONS DELCARED");
        return;
    }

    runOnce[currentFunctionCount] = true;
    toggleFunc(func,remoteButton);
}

void RemoteControl::eStop(Func func, uint8_t remoteButton) {
    if(currentFunctionCount >= maxNumberOfFunctions) { 
        Serial.println("ERROR: TOO MANY FUNCTIONS DELCARED");
        return;
    }
    eStopIndex = currentFunctionCount;
    onPress(func,remoteButton);
}

//run all defined functions so far
void RemoteControl::runCurrentFunctions() {
    for(int i = 0; i < currentFunctionCount; i++) {
        functions[i]();
    }
}


//check if the remote is being pressed for any functions, and runs functions that are active
void RemoteControl::checkRemoteButtons() {
    int16_t code = decoder.getKeyCode();
    if(!eStopped) {
        for(int i = 0; i < currentFunctionCount; i++) {
            if(code == remoteButtons[i]) {
                if(runOnce[i]){
                    functions[i]();
                    if(eStopIndex == i) {
                        Serial.println("ESTOP ENABLED");
                        eStopped = true;
                        for(int i2 = 0; i2 < currentFunctionCount; i2++) {
                            activeFunctions[i2] = false;
                        }
                    }
                } else {
                    activeFunctions[i] = !activeFunctions[i];
                }
            }

            if(activeFunctions[i]) {
                functions[i]();         
            }
        }
    } else {
        if(code == remoteButtons[eStopIndex]) {
            Serial.println("ESTOP DISABLED");
            eStopped = false;
        }
        for(int i = 0; i < currentFunctionCount; i++) {
            if(activeFunctions[i]) {
                Serial.println("ERROR");       
            }
        }
        
    }
}