#include <Arduino.h>
#include "RemoteControl.h"
#include "IRdecoder.h"


// #define maxNumberOfFunctions 20

IRDecoder irdecoder(-1);


// setup the remote Control 
void RemoteControl::setup() {
    irdecoder = IRDecoder(pin);
    irdecoder.init();
    return;
}

void RemoteControl::test() {
    Serial.println(pin);
}

// Used to define a function that should continuously run when the given button is pressed
void RemoteControl::running(Func func, uint8_t remoteButton) {
    if(currentFunctionCount >= maxNumberOfFunctions) { 
        Serial.println("ERROR: TOO MANY FUNCTIONS DELCARED");
        return;
    }
    functions[currentFunctionCount] = func;
    remoteButtons[currentFunctionCount] = remoteButton;

    currentFunctionCount++;
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
    for(int i = 0;i < currentFunctionCount; i++) {
        if(code == remoteButtons[i]) {
            activeFunctions[i] = !activeFunctions;
            Serial.print(i);
            Serial.println(" ACTIVE!");
        }
        
        if(activeFunctions[i]) {
            functions[i]();
        }
    }
}