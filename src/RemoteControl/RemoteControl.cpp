#include <Arduino.h>
#include "RemoteControl.h"
#include "IRdecoder.h"

IRDecoder irdecoder(-1);


void RemoteControl::setup() {
    irdecoder = IRDecoder(pin);
    irdecoder.init();
    return;
}

void RemoteControl::test() {
    Serial.println(pin);
}