#include <Arduino.h>
#include "RemoteControl.h"
#include "IRdecoder.h"

IRDecoder irdecoder;

RemoteControl::RemoteControl(uint8_t pin) {
    irdecoder = new IRDecoder(pin);
}

void RemoteControl::setup() {
    return;
}