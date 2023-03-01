#pragma once

#include <Arduino.h>

const uint16_t remoteAddressByte0 = 0x00;
const uint16_t remoteAddressByte1 = 0xBF;

const uint8_t remoteVolMinus = 0x00;
const uint8_t remotePlayPause = 0x01;
const uint8_t remoteVolPlus = 0x02;

const uint8_t remoteSetup = 0x04;
const uint8_t remoteUp = 0x05;
const uint8_t remoteStopMode = 0x06;

const uint8_t remoteLeft = 0x08;
const uint8_t remoteEnterSave = 0x09;
const uint8_t remoteRight = 0x0A;

const uint8_t remote0 = 0x0C;
const uint8_t remoteDown = 0x0D;
const uint8_t remoteBack = 0x0E;

const uint8_t remote1 = 0x10;
const uint8_t remote2 = 0x11;
const uint8_t remote3 = 0x12;

const uint8_t remote4 = 0x14;
const uint8_t remote5 = 0x15;
const uint8_t remote6 = 0x16;

const uint8_t remote7 = 0x18;
const uint8_t remote8 = 0x19;
const uint8_t remote9 = 0x1A;

const uint8_t lineFollowingConst = 0x1B;
const uint8_t openGripperConst = 0x1C;
const uint8_t closeGripperConst = 0x1D;
const uint8_t raiseSlightlyConst = 0x1E;
const uint8_t takeOffHighPlateConst = 0x1F;
const uint8_t takeOffLowPlateConst = 0x21;
const uint8_t fourBarLowConst = 0x22;
const uint8_t lowerSlightlyConst = 0x23;
const uint8_t calebFunctionConst = 0x24;
const uint8_t rotateLeftUntilLineConst = 0x25;
const uint8_t rotateRightUntilLineConst = 0x26;
const uint8_t forwardUntilLineConst = 0x27;
const uint8_t forwardFunctConst = 0x28;