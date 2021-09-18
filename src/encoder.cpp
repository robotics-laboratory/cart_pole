#include "encoder.h"

#include <Arduino.h>

#include <cmath>
#include <iomanip>
#include <sstream>

#include "globals.h"
#include "protocol_processor.h"

namespace {
const int ENCODER_MAX_VALUE = 4096;  // 12 bit
const unsigned long REFRESH_INTERVAL_MILLIS = 10;
const bool REVERSE = true;
const float ROTATION_CARRY_THRESHOLD = 1.8 * PI;
const int SECONDARY_ENCODER_SDA = 13;
const int SECONDARY_ENCODER_SCL = 32;
}  // namespace

Encoder::Encoder(TwoWire *wire, float *x_ref, float *v_ref, float zero_angle)
    : wire(wire), x_ref(x_ref), v_ref(v_ref), prevAngle(0), prevTime(0), zero_angle(zero_angle) {
    ProtocolProcessor &P = GetProtocolProcessor();
    P.Log("Encoder init...");
    // TODO: Check if magnet is detected?
}

float Encoder::getRawAngle() {
    // Borrowed from https://github.com/kanestoboi/AS5600/blob/master/AS5600.cpp
    return getRegisters2(RAWANGLEAddressMSB, RAWANGLEAddressLSB);
}

uint8_t Encoder::getRegister(byte reg) {
    wire->beginTransmission(AS5600Address);
    wire->write(reg);
    wire->endTransmission();
    wire->requestFrom(AS5600Address, 1);

    while (wire->available() == 0) {
    };
    uint8_t _b = wire->read();
    return _b;
}

uint16_t Encoder::getRegisters2(byte regMSB, byte regLSB) {
    uint16_t _hi = getRegister(regMSB);
    uint16_t _lo = getRegister(regLSB);
    return (_hi << 8) | (_lo);
}

void Encoder::Poll() {
    unsigned long currTime = millis();
    if (currTime - prevTime < REFRESH_INTERVAL_MILLIS) return;
    float rawAngle = getRawAngle();
    float currAngle = rawAngle / ENCODER_MAX_VALUE * 2 * PI - zero_angle;
    if (currAngle < 0) currAngle += 2 * PI;
    if (REVERSE) currAngle = 2 * PI - currAngle;

    float deltaAngle = currAngle - prevAngle;
    if (std::abs(deltaAngle) > ROTATION_CARRY_THRESHOLD) {
        // Fix angle delta if we're transitioning between 0 -> 2PI or 2PI -> 0
        deltaAngle -= (deltaAngle > 0) ? 2 * PI : -2 * PI;
    }
    unsigned long deltaTime = currTime - prevTime;
    *v_ref = deltaAngle / deltaTime * 1000;
    *x_ref = currAngle;
    prevAngle = currAngle;
    prevTime = currTime;
}

Encoder &GetPoleEncoder() {
    static TwoWire *primaryWire = []() {
        Wire.begin();
        return &Wire;
    }();
    Globals &G = GetGlobals();
    static Encoder encoder(primaryWire, &G.pole_x, &G.pole_v, 2.9406);
    return encoder;
}

Encoder &GetMotorEncoder() {
    static TwoWire secondaryWire = []() {
        TwoWire secondaryWire(1);
        secondaryWire.begin(SECONDARY_ENCODER_SDA, SECONDARY_ENCODER_SCL);
        return secondaryWire;
    }();
    Globals &G = GetGlobals();
    static Encoder encoder(&secondaryWire, &G.motor_x, &G.motor_v, 0.0);
    return encoder;
}
