#include "encoder.h"

#include <Arduino.h>

#include <cmath>
#include <iomanip>
#include <sstream>

#include "globals.h"
#include "protocol_processor.h"

namespace {
const int ENCODER_MAX_VALUE = 4096;  // 12 bit
const unsigned long VELOCITY_DELTA_TIME_MICROS = 20 * 1000;  // 20ms
const float VELOCITY_SMOOTHING_ALPHA = 0.85; // curr = alpha * curr + (1 - alpha) * prev
const float MAX_VELOCITY = 5 * 2 * PI; // rad/s, used to filter spikes
const bool REVERSE = true;
const float ROTATION_CARRY_THRESHOLD = 1.8 * PI;
const int SECONDARY_ENCODER_SDA = 13;
const int SECONDARY_ENCODER_SCL = 32;
}  // namespace

Encoder::Encoder(TwoWire *wire, float *x_ref, float *v_ref, float zero_angle)
    : wire(wire), x_ref(x_ref), v_ref(v_ref), zero_angle(zero_angle), prevAngle(0), prevTime(0), prevVelocity(0), history() {
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
    unsigned long currTime = micros();
    float rawAngle = getRawAngle();
    float currAngle = rawAngle / ENCODER_MAX_VALUE * 2 * PI - zero_angle;
    if (currAngle < 0) currAngle += 2 * PI;
    if (REVERSE) currAngle = 2 * PI - currAngle;

    float momentaryDeltaAngle = currAngle - prevAngle;
    float momentaryDeltaTime = static_cast<float>(currTime - prevTime) / 1000000;
    if (std::abs(momentaryDeltaAngle) > ROTATION_CARRY_THRESHOLD) {
        // Fix angle delta if we're transitioning between 0 -> 2PI or 2PI -> 0
        momentaryDeltaAngle -= (momentaryDeltaAngle > 0) ? 2 * PI : -2 * PI;
    }
    float momentaryVelocity = momentaryDeltaAngle / momentaryDeltaTime;
    if (std::abs(momentaryVelocity) > MAX_VELOCITY and !history.empty()) {
        // Spike detected!
        return;
    }

    unsigned long prevHistoryTime = currTime - 1;
    float prevHistoryAngle = currAngle;
    if (history.empty()) {
        ProtocolProcessor& P = GetProtocolProcessor();
        P.Log("Encoder deque is empty!");
    }
    while (!history.empty()) {
        auto item = history.front();
        prevHistoryTime = item.first;
        prevHistoryAngle = item.second;
        if (currTime - item.first <= VELOCITY_DELTA_TIME_MICROS) break;
        history.pop_front();
    }

    float deltaAngle = currAngle - prevHistoryAngle;
    float deltaTime = static_cast<float>(currTime - prevHistoryTime) / 1000000;
    if (std::abs(deltaAngle) > ROTATION_CARRY_THRESHOLD) {
        // Fix angle delta if we're transitioning between 0 -> 2PI or 2PI -> 0
        deltaAngle -= (deltaAngle > 0) ? 2 * PI : -2 * PI;
    }
    float currVelocity =
        VELOCITY_SMOOTHING_ALPHA * (deltaAngle / deltaTime)
        + (1 - VELOCITY_SMOOTHING_ALPHA) * prevVelocity;

    *x_ref = currAngle;
    *v_ref = currVelocity;
    prevTime = currTime;
    prevAngle = currAngle;
    prevVelocity = currVelocity;
    history.emplace_back(currTime, currAngle);
}

Encoder &GetPoleEncoder() {
    static TwoWire *primaryWire = []() {
        Wire.begin(-1, -1, 1000000);
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
