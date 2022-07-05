#pragma once
#include <Wire.h>

#include <deque>

#include "globals.h"

class Encoder {
    TwoWire *wire;
    float *x_ref;
    float *v_ref;
    float zero_angle;
    float prevAngle;
    unsigned long prevTime;
    float prevVelocity;
    std::deque<std::pair<unsigned long, float>> history;

public:
    Encoder(TwoWire *wire, float *x_ref, float *v_ref, float zero_angle);

    void Poll();

private:
    static const int AS5600Address = 0x36;
    static const byte RAWANGLEAddressMSB = 0x0C;
    static const byte RAWANGLEAddressLSB = 0x0D;

    float getRawAngle();
    uint8_t getRegister(byte reg);
    uint16_t getRegisters2(byte regMSB, byte regLSB);
};

Encoder &GetPoleEncoder();
Encoder &GetMotorEncoder();
