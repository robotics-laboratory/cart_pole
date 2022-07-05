#pragma once
#include <MPU6050.h>

class Accelerometer {
    MPU6050 mpu;

public:
    Accelerometer();

    void Poll();
};

Accelerometer &GetAccelerometer();
