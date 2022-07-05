#include "accelerometer.h"

#include <Wire.h>

#include <sstream>

#include "globals.h"
#include "protocol_processor.h"

namespace {
const int SENSITIVITY = MPU6050_ACCEL_FS_2;  // 16384 LSB / g
const float G_ACCELERATION = 9.81509;

float convertToG(int16_t lsb, int sensitivity = SENSITIVITY) {
    switch (sensitivity) {
        case MPU6050_ACCEL_FS_2:
            return static_cast<float>(lsb) / 16384;
        case MPU6050_ACCEL_FS_4:
            return static_cast<float>(lsb) / 8192;
        case MPU6050_ACCEL_FS_8:
            return static_cast<float>(lsb) / 4096;
        case MPU6050_ACCEL_FS_16:
            return static_cast<float>(lsb) / 2048;
    }
    throw std::runtime_error{"Unknown sensitivity in lsb to G conversion"};
}
}  // namespace

Accelerometer::Accelerometer() : mpu() {
    ProtocolProcessor &P = GetProtocolProcessor();

    Wire.begin();
    mpu.initialize();

    if (mpu.testConnection()) {
        P.Log("Accelerometer connection established");
    } else {
        P.Error("Failed to initialize accelerometer");
        throw std::runtime_error{"Failed to initialize accelerometer"};
    }
    mpu.setFullScaleAccelRange(SENSITIVITY);
}

void Accelerometer::Poll() {
    Globals &G = GetGlobals();

    int16_t y = mpu.getAccelerationY();
    G.imu_a = convertToG(y) / G_ACCELERATION;
}

Accelerometer &GetAccelerometer() {
    static Accelerometer A{};
    return A;
}