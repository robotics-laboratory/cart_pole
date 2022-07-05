#pragma once
#include <Arduino.h>

#include <string>
#include <vector>

enum class FieldID {
    max_x,
    max_v,
    max_a,
    hw_max_x,
    hw_max_v,
    hw_max_a,
    clamp_x,
    clamp_v,
    clamp_a,
    curr_x,
    trgt_x,
    curr_v,
    trgt_v,
    curr_a,
    trgt_a,
    pole_x,
    pole_v,
    errcode,
    imu_a,
    motor_x,
    motor_v,
    debug_led,
};

enum class Error {
    NO_ERROR = 0,       // This is fine
    NEED_RESET = 1,     // Homing procedure is required ("reset" command)
    X_OVERFLOW = 2,     // X overshoot detected, motion is disabled
    V_OVERFLOW = 3,     // V overshoot detected, motion is disabled
    A_OVERFLOW = 4,     // A overshoot detected, motion is disabled
    MOTOR_STALLED = 5,  // TMC StallGuard is triggered (stepper missed steps)
    ENDSTOP_HIT = 6,    // One of endstops is triggered during movement
};

struct Globals {
    /* CONFIG */
    float max_x;     // [m] Absolute max cart position
    float max_v;     // [m/s] Absolute max cart velocity
    float max_a;     // [m/s^2] Absolute max cart acceleration
    float hw_max_x;  // [m] Absolute max hardware-allowed position
    float hw_max_v;  // [m/s] Absolute max hardware-allowed velocity
    float hw_max_a;  // [m/s^2] Absolute max hardware-allowed acceleration
    bool clamp_x;    // Clamp X to allowed range instead of raising error
    bool clamp_v;    // Clamp V to allowed range instead of raising error
    bool clamp_a;    // Clamp A to allowed range instead of raising error
    bool debug_led;  // Debug LED pin to indicate start of session
    /* STATE */
    float curr_x;    // [m] Current cart position
    float curr_v;    // [m/s] Current cart velocity
    float curr_a;    // [m/s^2] Current cart acceleration
    float pole_x;    // [rad] Current pole angle
    float pole_v;    // [rad/s] Current pole angular velocity
    Error errcode;   // Current error code
    float imu_a;     // [m/s^2] Cart acceleration measured by IMU
    float motor_x;   // [rad] Rotation of the motor shaft (secondary encoder)
    float motor_v;   // [rad/s] Velocity of the motor shaft (secondary encoder)
    /* TARGET */
    float trgt_x;    // [m] Target cart position
    float trgt_v;    // [m/s] Target cart velocity
    float trgt_a;    // [m/s^2] Target cart acceleration
    /* MISC */
    float full_length_meters;  // [m] Total Length as determined during homing

    std::string Get(const std::string &group, const std::string &key) const;
    std::vector<std::pair<std::string, std::string>> Get(const std::string &group) const;
    void Prepare(const std::string &group, const std::string &key, const std::string &value);
    void Commit(const std::string &group, const std::string &key);
    void Reset();
};

Globals &GetGlobals();

template <typename T, FieldID F>
T validateField(T value);

template <typename T, FieldID F>
void updateField(T value);
