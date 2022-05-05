#pragma once
#include <FastAccelStepper.h>
#include <TMCStepper.h>
#undef min
#undef max

#include "globals.h"

class Stepper {
    HardwareSerial tmc_serial_port;
    TMC2209Stepper tmc_driver;
    FastAccelStepperEngine fas_engine;
    FastAccelStepper *fas_stepper;

public:
    Stepper();

    void Poll();
    void CheckStallGuard();
    void CheckEndstops();
    void CheckLimits();
    void SetError(Error err, std::string what);

    void Enable();
    void Disable();

    void ForceStop();
    float GetCurrentPosition();
    float GetCurrentVelocity();
    float GetCurrentAcceleration();

    void SetSpeed(float value);
    void SetAcceleration(float value);

    void SetTargetPosition(float value);
    void SetTargetVelocity(float value);
    void SetTargetAcceleration(float value);

    void Homing();
    void AsyncHoming();
    bool IsDoneHoming();
};

Stepper &GetStepper();