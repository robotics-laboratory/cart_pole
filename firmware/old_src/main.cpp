#include <Arduino.h>

#include "encoder.h"
#include "accelerometer.h"
#include "protocol_processor.h"
#include "stepper.h"

const int DEBUG_LED_PIN = 2;
const int DEBUG_PULSE_US = 10;

void setup() {
    pinMode(DEBUG_LED_PIN, OUTPUT);
}

void debug_pulse() {
    digitalWrite(DEBUG_LED_PIN, HIGH);
    delayMicroseconds(DEBUG_PULSE_US);
    digitalWrite(DEBUG_LED_PIN, LOW);
    delayMicroseconds(DEBUG_PULSE_US);
}

void loop() {
    GetStepper().Poll();
    debug_pulse();
    GetPoleEncoder().Poll();
    debug_pulse();
    // GetMotorEncoder().Poll();
    // debug_pulse();
    // GetAccelerometer().Poll();
    // debug_pulse();
    GetProtocolProcessor().Poll();
    debug_pulse();

    // delayMicroseconds(20);
    // if (Serial.available()) {
    //     char in = Serial.read();
    //     Serial.write(in);
    //     Serial.flush();
    // }
    // delayMicroseconds(10);
}
