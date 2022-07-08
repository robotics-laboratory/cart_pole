#include "protocol.pb-c.h"
#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctime>
#include <tuple>
#include <string>
hw_timer_t* timer = NULL;
State state = STATE__INIT;
uint8_t * buf;

State stateBuilder() {
    state.curr_cart_x = static_cast<float> (rand());
    state.curr_cart_v = static_cast<float> (rand());
    state.curr_cart_a = static_cast<float> (rand());
    state.curr_imu_a = static_cast<float> (rand());
    state.curr_pole_v =static_cast<float> (rand());
    state.curr_pole_x = static_cast<float> (rand());
}


std::tuple<int,int> benchmark() {
    int encodeTimeMicros;
    int decodeTimeMicros;
    stateBuilder();
    int buffer_length = state__get_packed_size(&state);
    buf = new uint8_t[buffer_length];
    timerStart(timer);
    state__pack(&state, buf);
    timerStop(timer);
    //timer spits out microseconds because 1MHz APB clock freq
    encodeTimeMicros = timerRead(timer);
    //free(buf);
    timerRestart(timer);

    buf = new uint8_t[buffer_length];
    State * msg;
    timerStart(timer);
    msg = state__unpack(NULL, buffer_length, buf);
    timerStop(timer);
    state__free_unpacked(msg, NULL);
    decodeTimeMicros = timerRead(timer);
    timerRestart(timer);
    
    return std::make_tuple(encodeTimeMicros,decodeTimeMicros);
}

void setup() {
    Serial.begin(115200);
    //1 tick per millisecond
    timer = timerBegin(0,80,true);
}

void loop() {

    for (int runNumber = 0; runNumber < 10000; runNumber++) {
        std::tuple <int, int> tuple= benchmark();
        Serial.print("Run number ");
        Serial.println(runNumber);
        Serial.print("Encode time ");
        Serial.println(std::get<0>(tuple));
        Serial.print("Decode time ");
        Serial.println(std::get<1>(tuple));
    }
}
