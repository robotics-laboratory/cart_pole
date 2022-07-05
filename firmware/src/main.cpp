#include <Arduino.h>
extern "C" {
#include "TinyFrame.h"
}

#include <pb.h>
#include <pb_decode.h>
#include <pb_encode.h>

#include <stdio.h>
#include <deque>
#include <string>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "protocol.pb.h"

#include <FastAccelStepper.h>
#include <TMCStepper.h>
//--------------------------GLOBAL CONSTANTS--------------------------

const int LED_PIN = 2;

//--------------------------STEPPER CONSTANTS--------------------------

const int TMC_EN = 25;
const int TMC_STEP = 33;
const int TMC_DIR = 32;
const int M0_PIN = 26;
const int M1_PIN = 27;
const int M2_PIN = 16;
const int TMC_STALLGUARD = 39;
const int ENDSTOP_LEFT = 34;
const int ENDSTOP_RIGHT = 35;
const bool INVERSE_ENDSTOPS = false;

const HardwareSerial STEPPER_SERIAL_PORT = Serial2;
const float STEPPER_CURRENT = 2.0;
const int SERIAL_SPEED = 115200;
const int ADDRESS = 0b00;
const float R_SENSE = 0.11;
const int TOFF_VALUE = 5;
const int MICROSTEPS = 16;
const bool REVERSE_STEPPER = false;
const int FULL_STEPS_PER_METER = 1666;
const float HOMING_SPEED = 0.1;
const float HOMING_ACCELERATION = 0.5;

const int METERS_TO_STEPS_MULTIPLIER = MICROSTEPS * FULL_STEPS_PER_METER;
const float LIMITS_EPS = 1e-3;

//--------------------------MAGNETIC ENCODER CONSTANTS--------------------------

const int ENCODER_MAX_VALUE = 4096;  // 12 bit
const unsigned long VELOCITY_DELTA_TIME_MICROS = 20 * 1000;  // 20ms
const float VELOCITY_SMOOTHING_ALPHA = 0.85; // curr = alpha * curr + (1 - alpha) * prev
const float MAX_VELOCITY = 5 * 2 * PI; // rad/s, used to filter spikes
const bool REVERSE = true;
const float ROTATION_CARRY_THRESHOLD = 1.8 * PI;

//--------------------------UART CONSTANTS--------------------------

const auto SERIAL_PORT_NUM = UART_NUM_0;
const int SERIAL_TX_PIN = 1;
const int SERIAL_RX_PIN = 3;
const int SERIAL_BUFFER_SIZE = 512;

//--------------------------I2C CONSTANTS--------------------------

static gpio_num_t I2C_MASTER_SCL = GPIO_NUM_22;
static gpio_num_t I2C_MASTER_SDA = GPIO_NUM_21;
const int i2c_master_freq_hz = 800000;
const int I2C_MASTER_TX_BUF_DISABLE = 0;
const int I2C_MASTER_RX_BUF_DISABLE = 0;
const int ENCODER_ADDR = 0x36;
const int ACK_CHECK_ENABLE = 0x1;
const int ACK_CHECK_DISABLE = 0x0;

//--------------------------GLOBAL OBJECTS/VARIBLES--------------------------
//--------------------------GLOBAL OBJECTS--------------------------

uint8_t buffer[SERIAL_BUFFER_SIZE];
Error errorCode = Error_NEED_RESET;
float fullLengthMeters;
float maxCartX = 0.0;
float maxCartV = 0.5;
float maxCartA = 1.0;
float hwMaxX = 0.0;
float hwMaxV = 10;
float hwMaxA = 10;
float currCartX;
float currCartV;
float currCartA;
float targetCartX;
float targetCartV;
float targetCartA;

//--------------------------STEPPER OBJECTS/VARS--------------------------

FastAccelStepper *fas_stepper_motor = nullptr;
TaskHandle_t HOMING_TASK_HANDLE = nullptr;
bool IS_DONE_HOMING = false;

//--------------------------MAGNETIC ENCODER OBJECTS/VARS--------------------------

float startAngle;
uint8_t magnetStatus;
std::deque<std::pair<unsigned long, float>> magnetHistory;
float prevPoleAngle;
unsigned long prevTime;
float prevPoleVelocity;
float currPoleAngle;
float currPoleVelocity;

//--------------------------TINYFRAME OBJECTS--------------------------

TinyFrame *slave_tf;
TF_Msg msg;

//--------------------------PROTOBUF OBJECTS--------------------------

State state;
Config config;
Target target;

//--------------------------FUNCTIONS--------------------------

//--------------------------MISC FUNCTIONS--------------------------

float randFloat() {
    float f = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    return f;
}

//--------------------------UART FUNCTIONS--------------------------

void idf_uart_init() {
    uart_config_t uart_config;
    uart_config.baud_rate = SERIAL_SPEED;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    ESP_ERROR_CHECK(
        uart_driver_install(SERIAL_PORT_NUM, SERIAL_BUFFER_SIZE, SERIAL_BUFFER_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(SERIAL_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(SERIAL_PORT_NUM, SERIAL_TX_PIN, SERIAL_RX_PIN, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
}

bool idf_uart_available() {
    size_t length = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(SERIAL_PORT_NUM, &length));
    return length > 0;
}

uint8_t idf_uart_read_byte() {
    uint8_t byte = 0;
    int length = uart_read_bytes(SERIAL_PORT_NUM, &byte, 1, 1);
    if (length != 1) ESP_ERROR_CHECK(ESP_FAIL);
    return byte;
}

void idf_uart_write_bytes(const uint8_t *src, size_t length) {
    int length_written = uart_write_bytes(SERIAL_PORT_NUM, (const char *)src, length);
    if (length_written != length) ESP_ERROR_CHECK(ESP_FAIL);
}

//--------------------------I2C FUNCTIONS--------------------------

void idf_i2c_init() {
    i2c_port_t i2c_master_port = I2C_NUM_0;
    i2c_config_t i2c_config;
    i2c_config.mode = I2C_MODE_MASTER;
    i2c_config.sda_io_num = I2C_MASTER_SDA;
    i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE, i2c_config.scl_io_num = I2C_MASTER_SCL,
    i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE, i2c_config.master.clk_speed = 500000;
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE,
                                       I2C_MASTER_TX_BUF_DISABLE, 0));
}

//--------------------------ENCODER FUNCTIONS--------------------------

uint16_t getRawAngle() {
    uint8_t angleHighByte;
    uint8_t angleLowByte;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // Read low byte
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ENCODER_ADDR << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, 0x0D, I2C_MASTER_ACK);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ENCODER_ADDR << 1) | I2C_MASTER_READ, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &angleLowByte, I2C_MASTER_ACK);
    // Read high byte
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ENCODER_ADDR << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, 0x0C, I2C_MASTER_ACK);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ENCODER_ADDR << 1) | I2C_MASTER_READ, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &angleHighByte, I2C_MASTER_ACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 0 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ((uint16_t)angleHighByte << 8) | (uint16_t)angleLowByte;
}

float getAbsoluteDegreeAngle(uint16_t rawAngle) { return startAngle = rawAngle * (360.0 / 4096); }

float getCorrectedRadAngle(uint16_t rawAngle) {
    float tempDegAngle = rawAngle * (360.0 / 4096);
    tempDegAngle = tempDegAngle - startAngle;
    tempDegAngle = 360.0 - abs(tempDegAngle);
    // Return angle in radians
    return tempDegAngle * (PI / 180);
}

void checkMagnetPresence() {
    while ((magnetStatus & 32) !=
           32)  // while the magnet is not adjusted to the proper distance - 32: MD = 1
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (ENCODER_ADDR << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
        i2c_master_write_byte(cmd, 0x0B, I2C_MASTER_ACK);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (ENCODER_ADDR << 1) | I2C_MASTER_READ, I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, &magnetStatus, I2C_MASTER_ACK);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_NUM_0, cmd, 0 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
    }
    // Spit out "MAG" once the magnet is found
    const uint8_t bufir[] = {77, 65, 71};
     idf_uart_write_bytes(bufir, sizeof(bufir));
}

void magneticEncoderPoll() {
    unsigned long currTime = micros();
    float rawAngle = getRawAngle();
    currPoleAngle = rawAngle / ENCODER_MAX_VALUE * 2 * PI - startAngle;
    if (currPoleAngle < 0) currPoleAngle += 2 * PI;
    if (REVERSE) currPoleAngle = 2 * PI - currPoleAngle;

    float momentaryDeltaAngle = currPoleAngle - prevPoleAngle;
    float momentaryDeltaTime = static_cast<float>(currTime - prevTime) / 1000000;
    if (std::abs(momentaryDeltaAngle) > ROTATION_CARRY_THRESHOLD) {
        // Fix angle delta if we're transitioning between 0 -> 2PI or 2PI -> 0
        momentaryDeltaAngle -= (momentaryDeltaAngle > 0) ? 2 * PI : -2 * PI;
    }
    float momentaryVelocity = momentaryDeltaAngle / momentaryDeltaTime;
    if (std::abs(momentaryVelocity) > MAX_VELOCITY and !magnetHistory.empty()) {
        // Spike detected!
        return;
    }

    unsigned long prevHistoryTime = currTime - 1;
    float prevHistoryAngle = currPoleAngle;
    if (magnetHistory.empty()) {
        //ProtocolProcessor& P = GetProtocolProcessor();
        //P.Log("Encoder deque is empty!");
    }
    while (!magnetHistory.empty()) {
        auto item = magnetHistory.front();
        prevHistoryTime = item.first;
        prevHistoryAngle = item.second;
        if (currTime - item.first <= VELOCITY_DELTA_TIME_MICROS) break;
        magnetHistory.pop_front();
    }

    float deltaAngle = currPoleAngle - prevHistoryAngle;
    float deltaTime = static_cast<float>(currTime - prevHistoryTime) / 1000000;
    if (std::abs(deltaAngle) > ROTATION_CARRY_THRESHOLD) {
        // Fix angle delta if we're transitioning between 0 -> 2PI or 2PI -> 0
        deltaAngle -= (deltaAngle > 0) ? 2 * PI : -2 * PI;
    }
    currPoleVelocity =
        VELOCITY_SMOOTHING_ALPHA * (deltaAngle / deltaTime)
        + (1 - VELOCITY_SMOOTHING_ALPHA) *prevPoleVelocity;
    prevTime = currTime;
    prevPoleAngle = currPoleAngle;
    prevPoleVelocity = currPoleVelocity;
    magnetHistory.emplace_back(currTime, currPoleAngle);
}

//--------------------------STEPPER FUNCTIONS--------------------------

void stepperMotorInit(){
    // TODO: probably fix init

    pinMode(TMC_EN, OUTPUT);
    pinMode(TMC_STEP, OUTPUT);
    pinMode(TMC_DIR, OUTPUT);
    pinMode(TMC_STALLGUARD, INPUT);
    pinMode(ENDSTOP_LEFT, INPUT);
    pinMode(ENDSTOP_RIGHT, INPUT);

    digitalWrite(TMC_EN, HIGH);
    delay(10);
    pinMode(M0_PIN, OUTPUT);
    pinMode(M1_PIN, OUTPUT);
    pinMode(M2_PIN, OUTPUT);

    // https://github.com/arkypita/LaserGRBL/issues/716
    digitalWrite(M0_PIN, LOW);
    digitalWrite(M1_PIN, LOW);
    digitalWrite(M2_PIN, LOW);
    if (MICROSTEPS == 2) {
        digitalWrite(M0_PIN, HIGH);
    } else if (MICROSTEPS == 4) {
        digitalWrite(M1_PIN, HIGH);
    } else if (MICROSTEPS == 8) {
        digitalWrite(M0_PIN, HIGH);
        digitalWrite(M1_PIN, HIGH);
    } else if (MICROSTEPS == 16) {
        digitalWrite(M2_PIN, HIGH);
    } else if (MICROSTEPS == 32) {
        digitalWrite(M0_PIN, HIGH);
        digitalWrite(M2_PIN, HIGH);
    }

    // Init FastAccelStepper
    fas_engine->init();
    fas_stepper_motor = fas_engine->stepperConnectToPin(TMC_STEP);
    assert(fas_stepper_motor != NULL);
    fas_stepper_motor->setDirectionPin(TMC_DIR, REVERSE_STEPPER);
}

float getCurrentStepperPosition(){
    int pos_steps = fas_stepper_motor->getCurrentPosition();
    return static_cast<float>(pos_steps) / METERS_TO_STEPS_MULTIPLIER - fullLengthMeters / 2;
}

float getCurrentStepperVelocity(){
    int vel_steps_per_ms = fas_stepper_motor->getCurrentSpeedInMilliHz();
    return static_cast<float>(vel_steps_per_ms) / METERS_TO_STEPS_MULTIPLIER / 1000;
}

float getCurrentStepperAcceleration(){
    int steps_per_ss = fas_stepper_motor->getCurrentAcceleration();
    return static_cast<float>(steps_per_ss) / METERS_TO_STEPS_MULTIPLIER;
}

void ForceStop() {
    fas_stepper_motor->forceStopAndNewPosition(fas_stepper_motor->getCurrentPosition());
}

void Enable() {
    digitalWrite(TMC_EN, LOW);
}

void Disable() {
    ForceStop();
    digitalWrite(TMC_EN, HIGH);
}

void CheckStallGuard() {
    if (digitalRead(TMC_STALLGUARD)) {
        //SetError(Error::MOTOR_STALLED, "Motor stall detected");
    }
}

void CheckEndstops() {
    if (INVERSE_ENDSTOPS ^ digitalRead(ENDSTOP_LEFT) ||
        INVERSE_ENDSTOPS ^ digitalRead(ENDSTOP_RIGHT)) {
        //SetError(Error::ENDSTOP_HIT, "Endstop hit detected");
    }
}

void CheckLimits() {
    //Globals &G = GetGlobals();
    if (std::abs(currCartX) > maxCartX + LIMITS_EPS)
    {

    }
        //return SetError(Error::X_OVERFLOW, "X overflow detected");
        
    if (std::abs(currCartV) > maxCartV + LIMITS_EPS)
    {

    }
        //return SetError(Error::V_OVERFLOW, "V overflow detected");
        
    if (std::abs(currCartA) > maxCartA + LIMITS_EPS)
    {

    }
        //return SetError(Error::A_OVERFLOW, "A overflow detected");

}

void stepperMotorPoll(){
    // TODO: This works???
    if (errorCode == Error_NO_ERROR){
        CheckStallGuard();
        CheckEndstops();
        CheckLimits();
    }
}

void SetSpeed(float value) {
    uint32_t speed_hz = static_cast<uint32_t>(value * METERS_TO_STEPS_MULTIPLIER);
    fas_stepper_motor->setSpeedInHz(speed_hz);

}

void SetAcceleration(float value) {
    uint32_t steps_per_ss = static_cast<uint32_t>(value * METERS_TO_STEPS_MULTIPLIER);
    fas_stepper_motor->setAcceleration(steps_per_ss);
}

void SetTargetPosition(float value) {
    
    int pos_steps =
        static_cast<int>((value + fullLengthMeters/ 2) * METERS_TO_STEPS_MULTIPLIER);
    fas_stepper_motor->moveTo(pos_steps);
}

void SetTargetAcceleration(float value) {
    SetSpeed(maxCartV);

    int32_t steps_per_ss = static_cast<int32_t>(value * METERS_TO_STEPS_MULTIPLIER);
    fas_stepper_motor->moveByAcceleration(steps_per_ss, true);
}

void SetError(Error err, std::string what) {
    errorCode = err; 
    Disable();

    //std::stringstream ss;
    //ss << "CURR X: " << G.curr_x << " ";
    //ss << "CURR V: " << G.curr_v << " ";
    //ss << "CURR A: " << G.curr_a;
    //P.Log(ss.str());
}

void Homing() {

    ForceStop();
    Enable();
    SetSpeed(HOMING_SPEED);
    SetAcceleration(HOMING_ACCELERATION);

    // RUN LEFT
    fas_stepper_motor->runBackward();
    while (!(INVERSE_ENDSTOPS ^ digitalRead(ENDSTOP_LEFT))) {
    }

    ForceStop();
    fas_stepper_motor->setCurrentPosition(0);
    delay(50);

    // RUN RIGHT
    fas_stepper_motor->runForward();
    while (!(INVERSE_ENDSTOPS ^ digitalRead(ENDSTOP_RIGHT))) {
    }

    ForceStop();
    int delta_steps = fas_stepper_motor->getCurrentPosition();
    fas_stepper_motor->setCurrentPosition(delta_steps);
    delay(50);

    // GOTO CENTER
    fas_stepper_motor->moveTo(delta_steps / 2);
    while (fas_stepper_motor->isRunning()) {
    }

    fullLengthMeters = static_cast<float>(delta_steps) / METERS_TO_STEPS_MULTIPLIER;
    maxCartX= fullLengthMeters / 2;

    errorCode = Error_NO_ERROR;

    // std::stringstream stream;
    // stream << std::fixed << std::setprecision(5) << "Full length: " << delta_steps << " steps"
    //        << G.full_length_meters << " meters";
    // P.Log(stream.str());
    // stream = std::stringstream{};
    // stream << std::fixed << std::setprecision(5) << "Valid X range: " << -G.hw_max_x << " ... "
    //        << G.hw_max_x;
    // P.Log(stream.str());
}

void homingTask(void *) {

    Homing();

    IS_DONE_HOMING = true;
    vTaskDelete(HOMING_TASK_HANDLE);
    while (true);
}

void AsyncHoming() {

    IS_DONE_HOMING = false;
    BaseType_t ret =
        xTaskCreate(homingTask, "homing", 8192, nullptr, tskIDLE_PRIORITY, &HOMING_TASK_HANDLE);

    if (ret != pdPASS) {
        //P.Error("Async Homing Failure");
    }
    if (ret == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) {
        //P.Log("Could not allocate required memory");
    }
}

bool IsDoneHoming() { return IS_DONE_HOMING; }

//--------------------------PROTOBUF FUNCTIONS--------------------------

void encodeState(State &state, uint8_t buffer[]) {
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, SERIAL_BUFFER_SIZE);
    pb_encode(&stream, State_fields, &state);
}

void decodeTarget(Target &target, uint8_t buffer[]) {
    pb_istream_t stream = pb_istream_from_buffer(buffer, SERIAL_BUFFER_SIZE);
    pb_decode(&stream, Target_fields, &target);
}

void decodeConfig(Config &config, uint8_t buffer[]) {
    pb_istream_t stream = pb_istream_from_buffer(buffer, SERIAL_BUFFER_SIZE);
    pb_decode(&stream, Config_fields, &target);
}

//--------------------------TINYFRAME FUNCTIONS--------------------------

void TF_WriteImpl(TinyFrame *tf, const uint8_t *buff, uint32_t len) {
    idf_uart_write_bytes(buff, len);
}

TF_Result targetListener(TinyFrame *tf, TF_Msg *msg) {
    
    decodeTarget(target, (unsigned char *)msg->data);
    target.target_cart_x;
    target.target_cart_v;
    target.target_cart_a;         
    TF_Msg responseMessage;
    TF_ClearMsg(&responseMessage);
    responseMessage.type = MessageType_TARGETSTATE;
    responseMessage.len = 512;

    state.curr_cart_a = currCartA;
    state.curr_cart_v = currCartV;
    state.curr_cart_x = currCartX;
    // TODO: start using real IMU readings
    state.curr_imu_a = currCartA;
    state.curr_pole_v = currPoleVelocity;
    state.curr_pole_x = currPoleAngle;

    const uint8_t bufir[] = {77, 65, 71};
    idf_uart_write_bytes(bufir, sizeof(bufir));
    // State has 6 float fields, target has 3 float fields

    encodeState(state, buffer);
    responseMessage.data = (unsigned char *)buffer;
    TF_Respond(tf, &responseMessage);
    return TF_STAY;
}

TF_Result dryRunListener(TinyFrame *tf, TF_Msg *msg) {
    TF_Msg responseMessage;
    TF_ClearMsg(&responseMessage);
    responseMessage.type = MessageType_TARGETSTATE;
    responseMessage.len = 512;
    state.curr_cart_a = currCartA;
    state.curr_cart_v = currCartV;
    state.curr_cart_x = currCartX;
    // TODO: start using real IMU readings
    state.curr_imu_a = currCartA;
    state.curr_pole_v = currPoleVelocity;
    state.curr_pole_x = currPoleAngle;
    encodeState(state,buffer);
    responseMessage.data = (unsigned char *)buffer;
    TF_Respond(tf, &responseMessage);
    return TF_STAY;
}

TF_Result configListener(TinyFrame *tf, TF_Msg *msg) {
    errorCode = Error_NO_ERROR;
    decodeConfig(config,(unsigned char *)msg->data);
    hwMaxX = config.hw_max_x;
    hwMaxV = config.hw_max_v;
    hwMaxA = config.hw_max_a;
    maxCartX = config.max_cart_x;
    maxCartV = config.max_cart_v;
    maxCartA = config.max_cart_a; 
    return TF_STAY;
}

void tfRead(){
    while (idf_uart_available()) {
        uint8_t b = idf_uart_read_byte();
        TF_AcceptChar(slave_tf, b);
    }
}

//--------------------------MAIN--------------------------

void setup() {
    idf_uart_init();
    idf_i2c_init();
    slave_tf = TF_Init(TF_SLAVE);
    TF_ClearMsg(&msg);
    TF_AddTypeListener(slave_tf, MessageType_TARGETSTATE, targetListener);
    TF_AddTypeListener(slave_tf, MessageType_UPDATESTATE, dryRunListener);
    TF_AddTypeListener(slave_tf, MessageType_RESET, configListener);
    // TODO: get start angle
    while(errorCode == Error_NEED_RESET)
    {
        tfRead();
    }

}

void loop() {
    stepperMotorPoll();
    magneticEncoderPoll();
    tfRead();
    // TODO: Нужна проверка операций на ESP на атомарность, лок на нужные (обновление угла/скорости)
    // uint16_t rawAngle;
    // float radAngle;
    // TODO: Simplify the way these two functions call eachother
    // rawAngle = getRawAngle();
    // radAngle = getCorrectedRadAngle(rawAngle);
    // Serial.print("HighByte\n");
    // Serial.print(angleHighByte);
    // Serial.print("LowByte\n");
    // Serial.print(angleLowByte);
    // Serial.print(radAngle);
    // Serial.print("\n");
    // idf_uart_write_bytes((uint8_t*)(str.data()), str.size());
    // idf_uart_write_bytes(&dataLow, 1);
    // idf_uart_write_bytes(&dataHigh, 1);
    // TF_Send(slave_tf, &msg);
}
