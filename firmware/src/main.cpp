#include <Arduino.h>
extern "C" {
#include "TinyFrame.h"
}
#include <pb.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <stdio.h>

#include <string>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "protocol.pb.h"

//--------------------------GLOBAL CONSTANTS--------------------------

const int LED_PIN = 2;

//--------------------------UART CONSTANTS--------------------------

const auto SERIAL_PORT_NUM = UART_NUM_0;
const int SERIAL_TX_PIN = 1;
const int SERIAL_RX_PIN = 3;
const int SERIAL_SPEED = 115200;
const int SERIAL_BUFFER_SIZE = 256;

//--------------------------I2C CONSTANTS--------------------------

static gpio_num_t I2C_MASTER_SCL = GPIO_NUM_22;
static gpio_num_t I2C_MASTER_SDA = GPIO_NUM_21;
const int i2c_master_freq_hz = 800000;
const int I2C_MASTER_TX_BUF_DISABLE = 0;
const int I2C_MASTER_RX_BUF_DISABLE = 0;
const int ENCODER_ADDR = 0x36;
const int ACK_CHECK_ENABLE = 0x1;
const int ACK_CHECK_DISABLE = 0x0;

//--------------------------GLOBAL STATIC OBJECTS/VARIBLES--------------------------

//--------------------------GLOBAL OBJECTS--------------------------

float startAngle;
uint8_t buffer[SERIAL_BUFFER_SIZE];
uint8_t magnetStatus;

//--------------------------TINYFRAME OBJECTS--------------------------

TinyFrame *master_tf;
TinyFrame *slave_tf;
TF_Msg msg;

//--------------------------PROTOBUF OBJECTS--------------------------

GetStateSetTargetResponse getStateSetTargetResponseMessage;
ResetResponse resetResponseMessage;

void encodeGetStateSetTargetResponse(GetStateSetTargetResponse message, uint8_t buffer[]);
void decodeGetStateSetTargetReponse(GetStateSetTargetResponse message, uint8_t buffer[]);
void encodeResetResponse(ResetResponse message, uint8_t buffer[]);
void decodeResetResponse(ResetResponse message, uint8_t buffer[]);

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

//--------------------------SENSOR FUNCTIONS--------------------------

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

float getAbsoluteDegreeAngle(uint16_t rawAngle) { startAngle = rawAngle * (360.0 / 4096); }

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
    Serial.print("MAG");
    // idf_uart_write_bytes(bufir, sizeof(bufir));
}
//--------------------------TINYFRAME FUNCTIONS--------------------------

void TF_WriteImpl(TinyFrame *tf, const uint8_t *buff, uint32_t len) {
    // Serial.write(buff, len);
    idf_uart_write_bytes(buff, len);
}

TF_Result queryListener1(TinyFrame *tf, TF_Msg *msg) {
    // THE LISTENER ONLY (!) ENCODES A RESPONSE FOR NOW
    getStateSetTargetResponseMessage.currCartA = randFloat();
    getStateSetTargetResponseMessage.currCartV = randFloat();
    getStateSetTargetResponseMessage.currCartX = randFloat();
    getStateSetTargetResponseMessage.currImuA = randFloat();
    getStateSetTargetResponseMessage.currMotorV = randFloat();
    getStateSetTargetResponseMessage.currMotorX = randFloat();
    getStateSetTargetResponseMessage.currPoleAngle = randFloat();
    getStateSetTargetResponseMessage.currPoleV = randFloat();
    encodeGetStateSetTargetResponse(getStateSetTargetResponseMessage, buffer);
    msg->data = (unsigned char *)buffer;
    TF_Respond(tf, msg);
    return TF_STAY;
}

TF_Result queryListener2(TinyFrame *tf, TF_Msg *msg) {
    // decodeResetResponse(resetResponseMessage,(unsigned char *) msg->data);
    TF_Respond(tf, msg);
    return TF_STAY;
}
//--------------------------PROTOBUF FUNCTIONS--------------------------

void encodeGetStateSetTargetResponse(GetStateSetTargetResponse message, uint8_t buffer[]) {
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, SERIAL_BUFFER_SIZE);
    pb_encode(&stream, GetStateSetTargetResponse_fields, &message);
}

void decodeGetStateSetTargetReponse(GetStateSetTargetResponse message, uint8_t buffer[]) {
    pb_istream_t stream = pb_istream_from_buffer(buffer, SERIAL_BUFFER_SIZE);
    pb_decode(&stream, GetStateSetTargetResponse_fields, &message);
}

void encodeResetResponse(ResetResponse message, uint8_t buffer[]) {
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, SERIAL_BUFFER_SIZE);
    pb_encode(&stream, ResetResponse_fields, &message);
}

void decodeResetResponse(ResetResponse message, uint8_t buffer[]) {
    pb_istream_t stream = pb_istream_from_buffer(buffer, SERIAL_BUFFER_SIZE);
    pb_decode(&stream, ResetResponse_fields, &message);
}

//--------------------------MAIN--------------------------

void setup() {
    uint16_t rawAngle;
    float radAngle;
    pinMode(LED_PIN, OUTPUT);
    idf_uart_init();
    idf_i2c_init();
    // pb
    getStateSetTargetResponseMessage.currCartX = 1337;
    getStateSetTargetResponseMessage.currCartA = 1488;
    getStateSetTargetResponseMessage.currCartV = 600;
    getStateSetTargetResponseMessage.currMotorX = 1200;
    getStateSetTargetResponseMessage.currPoleAngle = 69;
    getStateSetTargetResponseMessage.currPoleV = 1200;

    // tf
    // encodeMessage(myMessage, buffer);
    // tf_slave
    slave_tf = TF_Init(TF_SLAVE);
    TF_ClearMsg(&msg);
    msg.type = 1;
    // msg.data = (unsigned char *)"Hello TF";
    //(unsigned char*)
    msg.len = 256;
    TF_AddTypeListener(slave_tf, 1, queryListener1);
    TF_AddTypeListener(slave_tf, 2, queryListener2);
    checkMagnetPresence();
    rawAngle = getRawAngle();
    startAngle = getAbsoluteDegreeAngle(rawAngle);
}

void loop() {
    uint16_t rawAngle;
    float radAngle;
    rawAngle = getRawAngle();
    radAngle = getCorrectedRadAngle(rawAngle);
    // Serial.print("HighByte\n");
    // Serial.print(angleHighByte);
    // Serial.print("LowByte\n");
    // Serial.print(angleLowByte);
    Serial.print(radAngle);
    Serial.print("\n");
    // idf_uart_write_bytes((uint8_t*)(str.data()), str.size());
    // idf_uart_write_bytes(&dataLow, 1);
    // idf_uart_write_bytes(&dataHigh, 1);
    // TF_Send(slave_tf, &msg);
    // while(idf_uart_available())
    //{

    // uint8_t b = idf_uart_read_byte();
    // TF_AcceptChar(slave_tf,b);

    //}
}
