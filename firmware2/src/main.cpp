#include <Arduino.h>
extern "C" {
#include "TinyFrame.h"
}
#include <stdio.h>
#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "brain_controller.pb.h"
//#include "utils.h"
TinyFrame * master_tf;
TinyFrame * slave_tf;
TF_Msg msg;
getStateSetTargetResponse getStateSetTargetResponseMessage;
resetResponse resetResponseMessage;
#define BUFFER_LENGTH 256
#define led 2
uint8_t buffer[BUFFER_LENGTH];

const auto SERIAL_PORT_NUM = UART_NUM_0;
const int SERIAL_TX_PIN = 1;
const int SERIAL_RX_PIN = 3;
const int SERIAL_SPEED = 115200;
const int SERIAL_BUFFER_SIZE = 256;



void processDataGetStateSetTarget();
void encodeGetStateSetTargetResponse(getStateSetTargetResponse message, uint8_t buffer[]);
void decodeGetStateSetTargetReponse(getStateSetTargetResponse message, uint8_t buffer[]);
void encodeResetResponse(resetResponse message, uint8_t buffer[]);
void decodeResetResponse(resetResponse message, uint8_t buffer[]);
void idf_uart_init();
bool idf_uart_available();
uint8_t idf_uart_read_byte();
void idf_uart_write_bytes(const uint8_t *src, size_t length);

void idf_uart_init() {
    uart_config_t uart_config = {
        .baud_rate = SERIAL_SPEED,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    ESP_ERROR_CHECK(uart_driver_install(SERIAL_PORT_NUM, SERIAL_BUFFER_SIZE, SERIAL_BUFFER_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(SERIAL_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(SERIAL_PORT_NUM, SERIAL_TX_PIN, SERIAL_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
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
    int length_written = uart_write_bytes(SERIAL_PORT_NUM, (const char*) src, length);
    if (length_written != length) ESP_ERROR_CHECK(ESP_FAIL);
}

void TF_WriteImpl(TinyFrame *tf, const uint8_t *buff, uint32_t len)
{
        //Serial.write(buff, len);
        idf_uart_write_bytes(buff, len);
}

float randFloat()
{
    float f = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    return f;
}

TF_Result queryListener1(TinyFrame *tf, TF_Msg *msg)
{
    //THE LISTENER ONLY (!) ENCODES A RESPONSE FOR NOW
        getStateSetTargetResponseMessage.CURR_CART_A = randFloat();
        getStateSetTargetResponseMessage.CURR_CART_V = randFloat();
        getStateSetTargetResponseMessage.CURR_CART_X = randFloat();
        getStateSetTargetResponseMessage.CURR_IMU_A = randFloat();
        getStateSetTargetResponseMessage.CURR_MOTOR_V = randFloat();
        getStateSetTargetResponseMessage.CURR_MOTOR_X = randFloat();
        getStateSetTargetResponseMessage.CURR_POLE_ANGLE = randFloat();
        getStateSetTargetResponseMessage.CURR_POLE_V = randFloat();
        //Epilepsy-inducing blinking
        digitalWrite(led, HIGH);
        delay(1000);
        digitalWrite(led, LOW);
        delay(1000);
        digitalWrite(led, HIGH);
        delay(250);
        digitalWrite(led, LOW);
        delay(250);
        digitalWrite(led, HIGH);
        delay(250);
        digitalWrite(led, LOW);
        delay(250);
        digitalWrite(led, HIGH);
        delay(250);
        digitalWrite(led, LOW);
        delay(250);
        digitalWrite(led, HIGH);
        delay(500);
        encodeGetStateSetTargetResponse(getStateSetTargetResponseMessage, buffer);
        msg->data = (unsigned char *)buffer;
        TF_Respond(tf, msg);
//    TF_Respond();
    return TF_STAY;
}

TF_Result queryListener2(TinyFrame *tf, TF_Msg *msg)
{
    //decodeResetResponse(resetResponseMessage,(unsigned char *) msg->data);
    TF_Respond(tf, msg);
    digitalWrite(led, HIGH);
    delay(500);
    digitalWrite(led, LOW);
    delay(500);
    digitalWrite(led, HIGH);
    delay(500);
    digitalWrite(led, LOW);
    delay(1000);
    return TF_STAY;
}

 
void encodeGetStateSetTargetResponse(getStateSetTargetResponse message, uint8_t buffer[])
{
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, BUFFER_LENGTH);
    pb_encode(&stream, getStateSetTargetResponse_fields , &message);
 
}
 
void decodeGetStateSetTargetReponse(getStateSetTargetResponse message, uint8_t buffer[])
{
    pb_istream_t stream = pb_istream_from_buffer(buffer, BUFFER_LENGTH);
    pb_decode(&stream, getStateSetTargetResponse_fields, &message);
}
 
void encodeResetResponse(resetResponse message, uint8_t buffer[])
{
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, BUFFER_LENGTH);
    pb_encode(&stream, resetResponse_fields, &message);
}
 
void decodeResetResponse(resetResponse message, uint8_t buffer[])
{
    pb_istream_t stream = pb_istream_from_buffer(buffer, BUFFER_LENGTH);
    pb_decode(&stream, resetResponse_fields, &message);
}

void setup()
{   
    pinMode(led, OUTPUT);
    //Serial.begin(115200);
    idf_uart_init();
    //pb
    getStateSetTargetResponseMessage.CURR_CART_X = 1337;
    getStateSetTargetResponseMessage.CURR_IMU_A = 1488;
    getStateSetTargetResponseMessage.CURR_MOTOR_V = 600;
    getStateSetTargetResponseMessage.CURR_MOTOR_X = 1200;
    getStateSetTargetResponseMessage.CURR_POLE_ANGLE = 69;
    getStateSetTargetResponseMessage.CURR_POLE_V = 1200;

    //tf
    //encodeMessage(myMessage, buffer);
    //tf_slave 
    slave_tf = TF_Init(TF_SLAVE);
    TF_ClearMsg(&msg);
    msg.type = 1;
    //msg.data = (unsigned char *)"Hello TF";
    //(unsigned char*)
    msg.len = 256;
    TF_AddTypeListener(slave_tf, 1, queryListener1);
    TF_AddTypeListener(slave_tf,2,queryListener2);
}


void loop()
{
   // if(Serial.available() > 0)
   // {
   //     int b = Serial.read();

   //     TF_AcceptChar(slave_tf, b);
   // }
    while(idf_uart_available())
    {
        
        uint8_t b = idf_uart_read_byte();
        TF_AcceptChar(slave_tf,b);

    }
}