#include "protocol_processor.h"

#include <vector>

#include "globals.h"
#include "stepper.h"

#include <pb_encode.h>
#include <pb_decode.h>
extern "C" {
    #include "controller.pb.h"
}

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"

namespace {
// const HardwareSerial PROTOCOL_SERIAL_PORT = Serial;
using ErrorEnum = Error;

// bool encode_callback(pb_ostream_t *stream, const uint8_t *buf, size_t count) {
//     auto *P = (ProtocolProcessor *)stream->state;
//     P->serial_port.write(buf, count);
//     return true;
// }

const auto SERIAL_PORT_NUM = UART_NUM_0;
const int SERIAL_TX_PIN = 1;
const int SERIAL_RX_PIN = 3;
const int SERIAL_SPEED = 500000;
const int SERIAL_BUFFER_SIZE = 1024;

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
}  // namespace

ProtocolProcessor::ProtocolProcessor() {
    // serial_port.begin(SERIAL_SPEED);
    // while (!serial_port) {
    // }  // Wait for init
    idf_uart_init();
    Log("CARTPOLE CONTROLLER STARTED");
}

void ProtocolProcessor::Poll() {
    static std::string buffer;
    static uint64_t payloadSize = 0;
    static bool waitingForHeader = true;
    while (idf_uart_available()) {
        buffer += idf_uart_read_byte();
        if (waitingForHeader) {
            pb_istream_t stream = pb_istream_from_buffer((const uint8_t*) buffer.c_str(), buffer.size());
            bool status = pb_decode_varint(&stream, &payloadSize);
            if (!status) continue;

//            std::ostringstream tmp;
//            tmp << "PSIZE: " << payloadSize << "BUFF: " << buffer;
//            Log(tmp.str());
            
            waitingForHeader = false;
            buffer.clear();
        }
        if (payloadSize == buffer.length()) {
            handleCommand(buffer);
            buffer.clear();
            waitingForHeader = true;
            break;
        }
    }
}

void ProtocolProcessor::handleCommand(const std::string &line) {
    try {
        pb_istream_t stream = pb_istream_from_buffer((const uint8_t*) line.c_str(), line.size());
        Request request = Request_init_zero;
        bool status = pb_decode(&stream, Request_fields, &request);
        if (!status) throw std::runtime_error{"Failed to decode request"};
        Response response = dispatch(request);
        response.status = ResponseStatus_OK;
        sendResponse(response);
    } catch (std::exception &e) {
        return Error(e.what());
    }
}

Response ProtocolProcessor::dispatch(Request &request) {
    switch (request.type) {
        case RequestType_GET_STATE:
            return handleGetState(request);
        case RequestType_SET_TARGET:
            return handleSetTarget(request);
        case RequestType_SET_CONFIG:
            return handleSetConfig(request);
        case RequestType_GET_TARGET:
            return handleGetTarget(request);
        case RequestType_GET_CONFIG:
            return handleGetConfig(request);
        case RequestType_RESET:
            return handleReset(request);
        default:
            throw std::runtime_error{"Unknown request type"};
    }
}

#define SET_OPTIONAL_FIELD(object, name, value) object.name = value; object.has_##name = true;
#define VALIDATE_FIELD(object, name, type) object.name = validateField<type, FieldID::name>(object.name);
#define UPDATE_FIELD(object, name, type) updateField<type, FieldID::name>(object.name);
#define VALIDATE_IF_PRESENT(object, name, type) if (object.has_##name) VALIDATE_FIELD(object, name, type);
#define UPDATE_IF_PRESENT(object, name, type) if (object.has_##name) UPDATE_FIELD(object, name, type);
#define SET_IF_PRESENT(object, name, target) if (object.has_##name) target.name = object.name;

Response ProtocolProcessor::handleGetState(Request &request) {
    Globals &G = GetGlobals();
    Response response = Response_init_zero;
    SET_OPTIONAL_FIELD(response.payload.state, curr_x, G.curr_x);
    SET_OPTIONAL_FIELD(response.payload.state, curr_v, G.curr_v);
    SET_OPTIONAL_FIELD(response.payload.state, curr_a, G.curr_a);
    SET_OPTIONAL_FIELD(response.payload.state, pole_x, G.pole_x);
    SET_OPTIONAL_FIELD(response.payload.state, pole_v, G.pole_v);
    SET_OPTIONAL_FIELD(response.payload.state, errcode, (int) G.errcode);
    SET_OPTIONAL_FIELD(response.payload.state, imu_a, G.imu_a);
    SET_OPTIONAL_FIELD(response.payload.state, motor_x, G.motor_x);
    SET_OPTIONAL_FIELD(response.payload.state, motor_v, G.motor_v);
    response.which_payload = Response_state_tag;
    return response;
}

Response ProtocolProcessor::handleSetTarget(Request &request) {
    Globals &G = GetGlobals();
    Target target = request.payload.target;
    VALIDATE_IF_PRESENT(target, trgt_x, float);
    VALIDATE_IF_PRESENT(target, trgt_v, float);
    VALIDATE_IF_PRESENT(target, trgt_a, float);
    SET_IF_PRESENT(target, trgt_x, G);
    SET_IF_PRESENT(target, trgt_v, G);
    SET_IF_PRESENT(target, trgt_a, G);
    UPDATE_IF_PRESENT(target, trgt_x, float);
    UPDATE_IF_PRESENT(target, trgt_v, float);
    UPDATE_IF_PRESENT(target, trgt_a, float);
    return handleGetTarget(request);
}

Response ProtocolProcessor::handleSetConfig(Request &request) {
    Globals &G = GetGlobals();
    Config config = request.payload.config;
    VALIDATE_IF_PRESENT(config, max_x, float);
    VALIDATE_IF_PRESENT(config, max_v, float);
    VALIDATE_IF_PRESENT(config, max_a, float);
    // VALIDATE_IF_PRESENT(config, hw_max_x, float);
    // VALIDATE_IF_PRESENT(config, hw_max_v, float);
    // VALIDATE_IF_PRESENT(config, hw_max_a, float);
    // VALIDATE_IF_PRESENT(config, clamp_x, bool);
    // VALIDATE_IF_PRESENT(config, clamp_v, bool);
    // VALIDATE_IF_PRESENT(config, clamp_a, bool);
    // VALIDATE_IF_PRESENT(config, debug_led, bool);
    
    SET_IF_PRESENT(config, max_x, G);
    SET_IF_PRESENT(config, max_v, G);
    SET_IF_PRESENT(config, max_a, G);
    SET_IF_PRESENT(config, hw_max_x, G);
    SET_IF_PRESENT(config, hw_max_v, G);
    SET_IF_PRESENT(config, hw_max_a, G);
    SET_IF_PRESENT(config, clamp_x, G);
    SET_IF_PRESENT(config, clamp_v, G);
    SET_IF_PRESENT(config, clamp_a, G);
    SET_IF_PRESENT(config, debug_led, G);

    // UPDATE_IF_PRESENT(config, max_x, float);
    UPDATE_IF_PRESENT(config, max_v, float);
    UPDATE_IF_PRESENT(config, max_a, float);
    // UPDATE_IF_PRESENT(config, hw_max_x, float);
    // UPDATE_IF_PRESENT(config, hw_max_v, float);
    // UPDATE_IF_PRESENT(config, hw_max_a, float);
    // UPDATE_IF_PRESENT(config, clamp_x, bool);
    // UPDATE_IF_PRESENT(config, clamp_v, bool);
    // UPDATE_IF_PRESENT(config, clamp_a, bool);
    // UPDATE_IF_PRESENT(config, debug_led, bool);
    return handleGetConfig(request);
}

Response ProtocolProcessor::handleGetTarget(Request &request) {
    Globals &G = GetGlobals();
    Response response = Response_init_zero;
    SET_OPTIONAL_FIELD(response.payload.target, trgt_x, G.trgt_x);
    SET_OPTIONAL_FIELD(response.payload.target, trgt_v, G.trgt_v);
    SET_OPTIONAL_FIELD(response.payload.target, trgt_a, G.trgt_a);
    response.which_payload = Response_target_tag;
    return response;
}

Response ProtocolProcessor::handleGetConfig(Request &request) {
    Globals &G = GetGlobals();
    Response response = Response_init_zero;
    SET_OPTIONAL_FIELD(response.payload.config, max_x, G.max_x);
    SET_OPTIONAL_FIELD(response.payload.config, max_v, G.max_v);
    SET_OPTIONAL_FIELD(response.payload.config, max_a, G.max_a);
    SET_OPTIONAL_FIELD(response.payload.config, hw_max_x, G.hw_max_x);
    SET_OPTIONAL_FIELD(response.payload.config, hw_max_v, G.hw_max_v);
    SET_OPTIONAL_FIELD(response.payload.config, hw_max_a, G.hw_max_a);
    SET_OPTIONAL_FIELD(response.payload.config, clamp_x, G.clamp_x);
    SET_OPTIONAL_FIELD(response.payload.config, clamp_v, G.clamp_v);
    SET_OPTIONAL_FIELD(response.payload.config, clamp_a, G.clamp_a);
    SET_OPTIONAL_FIELD(response.payload.config, debug_led, G.debug_led);
    response.which_payload = Response_config_tag;
    return response;
}

Response ProtocolProcessor::handleReset(Request &request) {
    Stepper &S = GetStepper();
    Globals &G = GetGlobals();

    S.AsyncHoming();
    while (!S.IsDoneHoming()) {
        KeepAlive();
        delay(100);
    }
    KeepAlive();
    delay(100);

    float hw_max_x = G.hw_max_x;
    G.Reset();
    G.errcode = ErrorEnum::NO_ERROR;
    G.hw_max_x = hw_max_x;
    G.max_x = G.hw_max_x;
    
    // Fix creeping movement bug
    S.SetAcceleration(G.max_a);
    S.SetSpeed(G.max_v);
    S.ForceStop();

    return Response Response_init_zero;
}

void ProtocolProcessor::sendResponse(Response &response) {
    static uint8_t buffer[1024];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    bool status = pb_encode(&stream, Response_fields, &response);
    if (!status) throw std::runtime_error{"Protobuf encoding failed"};
    size_t payloadSize = stream.bytes_written;
    status = pb_encode_varint(&stream, payloadSize);
    if (!status) throw std::runtime_error{"Varint encoding failed"};
    size_t headerSize = stream.bytes_written - payloadSize;
    // serial_port.printf("HSIZE: %d, PSIZE: %d\n", headerSize, payloadSize);
    // serial_port.write(buffer + payloadSize, headerSize);
    // serial_port.write(buffer, payloadSize);
    // serial_port.flush();
    idf_uart_write_bytes(buffer + payloadSize, headerSize);
    idf_uart_write_bytes(buffer, payloadSize);
}

void ProtocolProcessor::sendTextResponse(const std::string &text, ResponseStatus status) {
    Response response = Response_init_zero;
    response.status = status;
    text.copy(response.message, sizeof(response.message) - 1);
    response.message[sizeof(response.message) - 1] = '\0';
    sendResponse(response);
}

void ProtocolProcessor::Log(const std::string &text) {
    // TODO: Compilation flag to disable logs?
    sendTextResponse(text, ResponseStatus_DEBUG);
}

void ProtocolProcessor::Error(const std::string &text) {
    sendTextResponse(text, ResponseStatus_ERROR);
}

void ProtocolProcessor::KeepAlive() {
    sendTextResponse("", ResponseStatus_PROCESSING);
}

// std::string ProtocolProcessor::get(const std::string &group, std::stringstream &stream) {
//     Globals &G = GetGlobals();
//     std::stringstream result;

//     std::string key;
//     bool first = true;
//     while (stream >> key) {
//         if (first) {
//             first = false;
//         } else {
//             result << ' ';
//         }
//         result << key << "=" << G.Get(group, key);
//     }
//     if (first) {
//         auto res = G.Get(group);
//         for (auto &&kv : res) {
//             if (first) {
//                 first = false;
//             } else {
//                 result << ' ';
//             }
//             result << kv.first << '=' << kv.second;
//         }
//     }

//     return result.str();
// }

// std::string ProtocolProcessor::set(const std::string &group, std::stringstream &stream) {
//     Globals &G = GetGlobals();
//     if (G.errcode != ErrorEnum::NO_ERROR) {
//         throw std::runtime_error{"Global error code is set"};
//     }

//     std::stringstream result;

//     std::vector<std::string> request_keys;
//     std::string kv;

//     while (stream >> kv) {
//         size_t pos = kv.find('=');
//         if (pos == std::string::npos) {
//             throw std::runtime_error{"Incorrect key-value pair format"};
//         }
//         std::string key = kv.substr(0, pos);
//         std::string value = kv.substr(pos + 1);
//         G.Prepare(group, key, value);
//         request_keys.emplace_back(std::move(key));
//     }

//     bool first = true;
//     for (const auto &key : request_keys) {
//         G.Commit(group, key);
//         if (first) {
//             first = false;
//         } else {
//             result << ' ';
//         }
//         result << key << '=' << G.Get(group, key);
//     }

//     return result.str();
// }

ProtocolProcessor &GetProtocolProcessor() {
    static ProtocolProcessor processor{};
    return processor;
}
