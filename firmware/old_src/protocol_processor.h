#pragma once
#include <Arduino.h>

#include <sstream>
#include <string>
#include "controller.pb.h"

struct ProtocolProcessor {
    // HardwareSerial serial_port;

    ProtocolProcessor();

    void Poll();

    // void Success(const std::string &text);
    void Log(const std::string &text);
    void Error(const std::string &text);
    void KeepAlive();

private:
    void handleCommand(const std::string &line);

    // std::string get(const std::string &group, std::stringstream &stream);
    // std::string set(const std::string &group, std::stringstream &stream);
    // std::string reset();

    Response dispatch(Request &request);
    Response handleGetState(Request &request);
    Response handleSetTarget(Request &request);
    Response handleSetConfig(Request &request);
    Response handleGetTarget(Request &request);
    Response handleGetConfig(Request &request);
    Response handleReset(Request &request);
    void sendResponse(Response &response);
    void sendTextResponse(const std::string &text, ResponseStatus status);
};

ProtocolProcessor &GetProtocolProcessor();
