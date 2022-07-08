# CartPole interface protocol & framing

- [Overview](#overview)
- [Message types](#message-types)
 - [MessageType enumeration](#messagetype-enumeration)
 - [Config](#config)
 - [State](#state)
 - [Target](#target)
 - [Request](#request)
 - [Error codes](#error-codes)
- [Framing & Callbacks](#framing-and-callbacks)


## Overview
The protocol facilitates an interface between a higher-level, non-real-time capable computer and lower-level microcontroller. The computer deals with high-level commands, such as "move at speed of 0.5 m/s" or "retrieve cart-pole system state". Controller then translates these commands and manipulates lower-level devices such as motors and sensors in accordance.

Communication is done over serial port, the message data is encoded using Protobuf in plaintext and is than dispatched in separate frames (data units) using TinyFrame. This allows manual debugging and enables a callback-driven internal software architecture, where specific messages automatically trigger an execution of a particular piece of code.

The computer sends configs for the cart-pole system, mostly defining how fast and far on the x-axis it can go, as well as sending target values for the microcontroller, such as required motor speed which it will then try to achieve. 

The computer also continiously sends state requests, which are messages with no variables except for a specific ID (see MessageType enumeration), in response to which  the cart-pole system controller reports back its' state, such as current speed, and position of the cart, as well as pole angle.

## Message Types 
### MessageType enumeration
Message types are both defined in Protobuf and in TinyFrame for flexibility, in TinyFrame frames the Type field is employed for this purpose (see framing section).
| KEY | INT|  COMMENT                  |
|-----|----|---------------------------|
| RESET| 0 | This is a reset message which forces the system to perform a homing operation and procure new values for all hardware limit-related fields       | 
| TARGETSTATE| 1  | This message contains either a target sent to a microcontroller from a computer, or a state from a microcontoller sent to a computer | 
| KEEPALIVE| 2 | This emply (no payload, see framing section for what a payload is) message is sent when there are no commands from pc to be sent, just so that the serial port does not timeout |
|UPDATESTATE| 3 | This empty message (no payload, see framing section for payload definition) is sent just to ask for a new state message from the microcontroller  | 
### Config
| KEY       | TYPE/DIMENSION         | DEFAULT VALUE| RANGE                        | COMMENT                                  |
|---------- |--------------|---------|------------------------------|------------------------------------------|
| max_cart_x| m, float     | 0       | [-hw_max_x, +hw_max_x]  | [Readonly] Max cart position (x=0 is the rail center)        |
| max_cart_v| m/s, float   | 0.5     | [0, hw_max_v]                | [Readonly] Max cart velocity                        |
| max_cart_a| m/s^2, float | 1.0     | [0, hw_max_a]                | [Readonly] Max cart acceleration                    |
| hw_max_x  | m, float     | 0       | -                            | [Calibraion-determined] Max allowed cart position     |
| hw_max_v  | m/s, float   | TBD     | -                            | [Calibraion-determined] Max allowed cart velocity     |
| hw_max_a  | m/s^2, float | TBD     | -                            | [Calibraion-determined] Max allowed cart acceleration |

### State

| KEY     | TYPE/DIMENSION        | DEFAULT VALUE| RANGE | COMMENT                                        |
|---------|--------------|---------|-------|------------------------------------------------|
| curr_cart_x       | m, float     | 0       | -    | [Readonly] Current cart position               |
| curr_cart_v       | m/s, float   | 0       | -     | [Readonly] Current cart velocity               |
| curr_cart_a       | m/s^2, float | 0       | -     | [Readonly] Current cart acceleration           |
| curr_pole_x | rad, float   | 0      | [0, $2\pi$] | [Readonly] Current pole angle. The reference point (0 rad angle) is the angle of the pole steadily hanging downwards without any motion.               |.
| curr_pole_v| rad/s, float | 0       | -     | [Readonly] Current pole angular velocity. Clockwise velocity is positive, counterclockwise velocity is negative       |
| curr_imu_a | m/s^2, float | 0       | -     | [Readonly] Current pole angular velocity       |
| error_code | enum bitmask, int32    | 1     | -     | [Readonly] Bitmask-based error code (see enum below) |


### Target

| KEY | TYPE/DIMENSION         | DEFAULT VALUE| RANGE           | COMMENT                  |
|-----|--------------|---------|-----------------|--------------------------|
| target_cart_x   | m, float     | 0       | [-max_cart_x, +max_cart_x] | Target cart position     |
| target_cart_v   | m/s, float   | 0       | [-max_cart_v, +max_cart_v] | Target cart velocity     |
| target_cart_a  | m/s^2, float | 0       | [-max_cart_v, +max_cart_v] | Target cart acceleration |
### Error codes
The error code is a bitmask-driven enum, which enables reporting multiple errors in a single variable by means of bitwise disjuncting (applying "OR" operator) multiple error codes into a single value and than parsing them using a set of bitmasks.
| KEY           | INT |HEX| COMMENT                                                            |
|---------------|-----|---|--------------------------------------------------------------------|
| NO_ERROR      | 0   |0x00| This is fine (no errors, motion is allowed)                           |
| NEED_RESET    | 1   |0x01| Device needs homing (see reset command)                               |
| X_OVERFLOW    | 2   |0x02| Current or target position was out of allowed range                   |
| V_OVERFLOW    | 4   |0x04| Current or target velocity was out of allowed range                   |
| A_OVERFLOW    | 8   |0x08| Current or target acceleration was out of allowed range               |
| MOTOR_STALLED | 16   |0x10| Stepper driver detected motor failure (TMC’s StallGuard is triggered) |
| ENDSTOP_HIT   | 32   |0x20| One of endstops is triggered during movement                          |
#### Example
The cart has oversped (V_OVERFLOW) and hit an endstop (ENDSTOP_HIT), then error_code == (V_OVERFLOW | ENDSTOP_HIT) == 0010 1000 in binary, which can later be parsed into multiple error with bitwise "AND" and enumerated.

### Request
Request is an empty TinyFrame message (with no payload), defined only by its' TinyFrame type. It triggers a report of the current system state by the microcontroller without sending a new target to it.

## Framing and Callbacks
The Protobuf encoded messages are then "wrapped" using a framing protocol provided by protobuf. It provides a means of encapsulating data into discrete "portions". Below is what a typical frame structure looks like. "DATA" is our Protobuf message, the frame's payload.

```
,-----+-----+-----+------+------------+- - - -+-------------,
| SOF | ID  | LEN | TYPE | HEAD_CKSUM | DATA  | DATA_CKSUM  |
| 0-1 | 1-4 | 1-4 | 1-4  | 0-4        | ...   | 0-4         | <- size (bytes)
'-----+-----+-----+------+------------+- - - -+-------------'

SOF ......... start of frame, usually 0x01 (optional, configurable)
ID  ......... the frame ID (MSb is the peer bit)
LEN ......... number of data bytes in the frame
TYPE ........ message type (used to run Type Listeners, pick any values you like)
HEAD_CKSUM .. header checksum

DATA ........ LEN bytes of data
DATA_CKSUM .. data checksum (left out if LEN is 0)
```

Than TinyFrame message handlers (which are called "listeners" above) are set up both on the computer and microcontroller side, which parse every byte received over serial port, and as soon as a message of a certain type is received, it is handled and a particular piece of code is executed based on messages' ID.

These are callback-based interactions of the computer and microcontroller:
```
Computer                     |                Microcontroller
Send target -----------------|--------------->
                                        Try to execute target
        <--------------------|-- Send fresh state in response 
*************************************************************
Send state request-----------|--------------->
                                              Process request
        <--------------------|-- Send fresh state in response 
*************************************************************
Send keepalive message ------|--------------->
*************************************************************
Send configuration ----------|--------------->
                                            Assume new config
        <--------------------|------- Send configuration back
```
