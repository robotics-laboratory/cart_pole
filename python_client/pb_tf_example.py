from urllib import request
import brain_controller_pb2
import sys
import serial
import os
import time
import TinyFrame as TF 

resetMessage = brain_controller_pb2.resetRequest()
getStateMessageResponse = brain_controller_pb2.getStateSetTargetResponse()

#Dummy reset data to test echoing
resetMessage.MAX_CART_X = 1337
resetMessage.MAX_CART_V = 1488
resetMessage.MATX_CART_A = 0
resetMessage.HW_MAX_X = 80085
resetMessage.HW_MAX_V = 7331
resetMessage.HW_MAX_A = 707
resetMessage.MAX_THRESHOLD_X = 95
resetMessage.MAX_THRESHOLD_V = 91
resetMessage.MAX_THRESHOLD_A = 88

#float CURR_CART_X = 1; // current cart position float 
#float CURR_CART_V = 2; // current cart velocity float 
#float CURR_CART_A = 3; // current cart acceleration
#float CURR_POLE_ANGLE = 4; // current pole angle
#float CURR_POLE_V = 5; // current pole angular velocity
#float CURR_IMU_A = 6; // current IMU measured cart acceleration
#float CURR_MOTOR_X = 7; // current motor shaft position (radians)
#float CURR_MOTOR_V = 8; // motor shaft velocity (rad/sec)
#Listens to type1 - GetStateSetTarget
def type_listener1(_, frame):
    getStateMessageResponse.ParseFromString(frame.data)
    print("CURR_X: " + str(getStateMessageResponse.CURR_CART_X))
    print("CURR_V: " + str(getStateMessageResponse.CURR_CART_V))
    print("CURR_A: " + str(getStateMessageResponse.CURR_CART_A))
    print("CURR_POLE_ANGLE: " + str(getStateMessageResponse.CURR_POLE_ANGLE))
    print("CURR_POLE_V: " + str(getStateMessageResponse.CURR_POLE_V))
    print("CURR_IMU_A: " + str(getStateMessageResponse.CURR_IMU_A))
    print("CURR_MOTOR_X: " + str(getStateMessageResponse.CURR_MOTOR_X))
    print("CURR_MOTOR_V: " + str(getStateMessageResponse.CURR_MOTOR_V))
    
    
#Listens to type2 - Reset echo
def type_listener2(_, frame):
    resetMessage.ParseFromString(frame.data)
    print("MAX_X: " + str(resetMessage.MAX_CART_X))
    print("MAX_V: " + str(resetMessage.MAX_CART_V))
    print("MAX_A: " + str(resetMessage.MAX_CART_A))
    print("MAX_THRSHLD_X: " +str(resetMessage.MAX_THRESHOLD_X))
    print("MAX_THRSHLD_V: " +str(resetMessage.MAX_THRESHOLD_V))
    print("MAX_THRSHLD_A: " +str(resetMessage.MAX_THRESHOLD_A))


ser = serial.Serial(port="/dev/ttyUSB0", baudrate=115200)
tf = TF.TinyFrame()
tf.ID_BYTES = 1
tf.TYPE_BYTES = 1
tf.LEN_BYTES = 4
tf.CKSUM_TYPE = 'crc16'
tf.SOF_BYTE = 0x01
tf.write = ser.write
tf.add_type_listener(2, type_listener2)

while True:
    #tf.query(2, type_listener2, resetMessage.SerializeToString())
    #The request message contents don't matter for type 1 in this demo
    tf.query(1, type_listener1, resetMessage.SerializeToString())
    while ser.in_waiting:
        tf.accept(ser.read(1))

   # tf.accept(ser.read(256))
   # print("msg read")
   # tf.send()
   # print("Iteration " + str(a) + "\n")
   # a +=1
   # time.sleep()
   # tf.accept(ser.read(100))
   # time.sleep(1)
    #tf.accept(ser.read(1))
    # print("+ byte")