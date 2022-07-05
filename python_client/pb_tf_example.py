from urllib import request
import sys
import serial
import os
import time
import TinyFrame as TF 
import protocol_pb2

state = protocol_pb2.State()
target = protocol_pb2.Target()

#Dummy reset data to test echoing
target.target_cart_x = 1337;
target.target_cart_v = 1488;
target.target_cart_a = 1;

#Listens to get state set target
def state_listener(_, frame):
    state.ParseFromString(frame.data)
    print("CURR_X: " + str(state.curr_cart_x))
    print("CURR_V: " + str(state.curr_cart_v))
    print("CURR_A: " + str(state.curr_cart_a))
    print("CURR_POLE_ANGLE: " + str(state.curr_pole_x))
    print("CURR_POLE_V: " + str(state.curr_pole_v))
    print("CURR_IMU_A: " + str(state.curr_imu_a))

#Listens to type2 - Reset echo
#def type_listener2(_, frame):
    #resetMessage.ParseFromString(frame.data)
    #print("MAX_X: " + str(resetMessage.MAX_CART_X))
    #print("MAX_V: " + str(resetMessage.MAX_CART_V))
    #print("MAX_A: " + str(resetMessage.MAX_CART_A))
    #print("MAX_THRSHLD_X: " +str(resetMessage.MAX_THRESHOLD_X))
    #print("MAX_THRSHLD_V: " +str(resetMessage.MAX_THRESHOLD_V))
    #print("MAX_THRSHLD_A: " +str(resetMessage.MAX_THRESHOLD_A))


ser = serial.Serial(port="/dev/ttyUSB0", baudrate=115200)
tf = TF.TinyFrame()
tf.ID_BYTES = 1
tf.TYPE_BYTES = 1
tf.LEN_BYTES = 4
tf.CKSUM_TYPE = 'crc16'
tf.SOF_BYTE = 0x01
tf.write = ser.write

tf.add_type_listener(protocol_pb2.MessageType.TARGETSTATE, state_listener)

while True:
    #tf.query(2, type_listener2, resetMessage.SerializeToString())
    #The request message contents don't matter for type 1 in this demo
    tf.query(protocol_pb2.MessageType.TARGETSTATE, state_listener, target.SerializeToString())
    while ser.in_waiting:
        tf.accept(ser.read(1))
    
