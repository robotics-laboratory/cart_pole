from cartpole.device import CartPoleDevice, protocol_pb2
from cartpole.device import (
   DeviceTarget,
   DeviceConfig,
   DeviceState,
)
import time
cock_ball_device = CartPoleDevice()

while True:
   target = 25
   cock_ball_device.set_target(target);
   # print('The current velocity is {}'.format(state.cart_velocity))
   # print('The current acceleration is {}'.format(state.cart_acceleration))
   # print('The current position is {}'.format(state.cart_position))
   # #print('The current IMU acceleration is{}'.format(state.))
   # print('The current pole velocity is {}'.format(state.pole_angular_velocity))
   # print('The current angle is {}'.format(state.pole_angle))
   state = cock_ball_device.get_state()
   print("I am working!")
   print('The current velocity is {}'.format(state.cart_velocity))
   print('The current acceleration is {}'.format(state.cart_acceleration))
   print('The current position is {}'.format(state.cart_position))
   #print('The current IMU acceleration is{}'.format(state.))
   print('The current pole velocity is {}'.format(state.pole_angular_velocity))
   print('The current angle is {}'.format(state.pole_angle))
