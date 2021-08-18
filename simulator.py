from typing import Tuple

import numpy as np
import pybullet as pb

from pybullet_utils import bullet_client as pbc
from interface import CartPoleBase,CartPoleConfig, Error, State

import logging


world_center = [0,0,0]
no_rotation = [0, 0, 0, 1]

debug_state_position = [0.30, 0.05, 0.3]
debug_state_color = [255, 0 , 0] 
debug_state_orientation = pb.getQuaternionFromEuler([np.pi/2, 0, np.pi])
debug_state_size = 0.06

DEFAULT_DELAY = 1/50


def short_view(state, target):
    return 'p={p:+.2f}/{t:+.2f} v={v:+.2f} a={a:+.2f} w={w:+.2f}'.format(
        p=state.cart_position,
        t=target,
        v=state.cart_velocity,
        a=state.pole_angle,
        w=state.pole_velocity,
        e=int(state.error)
    )


log = logging.getLogger('simulator')

class CartPoleSimulator(CartPoleBase):
    '''
    Description:
        Сlass implements a physical simulation of the cart-pole device.
        A pole is attached by an joint to a cart, which moves along guide axis.
        The pendulum is initially at rest state. The goal is to maintain it in
        upright pose by increasing and reducing the cart's velocity.
    Source:
        This environment is some variation of the cart-pole problem
        described by Barto, Sutton, and Anderson
    Initial state:
        A pole is at starting position 0 with no velocity and acceleration.
    Session termination:
        - Cart position is violated.
    Technical details:
        Each environment runs its own isolated pybullet physics engine.
    '''

    def __init__(self,
            gravity: float = 9.8,
            debug_mode: bool = False):
        '''
        Args:
          * position_limit – cart center must be in [-position_limit, +position_limit] range.
          * gravity – gravity force
          * debug_mode – enable engine GUI
        '''

        log.info(f'gravity={gravity}, debug_mode={debug_mode}')

        self.config = None
        self.step_count = 0

        self.debug_mode = debug_mode
        self.debug_state_id = None

        if self.debug_mode:
            self.client = pbc.BulletClient(pb.GUI)
            self.client.configureDebugVisualizer(pb.COV_ENABLE_DEPTH_BUFFER_PREVIEW, False)
            self.client.configureDebugVisualizer(pb.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, False)
            self.client.configureDebugVisualizer(pb.COV_ENABLE_RGB_BUFFER_PREVIEW, False)
            self.client.resetDebugVisualizerCamera(
                cameraDistance=0.5,
                cameraYaw=180,
                cameraPitch=0,
                cameraTargetPosition=[0, 0.5, 0],
            )

        else:
            self.client = pbc.BulletClient(pb.DIRECT)

        self.object_id = self.client.loadURDF(
            'cart_pole_v0.urdf',
            basePosition = world_center,
            useFixedBase=True,
            flags=pb.URDF_USE_SELF_COLLISION,
        )

        self.index_by_name = {}
        for index in range(self.client.getNumJoints(self.object_id)):
            _, name, *__ = self.client.getJointInfo(self.object_id, index)
            self.index_by_name[name] = index

        self.slider_to_cart_index = self.index_by_name[b'slider_to_cart']
        self.cart_to_pole_index = self.index_by_name[b'cart_to_pole']

        # enable free pole rotation
        self.client.setJointMotorControl2(
            bodyUniqueId=self.object_id, 
            jointIndex=self.cart_to_pole_index,
            controlMode=pb.VELOCITY_CONTROL,
            force=0
        )
        
        self.client.changeDynamics(
            bodyUniqueId=self.object_id, 
            linkIndex=self.cart_to_pole_index,
            linearDamping=0,
            angularDamping=0
        )

        self.client.setTimeStep(DEFAULT_DELAY)
        self.target = 0

        # configure cart contraints
        self.client.setJointMotorControl2(
            bodyUniqueId=self.object_id, 
            jointIndex=self.slider_to_cart_index,
            controlMode=pb.POSITION_CONTROL,
            targetPosition=self.target,
            force=0, # N
            positionGain=1.0,
            velocityGain=0.0,
        )
        
        # disable any damping
        self.client.changeDynamics(
            bodyUniqueId=self.object_id, 
            linkIndex=self.slider_to_cart_index,
            linearDamping=0,
            angularDamping=0
        )

        self.client.setGravity(0, 0, -gravity)
        self.client.setRealTimeSimulation(0)

        # Formally, we need reset env to reset error.
        # Done for similarity with real control, where need to make homing.
        self.error = Error.NOT_INITIALIZED

        if self.debug_mode:
            self.debug_state_id = self.client.addUserDebugText(
                text=short_view(self.get_state(), self.get_target()),
                textPosition=debug_state_position,
                textColorRGB=debug_state_color,
                textSize=debug_state_size,
                textOrientation=debug_state_orientation
            )

    def get_state(self) -> State:
        '''
        Returns:
            Current state of system.
        '''

        cart_position, cart_velocity, *_ = self.client.getJointState(
            bodyUniqueId=self.object_id,
            jointIndex=self.slider_to_cart_index
        )

        pole_angle, pole_velocity, *_ = self.client.getJointState(
            bodyUniqueId=self.object_id,
            jointIndex=self.cart_to_pole_index
        )

        return State(
            cart_position=cart_position,
            cart_velocity=cart_velocity,
            pole_angle=pole_angle,
            pole_velocity=pole_velocity,
            error=self.error
        )

    def get_target(self) -> float:
        return self.target

    def get_config(self):
        assert self.config
        return self.config

    def reset(self, config: CartPoleConfig = CartPoleConfig()) -> Tuple[State, float]:
        '''
        Description:
            Resets the environment to an initial state.
            The pole is at rest position and cart is centered.

        Returns:
            Returns (initial_state, initial_target).
        '''
        log.info(f'reset')

        self.config = config
        self.target = 0
        self.step_count = 0

        self.client.resetJointState(
            bodyUniqueId=self.object_id,
            jointIndex=self.cart_to_pole_index,
            targetValue=0,
            targetVelocity=0,
        )

        self.client.resetJointState(
            bodyUniqueId=self.object_id,
            jointIndex=self.slider_to_cart_index,
            targetValue=0,
            targetVelocity=0,
        )

        self.client.setJointMotorControl2(
            bodyUniqueId=self.object_id,
            jointIndex=self.slider_to_cart_index,
            controlMode=pb.POSITION_CONTROL,
            targetPosition=self.target,
            maxVelocity=self.config.cart_velocity_limit
        )

        self.error = Error.NO_ERROR

        return self.get_state(), self.get_target()

    def set_target(self, target: float) -> None:
        log.debug(f'set target {self.config.action_type.target_name()}={target}')
        if self.error:
            return

        config = self.get_config()
        if config.clamp_position:
            target = np.clip(target, -config.cart_position_limit, config.cart_position_limit)
        elif abs(target) > config.cart_position_limit:
            self.error = Error.INVALID_CART_POSITION
            return 

        self.target = target

        self.client.setJointMotorControl2(
            bodyUniqueId=self.object_id, 
            jointIndex=self.slider_to_cart_index,
            controlMode=pb.POSITION_CONTROL,
            targetPosition=self.target
        )

    def step(self, delta: float) -> None:
        if self.error:
            return
    
        log.debug(f'Make step {self.step_count}')

        self.step_count += 1
        self.client.setTimeStep(delta)
        self.client.stepSimulation()

        if self.debug_mode:
            state = self.get_state()
            self.client.addUserDebugText(
                text=short_view(self.get_state(), self.get_target()),
                textPosition=debug_state_position,
                textColorRGB=debug_state_color,
                textSize=debug_state_size,
                textOrientation=debug_state_orientation,
                replaceItemUniqueId=self.debug_state_id
            )

    def get_info(self):
        return {
            'step_count': self.step_count
        }

    def close(self):
        self.client.disconnect()
