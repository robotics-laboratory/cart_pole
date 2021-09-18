import dataclasses
import math
import logging
import os

from typing import Dict, Tuple

import pybullet as pb
from pybullet_utils import bullet_client as pbc

from interface import CartPoleBase, Error, Config, State

log = logging.getLogger('simulator')

@dataclasses.dataclass
class PhysicalParams:
    time_step: float = 0.004  # 250hz
    gravity: float = 9.8      # m/s^2
    motor_force: float = 50.0 # N

def get_urdf_path() -> str:
    return os.path.dirname(os.path.abspath(__file__)) + '/cart_pole_v0.urdf'


def clamp(value: float, limit: float) -> Tuple[float, bool]:
    if value > limit:
        return limit, True

    if value < -limit:
        return -limit, True

    return value, False

WORLD_CENTER = [0, 0, 0]
NO_ROTATION = [0, 0, 0, 1]

DEBUG_STATE_POSITION = [-0.40, 0.05, 0.40]
DEBUG_STATE_COLOR = [255, 0, 0]
DEBUG_STATE_ORIENTATION = pb.getQuaternionFromEuler([math.pi/2, 0, 0])
DEBUG_STATE_SIZE = 0.06

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
    Motor simulation:
        Current implementation is specified for stepper motor.
        It is controlled by discrete velocity changes every time step.
        The target is desired acceleration of cart.
    Technical details:
        Each environment runs its own isolated pybullet physics engine.
    '''

    def __init__(self, debug_mode: bool = False):
        '''
        Args:
          * debug_mode – enable embedded pybullet GUI
        '''

        self.client = None
        self.config = None
        self.physical_params = None
        self.error = Error.NEED_RESET # Formally, we need reset env to reset error.
        self.step_count = 0

        self.target_acceleration = 0
        self.delta_velocity = 0
        self.velocity = 0

        self.debug_mode = debug_mode
        self.debug_state_id = None

        # create client
        if self.debug_mode:
            self.client = pbc.BulletClient(pb.GUI)
        else:
            self.client = pbc.BulletClient(pb.DIRECT)

        self.client.setRealTimeSimulation(0)

        # load urdf model
        self.object_id = self.client.loadURDF(
            get_urdf_path(),
            basePosition=WORLD_CENTER,
            useFixedBase=True,
            flags=pb.URDF_USE_SELF_COLLISION,
        )

        self.index_by_name = {}
        for index in range(self.client.getNumJoints(self.object_id)):
            _, name, *__ = self.client.getJointInfo(self.object_id, index)
            self.index_by_name[name] = index

        self.slider_to_cart_index = self.index_by_name[b'slider_to_cart']
        self.cart_to_pole_index = self.index_by_name[b'cart_to_pole']

        # prepare debug
        if self.debug_mode:
            self.client.configureDebugVisualizer(pb.COV_ENABLE_DEPTH_BUFFER_PREVIEW, False)
            self.client.configureDebugVisualizer(pb.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, False)
            self.client.configureDebugVisualizer(pb.COV_ENABLE_RGB_BUFFER_PREVIEW, False)
            self.client.resetDebugVisualizerCamera(
                cameraDistance=0.5,
                cameraYaw=0,
                cameraPitch=0,
                cameraTargetPosition=[0, -0.5, 0],
            )

            self.debug_state_id = self.client.addUserDebugText(
                text='...',
                textPosition=DEBUG_STATE_POSITION,
                textColorRGB=DEBUG_STATE_COLOR,
                textSize=DEBUG_STATE_SIZE,
                textOrientation=DEBUG_STATE_ORIENTATION
            )

    def update_debug_info(self) -> None:
        '''
        Update visual information for debug mode.
        '''

        if not self.debug_mode:
            return

        state = self.get_state()
        text = 'p={p:+.2f} v={v:+.2f}/{acc:+.2f} a={a:+.2f} w={w:+.2f} err={e}'.format(
            p=state.position,
            v=state.velocity,
            acc=self.target_acceleration,
            a=state.pole_angle,
            w=state.pole_angular_velocity,
            e=str(state.error_code.value)
        )

        self.client.addUserDebugText(
            text=text,
            textPosition=DEBUG_STATE_POSITION,
            textColorRGB=DEBUG_STATE_COLOR,
            textSize=DEBUG_STATE_SIZE,
            textOrientation=DEBUG_STATE_ORIENTATION,
            replaceItemUniqueId=self.debug_state_id
        )

    def reset_physical_params(self, params: PhysicalParams) -> None:
        '''
        Reset physical parameters of device model.
        '''

        log.info('reset physical params %s', params)
        self.error = Error.NEED_RESET

        self.physical_params = params
        self.client.setGravity(0, 0, -params.gravity)
        self.client.setTimeStep(params.time_step)

        # enable free pole rotation
        self.client.changeDynamics(
            bodyUniqueId=self.object_id,
            linkIndex=self.cart_to_pole_index,
            linearDamping=0,
            angularDamping=0
        )

        self.client.setJointMotorControl2(
            bodyUniqueId=self.object_id,
            jointIndex=self.cart_to_pole_index,
            controlMode=pb.VELOCITY_CONTROL,
            force=0
        )

        # disable any damping
        self.client.changeDynamics(
            bodyUniqueId=self.object_id,
            linkIndex=self.slider_to_cart_index,
            linearDamping=0,
            angularDamping=0
        )

        self.client.setJointMotorControl2(
            bodyUniqueId=self.object_id,
            jointIndex=self.slider_to_cart_index,
            controlMode=pb.VELOCITY_CONTROL,
            force=params.motor_force,
        )

    def get_state(self) -> State:
        '''
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
            position=cart_position,
            velocity=cart_velocity,
            acceleration=self.target_acceleration,
            pole_angle=pole_angle,
            pole_angular_velocity=pole_velocity,
            error_code=self.error
        )

    def get_target(self) -> float:
        '''
        Current acceleration target of system.
        '''

        return self.target_acceleration

    def get_config(self) -> Config:
        '''
        Current config.
        '''

        assert self.config
        return self.config

    def reset(self, config: Config) -> None:
        '''
        Resets the environment to an initial state.
        The pole is at rest position and cart is centered.
        '''
        log.info('reset config %s', config)

        assert self.physical_params
        if config.clamp_velocity or config.clamp_position:
            raise NotImplementedError

        self.config = config
        self.target_acceleration = 0
        self.velocity = 0
        self.delta_velocity = 0
        self.step_count = 0
        self.client.resetJointState(
            bodyUniqueId=self.object_id,
            jointIndex=self.cart_to_pole_index,
            targetValue=math.pi + 0.1,
            # targetValue=0.5,
            targetVelocity=0,
        )

        self.client.setJointMotorControl2(
            bodyUniqueId=self.object_id,
            jointIndex=self.cart_to_pole_index,
            controlMode=pb.POSITION_CONTROL,
            force=0
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
            controlMode=pb.VELOCITY_CONTROL,
            targetVelocity=0,
            force=self.physical_params.motor_force
        )

        self.error = Error.NO_ERROR

    def __set_motor_velocity(self, velocity) -> None:
        '''
        Set joint velocity
        '''

        log.debug('set motor velocity %.2f', velocity)

        self.client.setJointMotorControl2(
            bodyUniqueId=self.object_id,
            jointIndex=self.slider_to_cart_index,
            controlMode=pb.VELOCITY_CONTROL,
            targetVelocity=velocity,
        )

    def set_target(self, target: float) -> None:
        '''
        Controll acceleration of cart.
        It is assumed that the torque is sufficient to execute the command.

        Args:
            * target – desired acceleration of cart m/s
        '''

        if self.error:
            log.warning('set target, error=%s', self.error)
            return

        config = self.get_config()
        self.target_acceleration, clamped = clamp(target, config.max_acceleration)

        if clamped and not config.clamp_acceleration:
            self.error = Error.A_OVERFLOW
            return

        log.debug('set acc=%.2f', target)
        self.delta_velocity = self.target_acceleration * self.physical_params.time_step

    def validate(self) -> bool:
        '''
        Validate current state.
        Returns:
            True if validation is ok.
        '''
        state = self.get_state()
        config = self.get_config()

        if abs(state.velocity) > config.max_velocity:
            self.error = Error.V_OVERFLOW
            return False

        if abs(state.position) > config.max_position:
            self.error = Error.X_OVERFLOW
            return False

        return True

    def make_step(self, step_n=1) -> None:
        '''
        Makes step_n sequential simulation steps.
        '''
        if self.error:
            log.warning('make step, error=%s', self.error)
            return

        for _ in range(step_n):
            log.debug('make step %i', self.step_count)
            self.velocity += self.delta_velocity
            self.__set_motor_velocity(self.velocity)
            self.client.stepSimulation()
            self.update_debug_info()

            if not self.validate():
                return

            self.step_count += 1

    def run(self, seconds) -> int:
        '''
        Run simulation for required time.
        It is proceeded discretely, so resulted simulation time equals
            simulation_time = step_n * time_step, where
            step_n = round(seconds / time_step)
        Returns:
            Pair (simulation_time, step_n)
        '''

        time_step = self.physical_params.time_step
        step_n = round(seconds / time_step)
        simulation_time = time_step * step_n

        self.make_step(step_n)
        return simulation_time, step_n

    def get_info(self) -> Dict:
        '''
        Returns useful debug info.
        '''

        return {
            'step_count': self.step_count,
            'target': self.target_acceleration,
        }

    def close(self) -> None:
        '''
        Free simulator resources.
        '''
        self.client.disconnect()
