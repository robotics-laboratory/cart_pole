import logging
import dataclasses as dc
import math
from re import A
import time
import serial
import TinyFrame as TF
import protocol_pb2
from typing import Union, Type, Any
# TODO: Timeouts using the TF library
# TODO: Upon calling get state/reset check if we timeout
# TODO: If we timeout, raise error

from common import util
from common.interface import (
    Config,
    State,
    CartPoleBase,
    Error,
)

#LOGGER = logging.getLogger(__name__)

class DeviceVariableGroup:
    '''
    Mixin intended to be used with @dataclass-decorated classes. Subclass should
    represent a variable group in a controller protocol spec. Group name should be
    set by overriding `GROUP_NAME` classvar. Mixin provides methods
    for convertion to and from wire formats.
    '''

    # Should be overridden in subclass
    GROUP_NAME: str = None
    SERIALIZATION_MAP: dict = None

    # Field formatters (value -> string)
    _to_string_lookup = {
        float: lambda v: f'{v:.5f}',
        bool: lambda v: 'true' if v else 'false',
        int: lambda v: str(v),
        Error: lambda v: str(v.value),
    }

    # Field parsers (string -> value)
    _from_string_lookup = {
        float: lambda s: float(s),
        bool: lambda s: s == 'true',
        int: lambda s: int(s),
        Error: lambda s: Error(int(s)),
    }

    def __init_subclass__(cls) -> None:
        super().__init_subclass__()
        assert cls.GROUP_NAME is not None, 'GROUP_NAME is not set'
        assert cls.SERIALIZATION_MAP is not None, 'SERIALIZATION_MAP is not set'

    @classmethod
    def full(cls) -> 'DeviceVariableGroup':
        '''
        Constructs instance initialized with filler value
        '''
        return cls(**{field.name: True for field in dc.fields(cls)})

    @classmethod
    def _type_lookup(cls, type: Type, kv: dict):
        for possible_type, value in kv.items():
            if issubclass(type, possible_type):
                return value
        raise ValueError(f'No matching item for type {type}')

    def to_dict_format(self) -> str:
        '''
        Serializes current dataclass instance to '<key>=<value>' format
        as used in 'set' method of the protocol.

        Returns:
            Command string in dict wire format.
        '''
        return ' '.join(
            f'{self.SERIALIZATION_MAP[field.name]}={self._type_lookup(field.type, self._to_string_lookup)(getattr(self, field.name))}'
            for field in dc.fields(self)
            if getattr(self, field.name) is not None
        )

    def to_list_format(self) -> str:
        '''
        Serializes current dataclass instance to '<key>' format
        as used in 'get' method of the protocol.

        Returns:
            Command string in list wire format.
        '''
        return ' '.join(
            self.SERIALIZATION_MAP[field.name] for field in dc.fields(self) if
            getattr(self, field.name) is not None)

    @classmethod
    def from_dict_format(cls, text: str) -> 'DeviceVariableGroup':
        '''
        Parses input text into new class instance.

        Returns:
            Class instance, constructed from wire format representation.
        '''
        field_lookup = {cls.SERIALIZATION_MAP[field.name]: field for field in
                        dc.fields(cls)}
        data_dict = {}
        for pair in text.split():
            wire_name, value = pair.split('=')
            field = field_lookup[wire_name]
            from_string = cls._type_lookup(field.type, cls._from_string_lookup)
            data_dict[field.name] = from_string(value)
        return cls(**data_dict)


@dc.dataclass
class DeviceConfig(DeviceVariableGroup, Config):
    GROUP_NAME = 'config'
    SERIALIZATION_MAP = {
        'max_position': 'max_cart_x',
        'max_velocity': 'max_cart_v',
        'max_acceleration': 'max_cart_a',
        'hard_max_position': 'hw_max_x',
        'hard_max_velocity': 'hw_max_v',
        'hard_max_acceleration': 'hw_max_a',
        #'clamp_position': 'clamp_x',
        #'clamp_velocity': 'clamp_v',
        #'clamp_acceleration': 'clamp_a',
        #'debug_led': 'debug_led'
    }

    #debug_led: bool = dc.field(default=None)


@dc.dataclass
class DeviceState(DeviceVariableGroup, State):
    GROUP_NAME = 'state'
    SERIALIZATION_MAP = {
        'cart_position': 'curr_cart_x',
        'cart_velocity': 'curr_cart_v',
        'cart_acceleration': 'curr_cart_a',
        'pole_angle': 'curr_pole_x',
        'pole_angular_velocity': 'curr_pole_v',
        'error_code': 'errorCode',
        'accelerometer_value': 'curr_imu_a',
        #'motor_angle': 'motor_x',
        #'motor_velocity': 'motor_v',
    }
    cart_position: float = dc.field(default=None)
    cart_velocity: float = dc.field(default=None)
    cart_acceleration: float = dc.field(default=None)
    pole_angle: float = dc.field(default=None)
    pole_angular_velocity: float = dc.field(default=None)
    error_code : int = dc.field(default=None)
    accelerometer_value: float = dc.field(default=None)
    #motor_angle: float = dc.field(default=None)
    #motor_velocity: float = dc.field(default=None)


@dc.dataclass
class DeviceTarget(DeviceVariableGroup):
    GROUP_NAME = 'target'
    SERIALIZATION_MAP = {
        'position': 'target_cart_x',
        'velocity': 'target_cart_v',
        'acceleration': 'target_cart_a',
    }

    position: float = dc.field(default=None)
    velocity: float = dc.field(default=None)
    acceleration: float = dc.field(default=None)

class CartPoleDevice(CartPoleBase):
    def __init__(self, target_key='acceleration') -> None:

        # TinyFrame fields
        self.ser = serial.Serial(port="/dev/ttyUSB0", baudrate=115200)
        self.tf = TF.TinyFrame()
        self.tf.ID_BYTES = 1
        self.tf.TYPE_BYTES = 1
        self.tf.LEN_BYTES = 4
        self.tf.CKSUM_TYPE = 'crc16'
        self.tf.SOF_BYTE = 0x01
        self.tf.write = self.ser.write
        self.tf.add_type_listener(protocol_pb2.TARGETSTATE,self.state_listener)
        self.tf.add_type_listener(protocol_pb2.UPDATESTATE,self.state_listener)

        self.state = DeviceState() 

        self.step_count = 0
        self.target_key = target_key
        self.prev_angle = 0
        self.rotations = 0
        self.is_received = False
        self.state_timestamp = 0
        
    def reset(self, config: Config = None) -> None:

        config = config or Config.default()
        self.make_request(protocol_pb2.RESET, None, self._dataclass_to_protobuf(DeviceConfig, protocol_pb2.Config()).SerializeToString())
        self.step_count = 0
        pass

    def error_listener(self,_,frame):
        pass

    def state_listener(self,_, frame):
        self.is_received = True
        pb_state = protocol_pb2.State()
        pb_state.ParseFromString(frame.data)
        # ! Update class field at this point
        # TODO: Is this questionable?
        self.state : DeviceState = self._protobuf_to_dataclass(DeviceState(),pb_state)
        #self.state = State(pb_state.curr_cart_x,pb_state.curr_cart_v,pb_state.curr_pole_x,
                          #pb_state.curr_pole_v,pb_state.curr_cart_a,Error.NO_ERROR)
        print("New state received!")
        self.state_timestamp = time.time()

    def make_request(self,request_type ,listener ,payload):
        self.is_received = False
        print("Pre-query")
        self.tf.query(request_type ,listener ,payload)
        print("Post-query")
        while not self.is_received:
            buf = self.ser.read(1)
           #print(buf)
            if buf == b"":
                # TODO: raise runtime error here
                print("timeout")
            self.tf.accept(buf)
        print("Out of the loop")

    def get_state(self) -> State:
        # Remake this to work with older code, I guess
        time_elapsed = abs(self.state_timestamp - time.time())
        if time_elapsed > 0.1:
            self.update_state()
            #return self.state
            return self.state
        else:
            #return self.state
            return self.state

    def update_state(self):
        self.make_request(protocol_pb2.UPDATESTATE,self.state_listener,None)

    def set_target(self, target: float) -> None:
        # Get state through serial port
        # TODO: this works lol????
        self.make_request(protocol_pb2.TARGETSTATE,self.state_listener,self._dataclass_to_protobuf(DeviceTarget(**{self.target_key: target}),protocol_pb2.Target()).SerializeToString())

        # Update the state impliticty        
        curr = self.state.pole_angle
        prev = self.prev_angle
        max_delta = math.pi
        delta = curr - prev
        if delta > max_delta:
            self.rotations -= 1
        elif delta < -max_delta:
            self.rotations += 1
        abs_angle = 2 * math.pi * self.rotations + curr
        self.prev_angle = curr
        self.state.pole_angle = abs_angle
    
    def timestamp(self):
        return time.time()

    def advance(self, delta) -> None:
        pass  # TODO: ???

    def close(self) -> None:
        self.ser.close()

    def _protobuf_to_dataclass(self, dataclass, proto_obj):
        assert proto_obj is not None
        for field in dc.fields(dataclass):
            wire_name = dataclass.SERIALIZATION_MAP.get(field.name, field.name)
            value = getattr(proto_obj, wire_name, None)
            if value is not None:
                setattr(dataclass, field.name, value)
        return dataclass

    def _dataclass_to_protobuf(self, dataclass, proto_obj):
        for field in dc.fields(dataclass):
            wire_name = dataclass.SERIALIZATION_MAP.get(field.name)
            value = getattr(dataclass, field.name, None)
            if wire_name is not None and value is not None:
                setattr(proto_obj, wire_name, value)
        return proto_obj