import dataclasses as dc
import logging
import time

import google.protobuf.message
import serial
import varint
import threading
from typing import Union, Type, Any

import cartpole.device.controller_pb2 as proto
from cartpole.common.interface import Error, Config, State
import os

LOGGER = logging.getLogger(__name__)
RAW_COMMANDS_LOGGER = logging.getLogger('raw_commands')


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
        'max_position': 'max_x',
        'max_velocity': 'max_v',
        'max_acceleration': 'max_a',
        'hard_max_position': 'hw_max_x',
        'hard_max_velocity': 'hw_max_v',
        'hard_max_acceleration': 'hw_max_a',
        'clamp_position': 'clamp_x',
        'clamp_velocity': 'clamp_v',
        'clamp_acceleration': 'clamp_a',
        'debug_led': 'debug_led'
    }

    debug_led: bool = dc.field(default=None)


@dc.dataclass
class DeviceState(DeviceVariableGroup, State):
    GROUP_NAME = 'state'
    SERIALIZATION_MAP = {
        'cart_position': 'curr_x',
        'cart_velocity': 'curr_v',
        'cart_acceleration': 'curr_a',
        'pole_angle': 'pole_x',
        'pole_angular_velocity': 'pole_v',
        'error_code': 'errcode',
        'accelerometer_value': 'imu_a',
        'motor_angle': 'motor_x',
        'motor_velocity': 'motor_v',
    }

    accelerometer_value: float = dc.field(default=None)
    motor_angle: float = dc.field(default=None)
    motor_velocity: float = dc.field(default=None)


@dc.dataclass
class DeviceTarget(DeviceVariableGroup):
    GROUP_NAME = 'target'
    SERIALIZATION_MAP = {
        'position': 'trgt_x',
        'velocity': 'trgt_v',
        'acceleration': 'trgt_a',
    }

    position: float = dc.field(default=None)
    velocity: float = dc.field(default=None)
    acceleration: float = dc.field(default=None)


class WireInterface:
    def __init__(self,
                 port: str = None,
                 baud_rate: int = None,
                 read_timeout: float = 3,
                 write_timeout: float = 3,
                 ) -> None:
        port = port or os.environ.get('SERIAL_PORT')
        assert port is not None, 'Please set "port" argument or "SERIAL_PORT" env var'
        baud_rate = baud_rate or int(os.environ.get('SERIAL_SPEED'))
        self.serial = serial.Serial(port=port, baudrate=baud_rate, timeout=read_timeout,
                                    write_timeout=write_timeout, exclusive=True)
        self._lock = threading.Lock()
        self.serial.flushInput()
        self.serial.flushOutput()
        LOGGER.info(f'Opened serial connection to {self.serial.name}')

    def close(self):
        self.serial.close()

    def request(self, command: str) -> str:
        with self._lock:
            command = command.strip()

            LOGGER.debug(f'Request to serial connection "{command}"')
            RAW_COMMANDS_LOGGER.debug(command)
            self.serial.write((command + '\n').encode('utf-8'))

            while True:
                received = self.serial.readline().decode('utf-8').strip()
                if received == '':
                    LOGGER.error(f'Serial read timeout during "{command}" request')
                    continue

                RAW_COMMANDS_LOGGER.debug(received)

                if received.startswith('~'):
                    LOGGER.debug(
                        f'Received processing message during "{command}" request')
                    continue

                stripped = received[1:].strip()
                if received.startswith('#'):
                    LOGGER.debug(
                        f'Received log message during "{command}" request: {stripped}')
                    continue
                elif received.startswith('!'):
                    LOGGER.error(
                        f'Received error response during "{command}" request: {stripped}')
                    raise RuntimeError(f'Received error response: {stripped}')
                elif received.startswith('+'):
                    LOGGER.debug(f'Responding to request "{command}" with "{received}"')
                    return stripped
                else:
                    LOGGER.debug(f'Received unknown response line: {received}')

    def _command(self, command: str, group: str = '', args: str = '') -> str:
        return self.request(f'{command} {group} {args}')

    def set(self, params: Union[DeviceConfig, DeviceTarget]) -> DeviceVariableGroup:
        response = self._command('set', params.GROUP_NAME, params.to_dict_format())
        return params.from_dict_format(response)

    def get(self, params: Union[
        DeviceConfig, DeviceState, DeviceTarget]) -> DeviceVariableGroup:
        response = self._command('get', params.GROUP_NAME, params.to_list_format())
        return params.from_dict_format(response)

    def reset(self) -> None:
        _ = self._command('reset')


class ProtobufWireInterface(WireInterface):
    def set(self, params: Union[DeviceConfig, DeviceTarget]) -> DeviceVariableGroup:
        if isinstance(params, DeviceConfig):
            config = self._dataclass_to_protobuf(params, proto.Config())
            res = self._request(proto.RequestType.SET_CONFIG, config=config)
            return self._protobuf_to_dataclass(DeviceConfig(), res.config)
        if isinstance(params, DeviceTarget):
            target = self._dataclass_to_protobuf(params, proto.Target())
            res = self._request(proto.RequestType.SET_TARGET, target=target)
            return self._protobuf_to_dataclass(DeviceTarget(), res.target)
        raise NotImplementedError

    def get(self, params: Union[DeviceConfig, DeviceState, DeviceTarget]) -> DeviceVariableGroup:
        if isinstance(params, DeviceConfig):
            res = self._request(proto.RequestType.GET_CONFIG)
            return self._protobuf_to_dataclass(DeviceConfig(), res.config)
        if isinstance(params, DeviceState):
            res = self._request(proto.RequestType.GET_STATE)
            return self._protobuf_to_dataclass(DeviceState(), res.state)
        if isinstance(params, DeviceTarget):
            res = self._request(proto.RequestType.GET_TARGET)
            return self._protobuf_to_dataclass(DeviceTarget(), res.target)
        raise NotImplementedError

    def reset(self) -> None:
        self._request(proto.RequestType.RESET)

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

    def _error(self, message):
        LOGGER.error(message)
        raise RuntimeError(message)

    def _request(
            self, type: proto.RequestType, config=None, state=None, target=None
    ) -> proto.Response:
        with self._lock:
            request = proto.Request()
            request.type = type
            if config is not None:
                request.config.CopyFrom(config)
            if state is not None:
                request.state.CopyFrom(state)
            if target is not None:
                request.target.CopyFrom(target)
            payload = request.SerializeToString()
            LOGGER.debug(f'Serial TX payload size: {len(payload)}')
            data = varint.encode(len(payload)) + payload
            LOGGER.debug(f'Serial TX data: {data!r}')
            self.serial.write(data)
            return self._read_loop()

    def _read_loop(self):
        while True:
            try:
                size = varint.decode_stream(self.serial)
            except TypeError:
                self._error('Varint decode failed ')
            LOGGER.debug(f'Serial RX payload size: {size}')
            data = self.serial.read(size)
            LOGGER.debug(f'Serial RX data: {data!r}')
            if len(data) < size:
                self._error('Serial read timeout')
            try:
                response = proto.Response()
                response.ParseFromString(data)
            except google.protobuf.message.DecodeError:
                self._error(f'Protobuf parse error (data: {data!r})')
            if response.status == proto.ResponseStatus.PROCESSING:
                LOGGER.info('Received processing message...')
                continue
            if response.status == proto.ResponseStatus.DEBUG:
                LOGGER.info(f'Received debug message: {response.message}')
                continue
            if response.status == proto.ResponseStatus.ERROR:
                self._error(f'Received error message: {response.message}')
            return response
