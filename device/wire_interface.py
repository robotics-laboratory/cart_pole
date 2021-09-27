import dataclasses as dc
import logging
import serial
from typing import Union, Type

from common.interface import Error, Config, State


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
        return ' '.join(self.SERIALIZATION_MAP[field.name] for field in dc.fields(self) if getattr(self, field.name) is not None)

    @classmethod
    def from_dict_format(cls, text: str) -> 'DeviceVariableGroup':
        '''
        Parses input text into new class instance.

        Returns:
            Class instance, constructed from wire format representation.
        '''
        field_lookup = {cls.SERIALIZATION_MAP[field.name]: field for field in dc.fields(cls)}
        data_dict = {}
        for pair in text.split():
            wire_name, value = pair.split('=')
            field = field_lookup[wire_name]
            from_string = cls._type_lookup(field.type, cls._from_string_lookup)
            data_dict[field.name] = from_string(value)
        return cls(**data_dict)


class DeviceConfig(DeviceVariableGroup, Config):
    GROUP_NAME = 'config'
    SERIALIZATION_MAP = {
        'max_position': 'max_x',
        'max_velocity': 'max_v',
        'max_acceleration': 'max_a',
        'hardware_max_position': 'hw_max_x',
        'hardware_max_velocity': 'hw_max_v',
        'hardware_max_acceleration': 'hw_max_a',
        'clamp_position': 'clamp_x',
        'clamp_velocity': 'clamp_v',
        'clamp_acceleration': 'clamp_a',
        'debug_led': 'debug_led'
    }

    debug_led: bool = dc.field(default=None)


class DeviceState(DeviceVariableGroup, State):
    GROUP_NAME = 'state'
    SERIALIZATION_MAP = {
        'position': 'x',
        'velocity': 'v',
        'acceleration': 'a',
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
        'position': 'x',
        'velocity': 'v',
        'acceleration': 'a',
    }

    position: float = dc.field(default=None)
    velocity: float = dc.field(default=None)
    acceleration: float = dc.field(default=None)


class WireInterface:
    def __init__(self, 
        port: str = '/dev/ttyS0', 
        baud_rate: int = 115200, 
        read_timeout: float = 0.2, 
        write_timeout: float = 0.2,
    ) -> None:
        self.serial = serial.Serial(port=port, baudrate=baud_rate, timeout=read_timeout, write_timeout=write_timeout, exclusive=True)
        LOGGER.info(f'Opened serial connection to {self.serial.name}')

    def close(self):
        self.serial.close()

    def request(self, command: str) -> str:
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
                LOGGER.debug(f'Received processing message during "{command}" request')
                continue

            stripped = received[1:].strip()
            if received.startswith('#'):
                LOGGER.debug(f'Received log message during "{command}" request: {stripped}')
                continue
            elif received.startswith('!'):
                LOGGER.error(f'Received error response during "{command}" request: {stripped}')
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

    def get(self, params: Union[DeviceConfig, DeviceState, DeviceTarget]) -> DeviceVariableGroup:
        response = self._command('get', params.GROUP_NAME, params.to_list_format())
        return params.from_dict_format(response)

    def reset(self) -> None:
        _ = self._command('reset')
