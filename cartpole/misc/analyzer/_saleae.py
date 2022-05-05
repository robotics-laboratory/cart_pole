import csv
import logging
import os
import time
from collections import defaultdict
from pathlib import Path
from typing import Optional, Union, NamedTuple
import psutil

LOGGER = logging.getLogger(__name__)

try:
    from saleae import Saleae
except ImportError:
    LOGGER.warning('Failed to import saleae package - proceed at your own risk')


def launch_logic(
        settings: Optional[Union[str, Path]] = None,
        kill_existing: bool = True,
) -> Saleae:
    if kill_existing:
        for proc in psutil.process_iter():
            if proc.name().lower().startswith('logic'):
                LOGGER.debug('Killing logic process: %s [%s]', proc.name(), proc.pid)
                proc.terminate()
                proc.wait(1)

    api = Saleae(args='-disablepopups')
    default_settings = Path(__file__).parent / 'cartpole.logicsettings'
    settings = Path(settings or os.environ.get('LOGIC_SETTINGS', default_settings))
    api.load_from_file(str(settings))
    return api


def prepare_output_path(
        path: Optional[Union[str, Path]],
        fallback: Optional[str] = None,
) -> Path:
    path = Path(path)
    if fallback is not None and not path.match('*.*'):
        path = path / fallback
    parent_dir = path.parent if path.match('*.*') else path
    parent_dir.mkdir(parents=True, exist_ok=True)
    return path


class SaleaeAnalyzer:
    '''
    Integration for Saleae logic analyzer software.

    Features:
    * Automatically launch and configure logic software via *.logicsettings file
    * Start and stop capture, load collected data to python
    * Parse analyzers output (SPI, I2C etc.) via custom python parsers (see below)

    To setup, please refer to Saleae docs and official python wrapper:
    * https://support.saleae.com/saleae-api-and-sdk/socket-api
    * https://github.com/saleae/SaleaeSocketApi/tree/master/Doc
    * https://github.com/ppannuto/python-saleae
    '''

    Flip = NamedTuple('Flip', time=float, value=bool)
    SerialData = NamedTuple(
        'SerialData', time=float, data=int, parity_err=bool, framing_err=bool
    )
    I2CData = NamedTuple(
        'I2CData', time=float, id=int, address=int, data=int, read=bool, ack=bool
    )

    def __init__(self):
        self.api = launch_logic()
        sample_rate = float(os.environ.get('SALEAE_SAMPLE_RATE', 1e6))
        self.api.set_sample_rate_by_minimum(digital_minimum=sample_rate)

    def start(self):
        self.api.capture_start()

    def stop(self):
        self.api.capture_stop()

    def save(self, export_path: Union[str, Path]):
        path = prepare_output_path(export_path)
        self._export_full_capture(path)
        self._export_signal_data(path)
        self._export_plugins_data(path)

    def _export_full_capture(self, path):
        path = path / 'capture.logicdata'
        LOGGER.info('Exporting full capture -> %s', path)
        self.api.save_to_file(str(path))

    def _export_signal_data(self, path):
        path = path / 'signal.csv'
        LOGGER.info('Exporting signal data -> %s', path)
        channels = self.api.get_active_channels()[0]  # Active digital channels
        self.api.export_data2(str(path), digital_channels=channels, display_base='bin')

    def _export_plugins_data(self, path):
        for name, index in self.api.get_analyzers():
            name = name.strip().lower().replace(' ', '_')
            export_path = path / f'{index}_{name}.csv'
            LOGGER.info('Exporting plugin data -> %s', export_path)
            self.api.export_analyzer(index, export_path)
    #
    # def _load_signal_data(self, path):
    #     signal_data = defaultdict(lambda: [self.Flip(time=-1, value=False)])
    #     with open(path) as file:
    #         reader = csv.DictReader(file, delimiter=',', fieldnames=['time', 'data'])
    #         next(reader, None)  # Skip header
    #         for item in reader:
    #             timestamp = float(item['time'])
    #             bits = map(lambda x: bool(int(x)), item['data'].strip())
    #             for channel, value in enumerate(bits):
    #                 _, prev_value = signal_data[channel][-1]
    #                 if value != prev_value:
    #                     flip = self.Flip(time=timestamp, value=bool(value))
    #                     signal_data[channel].append(flip)
    #     for values in signal_data.values():
    #         values.pop(0)  # Remove border element
    #     return signal_data
    #
    # def _load_plugin_data(self, path, name):
    #     with open(path) as file:
    #         if name == 'async_serial':
    #             return self._load_serial(file)
    #         elif name == 'i2c':
    #             return self._load_i2c(file)
    #         else:
    #             LOGGER.warning('Failed to load (unknown plugin): %s', name)
    #
    # def _load_serial(self, file):
    #     fields = ['time', 'data', 'parity_err', 'framing_err']
    #     reader = csv.DictReader(file, fieldnames=fields, delimiter=',')
    #     next(reader, None)  # Skip header
    #     return [
    #         self.SerialData(
    #             time=float(item['time']),
    #             data=int(item['data'], 16),
    #             parity_err=bool(item.get('parity_err')),
    #             framing_err=bool(item.get('framing_err')),
    #         )
    #         for item in reader
    #     ]
    #
    # def _load_i2c(self, file):
    #     fields = ['time', 'id', 'address', 'data', 'read', 'ack']
    #     reader = csv.DictReader(file, fieldnames=fields, delimiter=',')
    #     next(reader, None)  # Skip header
    #     return [
    #         self.I2CData(
    #             time=float(item['time']),
    #             id=int(item['id'], 16),
    #             address=int(item['address'], 16),
    #             data=int(item['data'], 16),
    #             read=item['read'].lower() == 'read',
    #             ack=item['ack'].lower() == 'ack',
    #         )
    #         for item in reader
    #     ]


if __name__ == '__main__':
    from device import CartPoleDevice
    from common.util import init_logging

    init_logging()
    device = CartPoleDevice()
    analyzer = SaleaeAnalyzer()
    analyzer.start()
    for _ in range(5):
        device.get_state()
    analyzer.stop()
    analyzer.save('data/logic_export/test')
