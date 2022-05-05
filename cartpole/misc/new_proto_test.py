from cartpole.device import ProtobufWireInterface, DeviceConfig, DeviceState, DeviceTarget
from cartpole.common.util import init_logging
import logging
import time

if __name__ == '__main__':

    # debug_message = proto.Response()
    # debug_message.status = proto.ResponseStatus.DEBUG
    # debug_message.message = 'hello!'
    # print(debug_message.SerializeToString())

    init_logging()
    interface = ProtobufWireInterface(read_timeout=100, write_timeout=100)

    # logging.info('RESET')
    # interface.reset()
    # logging.info('GET ALL')
    # logging.info(interface.get(DeviceState()))
    # logging.info(interface.get(DeviceConfig()))
    # logging.info(interface.get(DeviceTarget()))
    # logging.info('SET ALL')
    # logging.info(interface.set(DeviceTarget(position=0.1)))
    # logging.info(interface.set(DeviceConfig(max_velocity=0.1)))
    # time.sleep(2.0)
    # logging.info(interface.set(DeviceTarget(position=-0.1)))
    # logging.info('GET ALL')
    # logging.info(interface.get(DeviceState()))
    # logging.info(interface.get(DeviceConfig()))
    # logging.info(interface.get(DeviceTarget()))

    start = time.perf_counter()
    packets_count = 1000
    for _ in range(packets_count):
        logging.info('GET ALL')
        logging.info(interface.get(DeviceState()))
        # logging.info(interface.get(DeviceConfig()))
        # logging.info(interface.get(DeviceTarget()))
    delta = time.perf_counter() - start
    freq = packets_count / delta
    print(f'{packets_count} packets in {delta:.3f} s = {freq:.3f} Hz')
