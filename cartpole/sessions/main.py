import json

from cartpole.device.wire_interface import WireInterface, DeviceConfig
from cartpole.sessions.collector import CollectorProxy
from cartpole.sessions.runner import Runner
from cartpole.sessions.actor import OscillatingActor, Actor
from cartpole.device import CartPoleDevice
from cartpole.common.util import init_logging
from cartpole.misc.analyzer._saleae import SaleaeAnalyzer

import logging


class NoActor(Actor):
    def __call__(self, *args, **kwargs):
        return 0


def main():

    device = CartPoleDevice(WireInterface('COM4'))
    device_config = DeviceConfig(max_velocity=0.25, max_acceleration=1)
    actor_config = {
        'device_config': device_config,
        'acceleration': 0.5,
        'max_position': 0.1
    }
    runner = Runner(device, device_config, OscillatingActor, actor_config)
    analyzer = SaleaeAnalyzer()
    runner.proxy.reset_callbacks.append(analyzer.start)
    runner.proxy.close_callbacks.append(analyzer.stop)

    # runner.start_server()
    runner.run(100)
    session_id = 'v3'
    runner.proxy.save(f'data/sessions/{session_id}.json')
    analyzer.export_analyzers()
    with open(f'data/sessions/{session_id}-analyzer.json', 'w') as file:
        json.dump(analyzer.data, file)



if __name__ == '__main__':
    init_logging()
    main()
