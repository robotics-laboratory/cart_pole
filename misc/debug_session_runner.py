import time
from pathlib import Path
from device import CartPoleDevice, DeviceTarget
from sessions.actor import Actor
from sessions.collector import CollectorProxy
from common.interface import Config, CartPoleBase
from common.util import init_logging
from misc.analyzer._saleae import SaleaeAnalyzer
import logging
# Actor imports
from misc.oscillating_actor import OscillatingActor
# from misc.lqr_actor import LinearBalanceControl
from misc.demo_actor import DemoActor
from misc.const_actor import ConstantActor
from control.calibration import Calibrator

LOGGER = logging.getLogger('debug-session-runner')


def control_loop(device: CartPoleBase, actor: Actor, max_duration: float):
    start = time.perf_counter()
    while time.perf_counter() - start < max_duration:
        state = device.get_state()
        stamp = time.perf_counter() - start
        target = actor(state, stamp=stamp)
        logging.info("STAMP: %s", stamp)
        device.set_target(target)
        device.advance()


if __name__ == '__main__':
    from common.util import init_logging

    init_logging()

    SESSION_ID = 'calibrate + top_pos'
    ACTOR_CLASS = DemoActor
    DEVICE_CONFIG = Config(
        max_position=0.26,
        max_velocity=5,
        max_acceleration=10.0,
        clamp_velocity=True,
        clamp_acceleration=True, )
    ACTOR_CONFIG = dict(config=Config(
        max_position=0.20,
        max_velocity=4,
        max_acceleration=10.0,
        pole_length=0.28,
    ))

    SESSION_MAX_DURATION = 40.0
    OUTPUT_PATH = Path(f'data/sessions/{SESSION_ID}')
    init_logging()
    device = CartPoleDevice()
    # analyzer = SaleaeAnalyzer()
    proxy = CollectorProxy(
        cart_pole=device,
        actor_class=ACTOR_CLASS,
        actor_config=ACTOR_CONFIG,
        # reset_callbacks=[analyzer.start],
        # close_callbacks=[analyzer.stop],
    )

    try:
        calibrator = Calibrator(DEVICE_CONFIG, proxy)
        new_config = calibrator.calibrate()
        device.rotations = 0
        device.prev_angle = 0
        time.sleep(3.0)
        ACTOR_CONFIG["config"] = new_config
        #proxy.actor_config["pole_length"] = new_config.pole_length

         #proxy = CollectorProxy(
        #     cart_pole=device,
        #     actor_class=ACTOR_CLASS,
        #     actor_config=ACTOR_CONFIG,
        #     # reset_callbacks=[analyzer.start],
        #     # close_callbacks=[analyzer.stop],
        # )
        #proxy.reset(new_config)
        actor = DemoActor(config=Config(
            max_position=0.20,
            max_velocity=4,
            max_acceleration=10.0,
            pole_length=new_config.pole_length
        ))
        actor.proxy = proxy


        # LOGGER.info(">>>")
        # device.interface.set(DeviceTarget(position=0.05))
        # LOGGER.info("<<<")
        control_loop(proxy, actor, max_duration=SESSION_MAX_DURATION)
        # print(length)
    except Exception:
        LOGGER.exception('Aborting run due to error')
    finally:
        proxy.close()
        LOGGER.info('Run finished')

    proxy.save(OUTPUT_PATH / 'session.json')
    # analyzer.save(OUTPUT_PATH)
