from device.wire_interface import WireInterface, DeviceConfig, DeviceTarget
from collector import CollectorProxy
from device import CartPoleDevice
import logging
import balance_lqr


def main():
    wire = WireInterface('COM4')
    cartpole = CartPoleDevice(wire, target_key='acceleration')
    p = CollectorProxy(cartpole, interval=0.1)
    p.reset(DeviceConfig(debug_led=True, max_velocity=2.0, max_acceleration=10.0, clamp_acceleration=True))
    p.start()

    try:
        # CONTROL THE DEVICE HERE
        balance_lqr.main(p, max_iters=1000, timedelta=0)
    except:
        logging.exception("Exception during session")

    # p.cart_pole.interface.set(DeviceTarget(position=0))
    p.stop()
    p.cart_pole.interface.set(DeviceConfig(debug_led=False))


if __name__ == '__main__':
    FORMAT = '%(asctime)s [%(levelname)s] %(name)s :: %(message)s'
    logging.basicConfig(format=FORMAT)
    logging.getLogger().setLevel(logging.DEBUG)
    main()
