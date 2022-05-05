import time

from serial import Serial, PARITY_EVEN
from os import environ as env
from random import randint

port = env.get('SERIAL_PORT', 'COM4')
speed = 1_000_000
serial = Serial(port=port, baudrate=speed, timeout=1)

chunk_size = 1
ok_count = 0
ok_window_size = 1000
start = time.perf_counter()
while True:
    byte = bytes([randint(0, 255) for _ in range(chunk_size)])
    serial.write(byte)
    serial.flush()
    in_byte = serial.read(chunk_size)
    if byte != in_byte:
        print(f'ERR: {byte} != {in_byte}')
        print('Extra:', serial.read_all().decode(errors='ignore'))
    else:
        ok_count += 1
        if ok_count == ok_window_size:
            end = time.perf_counter()
            delta = end - start
            start = end
            speed_kbitps = ok_window_size / delta * 8 / 1024 * chunk_size
            print(f'{ok_window_size} ok packets in {delta:.3f} s = {speed_kbitps:.3f} kbit/s')
            ok_count = 0
    # print('OK')
