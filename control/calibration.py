from common import Config, State
from math import sin, cos
from IPython.display import HTML
import matplotlib.pyplot as plt
import numpy as np
import scipy.signal
import json
import time


class Calibrator:
    def __init__(self, config: Config, cart_pole):
        self.config = config
        self._cart_pole = cart_pole

        self._moves_num = 0
        self._acceleration = 0
        self._length = 0
        self._zero_angle = 0
        self._last_acc_change_timestamp = None
        self._first_acc_duration = None
        self._start_timestamp = None
        self.raw_data = []
        self.processed_data = []

    def _braking_distance(self):
        dist = -0.018 * self.raw_data[-1]["state"].cart_velocity
        return dist

    def _pos_turned_positive(self):
        return (self.raw_data[-1]["state"].cart_position >= self._braking_distance()) \
               and (self.raw_data[-4]["state"].cart_position <= self._braking_distance()) \
               and (self._last_acc_change_timestamp + 0.15 < self.raw_data[-1]["timestamp"])

    def _pos_turned_negative(self):
        return (self.raw_data[-1]["state"].cart_position <= self._braking_distance()) \
               and (self.raw_data[-4]["state"].cart_position >= self._braking_distance()) \
               and (self._last_acc_change_timestamp + 0.15 < self.raw_data[-1]["timestamp"])

    def _reset(self):
        self._cart_pole.reset(self.config)  # todo: update config's max_position
        time.sleep(3.0)

        self._length = self.config.max_position * 0.75
        self._acceleration = (min(self.config.max_acceleration, (self.config.max_velocity ** 2) / (2 * self._length))
                              / (self._moves_num + 1)) * 0.5
        self._first_acc_duration = (self._length / self._acceleration) ** 0.5
        self._zero_angle = self._cart_pole.get_state().pole_angle

    def _collect_data(self):
        target = self._acceleration
        self._start_timestamp = self._cart_pole.timestamp()
        timestamp = self._start_timestamp - self._start_timestamp
        state = self._cart_pole.get_state()
        self.raw_data.append({"state": state, "timestamp": timestamp, "target": 0})
        self._last_acc_change_timestamp = timestamp
        move_index = 0

        while timestamp < self._first_acc_duration:
            self._cart_pole.set_target(target)
            state = self._cart_pole.get_state()
            state.pole_angle -= self._zero_angle
            timestamp = self._cart_pole.timestamp() - self._start_timestamp
            self.raw_data.append({"state": state, "timestamp": timestamp, "target": target})

        target = -target

        while move_index < self._moves_num:
            self._cart_pole.set_target(target)
            state = self._cart_pole.get_state()
            state.pole_angle -= self._zero_angle
            timestamp = self._cart_pole.timestamp() - self._start_timestamp
            self.raw_data.append({"state": state, "timestamp": timestamp, "target": target})

            if self._pos_turned_negative():
                move_index += 1
                target = abs(target)
                self._last_acc_change_timestamp = timestamp

            if self._pos_turned_positive():
                target = (-abs(target)) * ((move_index + 1) / move_index)
                self._last_acc_change_timestamp = timestamp

    def _update_params(self):
        timestamps = []
        angles = []
        targets = []
        expressions = []  # expression: -((x¨)cos(θ) + g*sin(θ))
        omegas = []
        epsilons = []

        omega_delta = 3
        epsilon_indent = 5

        for i in range(0, len(self.raw_data)):
            timestamps.append(self.raw_data[i]["timestamp"])
            angles.append(self.raw_data[i]["state"].pole_angle)
            targets.append(self.raw_data[i]["target"])
        angles = scipy.signal.savgol_filter(angles, 21, 3)  # window size 21, polynomial order 3

        for i in range(omega_delta, len(angles) - omega_delta):
            angle_delta = angles[i + omega_delta] - angles[i - omega_delta]
            time_delta = timestamps[i + omega_delta] - timestamps[i - omega_delta]
            omegas.append(angle_delta / time_delta)
        omegas = scipy.signal.savgol_filter(omegas, 15, 3)  # window size 15, polynomial order 3

        for i in range(0, len(omegas) - epsilon_indent):
            omega_delta = omegas[i + epsilon_indent] - omegas[i]
            time_delta = timestamps[i + epsilon_indent + omega_delta] - timestamps[i + omega_delta]
            epsilons.append(omega_delta / time_delta)
        epsilons = list(scipy.signal.savgol_filter(epsilons, 15, 3))  # window size 15, polynomial order 3

        angles = angles[omega_delta:(- epsilon_indent - omega_delta)]
        timestamps = timestamps[omega_delta:(- epsilon_indent - omega_delta)]
        targets = targets[omega_delta:(- epsilon_indent - omega_delta)]
        omegas = omegas[:(-epsilon_indent)]

        for i in range(len(angles)):
            expressions.append(-((targets[i] * cos(angles[i])) + (self.config.gravity * sin(angles[i]))))
            self.processed_data.append({"angle": angles[i], "omega": omegas[i], "epsilon": epsilons[i],
                                        "timestamp": timestamps[i], "target": targets[i], "expression": expressions[i]})

        A = np.array(expressions)
        B = np.vstack([np.array(epsilons), np.zeros(len(epsilons))]).T

        self.config.pole_length, self.config.friction_coefficient = np.linalg.lstsq(B, A)[0]

    def calibrate(self, moves_num=10):
        self._moves_num = moves_num

        self._reset()
        self._collect_data()
        self._cart_pole.reset(self.config)
        self._update_params()

        return self.config

    def save_processed_data(self):
        with open("processed_data.json", 'w') as file:
            json.dump(self.processed_data, file)

    def save_raw_data(self):
        with open("raw_data.json", 'w') as file:
            json.dump(self.processed_data, file)

    def import_data(self):
        with open(r"data.json", "r") as file:
            self.raw_data = json.load(file)

    def plot_processed_data(self):
        pass

    def plot_raw_data(self):
        pass
