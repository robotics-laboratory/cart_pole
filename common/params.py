import dataclasses

@dataclasses.dataclass
class Params:
    cart_mass: float = 0     # kg
    pole_mass: float = 0.118 # kg
    pole_length: float = 0.3 # m
    gravity: float = 9.8     # m/s^2
