import matplotlib.pyplot as plt
import matplotlib.animation as animation

import math


def generate_pyplot_animation(
        config,
        trajectory,
        trajectory_expected=None,
        timestamps=None,
        margin=0.1):
    if trajectory_expected is not None:
        assert len(trajectory) == len(trajectory_expected)

    if timestamps is not None:
        assert len(trajectory) == len(timestamps)

    x_lim = config.max_position + config.pole_length + margin
    y_lim = config.pole_length + margin

    fig, ax = plt.subplots(figsize=(16, 8))
    ax.set_title('CartPole')
    ax.set_xlim(-x_lim, +x_lim)
    ax.set_ylim(-y_lim, +y_lim)
    ax.set_aspect('equal')
    ax.grid(linestyle='--')

    pole, = ax.plot([], [], 'o-', lw=2, c='red')
    pole_expected, = ax.plot([], [], 'o-', lw=4, c='gray', alpha=0.3)
    timestamp_text = ax.text(0.05, 0.9, '', transform=ax.transAxes, fontsize=16)

    def generate_pole(state):
        x = state.cart_position
        a = state.pole_angle
    
        l = config.pole_length

        return [x, x + math.sin(a)*l],  [0, -math.cos(a)*l]

    def animate(i):
        pole_x, pole_y = generate_pole(trajectory[i])
        pole.set_data(pole_x, pole_y)

        if trajectory_expected:
            pole_x, pole_y = generate_pole(trajectory_expected[i])
            pole_expected.set_data(pole_x, pole_y)
    
        if timestamps is not None:
            timestamp_text.set_text(f't={timestamps[i]:.3f}s')

        return pole, pole_expected

    anim = animation.FuncAnimation(fig, animate, len(trajectory))
    plt.close(fig)

    return anim


