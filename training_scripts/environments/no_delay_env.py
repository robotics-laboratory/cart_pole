import numpy as np

from simulator import CartPoleSimulator


class NoDelayEnvCartpoleEnv:
    DELAY = 0.02 # gym cartpole delay

    def __init__(self, steps_threshold=1000):
        self._simulator = CartPoleSimulator(debug_mode=False)
        self._curr_step = 0
        self._steps_threshold = steps_threshold
        self._state = self.reset()

    def close(self):
        self._simulator.close()

    def reset(self):
        s, _ = self._simulator.reset()
        return s

    def play_and_record(
        self,
        agent,
        experience_replay,
        n_steps,
        epsilon
    ):
        rewards = []

        for _ in range(n_steps):
            target = agent.action([self._state], epsilon=epsilon)[0]

            self._simulator.set_target(self.target_to_delta(target))
            self._simulator.step(self.DELAY)
            self._curr_step += 1

            done = self._curr_step == self._steps_threshold
            new_state = self._simulator.get_state()
            reward = self._compute_reward(new_state)

            experience_replay.append((self._state, target, reward, new_state, done))

            rewards.append(reward)
            self._state = new_state

            if done:
                self._state, _ = self._simulator.reset()
                self._curr_step = 0
        return rewards

    def evaluate(self, agent, n_games, epsilon):
        rewards = []

        for _ in range(n_games):
            self._state = self.reset()
            reward = 0
            for _ in range(self._steps_threshold):
                target = agent.action([self._state], epsilon=epsilon)[0]
                self._simulator.set_target(self.target_to_delta(target))
                self._simulator.step(self.DELAY)

                new_state = self._simulator.get_state()
                reward += self._compute_reward(new_state)

                self._state = new_state
        
            rewards.append(reward)

        self._state = self.reset()
        return np.mean(rewards)


    def _compute_reward(self, state):
        return np.exp(-np.cos(state.pole_angle))

    def target_to_delta(self, target):
        if target == 0:
            return -0.01
        elif target == 1:
            return +0.01
        else:
            raise NotImplementedError
