import argparse
from pydoc import locate

import yaml
from tqdm import trange
import numpy as np
import torch
import torch.nn as nn
from torch.utils.tensorboard import SummaryWriter

from training_scripts.models.dqn_agent import DQNAgent
from training_scripts.environments.no_delay_env import NoDelayEnvCartpoleEnv
from training_scripts.utils.replay_buffer import ReplayBuffer
from training_scripts.utils.epsilon_generator import LinearEpsilonGenerator


DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
NUM_ACTIONS = 2
STATE_SIZE = 4


def parse_config(config_path):
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)

    return config

def compute_loss(
    states,
    targets,
    rewards,
    next_states,
    done,
    agent,
    target_network,
    gamma
):
    np_states = np.array([state.as_np_vector() for state in states])
    np_next_states = np.array([state.as_np_vector() for state in next_states])

    torch_states = torch.tensor(np_states, device=DEVICE, dtype=torch.float32)
    targets = torch.tensor(targets, device=DEVICE, dtype=torch.int64)
    rewards = torch.tensor(rewards, device=DEVICE, dtype=torch.float32)
    torch_next_states = torch.tensor(np_next_states, device=DEVICE, dtype=torch.float32)
    is_done = torch.tensor(done, device=DEVICE, dtype=torch.uint8)

    predicted_q_values = agent(torch_states)

    with torch.no_grad():
        predicted_next_q_values = target_network(torch_next_states)

    predicted_q_values_for_actions = torch.gather(predicted_q_values, 1, targets.unsqueeze(1))

    next_state_values = torch.max(predicted_next_q_values, dim=1).values

    target_q_values_for_actions = rewards + gamma * next_state_values
    target_q_values_for_actions = torch.where(is_done, rewards, target_q_values_for_actions)

    loss = torch.mean((predicted_q_values_for_actions - target_q_values_for_actions) ** 2)

    return loss


def train(config, writer):
    agent = DQNAgent(state_size=STATE_SIZE, num_actions=NUM_ACTIONS,
                     **config['model_config']).to(DEVICE)
    target_network = DQNAgent(state_size=STATE_SIZE, num_actions=NUM_ACTIONS,
                              **config['model_config']).to(DEVICE)
    target_network.load_state_dict(agent.state_dict())

    opt_class = locate(config['optimizer_config'].pop('class'))
    opt = opt_class(agent.parameters(), **config['optimizer_config'])

    replay_buffer = ReplayBuffer(**config['buffer_config'])
    epsilon_generator = LinearEpsilonGenerator(**config['epsilon_generator_config'])
    env = NoDelayEnvCartpoleEnv(**config['env_config'])
    
    gamma = config['training_config']['gamma']
    num_steps = config['training_config']['num_steps']
    steps_per_update = config['training_config']['steps_per_update']
    target_update_steps = config['training_config']['target_update_steps']
    max_grad_norm = config['training_config']['max_grad_norm']
    eval_reward_freq = config['training_config']['eval_reward_freq']
    eval_reward_repeats = config['training_config']['eval_reward_repeats']
    
    for step in trange(num_steps):
        opt.zero_grad()
        current_epsilon = epsilon_generator.get_epsilon(step)

        env.play_and_record(agent, replay_buffer, steps_per_update, current_epsilon)

        states, targets, rewards, new_states, done = replay_buffer.sample()

        loss = compute_loss(states, targets, rewards, new_states, done,
                            agent, target_network, gamma)
        loss.backward()
        grad_norm = nn.utils.clip_grad_norm_(agent.parameters(), max_grad_norm)
        opt.step()

        if step % target_update_steps == 0:
            target_network.load_state_dict(agent.state_dict())

        writer.add_scalar('loss', loss.item(), step)
        writer.add_scalar('grad_norm', grad_norm, step)
        if step % eval_reward_freq == 0:
            writer.add_scalar('reward', env.evaluate(agent, eval_reward_repeats, 0.), step)



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--config-path', required=True)
    parser.add_argument('--log-subdirectory', required=True)
    args = parser.parse_args()

    config = parse_config(args.config_path)
    writer = SummaryWriter(args.log_subdirectory)

    train(config, writer)

    writer.close()
