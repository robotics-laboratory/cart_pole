from collections import namedtuple, deque
from itertools import count
from simulator import CartPoleSimulator

import numpy as np

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import torchvision.transforms as T

import logging
import math
import random
import time


logging.basicConfig(level=logging.INFO)

class Snapshot:
    def __init__(self, state, target):
        self.state = state
        self.target = target

    def to_tuple(self):
        return (
            self.state.cart_position,
            self.state.cart_velocity,
            self.state.pole_angle,
            self.state.pole_velocity,
            self.target
        )

Transition = namedtuple('Transition', ('src', 'dst', 'action', 'reward'))

class Memory(object):
    def __init__(self, capacity):
        self.memory = deque([], maxlen=capacity)

    def push(self, transition):
        self.memory.append(transition)

    def sample(self, batch_size):
        return random.sample(self.memory, batch_size)

    def __len__(self):
        return len(self.memory)


ACTION_N = 3

class EpsGreedySampler:
    def __init__(self, start=0.9, end=0.05, decay=200):
        self.start = start
        self.end = end
        self.decay = decay
        self.step = 0

    def get_eps(self):
        return self.end + (self.start - self.end) \
            * math.exp(-1. * self.step / self.decay)

    def __call__(self, policy, snapshot):
        eps = self.get_eps()
        self.step += 1

        if random.random() > eps:
            with torch.no_grad():
                x = torch.tensor([snapshot.to_tuple()])
                return policy(x).max(1)[1].view(1, 1)
        else:
            return torch.tensor([[random.randrange(ACTION_N)]], dtype=torch.long)

class DQN(nn.Module):
    def __init__(self, action_n):
        super(DQN, self).__init__()
        self.action_n = action_n
        self.l1 = nn.Linear(5, 32)
        self.l2 = nn.Linear(32, action_n)

    def forward(self, x):
        x = self.l1(x)
        x = F.relu(x)
        x = self.l2(x)
        return x

def action_to_delta(action):
    if action == 0:
        return -0.01
    elif action == 1:
        return 0
    elif action == 2:
        return +0.01
    else:
        raise NotImplementedError


EPISODE_N = 1000
STEP_N = 1000
TIME_STEP = 0.02
BATCH_SIZE = 128
GAMMA = 0.99
TARGET_UPDATE = 10

cart_pole = CartPoleSimulator(debug_mode=True)
sampler = EpsGreedySampler()
memory = Memory(100000)

policy_net = DQN(ACTION_N)
target_net = DQN(ACTION_N)
target_net.load_state_dict(policy_net.state_dict())
target_net.eval()

optimizer = optim.RMSprop(policy_net.parameters())

def to_tuple(s, t):
    return (s.cart_postion, s.cart_velocity, s.pole_angle, s.pole_velocity, t)

def reward(s):
    return np.exp(-np.cos(s.pole_angle))

def train(memory, optimizer, policy_net, target_net):
    if len(memory) < BATCH_SIZE:
        return
        
    src_states = []
    dst_states = []
    actions = []
    rewards = []
    for src, dst, a, r in memory.sample(BATCH_SIZE):
        src_states.append(src.to_tuple())
        dst_states.append(dst.to_tuple())
        actions.append([a])
        rewards.append(r)

    src_states = torch.tensor(src_states, dtype=torch.float32)
    dst_states = torch.tensor(dst_states, dtype=torch.float32)
    actions = torch.tensor(actions, dtype=torch.int64)
    rewards = torch.tensor(rewards, dtype=torch.float32)

    Q_src = policy_net(src_states).gather(1, actions).squeeze(1)
    Q_dst, _ = torch.max(target_net(dst_states), 1)


    criterion = nn.SmoothL1Loss()
    loss = criterion(Q_src, rewards + GAMMA * Q_dst)


    print(rewards.mean().item(), loss.item())

    # Optimize the model
    optimizer.zero_grad()
    loss.backward()
    for param in policy_net.parameters():
        param.grad.data.clamp_(-1, 1)
    optimizer.step()

for episode_i in range(EPISODE_N):
    # Initialize simulator and get initial state and target
    action = 1
    prev = Snapshot(*cart_pole.reset())

    cart_pole.step(TIME_STEP)

    action = sampler(policy_net, prev)
    current = Snapshot(cart_pole.get_state(), prev.target + action_to_delta(action))
    cart_pole.set_target(current.target)

    # Run episode 
    for step_i in range(STEP_N):
        cart_pole.step(TIME_STEP)

        action = sampler(policy_net, current)
        future = Snapshot(cart_pole.get_state(), current.target + action_to_delta(action))
        cart_pole.set_target(future.target)

        memory.push(Transition(prev, current, action, reward(future.state)))

        prev = current
        current = future
        
        train(memory, optimizer, policy_net, target_net)
    if episode_i % 5 == 0:
        target_net.load_state_dict(policy_net.state_dict())

cart_pole.close()
