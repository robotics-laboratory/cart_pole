import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F


class DQNAgent(nn.Module):
    def __init__(
        self,
        state_size,
        num_actions,
        hidden_size,
        num_layers,
        nonlinearity
    ):
        super().__init__()
        
        if nonlinearity == 'relu':
            nonlinearity_module = nn.ReLU()
        else:
            raise NotImplementedError()

        layers = []
        for i in range(num_layers):
            if i == 0:
                layers.append(nn.Linear(state_size, hidden_size))
                layers.append(nonlinearity_module)
            elif i == num_layers - 1:
                layers.append(nn.Linear(hidden_size, num_actions))
            else:
                layers.append(nn.Linear(hidden_size, hidden_size))
                layers.append(nonlinearity_module)

        self.net = nn.Sequential(*layers)

    def forward(self, x):
        return self.net(x)

    def get_device(self):
        return next(self.parameters()).device

    def get_qvalues(self, states):
        np_states = np.array([state.as_np_vector() for state in states])
        torch_states = torch.tensor(np_states, device=self.get_device(), dtype=torch.float32)
        q_values = self(torch_states)
        return q_values

    def action(self, states, epsilon):
        batch_size = len(states)
        q_values = self.get_qvalues(states).clone().detach().cpu().numpy()

        random_idx = np.random.binomial(1, epsilon, batch_size)
        random_actions = np.random.choice(q_values.shape[1], size=batch_size)
        best_actions = q_values.argmax(axis=1)

        return np.where(random_idx, random_actions, best_actions)
