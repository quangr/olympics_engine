# import ptvsd
# ptvsd.enable_attach(address=('localhost', 5678), redirect_output=True)
# print('Now is a good time to attach your debugger: Run: Python: Attach')
# ptvsd.wait_for_attach()
from typing import Dict
import gym
import tianshou as ts
import envpool
import numpy as np
from collections import deque
from tianshou.data import Batch, ReplayBuffer
import gym.spaces as spaces
from torch.distributions.normal import Normal
from torch.utils.tensorboard import SummaryWriter
from tianshou.utils import TensorboardLogger
writer = SummaryWriter('log/dqn')
logger = TensorboardLogger(writer)

actions_one_map=list(range(0,200,20))
actions_two_map=list(range(-30,30,5))
len1=len(actions_one_map)
len2=len(actions_two_map)
action_num=4

# def seed(self, seed)->None:
#     np.random.seed(seed)
# class EmitWrapper(gym.Wrapper):
#     def __init__(self, env) -> None:
#         super().__init__(env)
#         self.env=env
#         self.queue=deque([np.zeros((30,30),dtype=np.float32)],maxlen=2)
#         self.action_space=gym.spaces.Discrete(action_num)
#         # self.observation_space=gym.spaces.Box(np.zeros((2,30,30)),6*np.ones((2,30,30)))
#     def step(self, action):
#         myaction=[[actions_one_map[action//len2]],[actions_two_map[action%len2]]]
#         obs, reward, done, _ , info= self.env.step([myaction,[[0.0],[0.0]]])
#         r=obs[0]['release']
#         # if(r):
#         #     print(r)
#         self.queue.append(obs[0]['obs'][0])
#         return list(self.queue),1 if r else 0,r,{}
#     def reset(self):
#         obs=self.env.reset()
#         self.queue.append(obs[0]['obs'][0])
#         return list(self.queue)
#     def close(self):
#         return 


# train_envs = ts.env.SubprocVectorEnv([lambda:EmitWrapper(make("olympics-curling")) for _ in range(10)])
# test_envs = ts.env.SubprocVectorEnv([lambda:EmitWrapper(make("olympics-curling")) for _ in range(100)])
import envpool
game = envpool.make_gym("Curling-v1", num_envs=1)

import torch, numpy as np
from torch import nn
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
class CnnPolicy(nn.Module):
    def __init__(self)->None:
        super().__init__()
        self.model=nn.Sequential(
            nn.Flatten(),
            nn.Linear(4,10),
            nn.ReLU(),
            nn.Linear(10,10),
            nn.ReLU(),
            nn.Linear(10,10),
            nn.ReLU(),
            nn.Linear(10,2)
        ).to(device)
        # self.stdmodel=nn.Sequential(
        #     nn.Flatten(),
        #     nn.Linear(4,10),
        #     nn.ReLU(),
        #     nn.Linear(10,10),
        #     nn.ReLU(),
        #     nn.Linear(10,2)
        # ).to(device)
    def forward(self,input:torch.Tensor, state:torch.Tensor=None, info:Dict={})->torch.Tensor:
        return self.model(torch.tensor(input,dtype=torch.float32,device=device)),state
    def close(self)->None:
        return

class CnnValue(nn.Module):
    def __init__(self)->None:
        super().__init__()
        self.model=nn.Sequential(
            nn.Flatten(),
            nn.Linear(4,20),
            nn.ReLU(),
            nn.Linear(20,1)
        ).to(device)
    def forward(self,input:torch.Tensor)->torch.Tensor:
        return self.model(torch.tensor(input,dtype=torch.float32,device=device))

class MyProcessor:
    def __init__(self, size=100):# type: ignore
        self.episode_log = None
        self.main_log = deque(maxlen=size)# type: ignore
        self.main_log.append(0)
        self.baseline = 0

    def preprocess_fn(*args,**kwargs):# type: ignore
        """change reward to zero mean"""
        # if obs && env_id exist -> reset
        # if obs_next/act/rew/done/policy/env_id exist -> normal step
        if 'rew' not in kwargs:
            # means that it is called after env.reset(), it can only process the obs
            return Batch()  # none of the variables are needed to be updated
        else:
            # print(kwargs)
            # if((kwargs['rew']==1).any()):
            #     print(kwargs)
            return Batch(rew=kwargs['rew'])

#TODO init
test_processor = MyProcessor(size=100)
def MultivariateNormaldis(loc):# type: ignore
    return Normal(loc,torch.exp(torch.tensor(-.5)))
net = CnnPolicy()


net.load_state_dict(torch.load('dqn.pth'))

obs = train_envs.reset()
done = False
step = 0

while not done:
    step += 1
    action=policy.actor._distribution([obs]).sample()
    obs, reward, done, _ = game.step(action)
    # plt.imshow(obs[0])
    # plt.show()
