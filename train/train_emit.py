import numpy as np
import ptvsd
ptvsd.enable_attach(address=('localhost', 5678), redirect_output=True)
print('Now is a good time to attach your debugger: Run: Python: Attach')
ptvsd.wait_for_attach()
from typing import Dict,Union
import gym
import tianshou as ts
import envpool
from collections import deque
from tianshou.data import Batch, ReplayBuffer
import gym.spaces as spaces
from torch.distributions.normal import Normal
from torch.utils.tensorboard import SummaryWriter
from tianshou.utils import TensorboardLogger
writer = SummaryWriter('log/dqn')
logger = TensorboardLogger(writer)

import matplotlib.pyplot as plt
from matplotlib import animation
def plotgif(test_collector):# type: ignore
    test_collector.reset()
    plt.xlim([150, 450])
    plt.ylim([100, 700])
    plt.axhline(y=300, color='r', linestyle='-')
    arr=test_collector.data['obs'][0]
    plt.arrow(arr[0], arr[1], arr[2],arr[3], width = 0.5)
    plt.scatter(arr[4], 300, marker="x")
    
    def animate(i):# type: ignore
        batch=test_collector.collect(1)
        plt.cla()
        plt.xlim([150, 450])
        plt.ylim([100, 700])
        plt.axhline(y=300, color='r', linestyle='-')
        arr=test_collector.data['obs'][0]
        plt.scatter(arr[4], 300, marker="x")
        plt.arrow(arr[0], arr[1], arr[2],arr[3], width = 0.5)
        if('rew' in test_collector.data):
            plt.text(200,200,f"reward:{test_collector.data['rew'][0]}")
    anim = animation.FuncAnimation(plt.gcf(), animate, frames = (60)*3, interval=50)
    anim.save('./a.gif', writer='imagemagick', fps=60)

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
train_envs = envpool.make_gym("Curling-v1", num_envs=150)
test_envs = envpool.make_gym("Curling-v1", num_envs=1)

import torch, numpy as np
from torch import nn
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
class CnnPolicy(nn.Module):
    def __init__(self)->None:
        super().__init__()
        self.model=nn.Sequential(
            nn.Flatten(),
            nn.Linear(7,128),
            nn.ReLU(),
            nn.Linear(128,128),
            nn.ReLU(),
            nn.Linear(128,128),
            nn.ReLU(),
            nn.Linear(128,48)
        ).to(device)
        # self.anglemodel=nn.Sequential(
        #     nn.Flatten(),
        #     nn.Linear(4,10),
        #     nn.ReLU(),
        #     nn.Linear(10,10),
        #     nn.ReLU(),
        #     nn.Linear(10,3)
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
            nn.Linear(7,128),
            nn.ReLU(),
            nn.Linear(128,128),
            nn.ReLU(),
            nn.Linear(128,128),
            nn.ReLU(),
            nn.Linear(128,1)
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
def MultivariateNormaldis(logits):# type: ignore
    return torch.distributions.Categorical(logits=logits)
net = CnnPolicy()
crt=CnnValue()
for m in net.model.modules():
    if isinstance(m, torch.nn.Linear):
        torch.nn.init.orthogonal_(m.weight)
        torch.nn.init.zeros_(m.bias)

for m in crt.model.modules():
    if isinstance(m, torch.nn.Linear):
        torch.nn.init.orthogonal_(m.weight)
        torch.nn.init.zeros_(m.bias)

actions_one_map=list(range(-100,200,40))
actions_two_map=list(range(-30,31,12))
len1=len(actions_one_map)
len2=len(actions_two_map)
action_num=len1*len2

class mypolicy(ts.policy.PPOPolicy):
    def map_action(self, act: Union[Batch, np.ndarray]) -> Union[Batch, np.ndarray]:
        myaction=np.array([[actions_one_map[i] for i in act//len2],[actions_two_map[i] for i in act%len2]])
        return myaction.T

optim = torch.optim.Adam(list(net.parameters()) + list(crt.parameters()), lr=5*1e-5)
policy = mypolicy(net,crt, optim,MultivariateNormaldis,advantage_normalization=False, discount_factor=0.1,action_space =spaces.Box( np.array([-50,-5]), np.array([200,5])),action_bound_method="tanh")
train_collector = ts.data.Collector(policy, train_envs, ts.data.VectorReplayBuffer(200000, 4000), exploration_noise=True,preprocess_fn=test_processor.preprocess_fn)
test_collector = ts.data.Collector(policy, test_envs, preprocess_fn=test_processor.preprocess_fn)

trainer =ts.trainer.OnpolicyTrainer(
    policy, train_collector, None,
    max_epoch=3000, step_per_epoch=500, step_per_collect=8000,
    repeat_per_collect=2, episode_per_test=20, batch_size=8192,
    stop_fn=lambda mean_rewards: mean_rewards >= 0.6,logger=logger)
for epoch, epoch_stat, info in trainer:
    print("Epoch:", epoch)
    print(epoch_stat)
    print(info)
    # if(epoch_stat['rew']>10000):
    #     plotgif(test_collector)
        # print("a")
    # if(epoch_stat>100)
# result = ts.trainer.onpolicy_trainer(
#     policy, train_collector, test_collector,
#     max_epoch=10, step_per_epoch=10000, step_per_collect=10,
#     repeat_per_collect=2, episode_per_test=100, batch_size=64,
#     # train_fn=lambda epoch, env_step: policy.set_eps(0.1),
#     # test_fn=lambda epoch, env_step: policy.set_eps(0.05),
#     stop_fn=lambda mean_rewards: mean_rewards >= 0.5)
# print(f'Finished training! Use {result["duration"]}')
# print(result)
torch.save(net.state_dict(), 'dqn.pth')