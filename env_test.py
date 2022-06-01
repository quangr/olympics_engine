# import ptvsd
# ptvsd.enable_attach(address=('localhost', 5678), redirect_output=True)
# print('Now is a good time to attach your debugger: Run: Python: Attach')
# ptvsd.wait_for_attach()
import numpy as np

from envpool.classic_control import CurlingSimpleEnvSpec,CurlingSimpleGymEnvPool
config = CurlingSimpleEnvSpec.gen_config(num_envs=1,max_episode_steps=200, seed=0)
spec = CurlingSimpleEnvSpec(config)
env0 = CurlingSimpleGymEnvPool(spec)
a=env0.reset()
# print(a)
act_space = env0.action_space
# action = np.array([ for _ in range(num_envs)])
# print(act_space)pp
import yep

yep.start("test.prof")
# do something
action=np.array([[35,0] for _ in range(1)])
for i in range(50000):
    # print(f"step {i}",flush=True)
    # a,b,c,d=env0.step(np.array([[np.random.rand()*200,np.random.rand()*60-30] for _ in range(1)]))
    a,b,c,d=env0.step(action)
    # if(b!=0):
        # print(b)
    # print(a,b,c,d)
yep.stop()
