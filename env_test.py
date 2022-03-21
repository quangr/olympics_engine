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
print(a)
act_space = env0.action_space
# action = np.array([ for _ in range(num_envs)])
print(act_space)
for i in range(150):
    print(f"step {i}")
    a,b,c,d=env0.step(np.array([[200,0] for _ in range(1)]))
    if(b==1):
        print(a,b,c,d)
