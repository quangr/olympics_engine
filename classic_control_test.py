# Copyright 2021 Garena Online Private Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Unit tests for classic control environments."""

from tokenize import Double
from typing import Any, Tuple,Callable
from unittest import result
from PIL import Image

import numpy as np
from absl.testing import absltest
import testhelper.core
import classic_control_envpool
import testhelper.viewer
import testhelper.helperfunction
from testhelper.generator import create_scenario
import matplotlib.pyplot as plt
import cv2

# class _ClassicControlEnvPoolTest(absltest.TestCase):

#   # def run_space_check(self, spec_cls: Any) -> None:
#   #   """Check if envpool.observation_space == gym.make().observation_space."""
#   #   # TODO(jiayi): wait for #27

#   # def run_deterministic_check(
#   #   self,
#   #   spec_cls: Any,
#   #   envpool_cls: Any,
#   #   obs_range: Tuple[np.ndarray, np.ndarray],
#   #   **kwargs: Any,
#   # ) -> None:
#   #   num_envs = 4
#   #   env0 = envpool_cls(
#   #     spec_cls(spec_cls.gen_config(num_envs=num_envs, seed=0, **kwargs))
#   #   )
#   #   env1 = envpool_cls(
#   #     spec_cls(spec_cls.gen_config(num_envs=num_envs, seed=0, **kwargs))
#   #   )
#   #   env2 = envpool_cls(
#   #     spec_cls(spec_cls.gen_config(num_envs=num_envs, seed=1, **kwargs))
#   #   )
#   #   act_space = env0.action_space
#   #   eps = np.finfo(np.float32).eps
#   #   obs_min, obs_max = obs_range[0] - eps, obs_range[1] + eps
#   #   for _ in range(5000):
#   #     action = np.array([act_space.sample() for _ in range(num_envs)])
#   #     obs0 = env0.step(action)[0]
#   #     obs1 = env1.step(action)[0]
#   #     obs2 = env2.step(action)[0]
#   #     np.testing.assert_allclose(obs0, obs1)
#   #     self.assertFalse(np.allclose(obs0, obs2))
#   #     self.assertTrue(np.all(obs_min <= obs0), obs0)
#   #     self.assertTrue(np.all(obs_min <= obs2), obs2)
#   #     self.assertTrue(np.all(obs0 <= obs_max), obs0)
#   #     self.assertTrue(np.all(obs2 <= obs_max), obs2)

#   # def run_align_check(self, env0: gym.Env, env1: Any, reset_fn: Any) -> None:
#   #   for _ in range(10):
#   #     reset_fn(env0, env1)
#   #     d0 = False
#   #     while not d0:
#   #       a = env0.action_space.sample()
#   #       o0, r0, d0, _ = env0.step(a)
#   #       o1, r1, d1, _ = env1.step(np.array([a]), np.array([0]))
#   #       np.testing.assert_allclose(o0, o1[0], atol=1e-6)
#   #       np.testing.assert_allclose(r0, r1[0])
#   #       np.testing.assert_allclose(d0, d1[0])

#   # def test_Curling(self) -> None:
#   #   fmax = np.finfo(np.float32).max
#   #   obs_max = np.array([4.8, fmax, np.pi / 7.5, fmax])
#   #   self.run_deterministic_check(
#   #     CurlingEnvSpec, CurlingGymEnvPool, (-obs_max, obs_max)
#   #   )
#   def test_cross_prod(self)->None:
#     # return
#     self.assertTrue(classic_control_envpool.cross_prod([1,0],[5,2])==testhelper.core.cross_prod([1,0],[5,2]))
#   def test_point2line(self)->None:
#     self.assertTrue(classic_control_envpool.point2line([1,0],[5,2],[2,9])==testhelper.core.point2line([1,0],[5,2],[2,9]))
#   def test_line_intersect(self)->None:
#     for i in range(100):
#       testcase=[(np.random.rand(2,2)-.5)*100000,(np.random.rand(2,2)-.5)*100000]
#       self.assertTrue(classic_control_envpool.line_intersect(*testcase)==testhelper.core.line_intersect(*testcase))
#   def test_closest_point(self)->None:
#     for i in range(100):
#       testcase=[(np.random.rand(2)-.5)*100000,(np.random.rand(2)-.5)*100000,(np.random.rand(2)-.5)*100000]
#       self.assertTrue((classic_control_envpool.closest_point(*testcase)==testhelper.core.closest_point(*testcase)).all())
# class _ViewerTest(absltest.TestCase):
#   def randomtest(self,input:Callable,fun1:Callable,fun2:Callable)->None:
#       for i in range(1000):
#         testcase=input()
#         result=[fun1(*testcase),fun2(*testcase)]
#         self.assertTrue(np.equal(result[0],result[1]).all(),"result:"+str(result)+"\ntestcase:"+str(testcase))

#   def test_Build(self)->None:
#     setting={'height':120, 'width':100, 'edge':200}
#     v=classic_control_envpool.Viewer(**setting)
#     # self.assertTrue(v.width==100)
#     # self.assertTrue(v.height==120)

#   def test_check_radian(self)->None:
#     self.randomtest(lambda:[(np.random.rand()-.5)*1000,(np.random.rand()-.5)*1000,(np.random.rand()-.5)*1000],
#     classic_control_envpool.check_radian,
# testhelper.helperfunction.check_radian
#     )
    
#   def test_rotate(self)->None:
#     self.randomtest(lambda:[(np.random.rand()-.5)*1000,(np.random.rand()-.5)*1000,(np.random.rand()-.5)*1000],
#     classic_control_envpool.rotate,
# testhelper.helperfunction.rotate
#     )
 
#   def test_rotate2(self)->None:
#     self.randomtest(lambda:[(np.random.rand()-.5)*1000,(np.random.rand()-.5)*1000,(np.random.rand()-.5)*1000],
#     classic_control_envpool.rotate2,
# testhelper.helperfunction.rotate2
#     )
#   def get_distance(self)->None:
#     self.randomtest(lambda:[(np.random.rand()-.5)*1000,(np.random.rand(2)-.5)*1000,(np.random.rand()-.5)*1000,np.random.rand()>0.5],
#     classic_control_envpool.get_distance,
# testhelper.helperfunction.get_distance
#     )

# class _GeneratorTest(absltest.TestCase):
#   def test_readjson(self)->None:
#       print(classic_control_envpool.readjson("/app/envpool/classic_control/testhelper/scenario.json"))        



class _OlympicsBaseTest(absltest.TestCase):

  def test_Build(self)->None:
    a=classic_control_envpool.OlympicsBase()
    my=np.array(a.get_obs()[0])
    
    plt.imsave('/app/temp/test1.jpg', cv2.resize(my, (500, 500), interpolation=cv2.INTER_NEAREST))
    # print(testhelper.core.OlympicsBase(create_scenario("curling")).obs_list)
    self.assertTrue((testhelper.core.OlympicsBase(create_scenario("curling")).obs_list[0]==my).all())
    # self.assertTrue(v.height==120)

if __name__ == "__main__":
  absltest.main()