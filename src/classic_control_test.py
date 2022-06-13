#!/usr/bin/env python
# -*- coding: utf-8 -*-
# mypy: ignore-errors
"""Unit tests for classic control environments."""

from ast import Num
from cgitb import reset
import logging
from time import sleep
import time
from typing import Any, Tuple, Callable
from unittest import result
import sys
import numpy as np
from absl.testing import absltest
from python.generator import create_scenario
import pygame
import classic_control_envpool
from python.curling import curling, IDX_TO_COLOR
from olympics_engine.main import getgame
logger = logging.getLogger()
logger.level = logging.DEBUG
np.set_printoptions(precision=3)

class _CurlingTest(absltest.TestCase):
    # def testrender(self) -> None:
    #     a = curling(create_scenario("curling"))
    #     a.reset()
    #     while(True):
    #         a.render()
    #         a.step([[200, 0], [10, 10]])
    #         # if(a.step_cnt==16):
    #         #     time.sleep(20)
    #         time.sleep(.1)
    def testbuild(self) -> None:
        # import ptvsd
        # ptvsd.enable_attach(address=('localhost', 5678), redirect_output=True)
        # print('Now is a good time to attach your debugger: Run: Python: Attach')
        # ptvsd.wait_for_attach()
        a = curling(create_scenario("curling"))
        b = getgame("curling")
        self.assertTrue(np.equal(a.reset()[0]['agent_obs'],b.reset()[0]['agent_obs']).all())
        # print(a.step([[200,0],[200,0]]))
        action=[[200, 0], [10, 10]]
        actionhistory=[]
        try:
            while(True):
                # a.render()
                actionhistory.append(action)
                r1 = [x['agent_obs'] for x in a.step(action)[0]]
                r2 = [x['agent_obs'].astype(np.uint8) for x in b.step(action)[0]]
                self.assertTrue(np.equal(np.array(a.agent_pos),np.array(b.agent_pos)).all(),f'pos unexpectedly equals for player one in step {b.step_cnt}.')              
                # self._formatMessage(
                self.assertTrue(np.equal(r1[0],r2[0]).all(),f'output unexpectedly equals for player one in step {b.step_cnt}.\nactionhistory:{actionhistory}\n')
                self.assertTrue(np.equal(r1[1],r2[1]).all(),f'output unexpectedly equals for player two in step {b.step_cnt}.\nactionhistory:{actionhistory}\n')
        finally:
            logger.removeHandler(stream_handler)
            time.sleep(0.1)


if __name__ == "__main__":
    absltest.main()
