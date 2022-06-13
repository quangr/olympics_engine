import sys
from pathlib import Path
base_path = str(Path(__file__).resolve().parent.parent)
sys.path.append(base_path)
print(sys.path)
from olympics_engine.generator import create_scenario
import argparse
from olympics_engine.agent import *
import time

from scenario import Running, table_hockey, football, wrestling, billiard, \
    curling, billiard_joint, curling_long, curling_competition, Running_competition

from AI_olympics import AI_Olympics

import random
import numpy as np
import matplotlib.pyplot as plt
import json


RENDER = False
def getgame(map="curling"):
    for i in range(1):
        if map != 'all':
            Gamemap = create_scenario(map)
        #game = table_hockey(Gamemap)
        if map == 'running':
            game = Running(Gamemap)
            agent_num = 2
        elif map == 'running-competition':

            map_id = random.randint(1,10)
            # map_id = 3
            Gamemap = create_scenario(map)
            game = Running_competition(meta_map=Gamemap,map_id=map_id)
            agent_num = 2


        elif map == 'table-hockey':
            game = table_hockey(Gamemap)
            agent_num = 2
        elif map == 'football':
            game = football(Gamemap)
            agent_num = 2
        elif map == 'wrestling':
            game = wrestling(Gamemap)
            agent_num = 2
        # elif map == 'volleyball':
        #     game = volleyball(Gamemap)
        #     agent_num = 2
        elif map == 'billiard':
            game = billiard(Gamemap)
            agent_num = 2
        elif map == 'curling':
            game = curling(Gamemap)
            agent_num = 2

        elif map == 'curling-joint':
            game = curling_joint(Gamemap)
            agent_num = 2

        elif map == 'billiard-joint':
            game = billiard_joint(Gamemap)
            agent_num = 2

        elif map == 'curling-long':
            game = curling_long(Gamemap)
            agent_num = 2

        elif map == 'curling-competition':
            game = curling_competition(Gamemap)
            agent_num = 2

        elif map == 'all':
            game = AI_Olympics(random_selection = False, minimap=False)
            agent_num = 2
        return game

if __name__ == "__main__":
    b = getgame("curling")
    b.reset()
    # print(a.step([[200,0],[200,0]]))
    action=[[200, 0], [10, 10]]
    actionhistory=[]
    while(True):
        b.step(action)