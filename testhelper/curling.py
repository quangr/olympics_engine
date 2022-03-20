from .core import OlympicsBase
from .objects import Ball, Agent
import numpy as np
import math
import sys
import os
import random
import copy

# color 宏
COLORS = {
    'red': [255, 0, 0],
    'green': [0, 255, 0],
    'blue': [0, 0, 255],
    'yellow': [255, 255, 0],
    'grey':  [176,196,222],
    'purple': [160, 32, 240],
    'black': [0, 0, 0],
    'white': [255, 255, 255],
    'light green': [204, 255, 229],
    'sky blue': [0,191,255]
}

COLOR_TO_IDX = {
    'red': 7,
    'green': 1,
    'sky blue': 2,
    'yellow': 3,
    'grey': 4,
    'purple': 5,
    'black': 6,
    'light green': 0,
    'blue':8

}

IDX_TO_COLOR = {
    0: 'light green',
    1: 'green',
    2: 'sky blue',
    3: 'yellow',
    4: 'grey',
    5: 'purple',
    6: 'black',
    7: 'red',
    8: 'blue'
}

grid_node_width = 2     #for view drawing
grid_node_height = 2


def closest_point(l1, l2, point):
    """
    compute the coordinate of point on the line l1l2 closest to the given point, reference: https://en.wikipedia.org/wiki/Cramer%27s_rule
    :param l1: start pos
    :param l2: end pos
    :param point:
    :return:
    """
    A1 = l2[1] - l1[1]
    B1 = l1[0] - l2[0]
    C1 = (l2[1] - l1[1])*l1[0] + (l1[0] - l2[0])*l1[1]
    C2 = -B1 * point[0] + A1 * point[1]
    det = A1*A1 + B1*B1
    if det == 0:
        cx, cy = point
    else:
        cx = (A1*C1 - B1*C2)/det
        cy = (A1*C2 + B1*C1)/det

    return [cx, cy]

def distance_to_line(l1, l2, pos):
    closest_p = closest_point(l1, l2, pos)

    n = [pos[0] - closest_p[0], pos[1] - closest_p[1]]  # compute normal
    nn = n[0] ** 2 + n[1] ** 2
    nn_sqrt = math.sqrt(nn)
    cl1 = [l1[0] - pos[0], l1[1] - pos[1]]
    cl1_n = (cl1[0] * n[0] + cl1[1] * n[1]) / nn_sqrt

    return abs(cl1_n)


class curling(OlympicsBase):
    def __init__(self, map):
        super(curling, self).__init__(map)

        self.tau = 0.1
        self.wall_restitution = 1
        self.circle_restitution = 1
        self.print_log = False
        self.draw_obs = True
        self.show_traj = False
        self.start_pos = [300,150]
        self.start_init_obs = 90
        self.max_n = 4
        self.round_max_step = 100

        self.vis=300
        self.vis_clear = 10

        # self.purple_rock = pygame.image.load(os.path.join(CURRENT_PATH, "assets/purple rock.png"))
        # self.green_rock = pygame.image.load(os.path.join(CURRENT_PATH,"assets/green rock.png"))
        # self.curling_ground = pygame.image.load(os.path.join(CURRENT_PATH, "assets/curling ground.png"))
        # self.crown_image = pygame.image.load(os.path.join(CURRENT_PATH, "assets/crown.png"))
        # self.curling_ground.set_alpha(150)

    def reset(self, reset_game=False):
        self.release = False

        self.top_area_gamma = 0.98
        self.down_area_gamma = 0.95 #random.uniform(0.9, 0.95)

        self.gamma = self.top_area_gamma

        self.agent_num = 0
        self.agent_list = []
        self.agent_init_pos = []
        self.agent_pos = []
        self.agent_previous_pos = []
        self.agent_v = []
        self.agent_accel = []
        self.agent_theta = []
        self.temp_winner = -1
        self.round_step = 0

        if reset_game:
            assert self.game_round == 1
            self.current_team = 1   #start from green
            self.num_purple = 0
            self.num_green = 1

            map_copy = copy.deepcopy(self.map)
            map_copy['agents'][0].color = 'green'
            map_copy["agents"][0].original_color = 'green'


        else:
            self.num_purple = 1
            self.num_green = 0
            self.current_team = 0

            self.purple_game_point = 0
            self.green_game_point = 0

            self.game_round = 0
            map_copy = copy.deepcopy(self.map)

        self.obs_boundary_init = list()
        self.obs_boundary = self.obs_boundary_init

        #self.check_valid_map()
        self.generate_map(map_copy)
        self.merge_map()

        self.init_state()
        self.step_cnt = 0
        self.done = False
        self.release = False

        self.display_mode=False
        self.view_terminal = False

        obs = self.get_obs()

        if self.current_team == 0:
            return [obs, np.zeros_like(obs)-1]
        else:
            return [np.zeros_like(obs)-1, obs]

    def _reset_round(self):
        self.current_team = 1-self.current_team
        #convert last agent to ball
        if len(self.agent_list) != 0:
            last_agent = self.agent_list[-1]
            last_ball = Ball(mass = last_agent.mass, r = last_agent.r, position = self.agent_pos[-1],
                             color = last_agent.color)
            last_ball.alive = False
            self.agent_list[-1] = last_ball

        #add new agent
        if self.current_team == 0:
            #team purple
            new_agent_color = 'purple'
            self.num_purple += 1

        elif self.current_team == 1:
            new_agent_color = 'green'
            self.num_green += 1

        else:
            raise NotImplementedError

        new_agent = Agent(mass = 1, r= 15, position = self.start_pos, color = new_agent_color,
                          vis = self.vis, vis_clear = self.vis_clear)

        self.agent_list.append(new_agent)
        self.agent_init_pos[-1] = self.start_pos
        new_boundary = self.get_obs_boundaray(self.start_pos, 15, self.vis)
        self.obs_boundary_init.append(new_boundary)
        self.agent_num += 1

        self.agent_pos.append(self.agent_init_pos[-1])
        self.agent_v.append([0,0])
        self.agent_accel.append([0,0])
        init_obs = self.start_init_obs
        self.agent_theta.append([init_obs])
        self.agent_record.append([self.agent_init_pos[-1]])

        self.release = False
        self.gamma = self.top_area_gamma

        self.round_step = 0

        return self.get_obs()




    def cross_detect(self):
        """
        check whether the agent has reach the cross(final) line
        :return:
        """
        for agent_idx in range(self.agent_num):

            agent = self.agent_list[agent_idx]
            if agent.type != 'agent':
                continue

            for object_idx in range(len(self.map['objects'])):
                object = self.map['objects'][object_idx]

                if not object.can_pass():
                    continue
                else:
                    #print('object = ', object.type)
                    if object.color == 'red' and object.type=='cross' and \
                            object.check_cross(self.agent_pos[agent_idx], agent.r):
                        # print('agent type = ', agent.type)
                        agent.alive = False
                        #agent.color = 'red'
                        self.gamma = self.down_area_gamma            #this will change the gamma for the whole env, so need to change if dealing with multi-agent
                        self.release = True
                        self.round_countdown = self.round_max_step-self.round_step
                    # if the ball hasnot pass the cross, the relase will be True again in the new round
    def check_action(self, action_list):
        action = []
        for agent_idx in range(len(self.agent_list)):
            if self.agent_list[agent_idx].type == 'agent':
                action.append(action_list[0])
                _ = action_list.pop(0)
            else:
                action.append(None)

        return action



    def step(self, actions_list):

        actions_list = [actions_list[self.current_team]]

        #previous_pos = self.agent_pos
        action_list = self.check_action(actions_list)
        if self.release:
            input_action = [None for _ in range(len(self.agent_list))]       #if jump, stop actions
        else:
            input_action = action_list

        self.stepPhysics(input_action, self.step_cnt)

        if not self.release:
            self.cross_detect()
        self.step_cnt += 1
        self.round_step += 1

        obs_next = self.get_obs()


        done = self.is_terminal()

        if not done:
            round_end, end_info = self._round_terminal()
            if round_end:

                if end_info is not None:
                    #clean the last agent
                    del self.agent_list[-1]
                    del self.agent_pos[-1]
                    del self.agent_v[-1]
                    del self.agent_theta[-1]
                    del self.agent_accel[-1]
                    self.agent_num -= 1

                self.temp_winner, min_d = self.current_winner()
                #step_reward = [1,0.] if self.temp_winner == 0 else [0., 1]          #score for each round
                if self.temp_winner == -1:
                    step_reward=[0., 0.]
                elif self.temp_winner == 0:
                    step_reward=[1, 0.]
                elif self.temp_winner == 1:
                    step_reward=[0., 1]
                else:
                    raise NotImplementedError


                obs_next = self._reset_round()

            else:
                step_reward = [0., 0.]

        else:

            if self.game_round == 1:
                # self.final_winner, min_d = self.current_winner()
                # self.temp_winner = self.final_winner
                self._clear_agent()
                self.cal_game_point()

                if self.purple_game_point > self.green_game_point:
                    self.final_winner = 0
                    step_reward = [100., 0]
                elif self.green_game_point > self.purple_game_point:
                    self.final_winner = 1
                    step_reward = [0., 100.]
                else:
                    self.final_winner = -1
                    step_reward = [0.,0.]

                self.temp_winner = self.final_winner
                # step_reward = [100., 0] if self.final_winner == 0 else [0., 100]
                self.view_terminal = True

            elif self.game_round == 0:

                self._clear_agent()
                game1_winner = self.current_winner()
                step_reward = [10., 0] if game1_winner == 0 else [0., 10.]
                self.cal_game_point()
                self.game_round += 1
                next_obs = self.reset(reset_game=True)

                return next_obs, step_reward, False, 'game1 ends, switch position'
            else:
                raise NotImplementedError



        if self.current_team == 0:
            obs_next = [obs_next, np.zeros_like(obs_next)-1]
        else:
            obs_next = [np.zeros_like(obs_next)-1, obs_next]

        if self.release:
            # h_gamma = self.down_area_gamma + random.uniform(-1, 1)*0.001
            h_gamma = self.down_area_gamma 
            self.gamma = h_gamma

        #return self.agent_pos, self.agent_v, self.agent_accel, self.agent_theta, obs_next, step_reward, done
        return obs_next, step_reward, done, ''

    # def get_obs_encode(self):
    #     obs = self.get_obs()
    #     if self.current_team == 0:
    #         return [obs, np.zeros_like(obs)]
    #     else:
    #         return [np.zeros_like(obs), obs]



    def get_reward(self):

        center = [300, 500]
        pos = self.agent_pos[0]
        distance = math.sqrt((pos[0]-center[0])**2 + (pos[1]-center[1])**2)
        return [distance]

    def is_terminal(self):

        # if self.step_cnt >= self.max_step:
        #     return True

        if (self.num_green + self.num_purple == self.max_n*2):
            if not self.release and self.round_step > self.round_max_step:
                return True

            if self.release:
                L = []
                for agent_idx in range(self.agent_num):
                    if (self.agent_v[agent_idx][0] ** 2 + self.agent_v[agent_idx][1] ** 2) < 1e-1:
                        L.append(True)
                    else:
                        L.append(False)
                return all(L)
        else:
            return False

        # for agent_idx in range(self.agent_num):
        #     if self.agent_list[agent_idx].color == 'red' and (
        #             self.agent_v[agent_idx][0] ** 2 + self.agent_v[agent_idx][1] ** 2) < 1e-5:
        #         return True

    def _round_terminal(self):

        if self.round_step > self.round_max_step and not self.release:      #after maximum round step the agent has not released yet
            return True, -1

        #agent_idx = -1
        L = []
        for agent_idx in range(self.agent_num):
            if (not self.agent_list[agent_idx].alive) and (self.agent_v[agent_idx][0] ** 2 +
                                                        self.agent_v[agent_idx][1] ** 2) < 1e-1:
                L.append(True)
            else:
                L.append(False)

        return all(L), None

    def _clear_agent(self):
        if self.round_step > self.round_max_step and not self.release:
            # clean the last agent
            del self.agent_list[-1]
            del self.agent_pos[-1]
            del self.agent_v[-1]
            del self.agent_theta[-1]
            del self.agent_accel[-1]
            self.agent_num -= 1

    def current_winner(self):

        center = [300, 500]
        min_dist = 1e4
        win_team = -1
        for i, agent in enumerate(self.agent_list):
            pos = self.agent_pos[i]
            distance = math.sqrt((pos[0]-center[0])**2 + (pos[1]-center[1])**2)
            if distance < min_dist:
                win_team = 0 if agent.color == 'purple' else 1
                min_dist = distance

        return win_team, min_dist

    def cal_game_point(self):

        center = [300, 500]
        purple_dis = []
        green_dis = []
        min_dist = 1e4
        closest_team = -1
        for i, agent in enumerate(self.agent_list):
            pos = self.agent_pos[i]
            distance = math.sqrt((pos[0]-center[0])**2 + (pos[1]-center[1])**2)
            if agent.color == 'purple':
                purple_dis.append(distance)
            elif agent.color=='green':
                green_dis.append(distance)
            else:
                raise NotImplementedError

            if distance < min_dist:
                closest_team = 0 if agent.color == 'purple' else 1
                min_dist = distance

        purple_dis = np.array(sorted(purple_dis))
        green_dis = np.array(sorted(green_dis))

        if closest_team == 0:
            if len(green_dis) == 0:
                winner_point = len(purple_dis)
            else:
                winner_point = purple_dis < green_dis[0]
            self.purple_game_point += np.float64(winner_point).sum()
        elif closest_team == 1:
            if len(purple_dis) == 0:
                winner_point = len(green_dis)
            else:
                winner_point = green_dis < purple_dis[0]
            self.green_game_point += np.float64(winner_point).sum()
        elif closest_team == -1:
            pass
        else:
            raise NotImplementedError

        #print('purple dis = {}, green dis = {}'.format(purple_dis, green_dis))



