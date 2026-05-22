# -*- coding: utf-8 -*-
"""
@author: cenrong.dai@dorabot.com
@brief: Provide functions that are needed in both RRT* and MARRT*
"""
import pygame
from pygame.locals import KEYDOWN
import json
from math import log, ceil, sqrt
from visualisation import rnd_color
from geometry import Point, Vector

'''rrt family global parameter'''
POS_INF = float("inf")
# EPSILON = 1.0 # max step distance when steering
# MAX_NODES_EXPANDED = 1000
# MAX_SEARCH_TIME = 5 # 5s
# RADIUS=2
AGENT_DIMENSION = 0.5
ETA = 1.0 # minimum near neighbor radius in iNash search
'''for debug visualization'''
red = (255, 0, 0)
yellow = (255, 255, 0)
green = (0, 255, 0)
bg = (216, 222, 228, 0)
'''only for debug rrt family'''
# debug_rrtstar = True
color_dict = rnd_color(100)
debug_pixels_per_meter = None
debug_reference_point = None
with open('config.json') as file:
    config_data = json.load(file)
    debug_pixels_per_meter = config_data['visualization']['pixels_per_meter']
    debug_grids_num_per_meter = config_data["environment"]["resolution"]
    debug_reference_point = (debug_pixels_per_meter / debug_grids_num_per_meter*2, debug_pixels_per_meter / debug_grids_num_per_meter*2)


class State(Point):

    def __init__(self, xcoord, ycoord, cost = POS_INF):
        self.x = xcoord
        self.y = ycoord
        self.cost = cost
        self.parent = None
        self.point = Point(xcoord,ycoord)

    def __str__(self):
        return "State({x},{y})".format(x=self.x, y=self.y)

    def square_state_distance(self, state):
        return (self.x - state.x)**2 + (self.y - state.y)**2

    def state_distance(self, state):
        return sqrt((self.x - state.x)**2 + (self.y - state.y)**2)

    def state_update_parent(self, environment, parent, cost_heuristic_function):
        self.parent = parent
        self.cost = self.parent.cost+cost_heuristic_function(environment, self.parent, self, False)

class JointState:
    agents_state = {} # store the node of an agent by its id
    t = POS_INF
    cost = POS_INF
    parent = None
    active_agents_id_set = set()
    def __init__(self, agents_state, timestamp):
        self.agents_state = agents_state
        self.t = timestamp
        self.active_agents_id_set = set(agents_state.keys())
        if timestamp == 0: # root of tree
            self.cost = 0

    def __str__(self):
        return "JointState({states});time:{t};cost:{cost};active:{active}".\
            format(states='; '.join([str(aid)+":"+str(state) for aid, state in self.agents_state.items()]), \
                t=self.t, cost=self.cost, active=self.active_agents_id_set)

    def __getitem__(self, agent_id):
        """use agent id to get its state within the joint-state"""
        try:
            return self.agents_state[agent_id]
        except Exception as e:
            print "rrtstar_helper.py:", e

    def __setitem__(self, agent_id, agent_state):
        """use agent id to set its state within the joint-state"""
        self.agents_state[agent_id] = agent_state

    def update_parent(self, parent_joint_state, cost_heuristic_function, cost_time_function, current_is_goal = False):
        """if current joint-state is goal, do not update the state of inactive agents"""
        self.parent = parent_joint_state
        self.cost = self.parent.cost+cost_heuristic_function(self.parent, self, False)
        self.t = self.parent.t+cost_time_function(self.parent, self)
        if current_is_goal:
            return
        self.active_agents_id_set = self.parent.active_agents_id_set.copy()
        for aid in self.agents_state.keys():
            if aid not in self.active_agents_id_set:
                self.agents_state[aid] = self.parent[aid]

    def check_agents_arrive_goal(self, environment_map, goal_joint_state, radius):
        """ check the agent's parent step has arrived the region within RADIUS distance to goal-state and inactivate it """
        for aid in list(self.active_agents_id_set):
            goal_state = goal_joint_state[aid]
            parent_state = self.parent[aid]
            if parent_state.square_state_distance(goal_state) <= radius**2:
                temp_state = self.agents_state[aid]
                self.agents_state[aid] = goal_state
                if self.parent.to_joint_state_collision(environment_map, self):
                    self.agents_state[aid] = temp_state # revert back
                    continue
                else:
                    self.active_agents_id_set.remove(aid)

    def to_joint_state_active_distance(self, to_joint_state):
        """return the distance between the current joint-state's ***active*** agents and corresponding agents in to-joint-state"""
        self.check_joint_state_consistency(to_joint_state)
        
        uniform_active_sum = 0
        for aid in self.active_agents_id_set:
            from_state = self.agents_state[aid]
            to_state = to_joint_state[aid]
            uniform_active_sum += from_state.state_distance(to_state)
        return uniform_active_sum

    def to_joint_state_total_distance(self, to_joint_state):
        """return the distance between the current joint-state and to-joint-state, ***no matter*** active or not"""
        self.check_joint_state_consistency(to_joint_state)
        uniform_total_sum = 0
        for aid in self.agents_state.keys():
            from_state = self.agents_state[aid]
            to_state = to_joint_state[aid]
            uniform_total_sum += from_state.state_distance(to_state)
        return uniform_total_sum

    def is_each_state_distance_to_other_within_radius(self, to_joint_state, radius):
        """***no matter active or not*** agents are considered"""
        self.check_joint_state_consistency(to_joint_state)

        for aid in self.agents_state.keys():
            if self.agents_state[aid].square_state_distance(to_joint_state[aid]) > radius**2:
                return False
        return True

    # def is_each_active_state_distance_to_other_within_radius(self, to_joint_state, radius = RADIUS):
    #     """only ***active*** agents are considered"""
    #     check_joint_state_consistency(self, to_joint_state)
    #     for aid in self.active_agents_id_set:
    #         if self.agents_state[aid].state_distance(to_joint_state[aid]) > radius:
    #             return False
    #     return True

    def to_joint_state_collision(self, environment_map, to_joint_state):
        '''return True if from current joint-state to to-joint-state collide with any obstacle or path collision, only ***active*** agents
        in current joint-state is considered'''
        self.check_joint_state_consistency(to_joint_state)
        # edge obstacle collision
        for aid in self.active_agents_id_set:
            from_state = self.agents_state[aid]
            to_state = to_joint_state[aid]
            if environment_map.subpath_obstacles_collision(from_state, to_state): # agent half dimension margin is handled by environment_map
                return True
        if self.to_joint_state_subpaths_collision(environment_map, to_joint_state):
            return True
        return False

    def to_joint_state_subpaths_collision(self, environment_map, to_joint_state):
        '''return True if subpaths of consecutive states of indivisual agent collide'''
        self.check_joint_state_consistency(to_joint_state)

        active_agents_id = self.active_agents_id_set
        for aid1 in active_agents_id:
            for aid2 in active_agents_id:
                if aid1 == aid2:
                    continue
                from_state1, to_state1 = self.agents_state[aid1], to_joint_state[aid1]
                from_state2, to_state2 = self.agents_state[aid2], to_joint_state[aid2]
                if environment_map.subpath_subpath_collision(from_state1, to_state1, from_state2, to_state2):
                    return True
        return False

    '''joint state comparable assertion'''
    def check_joint_state_consistency(self, other_joint_state):
        """raise Exception if the agents are not the same between 2 joint-states"""
        if set(self.agents_state.keys()) != set(other_joint_state.agents_state.keys()):
            raise NotImplementedError("marrt*_planner.py: the agents in two joint-states are not the same;\nself: {from_state};\nother: {to_state}".\
                format(from_state=str(self), to_state=str(other_joint_state)))


'''visualization for debug'''
def draw_arrow(screen, color, from_state, to_state, width = 1):
    draw_line(screen, color, from_state, to_state, width)
    direction = Vector(from_state.x-to_state.x, from_state.y-to_state.y)
    direction = direction.normalize().scale(0.2)
    from_point = Point(to_state.x + direction.x, to_state.y + direction.y)
    draw_line(screen, (255-color[0], 255-color[1], 255-color[2]), from_point, to_state, 3)

def draw_line(screen, color, from_state, to_state, width = 1):
    pygame.draw.line(screen,color,list(rectify_meter_to_pixel(from_state)),list(rectify_meter_to_pixel(to_state)), width)

def draw_circle(screen, color, center, radius = 5):
    try:
        pygame.draw.circle(screen, color, rectify_meter_to_pixel(center), radius, 0)
    except Exception as e:
        print "marrt*.py: unable to draw point {}, {}".format(center, e)

def draw_joint_state(screen, joint_state, color = None, radius = 5):
    for aid, state in joint_state.agents_state.items():
        draw_circle(screen, color[aid], Point(state.x, state.y), radius)

def rectify_meter_to_pixel(point):
    def meter_to_pixel(value):
        return int(ceil(value * debug_pixels_per_meter))
    return (meter_to_pixel(point.x)+debug_reference_point[0],meter_to_pixel(point.y)+debug_reference_point[1])

def pausable_interface(cmd = False):
    pygame.display.flip()
    paused = cmd
    if paused:
        print "PAUSE searching ..."
    while paused:
        for event in pygame.event.get():
            if event.type == KEYDOWN:
                if event.key == pygame.K_p:
                    paused = not paused
                    print "RESUME searching ..."
                elif event.key == pygame.K_q: # Q/q is pressed
                    exit()
                # elif event.key == pygame.K_t:
                #     return True
    return False

def interaction_listener():
    pygame.display.flip()
    terminated = False
    for event in pygame.event.get():
        if event.type == KEYDOWN:
            if event.key == pygame.K_p:
                terminated = pausable_interface(True)
            elif event.key == pygame.K_q: # Q/q is pressed
                exit()
            # elif event.key == pygame.K_t:
            #     return True
    return terminated
