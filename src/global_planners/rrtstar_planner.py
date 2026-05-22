# -*- coding: utf-8 -*-
"""
@author: cenrong.dai@dorabot.com
@brief: RRT*, Code adapted from 
    http://mahbub029.blogspot.com/2016/01/motion-planning-algorithm-rrt-starrrt.html

    Originally written by Steve LaValle, UIUC for simple RRT inMay 2011
    Modified by Md Mahbubur Rahman, FIU for RRT* in January 2016

    Steer function can change to some local planner, see papers:
    Chiang, 2019, RL-RRT: Kinodynamic Motion Planning via Learning Reachability Estimators from RL Policies; 

    NOTE The distance function and cost function which computes the cost from a state to the next can be different;
    see State.square_state_distance(next_state) and RRTStar().cost_func
    NOTE choose_nearest_state use euclidean distance rather than cost_func; cost function can be polyline path instead of line
    NOTE RRTStar.EPSILON, RRTStar.RADIUS affects the performance
"""

import time
import random
import numpy as np
from math import sqrt,cos,sin,atan2
from global_planners.global_planner import GlobalPlanner, MapType
from global_planners.rrtstar_helper import *


class RRTStar(GlobalPlanner):
    EPSILON = 1.0 # max step distance when steering
    MAX_NODES_EXPANDED = 10000
    MAX_SEARCH_TIME = 50 # 50s
    RADIUS = 1.5 # max state distance when rewind, find nearest or parent
    VISUAL = False # show search process for debug/visualization
    
    MAP = MapType.CONTINUOUS
    '''for debug'''
    # np.random.seed(100)
    # random.seed(100)

    def compute_path(self, position, goal_pose, continous_space, sensor_observation):
        print "agent {} search path via RRT* ... ".format(self.agent.id)
        #main
        self.debug_screen = pygame.display.get_surface()
        start_search_time = time.time()

        '''for debug visualization'''
        if RRTStar.VISUAL:
            draw_circle(self.debug_screen, color_dict[self.agent.id], goal_pose, 7)
            pygame.display.flip()
        '''end for debug visualization'''
        
        init_state = State(position.x, position.y, 0)
        goal_state = State(goal_pose.x, goal_pose.y)
        states_list = [init_state]

        terminated = False

        '''set function'''
        sample_func = self.goal_orientated_guassian_sample_state # can be random search or informed search, see uniform_random_sample_state / goal_orientated_guassian_sample_state
        steer_func = self.straight_line_steer # can be steered by local planner, need further implementation
        self.cost_func = self.euclidean_cost # can be deep NN; see Chiang, 2019, RL-RRT
        # cost_func is used in functions for finding nearest_state, choose_parent and rewind

        while not terminated:
            sample_state = sample_func(continous_space, goal_state)
            nearest_state = self.choose_nearest_state(continous_space, states_list, sample_state)
            new_state = steer_func(continous_space, nearest_state, sample_state)
            if new_state == None: # not found
                continue
            (new_state, new_parent) = self.choose_parent(continous_space, new_state, states_list)
            if new_parent == None: # all collide
                continue
            states_list.append(new_state) # valid new-state
            
            '''for debug visualization'''
            if RRTStar.VISUAL:
                draw_circle(self.debug_screen, color_dict[self.agent.id], new_state, 2)
                draw_line(self.debug_screen, color_dict[self.agent.id], new_parent, new_state)
                pygame.display.flip()
            '''end for debug visualization'''

            states_list = self.rewind(continous_space, states_list, new_state)
            
            # terminate function currently return at the first result (not optimal)
            terminated = self.terminate(time.time() - start_search_time, len(states_list), continous_space, new_state, goal_state)

            interaction_listener()

        path = self.extract_path(goal_state)
        path.reverse()
        print "RRT* DONE in {} sec".format(time.time() - start_search_time)
        return path

    def extract_path(self, goal_state):
        path = []
        previous = goal_state
        while previous.parent != None:
            path.append(Point(previous.x, previous.y))
            previous = previous.parent
        path.append(Point(previous.x, previous.y)) # init_state
        return path

    def terminate(self, search_time_elapsed, num_states, continuous_space, new_state, goal_state):
        '''if new-state sits within RRT.RADIUS of goal-state and no obstacle in way, add edge from new-state to goal-state'''
        if new_state.square_state_distance(goal_state) <= RRTStar.RADIUS**2:
            # test whether it is a better path, if yes, update parent
            if goal_state.parent == None or goal_state.cost > new_state.cost + self.cost_func(continuous_space, new_state, goal_state, True):
                '''for debug visualization'''
                if RRTStar.VISUAL:
                    if goal_state.parent != None:
                        draw_line(self.debug_screen, bg, goal_state.parent, goal_state)
                '''end for debug visualization'''
                
                goal_state.state_update_parent(continuous_space, new_state, self.cost_func)
                
                '''for debug visualization'''
                if RRTStar.VISUAL:
                    draw_line(self.debug_screen, color_dict[self.agent.id], goal_state.parent, goal_state)
                '''end for debug visualization'''
                
        '''return at some threshold meet, better path can be otain at the cost of more search time+space'''
        if goal_state.parent != None: # or search_time_elapsed > RRTStar.MAX_SEARCH_TIME or num_states > RRTStar.MAX_NODES_EXPANDED:
            return True
        return False

    def rewind(self, continuous_space, states_list, new_state):
        for temp_state in states_list:
            if temp_state != new_state.parent and\
                new_state.square_state_distance(temp_state) <= RRTStar.RADIUS**2 and\
                    new_state.cost + self.cost_func(continuous_space, new_state, temp_state, True) < temp_state.cost:
                    '''for debug visualization'''
                    if RRTStar.VISUAL:
                        draw_line(self.debug_screen, bg, temp_state.parent, temp_state)
                    '''end for debug visualization'''
                    
                    temp_state.state_update_parent(continuous_space, new_state, self.cost_func)
                    
                    '''for debug visualization'''
                    if RRTStar.VISUAL:
                        draw_line(self.debug_screen, color_dict[self.agent.id], new_state, temp_state)
                    '''end for debug visualization'''
        return states_list

    def choose_parent(self, continuous_space, new_state, states_list):
        '''among the near-state seated within RRT.RADIUS around new-state, find the nearest-state which has the smallest cost distance towards new-state;
        if None is returned, all the edges from near-states to new-state collide with some obstacle'''
        cost_from_nearest_to_new = POS_INF
        nearest_state = None
        for temp_state in states_list:
            if temp_state.square_state_distance(new_state) <= RRTStar.RADIUS**2:
                temp_cost = temp_state.cost + self.cost_func(continuous_space, temp_state, new_state)
                if temp_cost < cost_from_nearest_to_new:
                    nearest_state = temp_state
                    cost_from_nearest_to_new = temp_cost
        if nearest_state != None:
            new_state.state_update_parent(continuous_space, nearest_state, self.cost_func)
        return (new_state, nearest_state)


    def straight_line_steer(self, continuous_space, nearest_state, sample_state):
        '''steer from the nearest-state towards sample-state, with a max length of RRT.EPSILON, if new-state in free space, return; otherwise, return None'''
        new_state = None
        if nearest_state.square_state_distance(sample_state) <= RRTStar.EPSILON**2:
            new_state = sample_state
        else:
            theta = atan2(sample_state.y-nearest_state.y, sample_state.x-nearest_state.x)
            new_state = State(nearest_state.x+RRTStar.EPSILON*cos(theta), nearest_state.y+RRTStar.EPSILON*sin(theta))
        if not continuous_space.is_in_free_space(new_state.to_tuple()):
            return None
        return new_state

    def uniform_random_sample_state(self, continuous_space, goal_state):
        width = continuous_space.width_in_meters
        height = continuous_space.height_in_meters
        sample_state = None
        while True:
            pos = (random.random()*width, random.random()*height)
            # if continuous_space.is_in_free_space(pos):
            sample_state = State(pos[0], pos[1])
            break
        return sample_state

    def goal_orientated_guassian_sample_state(self, continuous_space, goal_state):
        '''the probability distribution of sampling is Gaussian distribution centered at goal'''
        width = continuous_space.width_in_meters
        height = continuous_space.height_in_meters
        sample_state = None
        while True:
            mean = [goal_state.x, goal_state.y]
            cov = [[width*RRTStar.RADIUS, 0],[0, height*RRTStar.RADIUS]]
            pos = tuple(np.random.multivariate_normal(mean, cov, 1)[-1])
            # if continuous_space.is_in_free_space(pos):
            sample_state = State(pos[0], pos[1])
            break
        return sample_state

    def choose_nearest_state(self, continuous_space, states_list, sample_state):
        '''the distance is computed according to 2D distance without collision check, no based on cost_func;
        since sample-state can be far away from existing states (especially at beginning), collision check is turned off'''
        nearest_square_distance = POS_INF
        nearest_state = None
        for temp_state in states_list:
            temp_square_distance = temp_state.square_state_distance(sample_state)
            if temp_square_distance < nearest_square_distance:
                nearest_square_distance = temp_square_distance
                nearest_state = temp_state
        return nearest_state

    def euclidean_cost(self, continuous_space, from_state, to_state, collision_check = True):
        if collision_check and continuous_space.subpath_obstacles_collision(from_state, to_state): # half-agent-size inflation is handled by continuous-space
            return POS_INF

        return from_state.state_distance(to_state)