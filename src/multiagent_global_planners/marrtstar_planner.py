# -*- coding: utf-8 -*-
"""
@author: cenrong.dai@dorabot.com
@brief: Multi-agent RRT* search in joint-space, see paper:
    Cap, 2013, Multi-agent RRT*: Sampling-based Cooperative Pathfinding

    RRT* Code adapted from 
    http://mahbub029.blogspot.com/2016/01/motion-planning-algorithm-rrt-starrrt.html

    Originally written by Steve LaValle, UIUC for simple RRT inMay 2011
    Modified by Md Mahbubur Rahman, FIU for RRT* in January 2016

    MARRT* can be modified to accept local planner as steering function, see paper:
    Chiang, 2019, RL-RRT: Kinodynamic Motion Planning via Learning Reachability Estimators from RL Policies; 
    NOTE choose_nearest_state use euclidean distance rather than heuristic_func; heuristic function can be polyline path instead of line
"""

import numpy as np
import time
import random, math, pygame
from math import sqrt,cos,sin,atan2
from multiagent_global_planners.multiagent_planner import MultiAgentPlanner
from global_planners.global_planner import MapType
from global_planners.rrtstar_planner import RRTStar
from global_planners.rrtstar_helper import *

	
class MARRTStar(MultiAgentPlanner):
    EPSILON = 1.0 # max step distance when steering
    MAX_NODES_EXPANDED = 10000
    MAX_SEARCH_TIME = 60 # 50s
    RADIUS = 1.5 # max state distance when rewind, find nearest or parent
    VISUAL = True # show search process for debug/visualization
    report_time = 5 # report search progress every time interval
    
    MAP = MapType.CONTINUOUS
    '''for debug'''
    # np.random.seed(100)
    # random.seed(100)

    def compute_path(self):
        '''return a solution path dictionary recording all sequence_of_pose of all agents under control, otherwise return None if no path found'''
        self.debug_screen = pygame.display.get_surface() # for debug

        all_agents_dict = self.server.collect_agents_info(self.agents)
        init_joint_state_dict = {} # <agent.id, agent_state in MARRT*>
        goal_joint_state_dict = {}

        '''for debug'''
        if MARRTStar.VISUAL:
            for agent in all_agents_dict.values():
                if agent.destination_location != None:
                    draw_circle(self.debug_screen, color_dict[agent.id], agent.destination_location, 7)
        '''end for debug visualization'''
                
        try:
            for agent in all_agents_dict.values():
                agent_init_state = State(agent.position.x, agent.position.y)
                agent_goal_state = State(agent.destination_location.x, agent.destination_location.y)
                init_joint_state_dict[agent.id] = agent_init_state
                goal_joint_state_dict[agent.id] = agent_goal_state
        except Exception as e:
            print "marrtstar_planner.py: not all agents have destination, empty path is returned"
            return None

        solution_paths_dict = self.grow_MARRTStar(JointState(init_joint_state_dict, 0), JointState(goal_joint_state_dict, POS_INF), all_agents_dict)
        self.server.refresh_agents_path(solution_paths_dict)
        return solution_paths_dict

    def grow_MARRTStar(self, init_joint_state, goal_joint_state, all_agents_dict):
        #main
        joint_states_list = [init_joint_state]

        start_search_time = time.time()
        terminated = False

        '''set function'''
        rrtstar_paths_dict = {} # for hierarchical sampling, first find the rrt path of each agent and use as sampling reference
        sample_func = self.hierarchical_sample_joint_state # self.uniform_random_sample_joint_state, hierarchical_sample_joint_state, informed_sample_joint_state
        steer_func = self.straight_line_steer
        self.cost_func = self.euclidean_cost # can be deep NN; see Chiang, 2019, RL-RRT
        self.time_func = self.lockstep_time

        while not terminated:
            sample_joint_state = sample_func(goal_joint_state, all_agents_dict, rrtstar_paths_dict)
            '''note that sample_joint_state.active_agents_id_set = 0'''
            nearest_joint_state = self.choose_nearest_joint_state(joint_states_list, sample_joint_state)
            new_joint_state = steer_func(nearest_joint_state, sample_joint_state) # new_joint_state has time and cost infinity
            '''note that new_joint_state.active_agents_id_set = 0, time = inf'''
            # instead of directly set nearest as parent, check for a possible smaller one
            if new_joint_state == None:
                continue
            (new_joint_state, parent) = self.choose_parent(new_joint_state, joint_states_list)
            if parent == None:
                continue
            new_joint_state.check_agents_arrive_goal(self.environment_map, goal_joint_state, MARRTStar.EPSILON)
            joint_states_list.append(new_joint_state) # with parent as new parent
            
            """for debug"""
            if MARRTStar.VISUAL:
                self.visualize_joint_state(new_joint_state)
                self.visualize_consecutive_joint_state(parent, new_joint_state)
            """end for debug"""

            joint_states_list = self.rewind(joint_states_list, new_joint_state)
            terminated = self.terminate(time.time()-start_search_time, len(joint_states_list), new_joint_state, goal_joint_state)
            interaction_listener()

        joint_states_path = self.find_solution_path(goal_joint_state)

        solution_paths_dict = self.extract_paths_from_joint_states(joint_states_path)
        print "MARRT* DONE in {} sec".format(time.time() - start_search_time)
        return solution_paths_dict
    
    def extract_paths_from_joint_states(self, joint_states_path):
        goal_joint_state = joint_states_path[-1]
        agents_id_list = goal_joint_state.agents_state.keys()
        joint_states_path.reverse()
        agents_path_dict = {aid:[] for aid in agents_id_list}
        for joint_state in joint_states_path:
            for aid, state in joint_state.agents_state.items():
                agents_path_dict[aid].append(Point(state.x, state.y))
        return agents_path_dict

    def terminate(self, time_elapsed, num_joint_states, new_joint_state, goal_joint_state):
        if time_elapsed >= MARRTStar.report_time:
            print "marrt*.py: already search for {} second".format(int(time_elapsed))
            MARRTStar.report_time = (int(time_elapsed) // 5)*5 + 5

        '''check whether wire goal-joint-state to new-joint-state is possible and gives lower cost'''
        if new_joint_state.is_each_state_distance_to_other_within_radius(goal_joint_state,MARRTStar.RADIUS) and\
            goal_joint_state.cost > new_joint_state.cost + self.cost_func(new_joint_state, goal_joint_state):
            goal_joint_state.update_parent(new_joint_state, self.cost_func, self.time_func, True)

            path = self.find_solution_path(goal_joint_state)
            '''for debug'''
            if MARRTStar.VISUAL:
                self.visualize_solution_path(path)
                # print "DONE:", [str(js) for js in path], "terminated_cost:", goal_joint_state.cost, "time_elapsed:", time_elapsed
                '''end for debug'''
                # pausable_interface(True)
            return True

            # paused = True
            # while paused:
            #     for event in pygame.event.get():
            #         if event.type == KEYDOWN:
            #             if event.key == pygame.K_p:
            #                 paused = not paused
            #                 self.devisualize_solution_path(path)
            #             elif event.key == pygame.K_q: # Q/q is pressed
            #                 exit()
            #             elif event.key == pygame.K_t:
            #                 return True
                        
        return time_elapsed > MARRTStar.MAX_SEARCH_TIME# or num_joint_states > MAX_NODES_EXPANDED


    def hierarchical_sample_joint_state(self, goal_joint_state, all_agents_dict, reference_rrtstar_paths_dict):
        '''first, each agent search for its own path ignoring other agents; Then use the waypoints on the path as Gaussian distribution
        for searching in joint-space'''
        if len(reference_rrtstar_paths_dict) != len(all_agents_dict):
            # find indivisual rrt path if no prvious record
            RRTStar.VISUAL = MARRTStar.VISUAL
            
            for agent in all_agents_dict.values():
                rrtstar_planner = RRTStar(agent) # NOTE sample, steer, and cost function have to be manually changed if needed
                reference_rrtstar_paths_dict[agent.id] = rrtstar_planner.compute_path(agent.position, agent.destination_location, self.environment_map, agent.perception_module)
                
            '''for debug'''
            if MARRTStar.VISUAL:
                for aid, path in reference_rrtstar_paths_dict.items():
                    for waypoint in path:
                        draw_circle(self.debug_screen, color_dict[aid], waypoint, 4)
                # pausable_interface(True)

        width = self.environment_map.width_in_meters
        height = self.environment_map.height_in_meters
        sample_joint_state = JointState({}, POS_INF)
        agents_id_list = goal_joint_state.agents_state.keys()
        for aid in agents_id_list:
            found = False
            candidates = []
            while not found:
                waypoints = reference_rrtstar_paths_dict[aid]
                for point in waypoints:
                    mean = [point.x, point.y]
                    cov = [[width, 0],[0, height]]
                    candidates.extend(np.random.multivariate_normal(mean, cov, 10))
                random.shuffle(candidates)
                for candidate in candidates:
                    # if self.environment_map.is_in_free_space(candidate):
                    sample_joint_state[aid] = State(candidate[0], candidate[1])
                    found = True
                    break
        return sample_joint_state

    def informed_sample_joint_state(self, goal_joint_state, all_agents_dict, reference_rrtstar_paths_dict):
        width = self.environment_map.width_in_meters
        height = self.environment_map.height_in_meters
        sample_joint_state = JointState({}, POS_INF)
        agents_id_list = goal_joint_state.agents_state.keys()
        for aid in agents_id_list:
            pos = None
            goal_state = goal_joint_state[aid]
            while True:
                mean = [goal_state.x, goal_state.y]
                cov = [[width*MARRTStar.RADIUS, 0],[0, height*MARRTStar.RADIUS]]

                pos = tuple(np.random.multivariate_normal(mean, cov, 1)[-1])
                # if self.environment_map.is_in_free_space(pos):
                sample_joint_state[aid] = State(pos[0], pos[1])
                break
        return sample_joint_state

    def uniform_random_sample_joint_state(self, goal_joint_state, all_agents_dict, reference_rrtstar_paths_dict):
        """ return a joint-state which is formed by sample a valid state position for each agent, no matter active or not """
        width = self.environment_map.width_in_meters
        height = self.environment_map.height_in_meters
        sample_joint_state = JointState({}, POS_INF)
        agents_id_list = goal_joint_state.agents_state.keys()
        for aid in agents_id_list:
            pos = None
            while True:
                pos = (random.random()*width, random.random()*height)
                # if self.environment_map.is_in_free_space(pos):
                sample_joint_state[aid] = State(pos[0], pos[1])
                break
        return sample_joint_state
            
    def choose_nearest_joint_state(self, joint_states_list, sample_joint_state):
        """for newly sampled joint-state, return the nearest joint-state where the total ***euclidean distance*** of ***all*** is the smallest (not cost_func, not only active agents)
        """
        nearest_distance = POS_INF
        nearest_joint_state = None
        for temp_joint_state in joint_states_list:
            temp_distance = temp_joint_state.to_joint_state_active_distance(sample_joint_state)
            if temp_distance < nearest_distance:
                nearest_distance = temp_distance
                nearest_joint_state = temp_joint_state
        return nearest_joint_state

    def straight_line_steer(self, nearest_joint_state, sample_joint_state):
        """return a joint-state which drives every agent move from the nearest-joint-state towards the sample-joint-state """
        nearest_joint_state.check_joint_state_consistency(sample_joint_state)

        new_joint_state = JointState({}, POS_INF)
        for aid, state in nearest_joint_state.agents_state.items():
            sample_state = sample_joint_state[aid]
            new_state = None
            if state.square_state_distance(sample_state) <= MARRTStar.EPSILON**2:
                # sample_state is the new waypoint of current agent
                new_state = State(sample_state.x, sample_state.y)
            else:
                # move towards sample_state by a magnitude of EPSILON
                theta = atan2(sample_state.y-state.y, sample_state.x-state.x)
                new_state = State(state.x+MARRTStar.EPSILON*cos(theta), state.y+MARRTStar.EPSILON*sin(theta))
            if not self.environment_map.is_in_free_space(new_state.to_tuple()):
                return None
            new_joint_state[aid] = new_state
        return new_joint_state

    def choose_parent(self, new_joint_state, joint_states_list):
        """ compute the cost from new-joint-state to joint-states within a RADIUS distance neighborhood;
        return (new_joint_state, nearest_joint_state) """
        cost_from_nearest_to_new = POS_INF
        nearest_joint_state = None
        for temp_joint_state in joint_states_list:
            if temp_joint_state.is_each_state_distance_to_other_within_radius(new_joint_state, MARRTStar.RADIUS):
                temp_cost = temp_joint_state.cost + self.cost_func(temp_joint_state, new_joint_state, True)
                if temp_cost < cost_from_nearest_to_new:
                    nearest_joint_state = temp_joint_state
                    cost_from_nearest_to_new = temp_cost
        if nearest_joint_state != None:
            new_joint_state.update_parent(nearest_joint_state, self.cost_func, self.time_func)
        return (new_joint_state, nearest_joint_state)

    def rewind(self, joint_states_list, new_joint_state):
        for temp_joint_state in joint_states_list:
            if temp_joint_state != new_joint_state.parent and\
                new_joint_state.is_each_state_distance_to_other_within_radius(temp_joint_state, MARRTStar.RADIUS) and\
                    new_joint_state.cost + self.cost_func(new_joint_state, temp_joint_state, True) < temp_joint_state.cost:
                    '''for debug'''
                    if MARRTStar.VISUAL:
                        self.remove_consecutive_joint_state_visualization(temp_joint_state.parent, temp_joint_state)
                    '''end for debug'''
                    
                    temp_joint_state.update_parent(new_joint_state, self.cost_func, self.time_func)
                    # we do not update inactive agent of temp_joint_state here since it should already be correct and need no change
                    '''for debug'''
                    if MARRTStar.VISUAL:
                        self.visualize_consecutive_joint_state(temp_joint_state.parent, temp_joint_state)
                    '''end for debug'''
        return joint_states_list
    
    def find_solution_path(self, goal_joint_state):
        """find the parent of the goal-joint-state and return a solution path"""
        solution_path = []

        previous_joint_state = goal_joint_state
        while previous_joint_state.parent != None:
            solution_path.append(previous_joint_state)
            previous_joint_state = previous_joint_state.parent
        solution_path.append(previous_joint_state)
        return solution_path

    def euclidean_cost(self, from_joint_state, to_joint_state, collision_check = True):
        if (collision_check and from_joint_state.to_joint_state_collision(self.environment_map, to_joint_state)):
            return POS_INF
        '''change between joint_state_active_distance_to and joint_state_total_distance_to gives different searching actions'''
        return from_joint_state.to_joint_state_active_distance(to_joint_state)
        # return from_joint_state.to_joint_state_total_distance(to_joint_state)

    def lockstep_time(self, from_joint_state, to_joint_state):
        return 1



    '''for debug'''
    def visualize_solution_path(self, path):
        for joint_state in path:
            for aid, from_state in joint_state.agents_state.items():
                try:
                    to_state = joint_state.parent[aid]
                    draw_line(self.debug_screen, green, from_state, to_state, 3)
                except:
                    pass
        pygame.display.flip()

    def devisualize_solution_path(self, path):
        if not MARRTStar.VISUAL:
            return
        for joint_state in path:
            for aid, from_state in joint_state.agents_state.items():
                try:
                    to_state = joint_state.parent[aid]
                    draw_line(self.debug_screen, bg, from_state, to_state, 3)
                    draw_line(self.debug_screen, color_dict[aid], from_state, to_state)
                except:
                    pass
        pygame.display.flip()


    def visualize_joint_state(self, new_joint_state):
        if not MARRTStar.VISUAL:
            return
        color = (0, new_joint_state.t*10 % 256, 0)
        font = pygame.font.SysFont('Calibri', 20) 
        for aid, state in new_joint_state.agents_state.items():
            draw_circle(self.debug_screen, color, Point(state.x, state.y), 2)
            if new_joint_state.t % 5 == 0:
                text_obj=font.render(str(new_joint_state.t), True, color_dict[aid])
                text_pos=text_obj.get_rect()
                text_pos.center=list(rectify_meter_to_pixel(Point(state.x, state.y)))
                self.debug_screen.blit(text_obj,text_pos)

    def visualize_consecutive_joint_state(self, from_joint_state, to_joint_state):
        if not MARRTStar.VISUAL:
            return
        for aid, from_state in from_joint_state.agents_state.items():
            to_state = to_joint_state[aid]
            draw_line(self.debug_screen, color_dict[aid], from_state, to_state)
        pygame.display.flip()

    def remove_consecutive_joint_state_visualization(self, from_joint_state, to_joint_state):
        if from_joint_state == None: # the case when from-joint-state is root or goal
            return
        for aid, from_state in from_joint_state.agents_state.items():
            to_state = to_joint_state[aid]
            if MARRTStar.VISUAL:
                draw_line(self.debug_screen, bg, from_state, to_state)
    
    '''end for debug'''



# TODO: should we consider agents that require a goal within a short distance and a goal within a long distance, in which case,
# if we keep the RRT* state length the same for all, those with short goals may suffer from wandering? -> active agents
# the same problem exist in steer

# TODO: joint-state distance test is based on euclidean and uniform weighted sum, may be further improved?

# TODO: find the nearest state is by loop through all the joint-states in the tree any better solution?\

# TODO: self.cost; should cost related to rotation angle? safety?

# TODO: obstacle check collision can have some inflation; this shall be implemented in Obstacles.py

# TODO: wrap cost function just like the state machine

# TODO: can we also generate branch from goal
# No, since we require time, and search in joint-space

# TODO: if a path is found for an agent, can we increase the probability of sampling along that path?


# TODO: will the order of rewind affect the quality?

# TODO: steer has some problem

# TODO: dont search in joint-space, which gives really ungly result. try to use Zhu