# -*- coding: utf-8 -*-
"""
@author: cenrong.dai@dorabot.com
@breif: iNash Trajectory, ideas from papers:
    Zhu, 2014, Game theoretic controller synthesis for multi-robot motion planning Part I trajectory based algorithm

    Each agent search one step of RRT path in sequential order, annouce its choice and then all the other agents change
    trajectory based on the annoucement. It shall converge to Nash Equilibrium when the iterations go to infinity.
    NOTE this is an RRT-based algorithm, which means no rewind
    NOTE choose_nearest_state use euclidean distance rather than cost_func; cost  function can be polyline path instead of line
"""

import numpy as np
import time
import random, math, pygame, copy
from math import sqrt,cos,sin,atan2,pi
from geometry import *
from multiagent_global_planners.multiagent_planner import MultiAgentPlanner
from global_planners.global_planner import MapType
from global_planners.sample_global_planner import PriorityQueue
from global_planners.rrtstar_helper import *
from visualisation import draw_agent


class iNashState(State):
    '''the state has multiple parents, denoted as in_edges go from parent state to current state;
    here, state.parent points to the from_state which gives the lowest cost;
    state,best_edge is the edge from the best parent to current state which has the minimum cost
    state.cost is the minimum cost going from root to current state'''
    in_edges = None
    best_edge = None
    static_perception = None # record percetion at a certain time frame

    def add_in_edge(self, from_state, edge_cost, edge_traversal_time, from_state_velocity = 0.0, to_state_velocity = 0.0):
        '''velocity is recorded for local planner steering'''
        # print "from {}, distance {}, time {}, from_speed {}, to_speed {}".format(from_state, edge_cost, edge_traversal_time, from_state_velocity, to_state_velocity)
        if self.in_edges == None: # not initialized
            self.in_edges = []
        new_edge = iNashStateEdge(from_state, self, edge_cost, edge_traversal_time, from_state_velocity, to_state_velocity)
        self.in_edges.append(new_edge)
        # check whether need to update the min cost of the state
        new_cost = from_state.cost + edge_cost
        if new_cost < self.cost:
            self.cost = new_cost
            self.parent = from_state
            self.best_edge = new_edge

class iNashStateEdge:
    '''In iNash trajectory algorithm, we only add in_edge to a new state and do not rewind edge,
    thus the attributes of iNashEdge basically do not change'''
    def __init__(self, from_state, to_state, edge_cost, edge_traversal_time, from_state_velocity = 0.0, to_state_velocity = 0.0):
        self.from_state = from_state
        self.to_state = to_state
        self.edge_cost = edge_cost
        self.edge_traversal_time = edge_traversal_time
        self.from_state_velocity = from_state_velocity
        self.to_state_velocity = to_state_velocity

    def __str__(self):
        # return "{}--->{}, cost:{}, time:{}".format(self.from_state, self.to_state, self.edge_cost, self.edge_traversal_time)
        return "{}--->{}".format(self.from_state, self.to_state)

    def cost_so_far(self):
        '''return the cost so far from root to to-state of the edge, via the edge;
        note that to-state.cost return the minimum cost from root to current to-state, which not necessarily
        pass current edge'''
        return self.from_state.cost + self.edge_cost

class INashRRT(MultiAgentPlanner):
    EPSILON = 1.0 # max step distance when steering
    MAX_NODES_EXPANDED = 10000
    MAX_SEARCH_TIME = 10 # 50s
    RADIUS = 1.5 # max state distance when rewind, find nearest or parent
    VISUAL = True # show search process for debug/visualization
    ETA = 5 # max length of steering is at most ETA
    REACT_INTERVAL = 1 # recompute a best response after each agent has extend another several states
    LOCAL_STEPS_LIMIT = 100 # local simulation step limit

    MAP = MapType.CONTINUOUS
    report_time = 10 # report search progress every time interval
    '''for debug'''
    np.random.seed(1)
    random.seed(1)
    
    def compute_path(self):
        '''return a solution path dictionary recording all sequence_of_pose of all agents under control, otherwise return None if no path found'''
        self.debug_screen = pygame.display.get_surface() # for debug
        agents_abstraction_dict = self.server.collect_agents_info(self.agents) # can only request those under its control
        start_search_time = time.time()

        
        '''for debug : draw all agents destination'''
        if INashRRT.VISUAL:
            for agent in agents_abstraction_dict.values():
                if agent.destination_location != None:
                    draw_circle(self.debug_screen, color_dict[agent.id], agent.destination_location, 7)
        '''end for debug visualization'''

            
        '''set function'''
        self.sample_func = self.goal_orientated_guassian_sample_state # goal_orientated_guassian_sample_state | uniform_random_sample_state
        self.steer_func = self.straight_line_steer # straight_line_steer | local_planner_steer
        # self.cost_func = self.euclidean_cost
        # self.time_func = self.euclidean_time

        # each agent keeps its own states_list, choose best response from the list against the other agents' current paths
        agents_states_list_dict = {} # <aid, list of existing states of this agent>
        agents_goal_state_dict = {} # <aid, goal state>
        agents_best_edge_path_dict = {aid:[] for aid in agents_abstraction_dict.keys()} # <aid, agent's edge list which forms best reponse>
        active_agents_id_list = []

        for agent in agents_abstraction_dict.values():
            init_state = iNashState(agent.position.x, agent.position.y, 0)
            agents_states_list_dict[agent.id] = [init_state]
            agents_goal_state_dict[agent.id] = iNashState(agent.destination_location.x, agent.destination_location.y)
            self.add_edge_if_single_agent_path_free(init_state, agents_goal_state_dict[agent.id], agents_abstraction_dict[agent.id]) # connect_to_goal_state

        terminated = False
        counter_k = 1 # track for the number of nodes that each agent has extended
        sample_counter = 0 # track how many states has been extended, if reach REACT_INTERVAL, compute best response for all
        while not terminated:
            counter_k += 1
            sample_counter += 1
            # each agent computes one step
            for agent in agents_abstraction_dict.values():
                if INashRRT.VISUAL:
                    interaction_listener() # for interaction
                feasible_new_state_found = False
                while not feasible_new_state_found:
                    sample_state = self.sample_func(agents_goal_state_dict[agent.id])
                    (feasible_new_state_found, agents_states_list_dict[agent.id]) =\
                        self.extend(agents_states_list_dict[agent.id],
                                    sample_state, agents_goal_state_dict[agent.id],
                                    INashRRT.RADIUS, agents_abstraction_dict[agent.id]) #min(ETA, RADIUS*((log(counter_k)/counter_k)**(1/len(all_agents_dict)))))
                
                # TODO check agent activation, may need a mechanism to inactivate some agent in the case of large agent number, so that speed up computation
                for temp_agent in agents_abstraction_dict.values():
                    if temp_agent.id not in active_agents_id_list and agents_goal_state_dict[agent.id].in_edges == None:
                        active_agents_id_list.append(temp_agent.id)
                    
                '''draw current new state for debugging'''
                if INashRRT.VISUAL:
                    new_state = agents_states_list_dict[agent.id][-1]
                    draw_circle(self.debug_screen, color_dict[agent.id], new_state, 2)
                    for edge in new_state.in_edges:
                        draw_arrow(self.debug_screen, color_dict[agent.id], edge.from_state, edge.to_state)
                    # draw edges to goal if exist
                    try:
                        goal_edge = agents_goal_state_dict[agent.id].in_edges[-1]
                        draw_arrow(self.debug_screen, color_dict[agent.id], goal_edge.from_state, goal_edge.to_state)
                    except:
                        pass
                '''end for debug'''
            
            if sample_counter >= INashRRT.REACT_INTERVAL:
                sample_counter = 0
                # every agent tries to adjust its path after REACT_INTERVAL
                for active_agent_id in active_agents_id_list:
                    agents_best_edge_path_dict[active_agent_id] = self.best_response(active_agent_id,
                                                                                     agents_goal_state_dict[active_agent_id], 
                                                                                     agents_states_list_dict[active_agent_id],
                                                                                     agents_best_edge_path_dict, agents_abstraction_dict)
            else:
                continue # sample another number of REACT_INTERVAL states for each agent

            terminated = self.terminate(time.time()-start_search_time, counter_k, agents_best_edge_path_dict)

        # if existing solutions for all agent, extract path
        (agents_point_path_dict, agents_cost_dict) = self.extract_solution_paths_from_edges(agents_best_edge_path_dict)

        '''draw solution path for debugging'''
        if INashRRT.VISUAL:
            for _, path in agents_point_path_dict.items():
                if len(path) == 0: continue
                previous = path[0]
                for point in path[1:]:
                    draw_arrow(self.debug_screen, green, previous, point, 3)
                    previous = point
            pygame.display.flip()
        '''end for debug'''

        self.server.refresh_agents_path(agents_point_path_dict)
        print "INashRRT DONE in {} sec".format(time.time() - start_search_time)
        return agents_point_path_dict
    
    def terminate(self, time_elapsed, num_each_agent_states, agents_best_edge_path_dict):
        if time_elapsed >= INashRRT.report_time:
            print "inash_planner*.py: already search for {} second".format(int(time_elapsed))
            INashRRT.report_time = (int(time_elapsed) // 10)*10 + 10

        if all([len(path) for path in agents_best_edge_path_dict.values()]):
            return True
        return time_elapsed > INashRRT.MAX_SEARCH_TIME # or num_each_agent_states > MAX_NODES_EXPANDED
        # return False

    def extract_solution_paths_from_edges(self, agents_best_edge_path_dict):
        '''agents_best_edge_path_dict = dictionary of agent.id: [list of (edge, start_t, end_t)] pairs'''
        agents_point_path_dict = {aid:[] for aid in agents_best_edge_path_dict.keys()}
        agents_cost_dict = {aid:0 for aid in agents_best_edge_path_dict.keys()}
        for aid, edges in agents_best_edge_path_dict.items():
            if len(edges) == 0:
                continue
            (edge,_,_) = edges[0]
            agents_point_path_dict[aid].append(edge.from_state.point) # append the init state
            for (edge,_,_) in edges:
                agents_point_path_dict[aid].append(edge.to_state.point)
                agents_cost_dict[aid] += edge.edge_cost
        return (agents_point_path_dict, agents_cost_dict)

    def best_response(self, agent_id, goal_state, states_list, agents_best_edge_path_dict, agents_abstraction_dict):
        # perform a revert A* search in states_list graph
        if goal_state.cost == POS_INF:
            # no feasible path available
            return []
        # test all feasible path, starting from the least cost one
        frontier = PriorityQueue()
        '''The element stores in frontier is of the following structure:
        ([list of edges when the path diverges from the optimal], cost of the path);
        say we have 3 paths to goal: A->B->C->G, A->C->G, A->G, each edge cost 1; A->G is the minimum path to reach goal;
        thus, ([edge(A,G)], cost) records the path that following the minimum path [A->G], and the corresponding cost;
        if [A->G] collide with other agent's path, we need to compromise to a sub-optimal solution:
        ([edge(C,G)], cost), which records the path that following the minimum path from A to C, then concate [C, G] to that minimum path,
        which gives [A->C->G]'''

        for edge in goal_state.in_edges:
            frontier.put([edge], edge.from_state.cost + edge.edge_cost) # cost is the minimum cost of from state + the edge cost
        while not frontier.empty():
            current_path_cost, current_diverge_edge_path = frontier.get_pair()
            current_head = current_diverge_edge_path[0].from_state # current head is the edge where the diverge happens
            edge_timestamp_path = self.recover_edge_path(current_head, current_diverge_edge_path)
            # print [(str(edge), start_t, end_t) for edge, start_t, end_t in edge_timestamp_path]
            if self.multiple_agents_path_free(agent_id, edge_timestamp_path, agents_best_edge_path_dict, agents_abstraction_dict):
                return edge_timestamp_path # best_response
            # if collide, expand the node and go for a less optimal one
            if current_head.in_edges == None:
                continue
            for edge in current_head.in_edges:
                new_diverge_edge_path = [edge]+current_diverge_edge_path[:] # optimal until edge.from_state, then go along the diverge_path
                new_path_cost = edge.from_state.cost + edge.edge_cost + (current_path_cost - current_head.cost)
                frontier.put(new_diverge_edge_path, new_path_cost)
        # print "agent {}: cannot find best response".format(agent_id)
        return []

    def recover_edge_path(self, diverge_state, diverge_edge_path):
        '''return a path guiding from root to goal, using a list of edges'''
        edge_path = diverge_edge_path[::-1]
        previous = diverge_state
        while previous.parent != None:
            edge_path.append(previous.best_edge)
            previous = previous.parent # parent is the from_state which gives the least cost among all in_edges
        edge_path.reverse()

        edge_timestamp_path = []
        time_acc = 0
        for edge in edge_path:
            end_time = time_acc+edge.edge_traversal_time
            edge_timestamp_path.append( (edge, time_acc, end_time) )
            time_acc = end_time
        return edge_timestamp_path
        
    def multiple_agents_path_free(self, agent_id, edge_timestamp_path, agents_best_edge_path_dict, agents_abstraction_dict):
        # print "agent {}: check path collision...".format(agent_id)
        '''agents_best_edge_path_dict = dictionary of agent.id: [list of (edge, start_t, end_t)] pairs'''
        temp_agents_best_edge_path_dict = copy.deepcopy(agents_best_edge_path_dict)
        if agent_id in temp_agents_best_edge_path_dict: # remove itself
            temp_agents_best_edge_path_dict.pop(agent_id)

        if self.steer_func == self.straight_line_steer: # check timestamp
            for (edge, start_t, end_t) in edge_timestamp_path:
                # compare with the timestamp of the other agents
                for other_edge_t_path in temp_agents_best_edge_path_dict.values():
                    for (other_edge, other_start_t, other_end_t) in other_edge_t_path:
                        if other_end_t < start_t: # disjoint case 1: this other_edge happens before current edge
                            other_edge_t_path.pop(0) # can be discarded
                        elif other_start_t > end_t: # disjoint case 2:
                            break # for this other agent, safe
                        else: # all the other time-intersected case, need to check collision
                            if self.environment_map.subpath_subpath_collision(edge.from_state, edge.to_state, other_edge.from_state, other_edge.to_state):
                                # TODO HIGH PRIORITY: better to return the collide subpath, and when extend for next collision solution, avoid these two edges, so that avoid unncessary searching
                                return False
        elif self.steer_func == self.local_planner_steer:

            
            '''def update_agent_position_and_velocity(simulated_agent, new_position_tuple, new_velocity_tuple):
                simulated_agent.position = new_position_tuple
                simulated_agent.userData.position = Point(new_position_tuple[0], new_position_tuple[1])
                simulated_agent.linearVelocity = new_velocity_tuple
                simulated_agent.userData.linear_velocity = new_velocity_tuple
                new_angle = atan2(new_velocity_tuple[1], new_velocity_tuple[0])
                simulated_agent.angle = new_angle
                simulated_agent.userData.angle = new_angle

            def compute_velocity(agent_id, agent_abstraction):

                
            # when other agent running agents_best_edge_path_dict, current agent run edge_timestamp_path
            extract pose path
            while no any contact and not all agent arrive destination:
                # set all the other agent
                for other_agent_id, other_agent_path in temp_agents_best_edge_path_dict.items():

                    local_planner = agents_abstraction_dict[other_agent_id].current_local_planner
                    perception_module = agents_abstraction_dict[other_agent_id].perception_module
                    simulated_agent = perception_module.simulated_agent
                    position_tuple = simulated_agent.position
                    velocity_tuple = simulated_agent.linearVelocity
                    position = Point(position_tuple[0], position_tuple[1])
                    velocity_tuple = local_planner.compute_plan(position, velocity_tuple, self.environment_map, perception_module, [put pose path])
                    simulated_agent.linearVelocity = velocity_tuple

                # set current query agent

                self.world.Step(self.TIME_STEP, 10, 10)

                check if there is any collision'''
            raise NotImplementedError
        else:
            print "inash_planner.py: cannot find corresponding steering method in multi-agent free paths computation, do you implement one?"
            raise NotImplementedError
        return True

    def add_edge_if_single_agent_path_free(self, from_state, to_state, agent_abstraction):
        '''add_in_edge to to_state if moving from from_state to to_state using corresponding steering method (line/local planner) is safe (no collision)'''
        if self.steer_func == self.straight_line_steer:
            if from_state.square_state_distance(to_state) <= INashRRT.RADIUS**2 and\
                not self.environment_map.subpath_obstacles_collision(from_state, to_state):
                    cost = from_state.state_distance(to_state)
                    to_state.add_in_edge(from_state, cost, cost)
        elif self.steer_func == self.local_planner_steer:
            reachable, _, path_cost, time_cost, from_state_velocity, to_state_velocity = \
                self.run_local_planner(from_state, to_state, agent_abstraction)
            if reachable:
                to_state.add_in_edge(from_state, path_cost, time_cost, from_state_velocity, to_state_velocity)
        else:
            print "inash_planner.py: cannot find corresponding steering function, do you implement it?"
            raise NotImplementedError

    def extend(self, states_list, sample_state, goal_state, radius, agent_abstraction):
        '''return True and the extended agent's states_list if can steer from a nearest neighbor towards
        sample-state without any obstacle collision (path collision is not taken care of by far);
        if the newly generated state is close to goal, add an in_edge to goal going from new-state to goal-state.
        choose_nearest_state -> steer -> connect_legal_near_state'''
        nearest_state = self.choose_nearest_state(states_list, sample_state) # this can be changed to hierarchical method as in google's paper
        new_state = self.steer_func(nearest_state, sample_state, agent_abstraction)
        if new_state == None:
            return (False, states_list)
        near_states = self.find_near_states(states_list, new_state, radius)
        for near in near_states:
            if near == nearest_state:
                continue
            self.add_edge_if_single_agent_path_free(near, new_state, agent_abstraction)
        # check whether can be connected to goal_state
        self.add_edge_if_single_agent_path_free(new_state, goal_state, agent_abstraction)
        states_list.append(new_state)
        return (True, states_list)
        
    def uniform_random_sample_state(self, goal_state):
        '''return a point which does not fall into any obstacle'''
        width = self.environment_map.width_in_meters
        height = self.environment_map.height_in_meters
        sample_state = None
        while True:
            pos = (random.random()*width, random.random()*height)
            # if environment.is_in_free_space(pos):
            sample_state = iNashState(pos[0], pos[1])
            break
        return sample_state

    def goal_orientated_guassian_sample_state(self, goal_state):
        '''the probability distribution of sampling is Gaussian distribution centered at goal'''
        width = self.environment_map.width_in_meters
        height = self.environment_map.height_in_meters
        sample_state = None
        while True:
            mean = [goal_state.x, goal_state.y]
            cov = [[width*INashRRT.RADIUS, 0],[0, height*INashRRT.RADIUS]]
            pos = tuple(np.random.multivariate_normal(mean, cov, 1)[-1])
            # if self.environment_map.is_in_free_space(pos):
            sample_state = iNashState(pos[0], pos[1])
            break
        return sample_state

    def choose_nearest_state(self, states_list, sample_state):
        '''nearest based on euclidean distance rather than cost_func'''
        nearest_square_distance = POS_INF
        nearest_state = None
        for temp_state in states_list:
            temp_square_distance = temp_state.square_state_distance(sample_state)
            if temp_square_distance < nearest_square_distance:
                nearest_square_distance = temp_square_distance
                nearest_state = temp_state
        return nearest_state

    def find_near_states(self, states_list, new_state, radius):
        '''nearest based on euclidean distance rather than cost_func'''
        near_neighbors = []
        for state in states_list:
            if state.square_state_distance(new_state) <= radius**2:
                near_neighbors.append(state)
        return near_neighbors

    def euclidean_cost(self, from_state, to_state, collision_check = True):
        if collision_check and self.environment_map.subpath_obstacles_collision(from_state, to_state):
            return POS_INF

        return from_state.state_distance(to_state)

    def euclidean_time(self, from_state, to_state):
        return from_state.state_distance(to_state)

    def straight_line_steer(self, nearest_state, sample_state, agent_abstraction):
        new_state = None
        if nearest_state.square_state_distance(sample_state) <= INashRRT.EPSILON**2:
            new_state = sample_state
        else:
            theta = atan2(sample_state.y-nearest_state.y, sample_state.x-nearest_state.x)
            new_state = iNashState(nearest_state.x+INashRRT.EPSILON*cos(theta), nearest_state.y+INashRRT.EPSILON*sin(theta))
        if not self.environment_map.is_in_free_space(new_state.to_tuple())\
            or self.environment_map.subpath_obstacles_collision(nearest_state, new_state):
                return None
        cost = nearest_state.state_distance(new_state)
        new_state.add_in_edge(nearest_state, cost, cost) # or 1 in time_cost
        return new_state

    def local_planner_steer(self, nearest_state, sample_state, agent_abstraction):
        _, position, path_cost, time_cost, from_state_velocity, to_state_velocity = self.run_local_planner(nearest_state, sample_state, agent_abstraction)
        new_state = iNashState(position.x, position.y)
        new_state.add_in_edge(nearest_state, path_cost, time_cost, from_state_velocity, to_state_velocity)
        return new_state

    def run_local_planner(self, from_state, to_state, agent_abstraction):
        # return (reachable, terminated_position, path_cost, time_cost, from_state_velocity, to_state_velocity) if within simulation_steps_limit, the agent can be steered from from_state to to_state using current local planner
        '''everytime before compute local planner, the perception module should be updated
        if the current nearest_state is the root, to retrieve the starting perception, set agent speed to 0.0 and then step forward the world
        '''
        draw_circle(self.debug_screen, color_dict[agent_abstraction.id], to_state, 10)

        local_planner = agent_abstraction.current_local_planner
        perception_module = agent_abstraction.perception_module
        simulated_agent = perception_module.simulated_agent

        def update_agent_position_and_velocity(simulated_agent, new_position_tuple, new_velocity_tuple):
            simulated_agent.position = new_position_tuple
            simulated_agent.userData.position = Point(new_position_tuple[0], new_position_tuple[1])
            simulated_agent.linearVelocity = new_velocity_tuple
            simulated_agent.userData.linear_velocity = new_velocity_tuple
            new_angle = atan2(new_velocity_tuple[1], new_velocity_tuple[0])
            simulated_agent.angle = new_angle
            simulated_agent.userData.angle = new_angle

        def terminate_local_planner_simulation(position, goal, perception_module, agent_radius):
            '''terminate when expect collision or reach goal'''
            if position.arrive(goal):
                return True
            elif perception_module.ports_in_collision_of():
                return True
            elif not self.environment_map.in_bounds_in_meters(position.to_tuple()):
                return True
            return False

        position = from_state
        position_tuple = position.to_tuple()
        velocity_tuple = (0.0, 0.0)
        # print "before everything:", [b.userData for b in perception_module.detected_object]
        update_agent_position_and_velocity(simulated_agent, position_tuple, velocity_tuple)
        self.world.Step(self.TIME_STEP, 10, 10) # let the agent update its perception module
        # print "after update static:", [b.userData for b in perception_module.detected_object]

        # TODO may need to update velocity tuple if is not root
        if from_state.best_edge != None: # not root
            velocity_tuple = from_state.best_edge.to_state_velocity
        # print "first step: position:{}, vel:{}".format(position_tuple, velocity_tuple)
        from_state_velocity = velocity_tuple
        steps_count = 0
        distance_count = 0
        while not terminate_local_planner_simulation(position, to_state, perception_module, agent_abstraction.radius):
            # recompute speed
            velocity_tuple = local_planner.compute_plan(position, velocity_tuple, self.environment_map, perception_module, [to_state])
            update_agent_position_and_velocity(simulated_agent, position_tuple, velocity_tuple)
            self.world.Step(self.TIME_STEP, 10, 10) # simulated run
            # redraw the agent
            position_tuple = agent_abstraction.perception_module.simulated_agent.position # update position
            distance_count += pow((position_tuple[0]-position.x)**2.0+(position_tuple[1]-position.y)**2.0, 0.5)
            position = Point(position_tuple[0], position_tuple[1])
            update_agent_position_and_velocity(simulated_agent, position_tuple, velocity_tuple)
            draw_circle(self.debug_screen, color_dict[agent_abstraction.id], position_tuple, 1)
            steps_count += 1
            if steps_count >= INashRRT.LOCAL_STEPS_LIMIT:
                break

        to_state_velocity = velocity_tuple
        
        update_agent_position_and_velocity(simulated_agent, agent_abstraction.position.to_tuple(), agent_abstraction.root_linear_velocity)

        pausable_interface(True)
        if position.arrive(to_state):
            return (True, position, distance_count, steps_count*self.TIME_STEP, from_state_velocity, to_state_velocity)
        else:
            return (False, position, distance_count, steps_count*self.TIME_STEP, from_state_velocity, to_state_velocity)
        
    def ghost_move(self):
        self.world

# TODO when two agents are close, difficult to find an non-collision path
# TODO best-reponse = path collision check very slow

# TODO 8.28 need to do multiagent inter paths collision checking. need to take other agent's path into consideration, then plan for current; if no best path found, replan for all