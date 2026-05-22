# -*- coding: utf-8 -*-
"""
@Copyright Dorabot Inc.
@date : 2018-10
@author: tian.xiao@dorabot.com
@brief : a sample implementation of an Astart algorithm
"""
import heapq
from representation.gridmap import GridMap
from representation.gridmap_a import GridmapWithNeighbors
from global_planners.global_planner import GlobalPlanner, MapType
from global_planners.sample_global_planner import PriorityQueue
from geometry import *
from representation.float_to_grid import *
from math import ceil, floor, pi


class LayeredAStar(GlobalPlanner):
    MAP = MapType.GRID
    def compute_path(self, position, goal_pose, old_gridmap, sensor_observation):
        resolution = old_gridmap.resolution
        gridmap = GridmapWithNeighbors(old_gridmap)
        start= agent_to_gridmap(position, gridmap, resolution,goal_pose, centroid=False)
        goal= float_to_grid((goal_pose.x, goal_pose.y), resolution, centroid=False)
        # update dynamic layer
        dynamic_layer = self.get_dynamic_layer(gridmap, sensor_observation)
        came_from = self.layered_a_star_search(gridmap, start, goal, dynamic_layer)
        current = goal
        path = []
        #Hard code resolution of gridmap here for test
        while current != start:
            # print(current, "goal ", self.coordinates_to_grids(resolution, position))
            tmp=grid_to_float(current, resolution, centroid=False)
            #path.append(Point(self.grids_to_coordinates(resolution, current, centroid=False)))
            path.append(Point(tmp[0],tmp[1]))   
            try:
                current = came_from[current]
            except:
                # print(position, goal_pose)
                return None 
            """
            if no path got from A* just return none and use old way points
            """
            # try:
            #   current = came_from[current]
            # except:
            #   print(position, goal_pose)
            #   return None              
        #path.append(position)# optional
        path.reverse() # optional
        # print("done")
        return path

    def heuristic(self, a, b):
        (x1, y1) = a
        (x2, y2) = b
        return abs(x1 - x2) + abs(y1 - y2)

    def layered_a_star_search(self, graph, start, goal, dynamic_layer):
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            current = frontier.get()

            if current == goal:
                break
            try:
                for next in graph.neighbors(current):
                    try:
                        new_cost = cost_so_far[current] + graph.cost(current, next) + dynamic_layer[current][next]
                        #print "add dynamic layer cost", dynamic_layer[current][next]
                    except:
                        new_cost = cost_so_far[current] + graph.cost(current, next)
                    if next not in cost_so_far or new_cost < cost_so_far[next]:
                        cost_so_far[next] = new_cost
                        priority = new_cost + self.heuristic(goal, next)
                        frontier.put(next, priority)
                        came_from[next] = current
            except:
                return None
        # self.__debug_print_path(came_from)
        return came_from

    def __debug_print_path(self, paths):
        for key in paths:
            print(key, paths[key])
            
    def get_dynamic_layer(self, gridmap, sensor_observation, inflation = 4.0):
        dynamic_layer = {}
        for agent in sensor_observation.other_agents_state_in_range_of(4.0, pi/2):
            position = agent.position
            #rad = agent.shape.get_radius()
            u = agent_to_gridmap(position, gridmap)
            for v in gridmap.neighbors(u):
                self.add_inflation(dynamic_layer, u, v, inflation)
                self.add_inflation(dynamic_layer, v, u, inflation)
                for w in gridmap.neighbors(v):
                    if w != u:
                        self.add_inflation(dynamic_layer, v, w, inflation/2)
                        self.add_inflation(dynamic_layer, w, v, inflation/2)
        return dynamic_layer
    
    def add_inflation(self, dictionary, u, v, inflation = 4.0):
        try:
            dictionary[u][v] += inflation
        except: 
            dictionary[u] = {v:inflation}
        return
        
        
    def observe_path(self, gridmap, current_position, sensor_observation, sequence_of_poses, threshold = 1):
        """
        Observe current global_plan_path. If it is obstructed by other agents, may need to replan.
        sequence_of_poses: should be the waypoints that have not been reached and within the scope of sensor
        If threshold == 1, replan when an obstruction is detected
        If threshold > 1, replan when number of detected obstructions >= threshold
        """
        flag_replan = False
        waypoints_in_range = []
        remaining_distance = current_position.distance(sequence_of_poses[-1])
        
        detected_obstacles = sensor_observation.other_agents_state_in_range_of(4.0, pi/2)
        
        if len(detected_obstacles) > 4:
            return True
        for a in sequence_of_poses:
            if current_position.distance(a) < 4.0 and sequence_of_poses[-1].distance(a) < remaining_distance:
                waypoints_in_range.append(a)        
        for agent in detected_obstacles:
            position = agent.position
            for point in waypoints_in_range:
                if point.distance(position) < 1.0:
                    flag_replan = True
                    return flag_replan
        return flag_replan
        