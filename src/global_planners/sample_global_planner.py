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
from geometry import *
from representation.float_to_grid import *

class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]

    def get_pair(self):
        '''return a (priority, item) tuple; while get only return the item'''
        return heapq.heappop(self.elements)


class SimpleAStar(GlobalPlanner):
    MAP = MapType.GRID
    def compute_path(self, position, goal_pose, old_gridmap, sensor_observation):
        resolution = old_gridmap.resolution
        gridmap = GridmapWithNeighbors(old_gridmap)
        start= agent_to_gridmap(position, gridmap, resolution,goal_pose, centroid=False)
        goal= float_to_grid((goal_pose.x, goal_pose.y), resolution, centroid=False)
        came_from = self.a_star_search(gridmap, start, goal)
        current = goal
        path = []
        #Hard code resolution of gridmap here for test
        while current != start:
            tmp=grid_to_float(current, resolution, centroid=False)
            #path.append(Point(self.grids_to_coordinates(resolution, current, centroid=False)))
            path.append(Point(tmp[0],tmp[1]))   
            try:
                current = came_from[current]
            except:
                return None 
            """
            if no path got from A* just return none and use old way points
            """
        path.reverse() # optional
        return path

    def heuristic(self, a, b):
        (x1, y1) = a
        (x2, y2) = b
        return abs(x1 - x2) + abs(y1 - y2)

    def a_star_search(self, graph, start, goal):
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
                    new_cost = cost_so_far[current] + graph.cost(current, next)
                    if next not in cost_so_far or new_cost < cost_so_far[next]:
                        cost_so_far[next] = new_cost
                        priority = new_cost + self.heuristic(goal, next)
                        frontier.put(next, priority)
                        came_from[next] = current
            except:
                return None
        return came_from
