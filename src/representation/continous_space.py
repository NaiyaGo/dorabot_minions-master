# -*- coding: utf-8 -*-
# @Copyright Dorabot Inc.
# @date : 2019-08
# @author: cenrong.dai@dorabot.com
# @brief : continuous space for RRT-family

from geometry import *

class ContinuousSpace(object):
    """ inflate the obstacles in environment with half of the agent's dimension;
    return a freespace in which every point is passable;
    functions for collision check are attached
    """
    SQRT_2 = math.sqrt(2)
    def __init__(self, static_obstacles_list, width_in_meters, height_in_meters, agent_dimension, top_left_corner=(0, 0)):
        self.static_obstacles_list = static_obstacles_list
        self.width_in_meters = width_in_meters
        self.height_in_meters = height_in_meters
        self.agent_dimension = agent_dimension
        self.top_left_corner = top_left_corner

    def is_in_free_space(self, pos, dimension_2d_tuple = None):
        """input (x, y) tuple; return True if point not falls into any obstacle or within a distance of half of agent dimension"""
        if dimension_2d_tuple == None:
            dimension_2d_tuple = (self.agent_dimension/2.0, self.agent_dimension/2.0)
        for obstacle in self.static_obstacles_list:
            if not self.in_bounds_in_meters(pos) or obstacle.is_too_close(pos, dimension_2d_tuple):
                return False
        return True

    def in_bounds_in_meters(self, pos, dimension_2d_tuple = None):
        """pos: (x, y) tuple; return True if point falls within meters; different from in_bounds of gridmap Class which test based on the grid"""
        if dimension_2d_tuple == None:
            dimension_2d_tuple = (self.agent_dimension/2.0, self.agent_dimension/2.0)
        return 0.0+dimension_2d_tuple[0] <= pos[0] <= self.width_in_meters-dimension_2d_tuple[0] and 0.0+dimension_2d_tuple[1] <= pos[1] <= self.height_in_meters-dimension_2d_tuple[1]

    # TODO need to change straight collision to related to local planner
    def subpath_obstacles_collision(self, from_state, to_state, dimension_2d_tuple = None):
        """return True if the subpath (from_state, to_state) collide with any static obstacle;
        a radius parameter can be non-zero to account for half of agent's dimension"""
        if dimension_2d_tuple == None:
            dimension_2d_tuple = (self.agent_dimension/2.0, self.agent_dimension/2.0)
        radius = max(list(dimension_2d_tuple))
        for obstacle in self.static_obstacles_list:
            if obstacle.check_obstacle_edge_collision(from_state, to_state, radius):
                return True
        return False

    def subpath_subpath_collision(self, from_state1, to_state1, from_state2, to_state2, dimension_2d_tuple = None):
        if dimension_2d_tuple == None:
            dimension_2d_tuple = (self.agent_dimension/2.0, self.agent_dimension/2.0)
        square_radius = dimension_2d_tuple[0]**2+dimension_2d_tuple[1]**2 # largest measurement
        if edge_edge_collision(from_state1, to_state1, from_state2, to_state2, square_radius):
            return True
        return False