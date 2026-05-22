# -*- coding: utf-8 -*-
"""
@Copyright Dorabot Inc.
@date : 2018-10
@author: {tian.xiao, xiaoyu.ge}@dorabot.com
@brief : Implementation of obstacles
"""

""" Create Obstacles class
    all static physic bodies are Obstacles
""" 
from geometry import *
from math import sqrt

class Obstacles(object):
    counter = 0
    def __init__(self, location, dimension, identifier = None):
        self.location = location
        self.center = comptue_center_point(location, dimension)
        self.dimension = dimension
        self.shape = None
        self.type = 'obstacle'
        if identifier == None:
            identifier = Obstacles.counter
            Obstacles.counter += 1
        self.identifier = identifier

    def get_radius(self):
        '''return the max radius'''
        return pow(self.dimension[0]**2+self.dimension[1]**2,0.5)/2
        
    def is_clicked(self, pos):
        return self.location.x <= pos[0] <= self.location.x + self.dimension[0] and self.location.y <= pos[1] <= self.location.y + self.dimension[1]

    def is_too_close(self, pos, agent_half_dimension_2d):
        '''input is tuple; return True if pos falls within a distance around the obstacle'''
        return self.location.x-agent_half_dimension_2d[0] <= pos[0] <= self.location.x+self.dimension[0]+agent_half_dimension_2d[0] and\
             self.location.y-agent_half_dimension_2d[1] <= pos[1] <= self.location.y+self.dimension[1]+agent_half_dimension_2d[1]

    def check_obstacle_edge_collision(self, from_point, to_point, radius = 0):
        '''return True if the min distance between the edge (from_point, to_point) and the obstacle *** <= *** radius'''
        obstacle_top_left_point = self.location
        obstacle_top_right_point = Point(self.location.x+self.dimension[0],self.location.y)
        obstacle_bottom_left_point = Point(self.location.x, self.location.y+self.dimension[1])
        obstacle_bottom_right_point = Point(self.location.x+self.dimension[0], self.location.y+self.dimension[1])
        # check if the line has hit any of the rectangle's sides
        if edge_edge_collision(from_point, to_point, obstacle_top_right_point, obstacle_top_left_point, radius**2) or\
            edge_edge_collision(from_point, to_point, obstacle_top_left_point, obstacle_bottom_left_point, radius**2) or\
                edge_edge_collision(from_point, to_point, obstacle_bottom_left_point, obstacle_bottom_right_point, radius**2) or\
                    edge_edge_collision(from_point, to_point, obstacle_bottom_right_point, obstacle_top_right_point, radius**2):
                    return True
        return False
