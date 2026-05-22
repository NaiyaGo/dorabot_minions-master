# -*- coding: utf-8 -*-
"""
@Copyright Dorabot Inc.
@date : 2018-11
@author: tian.xiao@dorabot.com
@brief : Implementation of walls of workspace 
"""

from setup_environment.obstacles import Obstacles
from shape import RectangleWallCreater
class Wall(Obstacles):
    counter = 0 
    def __init__(self, location, dimension):
        super(Wall, self).__init__(location, dimension, identifier = Wall.counter )
        self.type = 'wall'
        self.shape = RectangleWallCreater(self.location, self.dimension)
        self.id = Wall.counter
        Wall.counter += 1

    def debug_walls(self):
        print self.location