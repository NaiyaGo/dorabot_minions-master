# -*- coding: utf-8 -*-
from enum import Enum
class MapType(Enum):
    GRID = 0 # collision based on grid (A* family)
    CONTINUOUS = 1 # collision based on continuous space (RRT family)

class GlobalPlanner(object):
    MAP = MapType.GRID
    def __init__(self, agent):
        self.agent = agent
        self.map_type = MapType.GRID
    def compute_path(self, position, goal_pose, gridmap, sensor_observation):
         raise NotImplementedError
