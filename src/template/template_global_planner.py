# -*- coding: utf-8 -*-
from global_planners.global_planner import GlobalPlanner, MapType
from geometry import Point
# sensor data structure
from agents.agent import Agent, SimulatedPerception, Box2DPerception
# agent workflow
from agents.naive_agent import NaiveAgent
# environment type data structure
from representation.continous_space import ContinuousSpace
from representation.gridmap_a import GridmapWithNeighbors


# Please do NOT change class name or script name
class TemplateGlobal(GlobalPlanner):
    """ Choose an environment type that your algorithm is based on
    GRID: If your planner works in grid-based environment (e.g. A*)
    CONTINOUS: if you planner works in continous environment (e.g. RRT) 
    """
    MAP = MapType.GRID # or MapType.CONTINUOUS
    
    """ This type of planner computes a path for the agent which has the planner installed, ignoring all the other agents positions and goals.
    It is triggered when the agent in the charge of the planner has goal_changed;
    The output of the function MUST be a list of Point(x,y).

    Keyword arguments:
        position: the agent's current position <Point type>
        goal_pose: the agent's goal position <Point type>
        environment: the environment in which the planner works on
        sensor_observation: the sensor data obtained by the agent's sensor
    Return:
        list of Point(x,y)
    """
    def compute_path(self, position, goal_pose, environment, sensor_observation):
                

        # ------------------------------- #
        #       Fill Your Code Here       #
        # ------------------------------- #


        # example        
        path = [goal_pose]
        return path