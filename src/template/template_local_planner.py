# -*- coding: utf-8 -*-
from local_planners.local_planner import LocalPlanner
from geometry import Point, Vector, compute_direction
# sensor data structure
from agents.agent import Agent, SimulatedPerception, Box2DPerception
# agent workflow
from agents.naive_agent import NaiveAgent
# environment type data structure
from representation.continous_space import ContinuousSpace
from representation.gridmap_a import GridmapWithNeighbors

class TemplateLocal(LocalPlanner):
    """ This type of planner computes the linear velocity vector in which the agent will move during the next simulated step;
    The output MUST be a (float, float) tuple, each float stands for the speed in (x, y) direction respectively.

    Keyword arguments:
        position: the agent's current position <Point type>
        velocity: the agent's last velocity <tuple type: (x-float, y-float)>
        environment: the environment in which the agent works on
        sensor_observation: the sensor data obtained by the agent's sensor
        global_planner_path: list of poses (way points <Point type>) obtained from global planner
    Return:
        A (x-float, y-float) tuple
    """
    def compute_plan(self, position, velocity, environment, sensor_observation, global_planner_path):
                

        # ------------------------------- #
        #       Fill Your Code Here       #
        # ------------------------------- #


        # example        
        try:
            # if the agent arrives the current point in global path, remove it and go for the next; else, go towards the current
            if position.arrive(global_planner_path[0]): 
                global_planner_path.pop(0)
            goal_pose = global_planner_path[0]
        except: # if no further way point in global_path, move directly towards destination location
            goal_pose = self.agent.destination_location

        speed = compute_direction(position, goal_pose) # return a unit vector
        return (speed[0], speed[1])

