# -*- coding: utf-8 -*-
"""
@Copyright Dorabot Inc.
@date : 2019-08
@author: cenrong.dai@dorabot.com
@brief : local planner which follows the waypoint of the given global_planner_path strictly
"""
from geometry import Vector, compute_direction, Point
from local_planner import LocalPlanner
from math import sqrt
class DullPlanner(LocalPlanner):
    """

    Keyword arguments:
    position -- agent position
    velocity -- linear velocity of the agent
    sensor_observation -- use perception module to emulate sensor observation
    global_planner_path -- list of poses (way points) obtained from global planner
    Return:
    velcocity -- velocity command [only linear velocity returned in current version]
    local_path -- optional
    """
    def compute_plan(self, position, velocity, gridmap, sensor_observation, global_planner_path):
        local_path = []
        
        try:
            if position.arrive(global_planner_path[0]):
                global_planner_path.popleft()  # deque requires popleft(), not pop(0)
            goal_pose = global_planner_path[0]
        except (IndexError, AttributeError):
            goal_pose = self.agent.destination_location

        result_vel = Vector(velocity[0], velocity[1])
        force_goal = self.__goal_attraction(position, goal_pose)
        result_vel = result_vel + force_goal
        ratio = sqrt(result_vel.x**2 + result_vel.y **2)
        try:
            result_vel = result_vel.scale(self.agent.cruise_speed/ratio)
        except: # divide zero
            return (0.0, 0.0)
        return result_vel.to_tuple()
    def __goal_attraction(self, pos_current, loc_destination):
        dist = pos_current.distance(loc_destination)
        vec = compute_direction(pos_current, loc_destination).normalize()
        return vec.scale(100/(dist+0.01))
