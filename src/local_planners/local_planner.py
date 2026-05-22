# -*- coding: utf-8 -*-
"""
@Copyright Dorabot Inc.
@date : 2018-10
@author: {tian.xiao, xiaoyu.ge}@dorabot.com
@brief : local planner template
"""
class LocalPlanner(object):
    def __init__(self, agent):
        self.agent = agent
    def compute_plan(self, position, velocity, gridmap, sensor_observation, global_planner_path):
	       raise NotImplementedError
