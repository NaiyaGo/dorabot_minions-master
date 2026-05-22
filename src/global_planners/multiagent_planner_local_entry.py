# -*- coding: utf-8 -*-
"""
@author by: cenrong.dai@dorabot.com
@brief: A global planner module installed locally in each agent. It provides functions talking
    to the multiagent global planner installed in server side, sending the compute_path request
    and receive back the solution specifically for the current agent. The multiagent global planner
    in server side computes paths for all agents at the same time for coordination.
"""

from global_planners.global_planner import GlobalPlanner
class MultiAgentPlannerLocalEntry(GlobalPlanner): # installed locally in each agent
    def compute_path(self, position, goal_pose, environment, sensor_observation):
        # each agent talk to the shared server global planner via this locally installed entry
        return self.agent.server.request_multiagent_global_planner_compute_path(self.agent)