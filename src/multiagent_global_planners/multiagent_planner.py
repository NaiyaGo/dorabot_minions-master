# -*- coding: utf-8 -*-
"""
@modified by: cenrong.dai@dorabot.com
@brief: MultiAgentPlanner plans paths for all agents with coordination.
"""
from global_planners.global_planner import GlobalPlanner

class MultiAgentPlanner(GlobalPlanner):
    def __init__(self, server, environment, world, TIME_STEP):
        self.server = server # the planner need to talk to server to retrieve positions and goals of agents under its control
        self.environment_map = environment # for discrete planner, gridmap is passed in; for continuous planner, continuous_space
        self.world = world # a copy of the physical b2World, for RL planner
        self.TIME_STEP = TIME_STEP # when use local planner to simulate steer, the observation interval should stay constant with simulator
        self.agents = {} # record the agents under its control
    
    def add_agent_under_control(self, agent):
        if agent.global_planner and MultiAgentPlanner.__subclasscheck__(type(agent.global_planner)): # not None and is a multi agent controller
            agent.global_planner.remove_agent_under_control(agent)
        self.agents[agent.id] = agent
    
    def remove_agent_under_control(self, agent):
        if agent.id in self.agents:
            del self.agents[agent.id]
        agent.global_planner = None
        if len(self.agents) == 0: # empty multiagent global planner, kill itself
            self.server.remove_multiagent_local_planner(self)
    
    def compute_path(self):
        '''turn to server to compute paths for all agents'''
        raise NotImplementedError

