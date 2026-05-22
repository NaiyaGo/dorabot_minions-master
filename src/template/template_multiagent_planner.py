# -*- coding: utf-8 -*-
from multiagent_global_planners.multiagent_planner import MultiAgentPlanner
from global_planners.global_planner import MapType
from geometry import Point
# agent abstracted detail
from server import AgentAbstraction
# sensor data structure
from agents.agent import Agent, SimulatedPerception, Box2DPerception
# agent workflow
from agents.naive_agent import NaiveAgent
# environment type data structure
from representation.continous_space import ContinuousSpace
from representation.gridmap_a import GridmapWithNeighbors

# Please do NOT change class name or script name
class TemplateMultiagent(MultiAgentPlanner):
    """ Choose an environment type that your algorithm is based on
    GRID: If your planner works in grid-based environment (e.g. A*)
    CONTINOUS: if you planner works in continous environment (e.g. RRT) 
    """
    MAP = MapType.GRID # or MapType.CONTINUOUS
    
    """ This type of planner computes collabrative paths for all agents under its control.
    It is triggered when one of the agents under control has goal_changed;
    The details of the agents under the planner's control should be accessed by a query to the server.
    The output of the function MUST be a <agent_id, list of Point(x,y)> dictionary. Then the server will send the new solution to each agent accordingly.
    """
    def compute_path(self):
        # agents_abstraction_dict: <agent_id, agent_abstraction instance> dictionary
        agents_abstraction_dict = self.server.collect_agents_info(self.agents) # request the details of the agents which are under the planner's control
        

        # ------------------------------- #
        #       Fill Your Code Here       #
        # ------------------------------- #



        # example
        # agents_solution_path_dict: <agent_id, list of Point(x,y)> dictionary
        agents_solution_path_dict = {agent_info.id:[agent_info.destination_location] for agent_info in agents_abstraction_dict.values()}
        return agents_solution_path_dict