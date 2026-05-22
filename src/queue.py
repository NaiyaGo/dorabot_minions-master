# -*- coding: utf-8 -*-
"""
@date : 2018-07
@author : {xiaoyu.ge, xiao.tian}@dorabot.com
@brief : Vechile's queuing system
"""
from pygame import Rect
from geometry import Point
from collections import deque
class SimpleQueue:
    """ This class implements a simple queuing system with a single lane
    Attributes:
    agent_slot_map -- a hash map: agent_id ---> assigned_slot
    """
    def __init__(self, region):
        # Create rectangle region
        self.region = region
        # Discretise the space into grids
        self.length = 0
        self.agents= []
        self.agent_slot_map = {}
        self.__setup_slots()

    def num_agents(self):
        return len(self.agents)

    def enter(self,agent):

        self.length = self.length + 1
        self.agents.append(agent)
        self.__arrange_slots()
        return self.length-1

    def exits(self):
        self.length = self.length - 1
        self.agents.pop(0)
        self.__arrange_slots()
        return

    def get_slot(self, agent):
        """ All Agents will be sorted each time an agent enters or exists from the queue
        """
        return self.agent_slot_map[agent.id]
    
    def __setup_slots(self):

        slot_side_length = 1
        num_of_slots = int(self.region.width/slot_side_length) + 1
        self.min_dist_to_port = 1
        y = self.region.y + self.region.height/2
        x = self.region.x 
        self.slots = [Point(x + i * slot_side_length, y) for i in range(num_of_slots)]
        
    def __arrange_slots(self, slots_reverse = False):
        self.agents.sort(reverse = slots_reverse, key = lambda agent: agent.position.distance(self.slots[0]))
        # try:
        #     self.self.agent[2::] = sorted(self.agent[2::],reverse = slots_reverse, key = lambda agent: agent.position.distance(self.slots[0]))
        # except:
        #     self.agents.sort(reverse = slots_reverse, key = lambda agent: agent.position.distance(self.slots[0]))
        assert len(self.agents) <= len(self.slots)
        for i, agent in enumerate(self.agents):
            self.agent_slot_map[agent.id] = self.slots[i]
