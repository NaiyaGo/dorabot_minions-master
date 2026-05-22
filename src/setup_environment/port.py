# -*- coding: utf-8 -*-
"""
@Copyright Dorabot Inc.
@date : 2018-10
@author: {tian.xiao, xiaoyu.ge}@dorabot.com
@brief : Implementation of ports
"""
from setup_environment.obstacles import Obstacles
import json

""" Ports are created by their location, dimension and identifier.
    The default identifier should be the counter from 0.
"""
class Port(Obstacles):
    simulator_step = 0
    """Ports location is the Top Left corner of the port
       Ports dimension should be a tuple with (width,height)
    """
    def __init__(self, location, dimension, port_process_time, identifier):
        super(Port, self).__init__(location, dimension, identifier)
        self.operation_time_in_secs = port_process_time
        self.type = 'port'
        self.items = []
        self.control_range = 2
        self.operation_range = 2
        """ Queuing Related """
        self.queue = None
        self.has_queue_area = False
        """ Operation Related """
        self.operation_start_time = None
        self.in_operation = False
        self.operation_zone = None
        """ Logging Related """
        self.task_count = 0
        self.simulator_time_step = 1.0/60
        """future using""" 
        self.entry_point = location
        self.__get_time_step()

    def __get_time_step(self):
        try:
            with open('config.json') as file:
                config_data = json.load(file)
            self.simulator_time_step = 1.0/config_data['simulator']['steps_per_sec']
        except:
            pass

    def enforce_queuing_mechanism(self, queuing_mechanism):
        self.queue = queuing_mechanism
        self.operation_zone = self.queue.slots[0]
        self.entry_point = self.queue.slots[-1]

    # ================ External Communication =================================
    def query_num_existing_agents(self):
        return self.queue.num_agents()

    def request_enter_permit(self):
        # An agent entered into the region
      if len(self.queue.agents) < len(self.queue.slots):
          return True, self.queue.num_agents()
      else:
          return False, self.queue.num_agents()

    def confirm_enter(self, agent):
        self.queue.enter(agent)

    def confirm_exit(self):
        self.queue.exits()
    def get_slot(self, agent):
        return self.queue.get_slot(agent)
    def operate(self):
        is_done = False
        if self.in_operation:
            if (Port.simulator_step  * self.simulator_time_step) - self.operation_start_time > self.operation_time_in_secs:
                self.in_operation = False
                is_done = True
                self.task_count += 1
        else:
            self.in_operation = True
            self.operation_start_time = Port.simulator_step  * self.simulator_time_step
        return is_done

    # ================ Local Computation =================================
    def in_control_range(self, agent_position):
        if self.queue:
            dist = min(
                [slot.distance(agent_position) for slot in self.queue.slots])
            #dist=abs(self.center.y-agent_position.y)+min([abs(slot.x-agent_position.x) for slot in self.queue.slots])
        else:
            dist = self.location.distance(agent_position)
        return dist < self.control_range

    def in_operation_zone(self, agent_position):
        return self.operation_zone.distance(agent_position) < 2e-1


    def is_clicked(self, pos):
        return   self.location.x <= pos[0] <= self.location.x + self.dimension[0] and self.location.y <= pos[1] <= self.location.y + self.dimension[1]
