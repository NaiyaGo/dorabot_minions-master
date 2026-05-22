# -*- coding: utf-8 -*-
"""
@Copyright Dorabot Inc.
@date : 2018-10
@author: {tian.xiao, xiaoyu.ge}@dorabot.com
@brief : Implementation of Loading ports 
"""
import random
from setup_environment.port import Port
from setup_environment.unloading_port import UnloadingPort
class Item:
    def __init__(self, source_port_id, destination_port_id):
        self.source_port_id = source_port_id
        self.destination_port_id = destination_port_id

"""Loading ports created by their location, dimension and identifier(ID no.)
If not set identifier, the default value would be counter
"""
class LoadingPort(Port):
    counter = 0

    def __init__(self, location, dimension, port_process_time = 2, identifier = counter):
        super(LoadingPort, self).__init__(
            location, dimension, port_process_time, identifier = LoadingPort.counter)
        self.id = LoadingPort.counter
        LoadingPort.counter = LoadingPort.counter + 1

    # def get_next_item(self, next_destination_port_id):
    #     item = Item(
    #         source_port_id=self.identifier,
    #         destination_port_id=next_destination_port_id)
    #     self.items.append(item)
    #     return item

    def get_random_item(self, num_unloading_ports ):
        self.num_unloading_ports = num_unloading_ports
        item = Item(source_port_id=self.identifier, destination_port_id=random.randrange(0, self.num_unloading_ports))
        # num_remaining_items = 999 # set to a constant when simulating infinite sequence
        self.items.append(item)
        
        return item

    def operate(self):
        if super(LoadingPort, self).operate()==True and len(self.items)>0:
            self.items.append(Item(source_port_id=self.identifier, destination_port_id=random.randrange(0, self.num_unloading_ports)))
            return True, self.items.pop(0)#self.get_random_item(UnloadingPort.counter)
        else:
            return False, None