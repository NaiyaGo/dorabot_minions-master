# -*- coding: utf-8 -*-
"""
@Copyright Dorabot Inc.
@date : 2018-07
@author : xiaoyu.ge@dorabot.com
@brief : task manager is responsible for assigning tasks to agents. It knows
the complete state of agents and ports A task is defined by a tuple <s,d> where
s refers to the point location of the source and d refers to the point location
of the destination.
"""
from enum import Enum
class TaskType(Enum):
    WAITING_FOR_ORDER_TASK = 0
    GO_TO_LOADING_PORT = 1 # stopped and no task assigned
    GO_TO_UNLOADING_PORT = 2 # stopped and no task assigned

class Task(object):
    def __init__(self, source_location, destination_location):
        self.source_location = source_location
        self.destination_location = destination_location
class WaitingForOrderTask(Task):
    def __init__(self):
        self.type = TaskType.WAITING_FOR_ORDER_TASK
class LoadingTask(Task):
    def __init__(self, source_location, destination_location, port):
        super(LoadingTask, self).__init__(source_location, destination_location)
        self.port = port
        self.type = TaskType.GO_TO_LOADING_PORT
class UnLoadingTask(Task):
    def __init__(self, source_location, destination_location, port):
        super(UnLoadingTask, self).__init__(source_location, destination_location)
        self.port = port
        self.type = TaskType.GO_TO_UNLOADING_PORT
class TaskMananger(object):
    def __init__(self, environment, agents):
        self.agents = agents
        self.loading_ports = environment.loading_ports.values()
        self.unloading_ports = environment.unloading_ports.values()

    # perform task allocation for IDLE agents
    def next_loading_task(self, agent):
        raise NotImplementedError
