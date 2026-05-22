# -*- coding: utf-8 -*-
"""
@Copyright Dorabot Inc.
@date : 2018-10
@author: {tian.xiao, xiaoyu.ge}@dorabot.com
@brief : Implementation of Unloading ports 
"""
from setup_environment.port import Port

#UnLoading ports created by their location, dimension and identifier(ID no.)
#If not set identifier, the default value would be counter
class UnloadingPort(Port):
    counter = 0
    def __init__(self, location, dimension, port_process_time = 2, identifier = counter):
        super(UnloadingPort, self).__init__(
            location, dimension, port_process_time, identifier = UnloadingPort.counter)
        self.id = UnloadingPort.counter
        UnloadingPort.counter = UnloadingPort.counter + 1

    def operate(self):
        return super(UnloadingPort, self).operate(), None