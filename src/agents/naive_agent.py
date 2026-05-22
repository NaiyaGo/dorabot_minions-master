# -*- coding: utf-8 -*-
# @date : 2018-07
# @author : xiaoyu.ge@dorabot.com
# @brief : Implementation of a agent (e.g., mars vheicles) using naive strategy
from agents.agent import *
from agents.agent_state_machine import AgentState
from geometry import Point, Vector, compute_direction
from global_planners.layered_astar_planner import LayeredAStar
from representation.gridmap_a import GridmapWithNeighbors
from random import *
import pprint
import time

class NaiveAgent(Agent):
    def act(self):
        pass
    def observe(self, observe):
        """ Observation
        Obtain updated map (if necessary)
        Obtain sensor data
        Obtain server commands (high-priority command such as emergent halt)
        """
        localization = self.perception_module.localization()
        self.position = Point(localization[0], localization[1])
        self.angle = localization[2]
        self.pose = localization
        self.ray_length_list = observe
        self.potential_collision = False
        self.server_command = None
        if len(self.sequence_of_poses)>0 and self.global_planner == LayeredAStar:
            self.replan = self.global_planner.observe_path(self.static_environment, self.position, self.perception_module, self.sequence_of_poses)
    def plan(self):
        """ Do high level state-transitions """
        func = self.state_machine.next_state(self)
        func(self, self.server)
        if self.has_destination():
            if self.state_machine.arrive_at_destination(self.position,self.destination_location)==False\
                or self.goal_changed == True or self.replan == True:
                """ Run global planner """
                goal_pose = self.task.destination_location
                if self.goal_changed == True or self.replan == True:
                    self.destination_location = self.task.destination_location
                    goal_pose = self.task.destination_location
                    """ If there is a global planner module """
                    if self.global_planner != None:
                        temp_sequence_of_poses = self.global_planner.compute_path(
                                self.position,
                                self.destination_location,
                                self.static_environment,
                                self.perception_module)
                        if temp_sequence_of_poses != None:
                            self.sequence_of_poses = temp_sequence_of_poses
                        else:
                            self.sequence_of_poses = deque([goal_pose])
                    else:
                        self.sequence_of_poses = deque([goal_pose])
                    self.goal_changed = False
                    """ Run local local_planner
                    Skip if there is no goal pose
                    """
                if len(self.sequence_of_poses) == 0:
                    # raise Exception(" No next pose while in mobile state ")
                    self.sequence_of_poses = deque([goal_pose])
                    return

                # Non-mobile states (QUEUING, LOADING, PREQUEUE) that have already
                # reached their slot should not drive the local planner — the slot
                # destination changed (goal_changed) but the agent is already there.
                if not self.state_machine.is_mobile_state(self.state) and \
                        self.state_machine.arrive_at_destination(self.position, self.destination_location):
                    self.stop()
                    return

                if self.position.distance(self.destination_location) < min(self.ray_length_list):
                    self.current_local_planner = self.local_planner[0]
                else:
                    index = randrange(0,2)
                    self.current_local_planner = self.local_planner[0]#[index]
                act = self.current_local_planner.compute_plan(
                            self.position,
                            self.linear_velocity,
                            self.static_environment,
                            self.perception_module,
                            self.sequence_of_poses)
                self.linear_velocity = act
                # if act[0] > self.cruise_speed:
                #     act[0] = self.cruise_speed
                # if act[1] > self.max_angular_velocity:
                #     act[1] = self.max_angular_velocity
                # elif act[1] < 0 and act[1] < -self.max_angular_velocity:
                #     act[1] = - self.max_angular_velocity
                # self.speed = act[0]
                # self.angular_velocity = act[1]
            else: # has destination, arrived current destination but still in CRUISE
                if self.state == AgentState.CRUISE:
                    self.go_internal_stations()
                else:
                    self.internal_stations = []
                    self.stop()
        else: # do not have destination
            self.stop()
        
    def go_internal_stations(self):
        if len(self.internal_stations) > 0:
            next_goal = self.internal_stations.pop(0)
            self.task.destination_location = next_goal
            self.destination_location = next_goal
            self.goal_changed = True