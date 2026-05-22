# -*- coding: utf-8 -*-
"""
@Copyright Dorabot Inc.
@date : 2018-07
@author : xiaoyu.ge@dorabot.com
@brief : Quick Impelmentation of agent state machine for python environment
States and State transitions are hardcoded for a specific logic
@TODO Gary: Not urgent, Load configuration from json.
"""
from enum import Enum
from task_managers.task_manager import TaskType
class AgentState(Enum):
    IDLE = 0 # stopped and no task assigned
    LOADING = 1 # do not distinguish loading/unloading state
    QUEUING = 2
    HALT = 3 # stopped during exeucting a task
    CRUISE = 4
    PREQUEUE= 6

class AgentStateMachine:
    """ A sample implementation of agent state machine """
    PREQUEUE_TIMEOUT = 300  # steps (~5 seconds at 60Hz) before backing out of PREQUEUE

    def __init__(self, agent):
        self.states = [
                    AgentState.IDLE,
                    AgentState.LOADING,
                    AgentState.QUEUING,
                    AgentState.HALT,
                    AgentState.CRUISE,
                    AgentState.PREQUEUE]
        agent.state = AgentState.IDLE
        self.mobile_states = [
                AgentState.CRUISE]
        self.prequeue_counter = 0  # counts steps spent in PREQUEUE
    def next_state(self, agent, server_command=None):
        """ State transition function
        delta: Q X Sigma -> Q
        Sigma = {perception, agent_information, server_command}

        Return:
            a function type and a function to be called in q \in Q
        """
        if self.is_mobile_state(agent.state):
            if agent.task.type == TaskType.GO_TO_LOADING_PORT or agent.task.type == TaskType.GO_TO_UNLOADING_PORT:
                if agent.task.port.in_control_range(agent.position):
                    return approaching
            """ As long as it is in a mobile state, it must has a next goal destination """
            if self.arrive_at_destination(agent.position, agent.destination_location):
                agent.stop()
                return go_for_next_pose
        else:
            if agent.state == AgentState.PREQUEUE:
                self.prequeue_counter += 1
                if self.prequeue_counter >= AgentStateMachine.PREQUEUE_TIMEOUT:
                    # Port is persistently full; back out to CRUISE so the agent
                    # can be re-routed by the server or CBS rather than waiting forever.
                    self.prequeue_counter = 0
                    agent.state = AgentState.CRUISE
                    agent.replan = True
                    return stay_in_current_state
                return approaching
            if agent.state == AgentState.QUEUING:
                if agent.task.port.in_operation_zone(agent.position):
                    return start_loading
                else:
                    return move_if_next_slot_available
            if agent.state == AgentState.LOADING:
                return operate
            if agent.state == AgentState.IDLE:
                return go_for_next_loading_task
        return stay_in_current_state
    def is_mobile_state(self, state):
        return state in self.mobile_states
    def arrive_at_destination(self, position, destination):
        if destination == None:
            return True
        if position.distance(destination) < 2e-1:
            return True
        return False

""" Functions
    Helper function an agent should call when tranisting to a new state
    Function will peform certain jobs and complete state transition in the end
"""
""" ================== Server-required (remote) functions ================== """
def go_for_next_loading_task(agent, server):
    task = server.get_loading_task(agent)
    agent.assign_task(task)
    agent.destination_location = task.destination_location
    agent.state = AgentState.CRUISE
    server.update_data(agent)
    agent.goal_changed = True


def approaching(agent, server):
    """ Port could be one type of server that is in reminiscent to a control tower in an airport
    We should further decompose the state
    """
    # Already queuing — just update slot if it changed, don't re-enter
    if agent.state == AgentState.QUEUING:
        move_if_next_slot_available(agent, server)
        return

    allowed, num_agents_ahead = agent.task.port.request_enter_permit()
    if allowed:
        agent.task.port.confirm_enter(agent)
        agent.state = AgentState.QUEUING
        new_slot = agent.task.port.get_slot(agent)
        if new_slot != agent.destination_location:
            agent.destination_location = new_slot
            agent.task.destination_location = new_slot
            agent.goal_changed = True
        server.update_data(agent)
    else:
        agent.state = AgentState.PREQUEUE
        server.update_data(agent)

def operate(agent, server):
    operation_is_done, item = agent.task.port.operate()
    if operation_is_done:
        agent.task.port.confirm_exit()
        if agent.task.type == TaskType.GO_TO_LOADING_PORT:
            agent.assign_task(server.get_unloading_task(agent, item))
        elif agent.task.type == TaskType.GO_TO_UNLOADING_PORT:
            agent.assign_task(server.get_loading_task(agent))
        else:
            raise Exception(" Unclassifed operation types ")
        agent.state = AgentState.CRUISE
        server.update_data(agent)
        agent.goal_changed = True

def move_if_next_slot_available(agent, server):
    """ The agent is supposed to query the port that could be either deployed at the
    central server or a local server near the port
    """
    slot = agent.task.port.get_slot(agent)
    if slot != agent.task.destination_location:
        agent.goal_changed = True
        agent.task.destination_location = slot

""" ================= Agent-side functions =================================="""
def go_for_next_pose(agent, server=None):
    if len(agent.sequence_of_poses) > 0:
        pass
    else:
        # agent.state = AgentState.HALT
        agent.stop()
def start_loading(agent, server=None):
    agent.state = AgentState.LOADING
    server.update_data(agent)

def stay_in_current_state(agent, server=None):
    return
