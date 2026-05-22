# -*- coding: utf-8 -*-
# @author : (xiaoyu.ge, tian.xiao)@dorabot.com
# @brief : partly adopted from the author's github repository: https://github.com/fantastdd/physical-reasoning-2d/blob/master/scenario_generator.py
import pygame
from Box2D import *
from pygame import Rect
from shape import Rectangle
from geometry import Vector, Point
# from framework import * # from local framework.py file
from setup_environment.environment import Environment
from setup_environment.port import Port
from server import Server
from agents.agent_state_machine import AgentState
from agents.naive_agent import NaiveAgent
from local_planners.local_planner import LocalPlanner
from local_planners.dull_local_planner import DullPlanner
from local_planners.virtual_force_planner import VirtualForcePlanner
from local_planners.rvo_planner import RVOPlanner
from local_planners.DD_planner import DDPlanner
from local_planners.hrvo_planner import HRVOPlanner
from local_planners.flc_local_planner import FLCPlanner
from local_planners.VFP import VFP
from global_planners.global_planner import MapType
from global_planners.sample_global_planner import SimpleAStar
from global_planners.layered_astar_planner import LayeredAStar
from global_planners.rrtstar_planner import RRTStar
from global_planners.multiagent_planner_local_entry import MultiAgentPlannerLocalEntry
from multiagent_global_planners.multiagent_planner import MultiAgentPlanner
from multiagent_global_planners.marrtstar_planner import MARRTStar
from multiagent_global_planners.inash_planner import INashRRT
from multiagent_global_planners.cbs_planner import CBSPlanner
from multiagent_global_planners.ecbs_planner import ECBSPlanner
from multiagent_global_planners.ecbsnew_planner import ECBSNewPlanner
from representation.gridmap_a import GridmapWithNeighbors
from visualisation import Visualisation
from math import *
from enum import Enum
from agents.sensor import Sensor, SensorContactListener, SensorRayCast
import sys, os
import time
import json
import random
import numpy as np
import matplotlib.pyplot as plt
import re
import Queue
from interaction_handler import *

"""entity category of different fixtures in simulator"""
class EntityCategory(Enum):
    """Add new physical body entity here
       The logical of contacts check is 
       Collide =
          (A.maskBits & B.categoryBits) != 0 &&
          (A.categoryBits & B.maskBits) != 0
    """
    wall = 1
    obstacle = 1
    port = 2
    sensor = 4
    agent = 4


"""Multi-Agents Simulator
"""
class Simulator(b2ContactListener):
    step_counter = 0
    sensor_history_length = 3
    def __init__(self, cmd_args):
        super(Simulator, self).__init__()
        # whether run headless
        self.simulation_times = cmd_args.time
        self.TIME_STEP = 1.0/60
        self.server = None
        self.agents = []
        self.loading_ports = []
        self.unloading_ports = []
        """store physical bodies of agents in the simulation"""
        self.b2_objects =  {}
        """store physical bodies of ports and obstacles"""
        self.b2_static_objects = []
        # self.clock = pygame.time.Clock()
        self.timespan = 0
        self.time = 0
        self.task_count = 0
        self.start_time = 0
        self.heatmap_data = []
        self.agent_global_planner = ''
        self.agent_local_planner = ''
        '''disable gravity, set contact listener for sensor using'''
        self.world = b2World(gravity=(0,0), doSleep=True, contactListener = SensorContactListener()) 
        self.ray_length_list = []
        self.ray_line_list = []
        self.free_control = cmd_args.free
        self.title = cmd_args.title
        self.agent_details = cmd_args.agent_details
        self.receive_q = None # is used to receive the cmd from controller
        self.send_q = None
    # could be loaded from a json file as well.
    def set_environment(self, obstacle_tuples_list = []):
        self.environment = Environment()
        self.environment.create_gridmap()
        self.environment.setup_walls()
        '''Choose ports setup place here'''
        if not self.free_control:
            self.environment.setup_loading_ports_on_row()
            self.environment.setup_unloading_ports_on_bottom()
        self.environment.setup_obstacles(obstacle_tuples_list)
        self.environment.create_static_continuous_space() # this must go after ports and obstacles setup
        return self.environment


    def set_agents(self, agents):
        self.agents = agents
        # create physical objects for agents
        for agent in agents:
            dynamic_body = self.world.CreateDynamicBody(position=[agent.position.x, agent.position.y], userData = agent)
            agent_fixture = dynamic_body.CreatePolygonFixture(box=agent.shape.get_half_dimension(),  userData = agent)
            sensor_fixture = dynamic_body.CreateFixture(shape = agent.sensor.shape, userData = agent.sensor)
            sensor_fixture.sensor = True
            sensor_fixture.filterData.categoryBits = EntityCategory.sensor.value
            sensor_fixture.filterData.maskBits = EntityCategory.sensor.value | EntityCategory.port.value | EntityCategory.wall.value
            agent_fixture.filterData.categoryBits = EntityCategory.agent.value
            agent_fixture.filterData.maskBits = EntityCategory.agent.value | EntityCategory.port.value | EntityCategory.wall.value
            self.b2_objects[agent.id] = dynamic_body

    def set_ports(self, ports):
        for port in ports.values():
            static_port_body = self.world.CreateStaticBody(position = [port.location.x+port.dimension[0]/2.0, port.location.y+port.dimension[1]/2.0], userData = port)
            port_fixture = static_port_body.CreatePolygonFixture(box = (port.dimension[0]/2.0, port.dimension[1]/2.0), userData = port)
            port_fixture.filterData.categoryBits = EntityCategory.port.value
            self.b2_static_objects.append(static_port_body)

    def set_obstacles(self, obstacles):
        for obstacle in obstacles.values():
            static_obstacle_body = self.world.CreateStaticBody(position = [obstacle.location.x+obstacle.dimension[0]/2.0, obstacle.location.y+obstacle.dimension[1]/2.0], userData = obstacle)
            static_obstacle_fixture = static_obstacle_body.CreatePolygonFixture(box = (obstacle.dimension[0]/2.0, obstacle.dimension[1]/2.0), userData = obstacle)
            static_obstacle_fixture.filterData.categoryBits = EntityCategory.obstacle.value
            self.b2_static_objects.append(static_obstacle_body)
         
    def set_walls(self, walls):
        for wall in walls.values():
            static_wall_body = self.world.CreateStaticBody(position = wall.shape.get_box2d_location(), userData = wall)
            static_wall_fixture = static_wall_body.CreatePolygonFixture(box = wall.shape.get_half_dimension(), userData = wall)
            static_wall_fixture.filterData.categoryBits = EntityCategory.wall.value

    def set_local_planner(self, test_agent, local_planner):
        self.agent_local_planner = local_planner.__name__
        test_agent.use_local_planner(local_planner(test_agent))

    def set_global_planner(self, test_agent, global_planner):
        '''All agents share the same (type) of global planner'''
        # deal_with_old_planner
        old_planner = test_agent.global_planner
        if MultiAgentPlanner.__subclasscheck__(type(old_planner)):
            old_planner.remove_agent_under_control(test_agent)
        elif GlobalPlanner.__subclasscheck__(type(old_planner)):
            pass
        else: # None
            pass
        # deal with newly assigned planner
        if not global_planner: # None
            test_agent.global_planner = None
        elif MultiAgentPlanner.__subclasscheck__(global_planner): # this type of global planner plan paths taking all agents under its control into consideration (collabrative)
            # see whether this type of multiagent global planner has been initialized
            server = test_agent.server
            ma_planner = None
            environment_map = self.environment.static_gridmap
            for planner in server.multiagent_global_planners:
                if type(planner) == global_planner: # already exist, not need to create new
                    ma_planner = planner
                    if ma_planner.MAP == MapType.CONTINUOUS:
                        environment_map = self.environment.static_continuous_space
                    break
            if not ma_planner: # not exist: create new, check the required environment type
                if global_planner.MAP == MapType.CONTINUOUS:
                    environment_map = self.environment.static_continuous_space
                ma_planner = global_planner(server, environment_map, self.world, self.TIME_STEP)
                server.add_multiagent_local_planner(ma_planner)
            # ask ma_planner to control the agent
            ma_planner.add_agent_under_control(test_agent)
            # locally installed ma_planner entry on each agent
            test_agent.use_global_planner(MultiAgentPlannerLocalEntry(test_agent))
            test_agent.static_environment = environment_map
        elif GlobalPlanner.__subclasscheck__(global_planner): # this type of global planner only concern the agent installed it and the static environment (non-collabrative)
            environment_map = self.environment.static_gridmap
            if global_planner.MAP == MapType.CONTINUOUS:
                environment_map = self.environment.static_continuous_space
            test_agent.use_global_planner(global_planner(test_agent))
            test_agent.static_environment = environment_map
        else: # Unknown
            print "simulator.py: unknown type of global planner", global_planner
            test_agent.global_planner = None
        test_agent.goal_changed = True # trigger replan

    def set_multiagent_global_planner(self, server, agents, general_multiagent_global_planner, general_static_environment, time_step):
        if general_multiagent_global_planner:
            ma_class = general_multiagent_global_planner
            multiagent_global_planner = ma_class(general_static_environment, server, self.world, time_step) # create one for all agents
            for agent in agents:
                agent.global_planner.connect_to_multiagent_global_planner(multiagent_global_planner) # overwrite
            self.agent_global_planner = ma_class.__name__

    def ini_perception(self):
        for agent in self.agents:
            agent.ini_perception_module(self.b2_objects[agent.id], self.b2_objects.values(), self.b2_static_objects)
    

    def set_agent_velocity(self, agent, agent_body):
        if abs(agent_body.angle) >= pi:
            temp_angle = abs(agent_body.angle)%pi
            if agent_body.angle > 0:
                agent_body.angle = -pi + temp_angle
            elif agent_body.angle < 0:
                agent_body.angle = pi - temp_angle
        speed = agent.speed
        agent_body.angularVelocity = agent.angular_velocity
        agent_body.linearVelocity = (cos(agent_body.angle)*speed, 
                sin(agent_body.angle)*speed)


    def step(self):
        Port.simulator_step = Simulator.step_counter
        for agent in self.agents:
            # obtain observation
            ray_length_list = []
            agent.ray_point_list = []
            agent_body = self.b2_objects[agent.get_id()]
            for line_num in range(0, 512):
                angle = line_num/511.0*pi - pi/2 + agent_body.angle
                length = self.ray_cast_callback(agent_body, angle)
                ray_length_list.append(length)
            agent.ray_length_list = ray_length_list
            
            if len(agent.history_ray_length_list) >= Simulator.sensor_history_length:
                agent.history_ray_length_list = agent.history_ray_length_list[-(Simulator.sensor_history_length-1):]
                agent.history_ray_point_list = agent.history_ray_point_list[-(Simulator.sensor_history_length-1):]
            agent.history_ray_length_list.append(agent.ray_length_list)
            agent.history_ray_point_list.append(agent.ray_point_list)

        for agent in self.agents:
            self.__update_agent_state(agent, agent.ray_length_list)
            agent_body = self.b2_objects[agent.get_id()]
            vx, vy = agent.linear_velocity
            agent_body.linearVelocity = (vx, vy)
            # Only update angle when moving — avoids atan2(0,0)=0 overwriting
            # heading during stopped states (e.g. DullPlanner at waypoint).
            if vx * vx + vy * vy > 1e-6:
                agent_body.angle = atan2(vy, vx)
            # uncomment here to make it become differential wheel
            # self.set_agent_velocity(agent, agent_body)

        self.world.Step(self.TIME_STEP, 10, 10)
        """
        self.time is the simulator world time with unit sec
        """
        self.time = self.get_simulator_time() -  self.start_time
        self.task_count = sum([port.task_count for port in self.environment.unloading_ports.values()])
        Simulator.step_counter += 1
        if Simulator.step_counter % 10 == 0:
            self.heatmap_data.extend([(agent.position.y, agent.position.x) for agent in self.agents])
      
        if self.server is not None:
            self.server.tick(self)
        if self.time % 60 ==0 and self.task_count >0:
            print "Current PPH:", self.task_count/self.time*3600

    """RayCast for sensor using"""
    def ray_cast_callback(self, agent_body, angle):
        ray_length = 4.0
        callback = SensorRayCast()
        position = agent_body.position
        cos_angle = cos(angle)
        sin_angle = sin(angle)
        dimension = agent_body.userData.shape.get_half_dimension()
        start_point = ((position[0] + dimension[0]*cos_angle), (position[1] + dimension[0]*sin_angle))
        end_point = ((position[0] + (dimension[0] + ray_length )*cos_angle), (position[1] + (dimension[0] + ray_length )*sin_angle))
        self.world.RayCast(callback, start_point, end_point)
        agent_body.userData.ray_point_list.append((start_point, end_point))
        if callback.hit:
            return self.__get_distance_between_points(callback.point, start_point)
        else:
            return ray_length


    def __get_distance_between_points(self, a, b):
        return sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]))

    def run(self, show_visualisation):
        if show_visualisation:
            # load visualisation config data
            with open('config.json') as file:
                config_data = json.load(file)
            vis = Visualisation(self, config_data)
            vis.run()
        else:
            target_simulation_steps = self.simulation_times * 60 /self.TIME_STEP
            print("---------------------------------------------------------------------")
            print('The duration will be ' + str(self.simulation_times * 60)+ ' seconds'
                +' or '  + str(self.simulation_times) + ' minutes in simulation world')
            print("---------------------------------------------------------------------")
            start_realworld_time = time.time()
            while(target_simulation_steps >= Simulator.step_counter):
                # print(target_simulation_steps , Simulator.step_counter)
                if self.get_simulator_time() % 60 == 0:
                    print(str(self.get_simulator_time()/60.0)+' minutes passed in simulation world')
                    end_realworld_time = time.time()
                    print int(end_realworld_time - start_realworld_time), "seconds passed in real world"
                    if self.get_simulator_time() > 0:
                        print "PPH is", float(self.task_count)/self.get_simulator_time()*3600, "now"
                    print("---------------------------------------------------------------------")
                self.step()
    def get_simulator_time(self):
        return Simulator.step_counter * self.TIME_STEP 

    def realworld_to_simulator_time(self, realworld_time):
        pass
    #################### simulation codes  #####################################
    def __update_agent_state(self, agent, observation):
        agent.observe(observation)
        agent.plan()
        agent.act()

def json_decoder_environment_data():
    with open('config.json') as file:
            data = json.load(file)
    return data

def print_heatmap(simulator, PPH):
    y,x =zip(*simulator.heatmap_data)
    heatmap, xedges, yedges = np.histogram2d(x, y, bins=(100,100))
    extent = [xedges[0], xedges[-1], yedges[0], yedges[-1]]

    # Plot heatmap
    plt.clf()
    plt.title('System Heatmap\n Simulation time ' + str(simulator.get_simulator_time()) 
        + ' sec\nAgents number '+ str(len(simulator.agents)) + "  PPH: "+ str(PPH))
    plt.ylabel('X')
    plt.xlabel('Y')
    plt.text(0.5, -6,
            'Local Planner: '+str(simulator.agent_local_planner)+', Global planner: '+str(simulator.agent_global_planner), verticalalignment='center')
    plt.imshow(heatmap, extent=extent)
    plt.colorbar(orientation='vertical')
    plt.show()


def start_simulator(args, receive_q = None, send_q = None):
    cmd_args = process_cmd(args)
    show_visualisation = True if cmd_args.time == -1 else False # if a simulation time limit is given, do not invoke graphical display 
    config_data = json_decoder_environment_data()

    # Create a simulator
    simulator = Simulator(cmd_args)
    if receive_q and send_q:
        simulator.receive_q = receive_q
        simulator.send_q = send_q
        sys.stdout = send_q
    simulator.TIME_STEP=1.0/config_data['simulator']['steps_per_sec']
    h6_2_workspace = simulator.set_environment(cmd_args.obstacle)
    agents_number = config_data['agents']['number']
    agents_speed = config_data['agents']['cruise_speed']
    agent_angular_velocity = pi/config_data['agents']['pi_divide_by_max_angular_velocity']
    agents_dimension = config_data["agents"]["dimension"]
    agents=[]
    workspace_width = config_data['environment']['width_in_meters']
    workspace_height = config_data['environment']['height_in_meters']
    """
    2D workspace array -> 1d array
    ignore four sides because ports spawn on sides
    """
    gridmap = GridmapWithNeighbors(h6_2_workspace.static_gridmap)
    available_index_list = gridmap.available_index_list()
    gridmap.static_obstacle_inflation()

    # Set random agents spawn location seed
    spawn_agents_random_seed = 200
    random.seed(spawn_agents_random_seed)

    # Get random location of agent
    random_agent_spawn_location = random.sample(available_index_list,
        agents_number)

    # assign local and global planner according to cmd input
    general_local_planner = process_local_planner_cmd(cmd_args.local_planner)
    general_global_planner = process_global_planner_cmd(cmd_args.global_planner)

    # Create agents
    # The size of agents should be at least one gird
    for num in range(agents_number):
        j = int(random_agent_spawn_location[num] / workspace_width)
        i = random_agent_spawn_location[num] % workspace_width
        test_agent=NaiveAgent(shape=Rectangle(i, j, agents_dimension, agents_dimension),
            position=Point(i, j), speed = agents_speed, angular_velocity = agent_angular_velocity)
        test_agent.angularVelocity = 1
        test_agent.sensor = Sensor(radius = 5, angle = pi, location = test_agent.shape.get_box2d_location(), shape = b2PolygonShape)
        agents.append(test_agent)

        if simulator.free_control:
            test_agent.state = AgentState.CRUISE
    
    # Create server
    server = Server(environment=h6_2_workspace, agents=agents) # server also connect to agents
    simulator.server = server  # give simulator a reference so step() can call server.tick()
    # Connect agent to server & set planner
    for agent in agents:
        agent.connect_to_central_server(server)
        agent.strict_path = getattr(cmd_args, 'strict_path', False)
        simulator.set_local_planner(agent, general_local_planner)
        simulator.set_global_planner(agent, general_global_planner)

    simulator.set_agents(agents)
    if not simulator.free_control:
        simulator.set_ports(h6_2_workspace.unloading_ports)
        simulator.set_ports(h6_2_workspace.loading_ports)
        simulator.loading_ports = h6_2_workspace.loading_ports.values()
        simulator.unloading_ports = h6_2_workspace.unloading_ports.values()
    simulator.set_walls(h6_2_workspace.walls)
    simulator.set_obstacles(h6_2_workspace.obstacles)
    
    simulator.ini_perception()

    if not simulator.free_control:
        for port in simulator.environment.loading_ports.values():
            for i in range(100):
                port.get_random_item(simulator.environment.num_unloading_ports)
    
    simulator.run(show_visualisation)


    print("Time in seconds:",simulator.time)
    print("Number of Packages Delivered:", simulator.task_count)
    #PPH = simulator.task_count/simulator.time*3600
    #print("PPH(Packages Per Hour: ", PPH)
    #print_heatmap(simulator, PPH)

if __name__ == "__main__":
    args = create_cmd_parser().parse_args()
    start_simulator(args)