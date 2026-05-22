# -*- coding: utf-8 -*-
"""
@Copyright Dorabot Inc.
@date : 2018-10
@author: {tian.xiao, xiaoyu.ge}@dorabot.com
@brief : Implementation of ports and workspace
"""
import json
import random
from math import ceil, floor
from geometry import Point, comptue_center_point
from representation.gridmap_a import GridmapWithNeighbors
from shape import Rectangle
from queue import SimpleQueue
from pygame import time
import re
from setup_environment.walls import Wall
from setup_environment.unloading_port import UnloadingPort
from setup_environment.loading_port import LoadingPort
from setup_environment.obstacles import Obstacles
from representation.float_to_grid import *
from representation.continous_space import ContinuousSpace
""" The grid map of environment is a 2D labeled map
 For the value in each grid,
 0 is empty, 4 is obstacles, 2 is unloading area,
 1 is loading area, 3 is queue area
"""

"""
   Environment included workspace, loading ports and unloading ports
   It will include obstacles and waiting area in future
"""
class Environment:
    def __init__(self):
        self.height_in_meters = 0
        self.width_in_meters = 0
        #Top left corner is fixed (0,0)
        self.top_left_corner = (0, 0)
        self.num_grids_in_height = 0
        self.num_grids_in_width = 0
        self.resolution = 0  # grids per meter
        self.static_gridmap = None
        self.static_continuous_space = None
        self.loading_ports = {}
        self.unloading_ports = {}
        self.walls = {}
        self.obstacles = {}
        self.num_loading_ports = 0
        self.num_unloading_ports = 0
        self.loading_ports_process_time = 0
        self.unloading_ports_process_time = 0
        self.agent_dimension = 0
        self.__import_decoder_config_data()

    def __json_decoder_config_data(self):
        with open('config.json') as file:
            config_data = json.load(file)
        return config_data

    def __import_decoder_config_data(self):
        config_data = self.__json_decoder_config_data()
        environment_data = config_data['environment']
        corner_coordinate = environment_data['top_left_corner']
        self.height_in_meters = environment_data['height_in_meters']
        self.width_in_meters = environment_data['width_in_meters']
        self.top_left_corner = (corner_coordinate['x'], corner_coordinate['y'])
        self.resolution = environment_data['resolution']
        self.num_loading_ports = environment_data['num_loading_ports']
        self.num_unloading_ports = environment_data['num_unloading_ports']
        self.loading_ports_process_time = environment_data['loading_ports_process_time']
        self.unloading_ports_process_time = environment_data['unloading_ports_process_time']
        self.agent_dimension = config_data['agents']['dimension']
        # self.waiting_areas = self.__json_decoder_dict(environment_data['waiting_areas'])

    #Polygon which have location and shape uses list of the vertices with order to describe

    def __json_decoder_vertices(self, data):
        vertices_list = []
        for vertice in data:
            tmp_vertice = (vertice['x'], vertice['y'])
            vertices_list.append(tmp_vertice)
        return vertices_list

    #Create a dictionary for Unloading, loading, obstacles and waiting area.
    #Using ID as the key

    def __json_decoder_dict(self, data):
        object_dict = {}
        for polygon in data:
            object_dict[polygon['id']] = self.__json_decoder_vertices(
                polygon['vertices'])
        return object_dict

    #Create grid map by height and width
    def create_gridmap(self):
        self.static_gridmap = GridMap(self.width_in_meters,
                                      self.height_in_meters,
                                      self.resolution,
                                      self.top_left_corner)
        self.num_grids_in_width = self.static_gridmap.num_x_grids
        self.num_grids_in_height = self.static_gridmap.num_y_grids
        self.static_gridmap = GridmapWithNeighbors(self.static_gridmap)
        # Mark the outermost ring as obstacles to prevent edge-hugging paths.
        max_x = self.num_grids_in_width - 1
        max_y = self.num_grids_in_height - 1
        for x in range(self.num_grids_in_width):
            self.static_gridmap[x, 0] = 4
            self.static_gridmap[x, max_y] = 4
        for y in range(self.num_grids_in_height):
            self.static_gridmap[0, y] = 4
            self.static_gridmap[max_x, y] = 4
    
    def create_static_continuous_space(self):
        '''Note that walls are not passed in as obstacle, since they are out of width/height-in-meter range'''
        static_obstacles_list = list(self.obstacles.values())+list(self.loading_ports.values())+list(self.unloading_ports.values())
        self.static_continuous_space = ContinuousSpace(static_obstacles_list,
                                                       self.width_in_meters,
                                                       self.height_in_meters,
                                                       self.agent_dimension,
                                                       self.top_left_corner)

    def setup_walls(self):
        horizontal_dimension = (2 + self.agent_dimension + self.width_in_meters,1)
        vertical_dimension = (1, 1 + self.agent_dimension + self.height_in_meters)
        ceiling_wall = Wall(Point(-(1 + self.agent_dimension/2.0), (-(1 + self.agent_dimension/2.0))), horizontal_dimension)
        ground_wall = Wall(Point(-(1 + self.agent_dimension/2.0), (self.height_in_meters + 
            (self.agent_dimension/2.0))), horizontal_dimension)
        left_wall = Wall(Point(-(1 + self.agent_dimension/2.0), -self.agent_dimension/2.0), vertical_dimension)
        right_wall = Wall(Point((self.width_in_meters + self.agent_dimension/2.0), 
            -self.agent_dimension/2.0), vertical_dimension)
        self.walls[ceiling_wall.identifier] = ceiling_wall
        self.walls[ground_wall.identifier] = ground_wall
        self.walls[left_wall] = left_wall
        self.walls[right_wall] = right_wall

    """General setup ports function"""

    def setup_loading_ports_on_location(self, location, dimension, loading_ports_process_time):
        tmp_port = LoadingPort(location, dimension, port_process_time = loading_ports_process_time)
        self.loading_ports[tmp_port.identifier] = tmp_port
        self.__set_port_value_to_gridmap(tmp_port, 1)
        return tmp_port
    def setup_unloading_ports_on_location(self, location, dimension, unloading_ports_process_time):
        tmp_port = UnloadingPort(location, dimension, port_process_time = unloading_ports_process_time)
        self.unloading_ports[tmp_port.identifier] = tmp_port
        self.__set_port_value_to_gridmap(tmp_port, 2)
        return tmp_port

    """setup Unloading point in mid of workspace"""
    def setup_unloading_ports_in_mid(self):
        default_port_dimension_size = 1
        unloading_ports_spawn_start_location = (2, 3)
        dimension = (default_port_dimension_size, default_port_dimension_size)
        unloading_ports_area_width = self.width_in_meters - 2 * unloading_ports_spawn_start_location[0]
        unloading_ports_area_height = self.height_in_meters - 2 * unloading_ports_spawn_start_location[1]
        """ hard code start location to 2,3
                     gap set to 2
            the ports will be like 
             Port    *     *

              *      *     *

              *      *     *  
        """
        gap_between_ports = 4
        num_ports_each_line = int(unloading_ports_area_width / gap_between_ports)
        for num in range(self.num_unloading_ports):
            i = num  % num_ports_each_line
            j = num // num_ports_each_line
            tmp_port = self.setup_unloading_ports_on_location(Point(unloading_ports_spawn_start_location[0] + 
                i*gap_between_ports, 
                unloading_ports_spawn_start_location[1] + j * gap_between_ports+2), 
            dimension, self.unloading_ports_process_time)
            tmp_port.has_queue_area = True
            self.setup_simple_queue_area(tmp_port, gap_between_ports)


    def setup_loading_ports_on_horizontal(self):
        default_port_dimension_size = 1
        dimension = (default_port_dimension_size, default_port_dimension_size)
        ports_num_on_top = int(floor(self.num_loading_ports/2.0))
        ports_num_on_bottom = int(ceil(self.num_loading_ports/2.0))
        gap_between_ports_on_top = int(floor((self.width_in_meters - 2 * default_port_dimension_size)
                                  / ports_num_on_top))
        gap_between_ports_on_bottom = int(floor((self.width_in_meters - 2 * default_port_dimension_size)
                                  / ports_num_on_bottom))
        if gap_between_ports_on_top == 0 or gap_between_ports_on_bottom == 0: 
            raise ValueError(
                'Loading ports number must be smaller than the width grid number minus two'
            )
        for x in range(ports_num_on_top):
            tmp_port = self.setup_loading_ports_on_location(Point(x % ports_num_on_top * 
                gap_between_ports_on_top + default_port_dimension_size, 
                1), dimension, self.loading_ports_process_time)
            tmp_port.has_queue_area = True
            self.setup_simple_queue_area(tmp_port, gap_between_ports_on_top)
        for x in range(ports_num_on_bottom):
            tmp_port = self.setup_loading_ports_on_location(Point(x % ports_num_on_bottom * 
                gap_between_ports_on_bottom + default_port_dimension_size, 
                self.height_in_meters - 2 * default_port_dimension_size), dimension, self.loading_ports_process_time)
            tmp_port.has_queue_area = True
            self.setup_simple_queue_area(tmp_port, gap_between_ports_on_bottom)

    """ Loading ports number must be smaller than (the width grid number - 2*dimension)
    Fill top on average by loading ports
    default set on top
    """
    def setup_loading_ports_on_row(self, row = 1, num_loading_ports = 1):
        num_loading_ports = self.num_loading_ports
        default_port_dimension_size = 1
        dimension = (default_port_dimension_size, default_port_dimension_size)
        #Not using the origin and the end point for ports
        gap_between_ports = floor((self.width_in_meters - 2 * default_port_dimension_size)
                                  / num_loading_ports)
        if gap_between_ports == 0:
            raise ValueError(
                'Loading ports number must be smaller than the width grid number minus two'
            )
        for x in range(num_loading_ports):
            tmp_port = self.setup_loading_ports_on_location(Point(x % num_loading_ports * 
                gap_between_ports +default_port_dimension_size, row), dimension, self.loading_ports_process_time)
            tmp_port.has_queue_area = True
            self.setup_simple_queue_area(tmp_port, gap_between_ports)
    """
    Unloading ports number must be smaller than (the width grid number + height grid number*2 -10*dimension )
    First, fill bottom side by unloading ports
    Then, fill left and right side on average by rest ports
    """
    def setup_unloading_ports_on_row(self, row = 1, num_unloading_ports = 1):
        row = self.height_in_meters - 2
        num_unloading_ports = self.num_unloading_ports
        default_port_dimension_size = 1
        dimension = (default_port_dimension_size, default_port_dimension_size)
        gap_between_ports = floor((self.width_in_meters - 2 * default_port_dimension_size)
                      / self.num_unloading_ports)
        if gap_between_ports == 0:
            raise ValueError(
                'Loading ports number must be smaller than the width grid number minus two'
            )
        for x in range(num_unloading_ports):
                tmp_port = self.setup_unloading_ports_on_location(
                    Point(x % num_unloading_ports * gap_between_ports +
                        default_port_dimension_size, row), dimension,
                 self.unloading_ports_process_time)
                tmp_port.has_queue_area = True
                self.setup_simple_queue_area(tmp_port, gap_between_ports)

    def setup_unloading_ports_on_bottom(self):
        default_port_dimension_size = 1
        dimension = (default_port_dimension_size, default_port_dimension_size)
        """
          If loading ports nu mber is bigger or same as unloading ports,
          the unloading ports will be aligned loading ports
        """
        gap_between_ports = floor((self.width_in_meters - 2 * default_port_dimension_size)
                              / self.num_unloading_ports)
        if gap_between_ports == 0:
            raise ValueError(
                'Loading ports number must be smaller than the width grid number minus two'
            )
        for x in range(self.num_unloading_ports):
            tmp_port = self.setup_unloading_ports_on_location(
                Point(x % self.num_unloading_ports * gap_between_ports +
                    default_port_dimension_size, self.height_in_meters-1), dimension,
             self.unloading_ports_process_time)
            tmp_port.has_queue_area = True
            self.setup_simple_queue_area(tmp_port, gap_between_ports)


    def setup_obstacles(self, obstacle_tuples_str_list = []):
        obstacle_tuples_list = []
        for obstacle_tuple_str in obstacle_tuples_str_list: # change string to tuple of numbers
            x_top_str, y_top_str, dimension_str = re.split('[ ]*,[ ]*', obstacle_tuple_str)
            x_top, y_top, x_dimension, y_dimension = float(x_top_str), float(y_top_str), float(dimension_str), float(dimension_str)
            obstacle_tuples_list.append((x_top, y_top, x_dimension, y_dimension))
        
        obstacle_tuples_list.sort(key=lambda t:t[-1], reverse=True) # sort according to dimension in descending order
        for (x_top, y_top, x_dimension, y_dimension) in obstacle_tuples_list:
            def check_contained_in_obstacle(x_top, y_top, x_dimension, y_dimension):
                """check whether current obstacle is contained in the existing obstacles"""
                corners = [(x_top, y_top), (x_top, y_top+y_dimension), (x_top+x_dimension, y_top), (x_top+x_dimension, y_top+y_dimension)]
                for other_obstacle in self.obstacles.values():
                    for corner in list(corners):
                        if other_obstacle.is_clicked(corner):
                            corners.remove(corner)
                    if len(corners) < 2:
                        return True
            
            if check_contained_in_obstacle(x_top, y_top, x_dimension, y_dimension):
                '''this checking aims at alleviating the possibility that a dead region get created, where no agent can go into or out of; ***but not 100% gurantee***;
                it also discards the obstacles which are completed contained by others, alleviating the number of obstacles that a collision check need
                to be run with during path planning stage'''
                continue # discard this obstacle
            
            obstacle = Obstacles(Point(x_top, y_top), (x_dimension, y_dimension))
            self.obstacles[obstacle.identifier] = obstacle
            # set obstacle value to gridmap
            origin_x = int(obstacle.location.x * self.resolution)
            origin_y = int(obstacle.location.y * self.resolution)
            dimension_x = int((obstacle.dimension[0])* self.resolution + 1)
            dimension_y = int((obstacle.dimension[1]) * self.resolution + 1)

            for i in range(dimension_x):
                for j in range(dimension_y):
                    self.static_gridmap[origin_x + i, origin_y + j] = 4

# """
#     O: (left, top)
#     | -------------- gap_between_ports ------------|
#     O---- port dim ----|---------------------------|
#     |                  |                           port
#     | Loading Port Area|     Queuing Area          dim
#     |                  |                           |
#     |------------------|---------------------------|
# """

    def setup_simple_queue_area(self, port, gap_between_ports):
        x = port.location.x + 2*port.dimension[0]
        y = port.location.y
        width = int(gap_between_ports - 3*port.dimension[0])
        height = 1
        region = Rectangle(x, y, width, height)
        mechanism = SimpleQueue(region)
        port.enforce_queuing_mechanism(mechanism)
        left_on_gridmap = x * self.resolution
        top_on_gridmap = y * self.resolution
        width_on_gridmap = width * self.resolution
        height_on_gridmap = height * self.resolution
        #Set queue value in gridmap
        for i in range(width_on_gridmap):
            for j in range(height_on_gridmap):
                self.static_gridmap[left_on_gridmap + i, top_on_gridmap + j] = 0

    """
        Set port value to gridmap
        2 is unloading ports
        3 is loading ports
    """

    def __set_port_value_to_gridmap(self, port, val):
        origin_x = int(port.location.x * self.resolution)
        origin_y = int(port.location.y * self.resolution)
        if port.dimension[0] == 1 and port.dimension[1]== 1:
            dimension_x = int((port.dimension[0])* self.resolution + 1)
            dimension_y = int((port.dimension[1]) * self.resolution + 1)
        else:
            dimension_x = int((port.dimension[0])* self.resolution)
            dimension_y = int((port.dimension[1]) * self.resolution)
        for i in range(dimension_x):
            for j in range(dimension_y):
                """loading ports"""
                if origin_y < self.num_grids_in_height - self.resolution:
                    self.static_gridmap[origin_x + i, origin_y + j] = val
                    """unloading ports"""
                else:
                    self.static_gridmap[origin_x + i, origin_y - 1 + j] = val

    def debug_agent_spawn_location(self):
        gridmap = GridmapWithNeighbors(self.static_gridmap)
        available_index_list = gridmap.available_index_list()
        # Set random agents spawn location seed
        spawn_agents_random_seed = 100
        random.seed(spawn_agents_random_seed)
        # Get random location of agent
        config_data = self.__json_decoder_config_data()
        agents_number = config_data['agents']['number']
        random_agent_spawn_location = random.sample(available_index_list,
            agents_number)

        for num in range(agents_number):
            j = int(random_agent_spawn_location[num] / self.width_in_meters)
            i = random_agent_spawn_location[num] % self.width_in_meters
            self.static_gridmap[i,j] = 7

