# -*- coding: utf-8 -*-
# @author : cenrong.dai@dorabot.com
# @brief : provide interaction control via command flags
import re
import imp
import time
import json
import random
import argparse
import importlib
from local_planners.local_planner import LocalPlanner
from local_planners.dull_local_planner import DullPlanner
from global_planners.global_planner import GlobalPlanner
from multiagent_global_planners.multiagent_planner import MultiAgentPlanner
from geometry import Point
from agents.agent_state_machine import AgentState
from agents.agent import Agent
from agents.sensor import Sensor
from setup_environment.obstacles import Obstacles
from setup_environment.port import Port
from protocol import data_book_pb2

SUBPROCESS_END_FLAG = "SUBPROCESS_END"
WARNING_FLAG = "WARNING"
RECEIVED_FLAG = "Received"
CLICKED_FLAG = "Clicked"
CONTROL_FLAG = "CONTROL"
GET_FLAG = "GET"
AGENT_TYPE_FLAG = "AGENT"
SENSOR_TYPE_FLAG = "SENSOR"

def create_cmd_parser(prog=None):
    """The parser for flags when starting a new simulator"""
    parser = argparse.ArgumentParser(prog=prog, description='Dorabot MARS simulator.')
    general_control = parser.add_argument_group('General Control', 'Commands for general control.')
    json_control = parser.add_argument_group('JSON Control', 'Commands for setting JSON data.')
    obstacle_control = parser.add_argument_group('Obstacle Control', 'Commands for obstacle generation.')
    # general_control
    general_control.add_argument('-d', action='store_true', dest='agent_details', help='Run the simulator with agents\' details showed')
    general_control.add_argument('-c', action='store_true', dest='free', help='Start a closed environment free of any other obstacle')
    general_control.add_argument('-t', action='store', type=float, default=-1, dest='time', help='Run the simulator for the given number of minutes without showing up visualisation')
    general_control.add_argument('--title', action='store', type=str, default="Dorabot Minions", dest='title', help='Set the title of the simulator')
    general_control.add_argument('--gp', action='store', type=str, dest='global_planner', help='Set the global planners for all agents, choices include: LayeredAStar | RRTStar | MARRTStar | INashRRT | CBSPlanner | ECBSPlanner')
    general_control.add_argument('--lp', action='store', type=str, dest='local_planner', help='Set the local planners for all agents, choices include: VirtualForcePlanner | FLCPlanner')
    general_control.add_argument('--strict-path', action='store_true', dest='strict_path', help='Force agents to follow global planner waypoints strictly in order (no sliding-window shortcuts). Recommended when using CBS/ECBS.')
    
    # json_control
    json_control.add_argument('--agent', action='store', type=int, dest='agents_num', help='Set the number of agents', metavar='N')
    json_control.add_argument('--port', action='store', type=int, nargs=2, dest='ports_num', help='Set the number of loading and unloading ports', metavar=('LOAD', 'UNLOAD'))
    json_control.add_argument('--size', action='store', type=int, nargs=2, dest='meter_size', help='Set the width and height (in meter) of the environment', metavar=('WIDTH', 'HEIGHT'))
    json_control.add_argument('--resolution', action='store', type=int, dest='resolution', help='Set the resolution of the grid (number of grids per meter)')
    json_control.add_argument('--speed', action='store', type=float, dest='cruise_speed', help='Set the cruise speed of the agents')
    json_control.add_argument('--step', action='store', type=int, dest='step', help='Set the number of steps simulated per second')
    json_control.add_argument('--default', action='store_true', dest='default', help='Set all settings back to default')

    # (name or flags...[, action][, nargs][, const][, default][, type][, choices][, required][, help][, metavar][, dest])
    obstacle_control.add_argument('--obstacle', action='store', type=str, dest='obstacle', help='Specify the positions and sizes of a list of square obstacles in the form of (top_left_x, top_left_y, width) OR Randomly generate obstacles upto a certain percetage of congestion level', metavar='"(TOP_LEFT_X,TOP_LEFT_Y,WIDTH), ..." OR VALUE_BETWEEN_[0.0,0.7]')
    obstacle_control.add_argument('--seed', action='store', type=int, dest='seed', help='The seed used in random generation of obstacles')
    obstacle_control.add_argument('--save', action='store', type=str, dest='save_map', help='Save the generated obstacle map into a file in MAP folder', metavar='FILENAME')
    obstacle_control.add_argument('--import', action='store', type=str, dest='import_map', help='Import an obstacle map from MAP folder', metavar='FILENAME')
    return parser


def process_cmd(args):
    """Process the cmd flags"""
    with open("config.json", "r+") as jsonFile:
        json_data = json.load(jsonFile)
        grid_width = json_data['environment']['width_in_meters']
        grid_height = json_data['environment']['height_in_meters']
        obstacle_map = {} # store the attributes related to obstacle generation
        obstacle_tuples_list = [] # sotre the obstacle generated 

        # JSON control
        if args.default: # default will be overwritten if specific settings also exist
            json_data['agents']['number'] = 10
            json_data['environment']['width_in_meters'] = 45
            json_data['environment']['height_in_meters'] = 20
            json_data["visualization"]["pygame_screen_width"] =  1900
            json_data["visualization"]["pygame_screen_height"] = 1000
            json_data['environment']['num_loading_ports'] = 10
            json_data['environment']['num_unloading_ports'] = 10
            json_data['simulator']['steps_per_sec'] = 60
            json_data['environment']['resolution'] = 1
            json_data['agents']['cruise_speed'] = 1.5
        if args.agents_num:
            json_data['agents']['number'] = args.agents_num
        if args.ports_num:
            loadings_num, unloadings_num = args.ports_num
            json_data['environment']['num_loading_ports'] = loadings_num
            json_data['environment']['num_unloading_ports'] = unloadings_num
        if args.meter_size:
            grid_width, grid_height = args.meter_size
            json_data['environment']['width_in_meters'] = grid_width
            json_data['environment']['height_in_meters'] = grid_height
        if args.resolution:
            json_data['environment']['resolution'] = args.resolution
        if args.cruise_speed:
            json_data['agents']['cruise_speed'] = args.cruise_speed
        if args.step:
            json_data['simulator']['steps_per_sec'] = args.step

        # obstacle control
        if args.obstacle:
            obstacle_map['obstacle'] = args.obstacle
        if args.seed:
            obstacle_map['seed'] = args.seed
        if args.save_map:
            obstacle_map['save'] = args.save_map
        if args.import_map:
            obstacle_map['import'] = args.import_map
        

        grid_width = json_data['environment']['width_in_meters']
        grid_height = json_data['environment']['height_in_meters']
        obstacle_tuples_list, grid_width, grid_height = obstacle_generator(obstacle_map, grid_width, grid_height)
        args.obstacle = obstacle_tuples_list

        # reset environment size according to map/port number
        json_data['environment']['width_in_meters'] = grid_width
        json_data['environment']['height_in_meters'] = grid_height
        ratio = 30 # also change screen size
        json_data["visualization"]["pygame_screen_width"] =  grid_width* ratio + ratio*5
        json_data["visualization"]["pygame_screen_height"] = grid_height*ratio + ratio*10
        
        # make sure number of ports can be fit into the map
        loadings_num = json_data['environment']['num_loading_ports']
        unloadings_num = json_data['environment']['num_unloading_ports']
        if grid_width < max(loadings_num, unloadings_num)*3+2:
            raise Exception("Unable to fit {} ports inline into {}-grid-width map, at least {}-grid-width is needed"\
                .format(max(loadings_num, unloadings_num), grid_width, \
                    max(loadings_num, unloadings_num)*3+2))
        if grid_height < 5:
            raise Exception("Unable to fit ports into {}-grid-height map, at least 5-grid-height is needed".format(grid_height))

        jsonFile.seek(0)  # rewind
        json.dump(json_data, jsonFile, indent=4, separators=(',', ': '))
        jsonFile.truncate()
        jsonFile.close()

    return args

def obstacle_generator(obstacle_map, width, height):
    """Generate obstacle according to input map which contains (top_left_x,top_left_y,dimension) lsit"""
    import_flag = "import"
    save_flag = "save"
    obstacle_flag = "obstacle"
    seed_flag = "seed"

    obstacle_tuples_list = [] # for generate obstacle instance
    obstacle_str = ""
    # import has the highest priority
    if import_flag in obstacle_map:
        try:
            f = open("map/{}".format(obstacle_map[import_flag]), "r")
            width_str, height_str = f.readline().split(',')[:2]
            width, height = int(width_str), int(height_str)
            obstacle_str = f.readline()
            obstacle_tuples_list = re.findall(r'\((.*?)\)',obstacle_str)
        except Exception as e:
            print "obstacle_generator:", e
    elif obstacle_flag in obstacle_map:
        obstacle_str = obstacle_map[obstacle_flag]
        obstacle_tuples_list = re.findall(r'\((.*?)\)',obstacle_str)
        spanning_center_x = int(width / 2)
        spanning_center_y = int(height / 2)
        if len(obstacle_tuples_list) == 0: # user use percentage rather than manually specified obstacle string
            percentage = float(obstacle_str)
            if percentage > 0.7:
                print "The congestion level is too high to be valid, cannot generate obstacle map"
                return [], width, height
            # try to generate random obstacles
            seed = time.time()
            if seed_flag in obstacle_map:
                seed = obstacle_map[seed_flag]
            obstacle_str = ""
            max_obstacle_dimension = 3
            remaining_space_to_fill = (width-max_obstacle_dimension-3) * (height-7) * percentage
            random.seed(seed)
            while remaining_space_to_fill > 0:
                # randomly generate a square
                current_dimension = random.randint(1, max_obstacle_dimension)
                try:
                    square = (random.randint(2, width-2-current_dimension), random.randint(4, height-3-current_dimension), current_dimension)
                    # should not cover the center point which is the root of spanning tree when applicable
                    if square[0] <= spanning_center_x <= square[0]+square[2] and\
                        square[1] <= spanning_center_y <= square[1]+square[2]:
                        continue
                    remaining_space_to_fill -= square[2]**2
                    if remaining_space_to_fill < 0: # early stop
                        break
                    obstacle_str += str(square) + ", "
                except:
                    pass
            obstacle_tuples_list = re.findall(r'\((.*?)\)',obstacle_str)
        else: # user specifies each obstacle
            valid_width = (2, width-2)
            valid_height = (4, height-3)
            inbound_tuples_list = []
            for square_str in obstacle_tuples_list:
                # check the validation of customized input
                x, y, dimension = tuple(map(float, re.split('[ ]*,[ ]*', square_str)))
                if x < valid_width[0] or y < valid_height[0] or x+dimension > valid_width[1] or y+dimension > valid_height[1]:
                    print "({}) is discarded since it will block the way to ports or out of bound;".format(square_str)
                    continue
                if x <= spanning_center_x <= x+dimension and y <= spanning_center_y <= y+dimension:
                    print "({}) is discarded since it covers the center of spanning tree".format(square_str)
                    continue
                inbound_tuples_list.append(square_str)
            obstacle_tuples_list = inbound_tuples_list
        
    if save_flag in obstacle_map:
        f = open("map/{}".format(obstacle_map[save_flag]), "w")
        size_str = "{},{}\n".format(width, height)
        f.write(size_str+obstacle_str)
        f.close()
    return obstacle_tuples_list, width, height

def process_local_planner_cmd(local_planner_name):
    """Return the corresponding local planner class to be initialized according to a given name"""
    if not local_planner_name: return DullPlanner # None
    for local_planner_class in LocalPlanner.__subclasses__():
        if local_planner_name == local_planner_class.__name__:
            return local_planner_class
    # if it is your own planner
    module_name, class_name = convert_planner_name_by_uppercase(str(local_planner_name))
    planner_class = dynamic_importer(module_name, class_name)
    if planner_class and LocalPlanner.__subclasscheck__(planner_class): # found and is local planner
        return planner_class
    return DullPlanner # default

def process_global_planner_cmd(global_planner_name):
    if not global_planner_name: return None # None
    for global_planner_class in GlobalPlanner.__subclasses__(): # LayeredAStar
        if global_planner_name == global_planner_class.__name__:
            return global_planner_class
    for multiagent_global_planner_class in MultiAgentPlanner.__subclasses__(): # INashRRT
        if global_planner_name == multiagent_global_planner_class.__name__:
            return multiagent_global_planner_class
    # if it is your own planner
    module_name, class_name = convert_planner_name_by_uppercase(str(global_planner_name))
    planner_class = dynamic_importer(module_name, class_name)
    if planner_class and (GlobalPlanner.__subclasscheck__(planner_class) or MultiAgentPlanner.__subclasscheck__(planner_class)): # found
        return planner_class
    return None # not found

def controller_cmd_handler(simulator, running):
    '''process cmd from control panel, return (running, update) tuple'''
    if not (simulator.receive_q and simulator.send_q): # not in controllable mode
        return (running, False) # do nothing
    if not running: # terminated, tell the controller is terminated
        print SUBPROCESS_END_FLAG
        return (running, False)
    def to_point(float_list):
        return Point(float_list[0], float_list[1])
    while not simulator.receive_q.empty():
        flag, cmd = simulator.receive_q.get()
        if flag == SUBPROCESS_END_FLAG:
            print SUBPROCESS_END_FLAG
            running = False
            return (running, False)
        elif flag == CONTROL_FLAG: # input from do_control
            if not cmd.ids:
                cmd.ids = range(len(simulator.agents))
            valid_path = None
            for aid in cmd.ids:
                if cmd.path: # parse path
                    try: # in case of no valid tuple
                        coord_list = list(map(lambda x:to_point(map(float, re.split('[ ]*,[ ]*',x))), re.findall(r'\((.*?)\)',cmd.path)))
                        valid_path = [(p.x, p.y) for p in coord_list]
                        agent_object = simulator.b2_objects[aid]
                        agent_object.position = coord_list[0].to_tuple() # object accept unmutable tuple only
                        agent = simulator.agents[aid]
                        agent.destination_location = coord_list[0]
                        agent.task.destination_location = coord_list[0]
                        agent.internal_stations = coord_list
                        agent.state = AgentState.CRUISE
                        agent.goal_changed = True
                    except Exception as e:
                        print "{}: Cannot extract path from {}, did you input a valid one? {}".format(WARNING_FLAG, cmd.path, e)
                if cmd.global_planner: # reset agent's global planner
                    try:
                        simulator.set_global_planner(simulator.agents[aid],process_global_planner_cmd(cmd.global_planner))
                    except:
                        print "{}: Cannot find agent of id {} when setting global planner {}".format(WARNING_FLAG,aid, cmd.global_planner)
                if cmd.local_planner:
                    try:
                        simulator.set_local_planner(simulator.agents[aid], process_local_planner_cmd(cmd.local_planner))
                    except:
                        print "{}: Cannot find agent of id {} when setting local planner {}".format(WARNING_FLAG,aid, cmd.local_planner)
            # for display back to control panel
            if not valid_path:
                cmd.path = valid_path
        elif flag == GET_FLAG:
            if cmd.require_agent:
                aid_str, var_name = cmd.require_agent
                try:
                    simulator.send_q.put((GET_FLAG, (var_name, AGENT_TYPE_FLAG, convert_to_protocol_data_string(int(aid_str), simulator.agents[int(aid_str)]))))
                except Exception as e:
                    print "{}: Cannot find agent of id {} when retrieving instance".format(WARNING_FLAG,int(aid_str))
            elif cmd.require_sensor:
                sid_str, var_name = cmd.require_sensor
                try:
                    simulator.send_q.put((GET_FLAG, (var_name, SENSOR_TYPE_FLAG, convert_to_protocol_data_string(int(sid_str), simulator.agents[int(sid_str)].sensor))))
                except Exception as e:
                    print "{}: Cannot find sensor of id {} when retrieving instance".format(WARNING_FLAG,int(sid_str))
        print "{}: {}".format(RECEIVED_FLAG, cmd)
    return (running, True)

def convert_to_protocol_data(type_str, obj_str):
    if type_str == AGENT_TYPE_FLAG:
        agent_type = data_book_pb2.Agent()
        agent_type.ParseFromString(obj_str)
        return agent_type
    elif type_str == SENSOR_TYPE_FLAG:
        sensor_type = data_book_pb2.Sensor()
        sensor_type.ParseFromString(obj_str)
        return sensor_type

def convert_to_protocol_data_string(id_int, obj):
    if Agent.__subclasscheck__(type(obj)):
        agent_type = data_book_pb2.Agent()
        agent_type.id = obj.id
        agent_type.shape.x = obj.shape.width
        agent_type.shape.y = obj.shape.height
        agent_type.linear_velocity.x = obj.linear_velocity[0]
        agent_type.linear_velocity.y = obj.linear_velocity[1]
        agent_type.angular_velocity = obj.angular_velocity
        agent_type.angle = obj.angle
        agent_type.position.x = obj.position.x
        agent_type.position.y = obj.position.y
        agent_type.destination_location.x = obj.destination_location.x if obj.destination_location else -1
        agent_type.destination_location.y = obj.destination_location.y if obj.destination_location else -1
        agent_type.local_planner.type = type(obj.current_local_planner).__name__ if obj.current_local_planner else str(None)
        agent_type.global_planner.type = type(obj.global_planner).__name__ if obj.global_planner else str(None)
        agent_type.sensor.id = obj.id
        agent_type.sensor.radius = obj.sensor.radius
        agent_type.sensor.angle = obj.sensor.angle
        agent_type.sensor.location.x = obj.sensor.location[0]
        agent_type.sensor.location.y = obj.sensor.location[1]
        for visible in obj.sensor.visible_object:
            data = visible.userData
            if data.type == 'agent':
                agent_type.sensor.visible_agents_id.append(data.id)
            else: # port, wall, obstacle
                obstacle_type = data_book_pb2.Obstacle()
                obstacle_type.center.x = data.center.x
                obstacle_type.center.y = data.center.y
                obstacle_type.dimension.x = data.dimension[0]
                obstacle_type.dimension.y = data.dimension[1]
                obstacle_type.type = data.type
                agent_type.sensor.visible_obstacles.append(obstacle_type)
        for val in obj.ray_length_list:
            agent_type.ray_length_list.append(val)
        for pose in obj.sequence_of_poses:
            point_type = data_book_pb2.Point()
            point_type.x = pose.x
            point_type.y = pose.y
            agent_type.sequence_of_poses.append(point_type)
        for station in obj.internal_stations:
            point_type = data_book_pb2.Point()
            point_type.x = station.x
            point_type.y = station.y
            agent_type.internal_stations.append(point_type)
        return agent_type.SerializeToString()
    elif Sensor.__subclasscheck__(type(obj)):
        sensor_type = data_book_pb2.Sensor()
        sensor_type.id = id_int
        sensor_type.radius = obj.radius
        sensor_type.angle = obj.angle
        sensor_type.location.x = obj.location[0]
        sensor_type.location.y = obj.location[1]
        for visible in obj.visible_object:
            data = visible.userData
            if data.type == 'agent':
                sensor_type.visible_agents_id.append(data.id)
            else: # port, wall, obstacle
                obstacle_type = data_book_pb2.Obstacle()
                obstacle_type.center.x = data.center.x
                obstacle_type.center.y = data.center.y
                obstacle_type.dimension.x = data.dimension[0]
                obstacle_type.dimension.y = data.dimension[1]
                obstacle_type.type = data.type
                sensor_type.visible_obstacles.append(obstacle_type)
        return sensor_type.SerializeToString()

def convert_planner_name_by_uppercase(name):
    segments = filter(lambda x:len(x)>0, re.split('([A-Z][a-z]*)', name))
    class_name = name
    module_name = '_'.join([s[0].lower()+s[1:] for s in segments]) + '_planner'
    return module_name, class_name

def convert_planner_name_by_underscore(name):
    segments = (name.split('_'))
    class_name = ''.join([s[0].upper()+s[1:] for s in segments])
    module_name = name+'_planner'
    return module_name, class_name

def dynamic_importer(module_name, class_name):
    """Dynamically imports modules / classes"""
    packages = ['local_planners', 'global_planners', 'multiagent_global_planners']
    for upper_pkg in packages:
        pkg = upper_pkg+'.user'
        try:
            module = importlib.import_module('.'+module_name, package=pkg)
            my_class = getattr(module, class_name)
            return my_class
        except:
            pass
    print "Unable to find {}".format(class_name)
    return None

if __name__ == "__main__":
    print convert_planner_name_by_uppercase("MyMAGlobal")
    print convert_planner_name_by_uppercase("JustTryIt")