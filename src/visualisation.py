# -*- coding: utf-8 -*-
"""
@Copyright Dorabot Inc.
@date : 2018-07
@author : {xiaoyu.ge, tian.xiao}@dorabot.com
@brief : visualization code of simulator partly adopted from the author's github repository: https://github.com/fantastdd/physical-reasoning-2d/blob/master/scenario_generator.py
"""
"""
WARNING: unpolished visualisation codes.
"""
import pygame
from pygame import Rect
import pygame.locals
import sys
from math import ceil, degrees, atan2
import json
from time import sleep
import random
import threading
import re
from Box2D.b2 import polygonShape
from pygame.locals import QUIT, KEYDOWN
from agents.agent import Agent
from representation.gridmap_a import GridmapWithNeighbors
from global_planners.global_planner import MapType
from representation.float_to_grid import float_to_grid
from geometry import Vector, Point
from interaction_handler import controller_cmd_handler, CLICKED_FLAG

"""only for debug: generate random color dictionary"""
def rnd_color(num):
    random.seed(num)
    colors_list = []
    r = lambda: random.randint(0,255)
    for _ in range(num):
        colors_list.append((r(),r(),r(), 255 // 2))
    return colors_list

"""only for debug"""
def draw_everything(polygon, body, fixture, screen, object_box, PPM, reference_point, is_sensor):
    vertices = [(body.transform * v) * PPM for v in polygon.vertices]
    for vertice in vertices:
        vertice[0] = vertice[0]+reference_point[0]
        vertice[1] = vertice[1]+reference_point[1]
    centroid = body.position * PPM
    centroid[0] = centroid[0]+reference_point[0]
    centroid[1] = centroid[1]+reference_point[1]
    arrow_end = centroid +  body.linearVelocity * PPM/2
    if is_sensor:
        pygame.draw.polygon(screen, (255, 0, 0 , 255), vertices, 2)
    else:     
        pygame.draw.polygon(screen, (125, 125, 125, 255), vertices, is_sensor)

"""draw walls"""
def draw_wall(polygon, body, fixture, screen,  PPM, reference_point):
    vertices = [(body.transform * v) * PPM for v in polygon.vertices]
    for vertice in vertices:
        vertice[0] = vertice[0]+reference_point[0]
        vertice[1] = vertice[1]+reference_point[1]
    centroid = body.position * PPM
    centroid[0] = centroid[0]+reference_point[0]
    centroid[1] = centroid[1]+reference_point[1]
    arrow_end = centroid +  body.linearVelocity * PPM/2
    pygame.draw.polygon(screen, (0, 0, 0, 255), vertices, 2)

"""draw agents"""
def draw_agent(polygon, body, fixture, screen, agent, PPM, workspace_dimension, reference_point, show_agent_details, is_sensor, debug_color = (125, 125, 125, 255)):
    vertices = [(body.transform * v) * PPM for v in polygon.vertices]
    for vertice in vertices:
        vertice[0] = vertice[0]+reference_point[0]
        vertice[1] = vertice[1]+reference_point[1]
    centroid = body.position * PPM
    centroid[0] = centroid[0]+reference_point[0]
    centroid[1] = centroid[1]+reference_point[1]
    arrow_end = centroid +  body.linearVelocity * PPM/2
    agent_color = (125, 125, 125, 255)
    font=pygame.font.SysFont("Calibri",int(20*PPM/30.0))
    font.set_bold(True)
    if is_sensor:
        pygame.draw.polygon(screen, (255, 0, 0 , 255), vertices, 2)
    else:
        if show_agent_details:
            agent_color = debug_color
        pygame.draw.polygon(screen, agent_color, vertices, is_sensor)
        pygame.draw.line(screen, (0, 255, 0, 255), centroid, arrow_end, 3)
        pygame.draw.circle(screen, (0, 255, 0, 255), [int(arrow_end[0]) , int(arrow_end[1])], 4)
        text_obj=font.render(str(agent.id), True, (255,255,0)) #+str(agent.state)[11:]
        text_pos=text_obj.get_rect()
        text_pos.center=centroid
        screen.blit(text_obj,text_pos)
    #Agent status
    #Agent ID, current position, State
    if show_agent_details:
        # show agent's path
        def rectify_vertex_position(vertex):
            # Handle both Point objects and tuples
            x = vertex[0] if isinstance(vertex, tuple) else vertex.x
            y = vertex[1] if isinstance(vertex, tuple) else vertex.y
            return [int(x*PPM+reference_point[0]), int(y*PPM+reference_point[1])]

        text_obj2=font.render("Agent"+str(agent.id)+":("+str(int(agent.position.x))+","+str(int(agent.position.y))+
        ") STATE:"+str(agent.state)[11:],True, (0,0,0))    #STATE:"+str(agent.state)[11:]+'        DEST:'+str(agent.destination_location), True, (0,0,0)) #+str(agent.state)[11:]
        text_pos2 = text_obj2.get_rect()
        workspace_height = workspace_dimension[1]
        text_pos2.topleft=(PPM+PPM*10*(agent.id//10)+reference_point[0],workspace_height+PPM*4+PPM*(agent.id%10)+reference_point[1])
        screen.blit(text_obj2,text_pos2)
        #Agent Destination
        text_obj3=font.render("DEST: None",True, (0,0,0))
        if agent.destination_location:
            text_obj3 = font.render("DEST: ("+ str(int(agent.destination_location.x))+","+str(int(agent.destination_location.y))+")",True, (0,0,0))
            pygame.draw.circle(screen, agent_color, rectify_vertex_position(agent.destination_location), 7, 0)
        text_pos3 = text_obj3.get_rect()
        workspace_height = workspace_dimension[1]
        text_pos3.topleft=(PPM+PPM*10*(agent.id//10)+reference_point[0],workspace_height+PPM*4.5+PPM*(agent.id%10)+reference_point[1])
        screen.blit(text_obj3,text_pos3)

        for pose in agent.sequence_of_poses:
            pygame.draw.circle(screen, agent_color, rectify_vertex_position(pose), 4, 0)
        for internal in agent.internal_stations:
            pygame.draw.circle(screen, agent_color, rectify_vertex_position(internal), 4, 0)


class Visualisation:
    def __init__(self, simulator, config_data):
        self.simulator = simulator
        self.pixels_per_meter = config_data['visualization']['pixels_per_meter']
        self.grids_num_per_meter = config_data["environment"]["resolution"]
        self.screen_width = config_data['visualization']['pygame_screen_width']
        self.screen_height = config_data['visualization']['pygame_screen_height']
        self.workspace_dimension = self.get_workspace_dimension_by_workspace(
            self.simulator.environment.width_in_meters,
            self.simulator.environment.height_in_meters)
        self.workspace_width = self.workspace_dimension[0]
        self.workspace_height = self.workspace_dimension[1]
        self.reference_point = (self.pixels_per_meter / self.grids_num_per_meter*2, self.pixels_per_meter / self.grids_num_per_meter*2)
        #All font size should be the size * PPH/30 because the default PPH is 30
        self.font_scale = self.pixels_per_meter/30.0
        self.clicked_agent = None
        self.clicked = False
        self.choose = False
        self.clicked_pos = None
        self.release_pos = None
        self.chosen_agents = []

        self.debug_colors = rnd_color(100)

    def run(self):
        pygame.init()
        # TARGET_FPS = self.simulator.TARGET_FPS
        # Need set screen width and height by different PC screen
        win = pygame.display
        screen =  win.set_mode(
                [self.screen_width, self.screen_height]) # , pygame.RESIZABLE: for macOS
        img = pygame.image.load('logo/logo.png')
        win.set_icon(img)
        win.set_caption(self.simulator.title, ('logo.png'))
        # clock = pygame.time.Clock()
        self.__terminal_print()
        pygame.font.init()
        polygonShape.draw = draw_agent
        config_data = self.__json_decoder_environment_data()
        show_agent = config_data['visualization']['agents']
        running = True
        show_grid = True
        show_control_range = False
        pause_simulator = True
        show_agent_details = self.simulator.agent_details # default False
        DEBUG_MODE = False
        show_sensor_range = False
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == KEYDOWN:
                    if event.key == 113 or event.key == 81: # Q/q is pressed
                        running = False
                        break
                    elif event.key == 100 or event.key == 68: #D/d is pressed
                        show_agent_details = not show_agent_details
                    elif event.key == 83 or event.key == 115: # S/s is pressed 
                        show_sensor_range = not show_sensor_range
                    elif event.key == 103 or event.key == 71: #G/g is pressed
                        show_grid = not show_grid
                    elif event.key == 112 or event.key == 80:
                        pause_simulator = not pause_simulator
                    elif event.key == 114 or event.key == 82: #R/r is pressed
                        show_control_range = not show_control_range
                    elif event.key == 66 or event.key == 98: #B/b is pressed
                        DEBUG_MODE = not DEBUG_MODE
                
                self.mouse_event_handler(event, screen)
            (running, update) = controller_cmd_handler(self.simulator, running)

            # if show_sensor_range:
            #     self.draw_raycast(screen)
            if not DEBUG_MODE:
                self.draw_everything(screen, show_grid, show_control_range, show_agent, show_agent_details, show_sensor_range)
            else:
                # Debug Mode
                for body in self.simulator.world.bodies:
                    if body.userData.type == 'wall':
                        polygonShape.draw = draw_wall
                        for fixture in body.fixtures:
                            fixture.shape.draw(body, fixture, screen, self.pixels_per_meter, self.reference_point)
                    else:
                        polygonShape.draw = draw_everything
                        for a in self.simulator.b2_objects.keys():
                            if self.simulator.b2_objects[a]==body:
                                box_object = self.simulator.agents[a]
                        for fixture in body.fixtures:
                            if not fixture.sensor:
                                fixture.shape.draw(body, fixture, screen, box_object, self.pixels_per_meter, self.reference_point, False)
                            if show_sensor_range and fixture.sensor:
                                fixture.shape.draw(body, fixture, screen, box_object, self.pixels_per_meter, self.reference_point, True)
            for contact in self.simulator.world.contacts:
                """the touching flag will be True, only when the objects got collision"""
                if contact.touching == True:
                    if contact.fixtureA.userData == Agent or contact.fixtureB.userData == Agent:
                        print contact.fixtureA.userData, contact.fixtureB.userData
                    n = contact.manifold.pointCount
                    if n > 0:
                        for point in contact.worldManifold.points[:n]:
                            self.draw_contact(screen, point)

            font1 = pygame.font.SysFont("arial",int(20*self.font_scale))
            try:
                PPH = int(self.simulator.task_count/self.simulator.get_simulator_time()*3600)
            except:
                PPH=0

            text_obj1=font1.render(
                "Time in simulation world (unit:sec): " + str(int(self.simulator.get_simulator_time())) +
                " Packages Delivered: " + str(self.simulator.task_count),
                True, (0,0,0))
            text_pos1=text_obj1.get_rect()
            text_pos1.center=(self.screen_width/2, self.workspace_height+self.pixels_per_meter+self.reference_point[1])
            screen.blit(text_obj1,text_pos1)
            text_obj2=font1.render(
                " Estimated PPH: "+ str(PPH)+
                " Number of Agents: "+str(len(self.simulator.agents)),
                True, (0,0,0))
            text_pos2=text_obj2.get_rect()
            text_pos2.center=(self.screen_width/2, self.workspace_height + 2*self.pixels_per_meter + self.reference_point[1])
            screen.blit(text_obj2,text_pos2)
            try:
                port = self.simulator.environment.loading_ports.values()[0]
                text_obj3=font1.render("Parameters: Port Process Time:" +
                    str(port.operation_time_in_secs),
                    True, (0,0,0))
                text_pos3=text_obj3.get_rect()
                text_pos3.center=(self.screen_width/2, self.workspace_height+3*self.pixels_per_meter+self.reference_point[1])
                screen.blit(text_obj3,text_pos3)
            except:
                pass
            pygame.display.flip()
            while pause_simulator:
                update = False
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                        pause_simulator = False
                        break
                    elif event.type == KEYDOWN:
                        if event.key == 113 or event.key == 81: # Q/q is pressed
                            running = False
                            pause_simulator = False
                            break
                        elif event.key == 112 or event.key == 80:#P/p is pressed
                            pause_simulator = not pause_simulator
                        elif event.key == 100 or event.key == 68: #D/d is pressed
                            show_agent_details = not show_agent_details
                        elif event.key == 83 or event.key == 115: # S/s is pressed 
                            show_sensor_range = not show_sensor_range
                        elif event.key == 103 or event.key == 71: #G/g is pressed
                            show_grid = not show_grid
                        elif event.key == 112 or event.key == 80:
                                pause_simulator = not pause_simulator
                        elif event.key == 114 or event.key == 82: #R/r is pressed
                            show_control_range = not show_control_range
                        elif event.key == 66 or event.key == 98: #B/b is pressed
                            DEBUG_MODE = not DEBUG_MODE
                    update = update or self.mouse_event_handler(event, screen)
                (running, update2) = controller_cmd_handler(self.simulator, running)
                if not running:
                    break
                update = update2 or update
                if update:
                    self.draw_everything(screen, show_grid, show_control_range, show_agent, show_agent_details, show_sensor_range)
                    pygame.display.flip()
            self.simulator.step()

    def draw_everything(self, screen, show_grid, show_control_range, show_agent, show_agent_details, show_sensor_range):
        '''non debug mode'''
        screen_color = (216, 222, 228, 0)
        screen.fill(screen_color)
        self.draw_workspace(screen)
        self.draw_grid(show_grid, screen, (0,50,50))
        # draw loading and unloading ports
        for port in self.simulator.environment.loading_ports.values():
            self.draw_loading_port(screen, port)
            self.draw_control_range(screen, port, show_control_range)
            if port.has_queue_area:
                self.draw_queuing_region(screen, port.queue)
        for port in self.simulator.environment.unloading_ports.values():
            self.draw_unloading_port(screen, port)
            self.draw_control_range(screen, port, show_control_range)
            if port.has_queue_area:
                self.draw_queuing_region(screen, port.queue)
        # draw physical bodies
        for body in self.simulator.world.bodies:
            if body.userData.type == 'wall':
                polygonShape.draw = draw_wall
                for fixture in body.fixtures:
                    fixture.shape.draw(body, fixture, screen, self.pixels_per_meter, self.reference_point)
            if body.userData.type == 'obstacle':
                x_in_pixels = int(ceil(body.userData.location.x * self.pixels_per_meter))
                y_in_pixels = int(ceil(body.userData.location.y * self.pixels_per_meter))
                depth_dimension_in_pixels = int(ceil(body.userData.dimension[1] * self.pixels_per_meter))
                width_dimension_in_pixels = int(ceil(body.userData.dimension[0] * self.pixels_per_meter))
                self.draw_rectangle(screen, x_in_pixels, y_in_pixels, depth_dimension_in_pixels, width_dimension_in_pixels, (0, 0, 0))
            
            for a in self.simulator.b2_objects.keys():
                polygonShape.draw = draw_agent
                if self.simulator.b2_objects[a]== body:
                    agent=self.simulator.agents[a]
                    if show_agent:
                        for fixture in body.fixtures:
                            if fixture and not fixture.sensor:
                                fixture.shape.draw(body, fixture, screen, agent, self.pixels_per_meter,
                                    self.workspace_dimension, self.reference_point, show_agent_details, False, self.debug_colors[agent.id])
                            if show_sensor_range and fixture.sensor:
                                fixture.shape.draw(body, fixture, screen, agent, self.pixels_per_meter,
                                    self.workspace_dimension, self.reference_point, show_agent_details, True, self.debug_colors[agent.id])

        if self.choose and not self.clicked:
            pos = ((pygame.mouse.get_pos()[0] - self.reference_point[0]*1.0) / self.pixels_per_meter,
                (pygame.mouse.get_pos()[1] - self.reference_point[1]*1.0) / self.pixels_per_meter)
            self.draw_rectangle(screen, 
                self.meter_to_pixel(self.clicked_pos[0]), 
                self.meter_to_pixel(self.clicked_pos[1]), 
                self.meter_to_pixel(pos[0] - self.clicked_pos[0]),
                self.meter_to_pixel(pos[1] - self.clicked_pos[1]), (0, 0, 0, 0), 1)

    def mouse_event_handler(self, event, screen):
        # added new feature here. The agent can be moved by mouse now
        if event.type == pygame.MOUSEBUTTONDOWN:
            pos = ((pygame.mouse.get_pos()[0] - self.reference_point[0]*1.0) / self.pixels_per_meter,
                (pygame.mouse.get_pos()[1] - self.reference_point[1]*1.0) / self.pixels_per_meter)
            clicked_agents = [agent for agent in self.simulator.agents if agent.is_clicked(pos)]
            clicked_ports = [port for port in self.simulator.loading_ports if port.is_clicked(pos)]
            clicked_ports += [port for port in self.simulator.unloading_ports if port.is_clicked(pos)]
            # left-click on agent, ready to be moved
            if clicked_agents != None and event.button == 1:
                for agent in clicked_agents:
                    self.clicked_agent = self.simulator.b2_objects[agent.get_id()] 
                    self.clicked = True
            # left-click not on agent -> create rectangle selection
            if event.button == 1 and self.choose == False and not self.clicked: 
                self.clicked_pos = pos
                self.chosen_agents = []
                self.release_pos = None
                self.choose = True
            # right-click to select internal station for agents
            if event.button == 3 and len(self.chosen_agents) > 0 and not self.clicked:
                for agent in self.chosen_agents:
                    if not agent.global_planner==None and agent.global_planner.MAP == MapType.GRID: # wrap to grid
                        temp_tuple = float_to_grid(pos, self.grids_num_per_meter)
                        internal_station = Point(temp_tuple[0], temp_tuple[1])
                    else:
                        internal_station = Point(pos[0], pos[1])
                    if not self.simulator.free_control:
                        # story mode, agents have original destination before user assignment
                        # check if it is the first time give internal station to an agent
                        if len(agent.internal_stations) == 0:
                            # remember the original last station
                            agent.internal_stations.append(agent.destination_location.copy())
                            # set new destination location
                            agent.destination_location = internal_station
                            agent.task.destination_location = internal_station
                            agent.goal_changed = True
                        else:
                            # put the new position as internal as the second last goal; (last goal is the orginal goal from task)
                            agent.internal_stations.insert(-1, internal_station)
                    else:
                        # listen to user's command
                        if agent.destination_location == None:
                            agent.destination_location = internal_station
                            agent.task.destination_location = internal_station
                            agent.goal_changed = True
                        else:
                            agent.internal_stations.append(internal_station)
                    
                self.release_pos = None
                self.clicked_pos = None
            # right-click shows port info
            if clicked_ports != None and event.button == 3:
                for port in clicked_ports:
                    print "--------------Port {}------------".format(port.identifier)
                    for key in port.queue.agent_slot_map:
                        print "agent id", key
                        print "should go slot", port.queue.slots.index(port.queue.agent_slot_map[key])
                        print "slot position", port.queue.agent_slot_map[key].x, port.queue.agent_slot_map[key].y
                        agent = self.simulator.agents[key]
                        print "agent position", agent.position
                        print "agent destination", agent.destination_location
                        print "agent state", agent.state
                        print "agent local planner: ", agent.current_local_planner
                        print ""
                print "--------------------------------"
            # right-click shows agent info
            if clicked_agents != None and event.button == 3:
                for agent in clicked_agents:
                    print "--------------------------------"
                    print "agent id", agent.id
                    print "agent position", agent.position
                    print "agent destination", agent.destination_location
                    print "agent state", agent.state
                    print "arrive destination?"
                    if agent.position.distance(agent.destination_location) < 5e-3:
                        print "yes"
                    else:
                        print "no"
                    for obj in agent.sensor.visible_object:
                        try:
                            print "detected objects: ", obj.userData.type, "id: ", obj.userData.id
                        except:
                            print "None "
                    print "agent goal vector", agent.get_linear_velocity()[0], agent.get_linear_velocity()[1]
                    print "agent goal angle in degree", degrees(atan2(
                        agent.get_linear_velocity()[1], agent.get_linear_velocity()[0])),'in radius:', atan2(agent.get_linear_velocity()[1], agent.get_linear_velocity()[0])
                    agent_body = self.simulator.b2_objects[agent.get_id()]
                    print "current angle in degree", degrees(agent_body.angle), 'in radius:',agent_body.angle
                    print "agent local planner: ", agent.current_local_planner
                    print "agent global planner: ", agent.global_planner
                    print "current angular velocity in degree", degrees(agent_body.angularVelocity), 'in radius:',agent_body.angularVelocity
                    print "replan(True or False):", agent.replan
                    print "--------------------------------"
            print "{}: ({:.2f}, {:.2f})".format(CLICKED_FLAG, pos[0], pos[1])

        if event.type == pygame.MOUSEBUTTONUP:
            pos = ((pygame.mouse.get_pos()[0] - self.reference_point[0]*1.0) / self.pixels_per_meter,
                (pygame.mouse.get_pos()[1] - self.reference_point[1]*1.0) / self.pixels_per_meter)
            # compute which agent in rectangle
            if len(self.chosen_agents) == 0 and not self.clicked and self.choose and not self.release_pos:
                self.release_pos = pos
                self.choose = False
                self.chosen_agents = [agent for agent in self.simulator.agents if agent.is_in_range(self.clicked_pos, self.release_pos)]
            else:
                if self.clicked_agent:
                    self.clicked_agent.position = pos
                    self.simulator.agents[self.clicked_agent.userData.id].goal_changed = True
                self.clicked = False
                self.clicked_agent = None
            return True
        if event.type == pygame.MOUSEMOTION:
            if self.clicked:
                pos = ((pygame.mouse.get_pos()[0] - self.reference_point[0]*1.0) / self.pixels_per_meter,
                    (pygame.mouse.get_pos()[1] - self.reference_point[1]*1.0) / self.pixels_per_meter)
                if self.clicked_agent:
                    self.clicked_agent.position = pos
                else:
                    print "clicked position is ", pos
                return True
            if self.choose and not self.clicked:
                return True
        return False

    """
    Read pygame config from Config.json
    """
    def get_workspace_dimension_by_workspace(self, depth_ws, width_ws):
        return (int( (depth_ws+1) * self.pixels_per_meter), int((width_ws + 1) * self.pixels_per_meter))
    def meter_to_pixel(self, value):
        return int(ceil(value * self.pixels_per_meter))
    """
    Show grids on the simulator
    """
    def draw_grid(self, show_grid, screen, color):
        pixels_per_grid = self.pixels_per_meter / self.grids_num_per_meter
        if show_grid:
            for i in range((self.workspace_height-self.pixels_per_meter) // pixels_per_grid):
                pygame.draw.line(screen, color, [self.reference_point[0], i * pixels_per_grid+self.reference_point[1]],
                    [self.workspace_width - self.pixels_per_meter + self.reference_point[0],
                    i * pixels_per_grid + self.reference_point[1]], 1)
            for j in range((self.workspace_width-self.pixels_per_meter) // pixels_per_grid):
                pygame.draw.line(screen, color, [j * pixels_per_grid + self.reference_point[0], self.reference_point[1]],
                    [j * pixels_per_grid + self.reference_point[0],
                    self.workspace_height - self.pixels_per_meter +  self.reference_point[1]], 1)
        else:
            pass
    
    """draw ray cast for debug"""
    def draw_raycast(self, screen):
        points_list = self.simulator.ray_line_list
        color = (0, 0, 0, 255)
        for points in points_list:
            line = [(points[0][0]  * self.pixels_per_meter + self.reference_point[0], points[0][1]* self.pixels_per_meter + self.reference_point[1]), 
            (points[1][0] * self.pixels_per_meter + self.reference_point[0], points[1][1] *self.pixels_per_meter + self.reference_point[1]) ]
            pygame.draw.lines(screen, color, True, line, 3)


    """
    Draw rectangular workspace area
    TODO draw polygon workspace in future
    """
    def draw_workspace(self, screen):
        color = (0, 0, 0, 255)
        workspace_point_list = [self.reference_point, (self.reference_point[0], self.workspace_height - self.pixels_per_meter + self.reference_point[1]),
        (self.workspace_width - self.pixels_per_meter + self.reference_point[0], self.workspace_height - self.pixels_per_meter + self.reference_point[1]),
        (self.workspace_width - self.pixels_per_meter + self.reference_point[0], self.reference_point[1])]
        pygame.draw.lines(screen, color, True, workspace_point_list, 1)
    """
    Draw control range of ports
    """
    def draw_control_range(self, screen, port, show_control_range):
        color1 = (255, 14, 15, 255)
        color2 = (15, 14, 255 , 255)
        if show_control_range:
            pygame.draw.circle(screen,
                color1,
                (self.meter_to_pixel(port.queue.region.x) + self.reference_point[0], self.meter_to_pixel(port.queue.region.y) + self.reference_point[1]),
                self.meter_to_pixel(port.control_range),
                2)
            pygame.draw.circle(screen,
                color2,
                ( self.meter_to_pixel(port.queue.region.x)+self.meter_to_pixel(port.queue.region.width)+self.reference_point[0],
                    self.meter_to_pixel(port.queue.region.y)+self.reference_point[1]),
                self.meter_to_pixel(port.control_range),
                2)
        else:
            pass


    """
    TODO change all draw functions to draw on grid map
    """
    def draw_rectangle(self, screen, x, y, depth, width, color, fill = 0):
        pygame.draw.rect(screen, color, Rect(x + self.reference_point[0], y + self.reference_point[1], depth, width), fill)

    def draw_loading_port(self, screen, port):
        self.draw_rectangle(
            screen,
            self.meter_to_pixel(port.location.x),
            self.meter_to_pixel(port.location.y),
            self.meter_to_pixel(port.dimension[0]),
            self.meter_to_pixel(port.dimension[1]),
            (2, 126, 72, 255))
        self.draw_port_id(screen, port)
    def draw_unloading_port(self, screen, port):
        x = port.location.x
        y = port.location.y
        self.draw_rectangle(
            screen,
            self.meter_to_pixel(x),
            self.meter_to_pixel(y),
            self.meter_to_pixel(port.dimension[0]),
            self.meter_to_pixel(port.dimension[1]),
            (205, 44, 18, 255))
        self.draw_port_id(screen, port)
        self.draw_port_id(screen, port)
    def draw_queuing_region(self, screen, queue):
        rect = Rect(
            self.meter_to_pixel(queue.region.x)+self.reference_point[0],
            self.meter_to_pixel(queue.region.y)+self.reference_point[1],
            self.meter_to_pixel(queue.region.width),
            self.meter_to_pixel(queue.region.height)
            )
        pygame.draw.rect(screen, (255,0, 0, 255), rect, 1)
    #0.5 is for the grids_num_per_meter  is 1
    def draw_port_id(self, screen, port):
        font=pygame.font.SysFont("Calibri",int(40*self.font_scale))
        font.set_bold(True)
        identifier = port.identifier
        text_obj=font.render(str(identifier), True, (255,255,255))
        text_pos=text_obj.get_rect()
        text_pos.center = (self.meter_to_pixel(port.location.x + port.dimension[0] /2.0) + self.reference_point[0],
            self.meter_to_pixel(port.location.y + port.dimension[0] /2.0) + self.reference_point[1])
        screen.blit(text_obj,text_pos)

    def draw_contact(self, screen, pos):
        cx = int(self.meter_to_pixel(pos[0]) + self.reference_point[0])
        cy = int(self.meter_to_pixel(pos[1]) + self.reference_point[1])
        pygame.draw.circle(screen, (255, 0, 0), (cx, cy), 5, 0)

    def __json_decoder_environment_data(self):
        with open('config.json') as file:
            config_data = json.load(file)
        return config_data
    def __terminal_print(self):
        print ("")
        print ("--------------------------------------------------------------------------")
        print ("The Blue Port is loading port, and the Red Port is unloading ports")
        print ("--------------------------------------------------------------------------")
        print ("press q to exit")
        print ("press g to show/hide the grid map ")
        print ("press s to show/hide the range of radar")
        print ("press r to show/hide the control range of ports")
        print ("press d to show/hide the agents' details")
        print ("Red circle is the start of the control range of ports")
        print ("Blue circle is the end of the control range of ports")
        print ("Press b to OPEN DEBUG MODE, which draw everything from BOX2D")
        print ("press p to pause the simulator")
