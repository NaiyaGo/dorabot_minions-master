# -*- coding: utf-8 -*-
"""
@Copyright Dorabot Inc.
@date : 2018-10
@author: tian.xiao@dorabot.com
@brief : physical sensor on the agent 
"""
from math import *
from Box2D import b2ContactListener, b2RayCastCallback
class Sensor(object):
    def __init__(self, radius, angle, location, shape, vertices_num = 16):
        self.radius = radius
        self.angle = angle
        self.location = location 
        self.vertices_num = vertices_num
        self.vertices = []
        self.__set_vertices()
        self.shape = shape(vertices = self.vertices)
        self.visible_object = []

    def __set_vertices(self):
        self.vertices.append((0,0)) 
        for i in range(self.vertices_num - 1):
            angle = self.angle / (self.vertices_num - 2) * i
            self.vertices.append((self.radius * sin(angle), self.radius*cos(angle)))
    def radar_acquired_object(self, agent):
        self.visible_object.append(agent)
    def radar_lost_object(self, agent):
        self.visible_object.remove(agent)


'''RayCast for agents sensor'''
class SensorRayCast(b2RayCastCallback):
    def __init__(self):
        b2RayCastCallback.__init__(self)
        self.fixture = None
        self.hit = False

    def ReportFixture(self, fixture, point, normal, fraction):
        if fixture.sensor:
            return -1
        else:
            self.hit = True
            self.fixture = fixture
            self.point = point
            self.normal = normal        
            return 0.0



'''This is a contact Listener for sensor using'''
class SensorContactListener(b2ContactListener):
    '''override the begin contact of Box2D'''
    def BeginContact(self, contact):
        is_detected, detected_object, radar = self.__get_radar_and_object(contact)
        if is_detected:
            radar.radar_acquired_object(detected_object)

    def EndContact(self, contact):
        is_detected, detected_object, radar = self.__get_radar_and_object(contact)
        if is_detected:
            if type(detected_object) == 'Agent':
                print type(detected_object)
            radar.radar_lost_object(detected_object)
            
    '''This function is a filter to get the object which is not sensor '''
    def __get_radar_and_object(self, contact):
        fixtureA = contact.fixtureA
        fixtureB = contact.fixtureB
        if not (fixtureA.sensor ^ fixtureB.sensor):
            return False, None, None
        else:
            if not fixtureA.sensor:
                return True, fixtureA.body, fixtureB.userData
            else:
                return True, fixtureB.body, fixtureA.userData
