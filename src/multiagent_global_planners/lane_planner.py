#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Aug 17 17:26:55 2018
@author: {chong.chen2, xiaoyu.ge}@dorabot.com
"""
from geometry import Point, compute_direction
from math import sqrt
class LanePlanner:
    def __init__(self, environment):
        """ Setup lanes given an environment """
        self.__setup_lanes(environment)
    def enforce_rules(self):
        return
    def __setup_lanes(self, environment):
        """ Setup lanes optimised for an environment
        Keyword arguments:
        environment -- an instance of Environment depciting the current workspace
                       Note, it is not necessarily to be a gridmap
        """ 

# class LaneGlobalPlanner(Global_Palnner):
#     def path(self,agent,sim):
#         start=agent.position
#         stop=agent.destination_location
#         tmp_path=[]
#         crowdy=[]
#         if start.x>stop.x and stop.y>start.y:
#             for y in range(8, 12):
#                 tmp_path.append([Point(start.x-1, start.y+1), Point(start.x-1, y),
#                         Point(stop.x+1,y), Point(stop.x+1, stop.y-1), stop])
#                 crowdy.append(crowdiness(y,sim))
#         elif start.x>stop.x and stop.y<=start.y:
#             for y in range(8, 12):
#                 tmp_path.append([Point(start.x+1, start.y-1), Point(start.x+1, y), Point(stop.x,y), stop])
#                 crowdy.append(crowdiness(y,sim))
#         elif start.x<stop.x and stop.y>start.y:
#             for y in range(4, 8):
#                 tmp_path.append([Point(start.x-1, start.y+1),Point(start.x-1, y),Point(stop.x-1,y),Point(stop.x-1,stop.y-1),stop])
#                 crowdy.append(crowdiness(y,sim))
#         else:
#             for y in range(4, 8):
#                 tmp_path.append([Point(start.x+1, start.y-1),Point(start.x+1, y),Point(stop.x,y),stop])
#                 crowdy.append(crowdiness(y,sim))
#         min_crowd=min(crowdy)
#         idx=crowdy.index(min_crowd)
#         return tmp_path[idx]
#
#     def crowdiness(self.y,sim):
#         count=0
#         for a in sim.agents:
#             if abs(a.position.y-y)<0.1 and abs(a.linear_velocity[1])<0.1:
#                 count+=1
#         return count
#
#     def policy0(agent1, agent2):
#         if agent1.id<agent2.id:
#             agent2.stop()
#         else:
#             agent1.stop()
#         return
#
#     def policy1(agent1, agent2):
#         v1=agent1.linear_velocity
#         speed1=v1[0]**2+v1[1]**2    #actually speed1=sqrt(...), but doesn't matter for our comparison here
#         v2=agent2.linear_velocity
#         speed2=v2[0]**2+v2[1]**2
#         if speed2<speed1:
#             agent2.stop()
#         else:
#             agent1.stop()
#         return
#
#     def policy2(agent1, agent2):
#         v1=agent1.linear_velocity
#         speed1=v1[0]**2+v1[1]**2    #actually speed1=sqrt(...), but doesn't matter for our comparison here
#         v2=agent2.linear_velocity
#         speed2=v2[0]**2+v2[1]**2
#         rel_p=(agent2.position.x-agent1.position.x, agent2.position.y-agent1.position.y)
#
#         #Current Case
#         rel_v=(v1[0]-v2[0], v1[1]-v2[1])
#         rel_speed_square=rel_v[0]**2+rel_v[1]**2
#         tmp=(rel_p[0]*rel_v[0]+rel_p[1]*rel_v[1])
#
#         #agent1 stops:
#         # rel_v = -v2
#         # rel_speed=speed2
#         #tmp1=-(rel_p[0]*v2[0]+rel_p[1]*v2[1])
#
#         #agent2 stops:
#         # rel_v = v1
#         # rel_speed=speed1
#         #tmp2=(rel_p[0]*v1[0]+rel_p[1]*v1[1])
#         try:
#             comparison=(v2[0]*rel_p[1]-v2[1]*rel_p[0])/(v1[0]*rel_p[1]-v1[1]*rel_p[0])
#         except:                 #Happens when agent2 is right on agent1's current path
#             return
#             #print("In My Way")
#             #if speed2>0.5:
#             #    return
#             #else:
#             #    agent1.stop()
#
#         '''
#         if tmp<0 or rel_speed_square==0:            #won't collide
#             return
#         else:
#             min_dist=tmp/rel_speed_square
#         if min_dist > 1:                          #won't collide
#             return
#         '''
#         if comparison>0.8:
#             agent1.potential_collision=True
#             agent1.stop()
#         #    return
#         #else:
#         #    agent2.potential_collision=True
#         #    agent2.stop()
#         #    return
#         return
