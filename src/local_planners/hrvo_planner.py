#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@data 14:04:32 2018
@author: chong.chen2@dorabot.com
@breif: hrvo for Differential Drive Model, from the following paper:
    Smooth and collision-free navigation for multiple robots under differential-drive constraints,
    by Snape, Jamie Van Den Berg, Jur Guy, Stephen J. Manocha, Dinesh
    IROS 2010
    https://github.com/snape/HRVO/tree/master/src
"""
from math import ceil, floor, sqrt
import copy
from math import cos, sin, tan, atan2, asin, fmod
from math import pi as PI
from geometry import *
from agents.agent_state_machine import AgentState
from local_planner import LocalPlanner
from rvo_planner import compute_V_des, compute_rvo_BA, norm


class HRVOPlanner(LocalPlanner):
    def compute_plan(self, position, velocity, gridmap, sensor_observation, global_planner_path):
        
        if self.agent.destination_location == None: # or self.agent.state == AgentState.QUEUING:
            return [0,0]
        agentA = self.agent
        pA = agentA.position
        vA = agentA.linear_velocity
        vA_des = compute_V_des(self.agent, global_planner_path)
        rA = agentA.shape.get_radius()
        RVO_BA_all = []

        for agentB in sensor_observation.other_agents_state_in_range_of(3, PI/2):
            # TODO Gary: @Gary do not modify agent status in hidden functions
            self.agent.potential_collision = True
            #if agentB.state==AgentState.CRUISE:
            pB = agentB.position
            vB = agentB.linearVelocity
            rB = agentB.userData.shape.get_radius()
            if agentB.userData.destination_location == None or self.agent.state == AgentState.QUEUING:
                RVO_BA = compute_rvo_BA(pA, vA, rA, pB, vB, rB, reciprocal = False)
            else:
                RVO_BA = compute_hrvo_DD_BA(agentA, agentB.userData)
            RVO_BA_all.append(RVO_BA)


        for body in sensor_observation.ports_in_range_of(2, PI/2):
            self.agent.potential_collision = True
            obstacle = body.userData
            pB = Point(obstacle.location.x+obstacle.dimension[0]*0.5,obstacle.location.y+obstacle.dimension[1]*0.5)
            vB = [0,0]
            rB = pow(obstacle.dimension[0]**2+obstacle.dimension[1]**2,0.5)/2
            VO_BA = compute_rvo_BA(pA,vA,rA, pB, vB, rB, reciprocal = False)
            RVO_BA_all.append(VO_BA)
            
        for wall in sensor_observation.walls_in_range_of(2, PI/2):
            self.agent.potential_collision = True
            obstacle = wall.userData
            pB = Point(obstacle.location.x+obstacle.dimension[0]*0.5, obstacle.location.y+obstacle.dimension[1]*0.5)
            #vel_A_B_dir = compute_direction(position, pB)
            vB = [0,0]
            rB = pow(obstacle.dimension[0]**2+obstacle.dimension[1]**2,0.5)/2
            VO_BA = compute_rvo_BA(pA,vA,rA, pB, vB, rB, reciprocal = False)
            RVO_BA_all.append(VO_BA)
            
        if RVO_BA_all != []:
            result_vel = intersect(agentA, RVO_BA_all)
        else:
            result_vel = vA_des
           
        speed, w = compute_angular_velocity(velocity, result_vel)
        
        return speed, w


def compute_hrvo_DD_BA(agentA, agentB):
    """ Rewritten from Snape's Agent.cpp, Line 107-128"""
    uncertaintyOffset = 0.0
    # Must match simulator steps_per_sec in config.json (60 Hz)
    TIME_STEP = 1.0/60
    
    rA = agentA.shape.get_radius()
    rB = agentB.shape.get_radius()
    pA = agentA.position
    pB = agentB.position
    vA = agentA.linear_velocity
    vB = agentB.linear_velocity
    
    sum_r = rA + rB
    dist_BA = pA.distance(pB)
    
    if dist_BA > sum_r:
        theta_BA = atan2(pB.y-pA.y, pB.x-pA.x)       # Line 108, angle
        theta_BAort = asin(sum_r/dist_BA)            # Line 109, openingAngle
        theta_ort_left = theta_BA+theta_BAort
        bound_left = [cos(theta_ort_left), sin(theta_ort_left)]     # Line 111, velocityObstacle.side1_
        theta_ort_right = theta_BA-theta_BAort
        bound_right = [cos(theta_ort_right), sin(theta_ort_right)]  # Line 112, velocityObstacle.side2_
        
        d = sin(2.0*theta_BAort)                     # Line 114, d = 2 * sin(openingAngle) * cos(openingAngle)
        vA_des=compute_V_des(agentA, agentA.sequence_of_poses)          
        vB_des=compute_V_des(agentB, agentB.sequence_of_poses)
    
        det = (pB.x-pA.x)*(vA_des[1]-vB_des[1])-(pB.y-pA.y)*(vA_des[0]-vB_des[0])  # Line 116
        if det >0:
            s = 0.5*((vA[0]-vB[0])*bound_right[1]-(vA[1]-vB[1])*bound_right[0])/d   # Line 117
            apex = [vB[0] + s * bound_left[0]-(uncertaintyOffset/sum_r) * (pB.x-pA.x),
               vB[1] + s * bound_left[1]-(uncertaintyOffset/sum_r) * (pB.y-pA.y)] # Line 119
        else:
            s = 0.5*((vA[0]-vB[0])*bound_left[1]-(vA[1]-vB[1])*bound_left[0])/d
            apex = [vB[0] + s * bound_right[0]-(uncertaintyOffset/sum_r) * (pB.x-pA.x),
               vB[1] + s * bound_right[1]-(uncertaintyOffset/sum_r) * (pB.y-pA.y)] # Line 127
    
    else:    # dist_BA<=sum_r
        tmp = (uncertaintyOffset + 0.5* (sum_r-dist_BA)/TIME_STEP/dist_BA)
        apex = [0.5*(vB[0]+vA[0])-tmp*(pB.x-pA.x), 0.5*(vB[1]+vA[1])-tmp*(pB.y-pA.y)]
        
        bound_left = compute_direction(pA, pB)
        bound_right = (-bound_left[0], -bound_left[1])                                                   # Line 130-132
    HRVO_BA = [apex, bound_left, bound_right, dist_BA, sum_r]
    return HRVO_BA

def intersect(agentA, RVO_BA_all):
    candidates = []
    vo1_candidate = 1e6
    vo2_candidate = 1e6
    max_speed=agentA.cruise_speed
    vA_des = compute_V_des(agentA, agentA.sequence_of_poses)
    speed_des = pow(vA_des[0]**2+vA_des[1]**2,0.5)
    if speed_des < max_speed:                                       # Line 144-149
        velocity_candidate = vA_des        
    else:
        velocity_candidate = (vA_des[0]/speed_des * max_speed, 
                            vA_des[1]/speed_des * max_speed)
    candidate = (velocity_candidate, vo1_candidate, vo2_candidate, diff_sq(velocity_candidate, vA_des))
    candidates.append(candidate)                                                    # Line 151  
    
    for i in range(len(RVO_BA_all)):                                            # Line 153
        vo1_candidate = i
        vo2_candidate = i
        apex, bound_left, bound_right, dist_BA, sum_r = RVO_BA_all[i]
        diff = sub(vA_des, apex)
        dotProduct1 = diff[0] * bound_left[0] + diff[1] * bound_left[1]
        dotProduct2 = diff[0] * bound_right[0] + diff[1] * bound_right[1]
        det1 = bound_left[0] * diff[1] - bound_left[1] * diff[0]
        det2 = bound_right[0] * diff[1] - bound_right[1] * diff[0]
        if dotProduct1 > 0 and det1 > 0:                                         # Line 160-166
            velocity_candidate = (apex[0] + dotProduct1 * bound_left[0], apex[1] + dotProduct1 * bound_left[1])
            if norm(velocity_candidate) < max_speed:
                candidates.append((velocity_candidate, vo1_candidate, vo2_candidate, diff_sq(velocity_candidate, vA_des)))
        if dotProduct2 > 0 and det2 < 0:                                         # Line 168-174
            velocity_candidate = (apex[0] + dotProduct2 * bound_right[0], apex[1] + dotProduct2 * bound_right[1])
            if norm(velocity_candidate) < max_speed:
                candidates.append((velocity_candidate, vo1_candidate, vo2_candidate, diff_sq(velocity_candidate, vA_des)))
    
    for j in range(len(RVO_BA_all)):                                             # Line 177-
        vo1_candidate = 1e6
        vo2_candidate = j
        apex, bound_left, bound_right, dist_BA, sum_r = RVO_BA_all[j]
        
        det = apex[0] * bound_left[1] - apex[1] * bound_left[0]
        discriminant = max_speed**2-det**2
        
        if discriminant > 0:
            t1 = -(apex[0] * bound_left[0] + apex[1] * bound_left[1]) + pow(discriminant, 0.5)
            t2 = -(apex[0] * bound_left[0] + apex[1] * bound_left[1]) - pow(discriminant, 0.5)
        
            if t1 >= 0:                                                           # Line 188
                velocity_candidate = (apex[0] + t1 * bound_left[0], apex[1] + t1 * bound_left[1])
                candidates.append((velocity_candidate, vo1_candidate, vo2_candidate, diff_sq(velocity_candidate, vA_des)))
            if t2 >= 0:
                velocity_candidate = (apex[0] + t2 * bound_left[0], apex[1] + t2 * bound_left[1])
                candidates.append((velocity_candidate, vo1_candidate, vo2_candidate, diff_sq(velocity_candidate, vA_des)))
         
        det = apex[0] * bound_right[1] - apex[1] * bound_right[0]                  # Line 199
        discriminant = max_speed**2-det**2
        if discriminant > 0:
           t1 = -(apex[0] * bound_right[0] + apex[1] * bound_right[1]) + pow(discriminant, 0.5)
           t2 = -(apex[0] * bound_right[0] + apex[1] * bound_right[1]) - pow(discriminant, 0.5)
    
           if t1 >= 0:                                                           # Line 188
               velocity_candidate = (apex[0] + t1 * bound_right[0], apex[1] + t1 * bound_right[1])
               candidates.append((velocity_candidate, vo1_candidate, vo2_candidate, diff_sq(velocity_candidate, vA_des)))
           if t2 >= 0:
               velocity_candidate = (apex[0] + t2 * bound_right[0], apex[1] + t2 * bound_right[1])
               candidates.append((velocity_candidate, vo1_candidate, vo2_candidate, diff_sq(velocity_candidate, vA_des)))
         
    for i in range(len(RVO_BA_all)-1):                        # Line 217
        for j in range(i+1, len(RVO_BA_all)):
            vo1_candidate=i
            vo2_candidate=j
            
            apex_i, bound_left_i, bound_right_i, dist_BA_i, sum_r_i = RVO_BA_all[i]
            apex_j, bound_left_j, bound_right_j, dist_BA_j, sum_r_j = RVO_BA_all[j]
            
            d = bound_left_i[0]*bound_left_j[1]-bound_left_i[1]*bound_left_j[0]
            
            if d != 0:
                apex_diff = (apex_j[0]-apex_i[0], apex_j[1]-apex_i[1])
                s = ( apex_diff[0]* bound_left_j[1] - apex_diff[1]* bound_left_j[0]) / d
                t = ( apex_diff[0]* bound_left_i[1] - apex_diff[1]* bound_left_i[0]) / d
                
                if s >= 0 and t >= 0:
                    velocity_candidate = (apex_i[0] + s * bound_left_i[0], apex_i[1] + s * bound_left_i[1])
                    
                    if norm(velocity_candidate) < max_speed:
                        candidates.append((velocity_candidate, vo1_candidate, vo2_candidate, diff_sq(velocity_candidate, vA_des)))

            d = bound_right_i[0]*bound_left_j[1]-bound_right_i[1]*bound_left_j[0]
            if d != 0:
                apex_diff = (apex_j[0]-apex_i[0], apex_j[1]-apex_i[1])
                s = ( apex_diff[0]* bound_left_j[1] - apex_diff[1]* bound_left_j[0]) / d
                t = ( apex_diff[0]* bound_right_i[1] - apex_diff[1]* bound_right_i[0]) / d
                
                if s >= 0 and t >= 0:
                    velocity_candidate = (apex_i[0] + s * bound_right_i[0], apex_i[1] + s * bound_right_i[1])
                    
                    if norm(velocity_candidate) < max_speed:
                        candidates.append((velocity_candidate, vo1_candidate, vo2_candidate, diff_sq(velocity_candidate, vA_des)))
            
            d = bound_left_i[0]*bound_right_j[1]-bound_left_i[1]*bound_right_j[0]
            if d != 0:
                apex_diff = (apex_j[0]-apex_i[0], apex_j[1]-apex_i[1])
                s = ( apex_diff[0]* bound_right_j[1] - apex_diff[1]* bound_right_j[0]) / d
                t = ( apex_diff[0]* bound_left_i[1] - apex_diff[1]* bound_left_i[0]) / d
                
                if s >= 0 and t >= 0:
                    velocity_candidate = (apex_i[0] + s * bound_left_i[0], apex_i[1] + s * bound_left_i[1])
                    
                    if norm(velocity_candidate) < max_speed:
                        candidates.append((velocity_candidate, vo1_candidate, vo2_candidate, diff_sq(velocity_candidate, vA_des)))
            
            d = bound_right_i[0]*bound_right_j[1]-bound_right_i[1]*bound_right_j[0]
            if d != 0:
                apex_diff = (apex_j[0]-apex_i[0], apex_j[1]-apex_i[1])
                s = ( apex_diff[0]* bound_left_j[1] - apex_diff[1]* bound_right_j[0]) / d
                t = ( apex_diff[0]* bound_right_i[1] - apex_diff[1]* bound_right_i[0]) / d
                
                if s >= 0 and t >= 0:
                    velocity_candidate = (apex_i[0] + s * bound_right_i[0], apex_i[1] + s * bound_right_i[1])
                    
                    if norm(velocity_candidate) < max_speed:
                        candidates.append((velocity_candidate, vo1_candidate, vo2_candidate, diff_sq(velocity_candidate, vA_des)))

    optimal = -1 # Line 284
    
    for i, x in enumerate(candidates):
        velocity_candidate, vo1_candidate, vo2_candidate, tmp = x
        valid = True        
        for j in range(len(RVO_BA_all)):
            diff = ( velocity_candidate[0]- RVO_BA_all[j][0][0], velocity_candidate[1]- RVO_BA_all[j][0][1])
            det1 = RVO_BA_all[j][2][0] * diff [1] - RVO_BA_all[j][2][1] * diff [0]
            det2 = RVO_BA_all[j][1][0] * diff [1] - RVO_BA_all[j][1][1] * diff [0]
            if j != vo1_candidate and j != vo2_candidate and det1 < 0 and det2 >0:
                valid = False
              
                if j > optimal:
                    optimal=j
                    newVelocity = velocity_candidate
                return newVelocity
        if valid:
            newVelocity = velocity_candidate
            return newVelocity
        
def compute_angular_velocity(old_velocity, target_velocity):

    wheelTrack = 0.4
    # Must match simulator steps_per_sec in config.json (60 Hz)
    timeToOrientation = 1.0/60
    max_speed = 1.0
    #wheelRadius = 0.15
    
    current_orientation = atan2(old_velocity[1], old_velocity[0])
    if target_velocity == (0,0):
        target_orientation = PI
    else:
        target_orientation = atan2(target_velocity[1], target_velocity[0])
    diff_orientation = fmod(target_orientation-current_orientation, 2*PI)
    if diff_orientation < - PI:
        diff_orientation += PI
    if diff_orientation > PI:
        diff_orientation -= PI
    
    speedDiff = diff_orientation * wheelTrack / timeToOrientation
    
    if speedDiff > 2.0* max_speed:
        speedDiff = 2.0* max_speed
    elif speedDiff > 2.0* max_speed:
        speedDiff = -2.0* max_speed
    
    targetSpeed = norm(target_velocity)
    
    if targetSpeed + 0.5* abs(speedDiff) > max_speed:
        if speedDiff >=0:
            v_right = max_speed
            v_left = max_speed - speedDiff
        else:
            v_left = max_speed
            v_right = max_speed + speedDiff
    elif targetSpeed - 0.5* abs(speedDiff) < -max_speed:
        if speedDiff >=0:
            v_left = -max_speed
            v_right = -max_speed + speedDiff
        else:
            v_right = -max_speed
            v_left = -max_speed - speedDiff
    else:
        v_right = targetSpeed + 0.5* speedDiff
        v_left = targetSpeed - 0.5* speedDiff
    
    new_speed = (v_right + v_left) /2
    #new_velocity = (new_speed * cos(target_orientation), new_speed * sin(target_orientation))
    angular_velocity = speedDiff/wheelTrack
    
    return new_speed, angular_velocity
        
            
    
    

def diff_sq(v1, v2):    
    v = sub(v1, v2)
    return norm(v)

def sub(v1, v2):
    if isinstance(v1, Vector):
        v1 = v1.tuple
    if isinstance(v2, Vector):
        v2 = v2.tuple
    return (v1[0]-v2[0],v1[1]-v2[1])



    
    
