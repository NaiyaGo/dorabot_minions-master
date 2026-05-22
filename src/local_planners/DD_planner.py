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
from rvo_planner import norm


class DDPlanner(LocalPlanner):
    def compute_plan(self, position, velocity, gridmap, sensor_observation, global_planner_path):
        if self.agent.destination_location == None or global_planner_path == None:
            return [0,0], []
        local_path = []
        start_pose = position
        goal_pose = global_planner_path[-1]
        direction = compute_direction(start_pose, goal_pose)
        result_vel = Vector(velocity[0], velocity[1])
        agentA = self.agent
        
        # Effective position, velocity and radius
        pA, vA, RA = effective_vals(agentA)
        theta = agentA.angle
        pref_vA = pref_velocity(agentA, global_planner_path)
        RVO_BA_all = []

        for agentB in sensor_observation.other_agents_state_in_range_of(4):
            pB, vB, RB = effective_vals(agentB.userData)
            pref_vB = pref_velocity(agentB.userData, agentB.userData.sequence_of_poses)
                       
            vel_A_B_dir = compute_direction(pA, pB).normalize()
            self.agent.potential_collision = True
            
            #if agentB.userData.destination_location ==None:# or agentB.userData.state == AgentState.QUEUING:
            HRVO_BA=compute_hrvo_BA(pA, vA, RA,pref_vA, pB, vB, RB,pref_vB)
            #else:
            #    RVO_BA=compute_rvo_BA(pA, vA, RA, pB, vB, RB, reciprocal=True)
            RVO_BA_all.append(HRVO_BA)


        for body in sensor_observation.ports_in_range_of(4, PI/2):
            self.agent.potential_collision = True
            obstacle=body.userData
            pB=Point(obstacle.location.x+obstacle.dimension[0]*0.5,obstacle.location.y+obstacle.dimension[1]*0.5)
            vel_A_B_dir = compute_direction(pA, pB).normalize()
            vB=[0,0]
            pref_vB=[0,0]
            rB=pow(obstacle.dimension[0]**2+obstacle.dimension[1]**2,0.5)/2
            HRVO_BA=compute_hrvo_BA(pA, vA, RA,pref_vA, pB, vB, rB,pref_vB)
            RVO_BA_all.append(HRVO_BA)
            
        for wall in sensor_observation.walls_in_range_of(4, PI/2):
            self.agent.potential_collision = True
            obstacle = wall.userData
            pB = Point(obstacle.location.x+obstacle.dimension[0]*0.5, obstacle.location.y+obstacle.dimension[1]*0.5)
            #vel_A_B_dir = compute_direction(position, pB)
            vB = [0,0]
            pref_vB=[0,0]
            rB = pow(obstacle.dimension[0]**2+obstacle.dimension[1]**2,0.5)/2
            VO_BA = compute_hrvo_BA(pA, vA, RA,pref_vA, pB, vB, rB,pref_vB)
            RVO_BA_all.append(VO_BA)
            
        if RVO_BA_all != []:
            result_vel = intersect(pA, vA, pref_vA, RVO_BA_all)
        else:
            result_vel=pref_vA
        L = 0.4
        #L = agentA.wheel_track
        DA = RA/2
        
        a = cos(theta) / 2 + DA * sin(theta)/L
        b = cos(theta) / 2 - DA * sin(theta)/L
        c = sin(theta) / 2 - DA * cos(theta)/L
        d = sin(theta) / 2 + DA * cos(theta)/L
        
        det = a*d-b*c
        
        if det != 0:
            u_right = (-c* result_vel[0]+a*result_vel[1])/det
            u_left = (d* result_vel[0]-b*result_vel[1])/det
            
        speed = (u_right + u_left)/2

        angular = (u_right - u_left)/L
        
        if angular > 1.5:
            speed = 0

        #linear_velocity = (speed*cos(theta), speed*sin(theta))
        
        return speed, angular
    
def compute_hrvo_BA(pA,vA,RA,vA_des, pB, vB, rB, vB_des, reciprocal=False):
    """ Rewritten from Snape's Agent.cpp, Line 107-128"""
    uncertaintyOffset = 0.0
    # Must match simulator steps_per_sec in config.json (60 Hz)
    TIME_STEP = 1.0/60
    
    sum_r=RA+rB
    dist_BA = pA.distance(pB)
    
    if dist_BA > sum_r:
        theta_BA = atan2(pB.y-pA.y, pB.x-pA.x)       # Line 108, angle
        theta_BAort = asin(sum_r/dist_BA)            # Line 109, openingAngle
        theta_ort_left = theta_BA-theta_BAort
        bound_left = [cos(theta_ort_left), sin(theta_ort_left)]     # Line 111, velocityObstacle.side1_
        theta_ort_right = theta_BA+theta_BAort
        bound_right = [cos(theta_ort_right), sin(theta_ort_right)]  # Line 112, velocityObstacle.side2_
        
        d = sin(2.0*theta_BAort)                     # Line 114, d = 2 * sin(openingAngle) * cos(openingAngle)
        
        det0 = det(sub(pB, pA), sub(vA_des, vB_des))  # Line 116
        #print "det0", det0
        if det0 >0:           
            s = 0.5 * det(sub(vA, vB), bound_right)/d         # Line 117
            apex= [vB[0] + s * bound_left[0]-(uncertaintyOffset/sum_r) * (pB.x-pA.x),
               vB[1] + s * bound_left[1]-(uncertaintyOffset/sum_r) * (pB.y-pA.y)] # Line 119
        else:
            s = 0.5 * det(sub(vA, vB), bound_left)/d          # Line 122
            apex= [vB[0] + s * bound_right[0]-(uncertaintyOffset/sum_r) * (pB.x-pA.x),
               vB[1] + s * bound_right[1]-(uncertaintyOffset/sum_r) * (pB.y-pA.y)] # Line 127
    
    else:    # dist_BA<=sum_r
        apex=[0.5*(vB[0]+vA[0])-(uncertaintyOffset + 0.5* (sum_r-dist_BA)/TIME_STEP/dist_BA)*(pB.x-pA.x),
              0.5*(vB[0]+vA[0])-(uncertaintyOffset + 0.5* (sum_r-dist_BA)/TIME_STEP/dist_BA)*(pB.y-pA.y)]
        bound_left = compute_direction(pA, pB).normalize()
        bound_right = [-bound_left[0], -bound_left[1]]                                                   # Line 130-132
    HRVO_BA = [apex, bound_left, bound_right, dist_BA, sum_r]
    return HRVO_BA

def intersect(pA, vA, pref_vA, RVO_BA_all):
    candidates = []
    vo1_candidate = 1e6
    vo2_candidate = 1e6
    max_speed=2
    vA_des = pref_vA
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
        det1 = det(bound_left, diff)
        det2 = det(bound_right, diff)
        if dotProduct1 > 0 and det1 > 0:                                         # Line 160-166
            velocity_candidate = (apex[0] + dotProduct1 * bound_left[0], 
                                  apex[1] + dotProduct1 * bound_left[1])
            
            if norm(velocity_candidate) < max_speed:
                candidates.append((velocity_candidate, vo1_candidate, vo2_candidate, diff_sq(velocity_candidate, vA_des)))
        
        if dotProduct2 > 0 and det2 < 0:                                         # Line 168-174
            velocity_candidate = (apex[0] + dotProduct2 * bound_right[0], 
                                  apex[1] + dotProduct2 * bound_right[1])
            if norm(velocity_candidate) < max_speed:
                candidates.append((velocity_candidate, vo1_candidate, vo2_candidate, diff_sq(velocity_candidate, vA_des)))
    
    for j in range(len(RVO_BA_all)):                                             # Line 177-
        vo1_candidate = 1e6
        vo2_candidate = j
        apex, bound_left, bound_right, dist_BA, sum_r = RVO_BA_all[j]
        
        det0 = det(apex, bound_left)
        discriminant = max_speed**2-det0**2
        
        if discriminant > 0:
            t1 = -(apex[0] * bound_left[0] + apex[1] * bound_left[1]) + pow(discriminant, 0.5)
            t2 = -(apex[0] * bound_left[0] + apex[1] * bound_left[1]) - pow(discriminant, 0.5)
        
            if t1 >= 0:                                                           # Line 188
                velocity_candidate = (apex[0] + t1 * bound_left[0], apex[1] + t1 * bound_left[1])
                candidates.append((velocity_candidate, vo1_candidate, vo2_candidate, diff_sq(velocity_candidate, vA_des)))
            if t2 >= 0:
                velocity_candidate = (apex[0] + t2 * bound_left[0], apex[1] + t2 * bound_left[1])
                candidates.append((velocity_candidate, vo1_candidate, vo2_candidate, diff_sq(velocity_candidate, vA_des)))
         
        det0 = det(apex, bound_right)                 # Line 199
        discriminant = max_speed**2-det0**2
        
        if discriminant > 0:                          # Line201
           t1 = -(apex[0] * bound_right[0] + apex[1] * bound_right[1]) + pow(discriminant, 0.5)
           t2 = -(apex[0] * bound_right[0] + apex[1] * bound_right[1]) - pow(discriminant, 0.5)
    
           if t1 >= 0:                                                           # Line 205
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
            apex_diff = (apex_j[0]-apex_i[0], apex_j[1]-apex_i[1])
            d = det(bound_left_i, bound_left_j)
            
            if d != 0:    # Line 224
                s = det(apex_diff, bound_left_j) / d
                t = det(apex_diff, bound_left_i) / d
                
                if s >= 0 and t >= 0:
                    velocity_candidate = (apex_i[0] + s * bound_left_i[0], 
                                          apex_i[1] + s * bound_left_i[1])
                    
                    if norm(velocity_candidate) < max_speed:
                        candidates.append((velocity_candidate, vo1_candidate, vo2_candidate, diff_sq(velocity_candidate, vA_des)))

            d = det(bound_right_i, bound_left_j) # Line 237
            if d != 0:
                s = det(apex_diff, bound_left_j) / d 
                t = det(apex_diff, bound_right_i) / d
                
                if s >= 0 and t >= 0:
                    velocity_candidate = (apex_i[0] + s * bound_right_i[0],
                                          apex_i[1] + s * bound_right_i[1])
                    
                    if norm(velocity_candidate) < max_speed:
                        candidates.append((velocity_candidate, vo1_candidate, vo2_candidate, diff_sq(velocity_candidate, vA_des)))
            
            d = det(bound_left_i,bound_right_j) # Line 252
            if d != 0:
                s = det(apex_diff, bound_right_j) / d
                t = det(apex_diff, bound_left_i) / d
                
                if s >= 0 and t >= 0:
                    velocity_candidate = (apex_i[0] + s * bound_left_i[0], 
                                          apex_i[1] + s * bound_left_i[1])
                    
                    if norm(velocity_candidate) < max_speed:
                        candidates.append((velocity_candidate, vo1_candidate, vo2_candidate, diff_sq(velocity_candidate, vA_des)))
            
            d = det(bound_right_i,bound_right_j) # Line 267
            if d != 0:
                s = det(apex_diff, bound_right_j) / d
                t = det(apex_diff, bound_right_i) / d
                
                if s >= 0 and t >= 0:
                    velocity_candidate = (apex_i[0] + s * bound_right_i[0], apex_i[1] + s * bound_right_i[1])
                    
                    if norm(velocity_candidate) < max_speed:
                        candidates.append((velocity_candidate, vo1_candidate, vo2_candidate, diff_sq(velocity_candidate, vA_des)))

    optimal = -1 # Line 284
    
    for i, x in enumerate(candidates):
        velocity_candidate, vo1_candidate, vo2_candidate, tmp = x
        valid = True        
        for j in range(len(RVO_BA_all)):
            apex, bound_left, bound_right, dist_BA, sum_r = RVO_BA_all[j]
            diff = ( velocity_candidate[0]- apex[0], velocity_candidate[1]- apex[1])
            det1 = det(bound_right, diff) 
            det2 = det(bound_left, diff)
            if j != vo1_candidate and j != vo2_candidate and det1 < 0 and det2 >0:
                valid = False
              
                if j > optimal:
                    optimal=j
                    newVelocity = velocity_candidate
                break
        if valid:
            newVelocity = velocity_candidate
            break
    return newVelocity
        
def compute_angular_velocity(old_angle, target_velocity, max_speed):
    wheelTrack = 0.4
    # Must match simulator steps_per_sec in config.json (60 Hz)
    timeToOrientation = 1.0/60    
    current_orientation = old_angle
    if target_velocity == (0,0):
        target_orientation= PI
    else:
        target_orientation = atan2(target_velocity[1], target_velocity[0])
    diff_orientation = fmod(target_orientation-current_orientation, 2*PI)
    if diff_orientation < - PI:
        diff_orientation += 2*PI
    if diff_orientation > PI:
        diff_orientation -= 2*PI
    
    speedDiff = diff_orientation * wheelTrack / timeToOrientation
    
    if speedDiff > 2.0* max_speed:
        speedDiff = 2.0* max_speed
    elif speedDiff < - 2.0* max_speed:
        speedDiff = -2.0* max_speed
    
    targetSpeed = norm(target_velocity)
    
    # Differential Drive Constraint. 
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
    # angular_velocity = min (2*max_speed/wheelTrack/timeToOriendtation, diff_orientation/timetoorientation)
    
    return new_speed, angular_velocity

def compute_wheel_speeds(agent):
    angular = agent.angular_velocity
    linear = agent.linear_velocity
    speed = norm(linear)
    L = 0.4
    #L = agent.wheel_track
    u_left = speed - L * angular/2
    u_right = speed + L * angular/2
    return u_left, u_right

def effective_vals(agent):
    D = agent.shape.get_radius()
    theta = agent.angle
    qA = agent.position
    # = agent.wheel_track
    L = 0.4
    u_left, u_right = compute_wheel_speeds(agent)
    
    effective_pos=Point(qA.x + D * cos(theta), qA.y + D * sin(theta))
    
    vel_x = (cos(theta)/2 + D* sin(theta)/L)*u_left + (cos(theta)/2 - D* sin(theta)/L)*u_right
    vel_y = (sin(theta)/2 - D* cos(theta)/L)*u_left + (sin(theta)/2 + D* cos(theta)/L)*u_right
    
    effective_vel = (vel_x, vel_y)
    
    return effective_pos, effective_vel, 2*D

def pref_velocity(agent, global_planner_path):
    """Sliding Window"""
    DifferentialDrive = True
    pA, vA, RA = effective_vals(agent)
    D = RA/2    
    try:
        sliding_window = [x for x in global_planner_path if x.distance(p)<4]
        #goal = sliding_window[-2]
        next_step = sliding_window[-1]
        theta = compute_direction(agent.position, next_step)    
    except:
        goal = agent.destination_location
        if goal == None:
            return [0,0]
        theta = compute_direction(agent.position, goal)       
    if agent.state == AgentState.QUEUING:
        theta = Vector(-1,0)
    effective_goal = (goal.x + D * theta.x , goal.y + D * theta.y)    
    if DifferentialDrive == True:
        V_max=agent.cruise_speed
        V_pref = V_max#*0.7       
    diff = sub(effective_goal, pA)    
    norm_dif = norm(diff)
    V_des = [diff[k]*V_pref/norm_dif for k in range(2)]    
    V_des = V_des[:]    
    if diff_sq(pA, effective_goal)<0.1:
        V_des[0] = 0
        V_des[1] = 0
    return V_des
        

def diff_sq(v1, v2):    
    v=sub(v1, v2)
    return norm(v)

def det(v1, v2):
    if isinstance(v1, Vector) or isinstance(v1, Point):
        v1=v1.to_tuple()
    if isinstance(v2, Vector) or isinstance(v2, Point):
        v2=v2.to_tuple()
    return v1[0]*v2[1]-v1[1]*v2[0]

def sub(v1, v2):
    if isinstance(v1, Vector) or isinstance(v1, Point):
        v1=v1.to_tuple()
    if isinstance(v2, Vector) or isinstance(v2, Point):
        v2=v2.to_tuple()
    return (v1[0]-v2[0],v1[1]-v2[1])

def dot(v1, v2):
    if isinstance(v1, Vector) or isinstance(v1, Point):
        v1=v1.to_tuple()
    if isinstance(v2, Vector) or isinstance(v2, Point):
        v2=v2.to_tuple()
    return (v1[0]*v2[0]+v1[1]*v2[1])



    
    
