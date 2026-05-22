#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@data 14:04:32 2018
@author: chong.chen2@dorabot.com
@breif: original from MengGuo at Github Modified to a localized version
"""
from math import ceil, floor, sqrt
import copy
from math import cos, sin, tan, atan2, asin, fmod, tanh
from math import pi as PI
from geometry import *
from agents.agent_state_machine import AgentState
from local_planner import LocalPlanner

class RVOPlanner(LocalPlanner):
    def compute_plan(self, position, velocity, gridmap, sensor_observation, global_planner_path):
        #FIXIT 
        TIME_STEP = 1.0/20
        if self.agent.destination_location == None or position.distance(global_planner_path[-1])< 1e-1:
            return [0,0], []
        """
        if self.agent.position.distance(self.agent.destination_location)< 1:
            vel = compute_V_des(self.agent, [self.agent.destination_location])
            speed = norm(vel)
            angle_diff = PI - self.agent.angle
            angular = angle_diff / TIME_STEP
            #if abs(angle_diff) > PI/4/TIME_STEP:
            #    speed = 0
            return speed, angular
        """
        current_orientation =  sensor_observation.localization()[2]
        #result_vel = Vector(velocity[0], velocity[1])
        agentA = self.agent
        pA = agentA.position
        vA = agentA.linear_velocity
        vA_des = compute_V_des(self.agent, global_planner_path)
        rA = agentA.shape.get_radius()+0.1
        RVO_BA_all = []

        for agentB in sensor_observation.other_agents_state_in_range_of(3):
            #vel_A_B_dir = compute_direction(position, agentB.position)
            # TODO Gary: @Gary do not modify agent status in hidden functions
            self.agent.potential_collision = True
            #if agentB.state==AgentState.CRUISE:
            pB = agentB.position
            vB = agentB.linearVelocity
            rB = agentB.userData.shape.get_radius()
            if agentB.userData.destination_location == None or agentB.userData.state == AgentState.QUEUING:
                RVO_BA = compute_rvo_BA(pA, vA, rA, pB, vB, rB, reciprocal = False)
            else:
                RVO_BA = compute_rvo_BA(pA, vA, rA, pB, vB, rB, reciprocal = True)
            RVO_BA_all.append(RVO_BA)


        for body in sensor_observation.ports_in_range_of(2):
            self.agent.potential_collision = True
            obstacle = body.userData
            pB = Point(obstacle.location.x+obstacle.dimension[0]*0.5, obstacle.location.y+obstacle.dimension[1]*0.5)
            #vel_A_B_dir = compute_direction(position, pB)
            vB = [0,0]
            rB = pow(obstacle.dimension[0]**2+obstacle.dimension[1]**2,0.5)/2
            VO_BA = compute_rvo_BA(pA,vA,rA, pB, vB, rB, reciprocal = False)
            RVO_BA_all.append(VO_BA)
            
        if RVO_BA_all != []:
            result_vel = intersect(pA, vA_des, RVO_BA_all)
        else:
            result_vel = vA_des
        return result_vel
        # speed = norm(result_vel)
        # if speed == 0:
        #     target_angle = PI
        # else:
        #     target_angle = atan2(result_vel[1], result_vel[0])
            
        # angle_diff = target_angle-current_orientation
        # if angle_diff >=0:
        #     n = floor(angle_diff/PI)
        # else: 
        #     n = floor(angle_diff/PI) + 1

        # angle_diff = angle_diff + n*PI
                    
        # angular = angle_diff/TIME_STEP
        # #if abs(angle_diff) > PI/4:
        # #    speed = 0
            
        # return speed, angular
        
    
    

def compute_rvo_BA(pA, vA, rA, pB, vB, rB, reciprocal=True):
    sum_r = rA+rB
    dist_BA = max(sum_r,pA.distance(pB))
    theta_BA = atan2(pB.y-pA.y, pB.x-pA.x)
    if reciprocal == True:
        transl_vB_vA = [pA.x+0.5*(vB[0]+vA[0]), pA.y+0.5*(vB[1]+vA[1])]
    else:
        transl_vB_vA = [pA.x+vB[0], pA.y+vB[1]]
    theta_BAort = asin(sum_r/dist_BA)
    theta_ort_left = theta_BA+theta_BAort
    bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
    theta_ort_right = theta_BA-theta_BAort
    bound_right = [cos(theta_ort_right), sin(theta_ort_right)]
    RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, sum_r]
    return RVO_BA



def intersect(pA, vA, RVO_BA_all):
    norm_v = norm(vA)
    suitable_V = []
    unsuitable_V = []
    sample_step_length=[0.2, 0.2]
    for theta in [x*sample_step_length[0] for x in range(0, int(2*PI/sample_step_length[0]))]:
        for rad in [x*sample_step_length[1]+0.02 for x in range(0, int(norm_v/sample_step_length[1]))]:
            new_v = [rad*cos(theta), rad*sin(theta)]
            suit = True
            for RVO_BA in RVO_BA_all:
                p_0 = RVO_BA[0]
                left = RVO_BA[1]
                right = RVO_BA[2]
                dif = [new_v[0]+pA.x-p_0[0], new_v[1]+pA.y-p_0[1]]
                theta_dif = atan2(dif[1], dif[0])
                theta_right = atan2(right[1], right[0])
                theta_left = atan2(left[1], left[0])
                if in_between(theta_right, theta_dif, theta_left):
                    suit = False
                    break
            if suit:
                suitable_V.append(new_v)
            else:
                unsuitable_V.append(new_v)
    new_v = vA[:]
    suit = True
    for RVO_BA in RVO_BA_all:
        p_0 = RVO_BA[0]
        left = RVO_BA[1]
        right = RVO_BA[2]
        dif = [new_v[0]+pA.x-p_0[0], new_v[1]+pA.y-p_0[1]]
        theta_dif = atan2(dif[1], dif[0])
        theta_right = atan2(right[1], right[0])
        theta_left = atan2(left[1], left[0])
        if in_between(theta_right, theta_dif, theta_left):
            suit = False
            break
    if suit:
        suitable_V.append(new_v)
    else:
        unsuitable_V.append(new_v)

    if suitable_V:
        vA_post = min(suitable_V, key = lambda v: distance(v, vA))
        new_v = vA_post[:]
        for RVO_BA in RVO_BA_all:
            p_0 = RVO_BA[0]
            left = RVO_BA[1]
            right = RVO_BA[2]
            dif = [new_v[0]+pA.x-p_0[0], new_v[1]+pA.y-p_0[1]]
            theta_dif = atan2(dif[1], dif[0])
            theta_right = atan2(right[1], right[0])
            theta_left = atan2(left[1], left[0])
    else:
        tc_V = dict()
        for unsuit_v in unsuitable_V:
            tc_V[tuple(unsuit_v)] = 0
            tc = []
            for RVO_BA in RVO_BA_all:
                p_0 = RVO_BA[0]
                left = RVO_BA[1]
                right = RVO_BA[2]
                dist = RVO_BA[3]
                rad = RVO_BA[4]
                dif = [unsuit_v[0]+pA.x-p_0[0], unsuit_v[1]+pA.y-p_0[1]]
                theta_dif = atan2(dif[1], dif[0])
                theta_right = atan2(right[1], right[0])
                theta_left = atan2(left[1], left[0])
                if in_between(theta_right, theta_dif, theta_left):
                    small_theta = abs(theta_dif-0.5*(theta_left+theta_right))
                    if abs(dist*sin(small_theta)) >= rad:
                        rad = abs(dist*sin(small_theta))
                    big_theta = asin(abs(dist*sin(small_theta))/rad)
                    dist_tg = abs(dist*cos(small_theta))-abs(rad*cos(big_theta))
                    if dist_tg < 0:
                        dist_tg = 0
                    tc_v = dist_tg/norm(dif)
                    tc.append(tc_v)
            tc_V[tuple(unsuit_v)] = min(tc)+0.001
        WT = 0.2
        vA_post = min(unsuitable_V, key = lambda v: ((WT/tc_V[tuple(v)])+distance(v, vA)))
    return vA_post

def in_between(theta_right, theta_dif, theta_left):
    if abs(theta_right - theta_left) <= PI:
        if theta_right <= theta_dif <= theta_left:
            return True
        else:
            return False
    else:
        if (theta_left <0) and (theta_right >0):
            theta_left += 2*PI
            if theta_dif < 0:
                theta_dif += 2*PI
            if theta_right <= theta_dif <= theta_left:
                return True
            else:
                return False
        if (theta_left >0) and (theta_right <0):
            theta_right += 2*PI
            if theta_dif < 0:
                theta_dif += 2*PI
            if theta_left <= theta_dif <= theta_right:
                return True
            else:
                return False

def compute_V_des(agent,global_planner_path):
    """Sliding Window"""
    p=agent.position
    try:
        goal=[x for x in global_planner_path if x.distance(p)<2][-1]         
    except:
        goal=agent.destination_location

    V_max=agent.cruise_speed
    dif_x = [goal.x-p.x, goal.y-p.y]
    norm_dif = norm(dif_x)
    norm_dif_x = [dif_x[k]*V_max/norm_dif for k in range(2)]
    V_des=norm_dif_x[:]
    if reach(p, goal, 0.1):
        V_des[0] = 0
        V_des[1] = 0
    return V_des

def reach(p1, p2, bound=0.3):
    if p1.distance(p2)< bound:
        return True
    else:
        return False

def norm(vector):
    sum_square=sum(i**2 for i in vector)
    return pow(sum_square, 0.5)+0.001

def distance(pose1, pose2):
    """" compute Euclidean distance for 2D """
    return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)+0.001
