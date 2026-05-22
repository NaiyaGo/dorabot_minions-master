# -*- coding: utf-8 -*-

import time
import math

import numpy as np

# Parameters
MAX_LINEAR = 1.5  
MAX_ANGULAR = 1.5  
SCALE_PARAM = 12.0
PID_MIN_OBS = 0.014
FAILSAFE_DIST = 0.
H_MAX_ANGULAR = 3.0


class MLSHAgent(object):

    def __init__(self):
        self.subpolicies = []
        self.action = None
        self.subpolicies.append(ClassController())
        self.subpolicies.append(FuzzyLogicController())


    def act(self, obs, terminated, target_pos, uwb_pos, rvo_action):
        if terminated:
            return np.zeros(2)

        scan = obs[0]
        goal = obs[1]

        FLC_action = self.subpolicies[1].act(scan*SCALE_PARAM,goal)

        s = np.min(FLC_action)
        k = 6.
        b = 8.

        
        H_action,whereisgoal = self.subpolicies[0].act(obs, target_pos, uwb_pos)
        RL_action = np.array(rvo_action)
        


 
        # beta: adaptive value
        # s: current minimum obstacle distance
        # k: shrinking factor
        # b: phase shift
        # O_contrller = beta*H_contrller + (1-beta)*RL_controller

        beta = 1 / (1 + math.exp(-s*k+b))
        goal_dist_sq = (target_pos[0] - uwb_pos[0])**2 + (target_pos[1] - uwb_pos[1])**2
        if ((abs(whereisgoal)>math.pi/3) and (s > 0.1)) or goal_dist_sq < s**2:
            action = H_action

        else:
            action = beta * H_action + (1.0-beta) * RL_action


        if action[0] < 0:
            action[0] = 0
        return action



class ClassController(object):
    def __init__(self):

        self.k = (H_MAX_ANGULAR - 0.5*MAX_LINEAR)/math.pi

    def act(self, obs, target_pos, uwb_pos):
        R = [[math.cos(uwb_pos[3]), math.sin(uwb_pos[3]), 0],
             [-math.sin(uwb_pos[3]), math.cos(uwb_pos[3]), 0],
             [0, 0, 1]]
             
        
        xBar = np.dot(R,[[target_pos[0] - uwb_pos[0]], [target_pos[1] - uwb_pos[1]], [uwb_pos[3] - target_pos[2]]])
        if xBar[2] >= 0:
            n = math.floor(xBar[2]/np.pi)
        else:
            n = math.floor(xBar[2]/np.pi) + 1
        xBar[2] = xBar[2] + n*np.pi

        e = math.sqrt(math.pow(xBar[0],2) + math.pow(xBar[1],2))
        alpha = math.atan2(xBar[1],xBar[0])
        if alpha >=0:
            n = math.floor(alpha/np.pi)
        else: 
            n = math.floor(alpha/np.pi) + 1

        alpha = alpha+n*np.pi

        action = np.zeros(2)
        action[0] = MAX_LINEAR * math.tanh(e) * math.cos(alpha)
        action[1] = self.k*alpha + MAX_LINEAR * (math.tanh(e)/e) * math.sin(alpha) * math.cos(alpha)

        goal = obs[1]
        if abs(alpha) > np.pi/4:
            action[0] = 0

        if action[0] < 0:
            action[0] = 0
        
        return action, alpha

class FuzzyLogicController(object):
    def __init__(self):
        self.scan = 4. * np.ones([512])
        self.angle_array = np.linspace(-np.pi/2.,np.pi/2.,512)
    def act(self, scan, goal):
        self.scan = scan[:,-1]
        self.scan += np.abs(self.angle_array)*0.5
        obs_zone = self.scan * np.abs(np.sin(self.angle_array))
        new_scan = np.where(obs_zone>0.35, 4, self.scan)
        return new_scan






class Environment(object):
    def __init__(self, agent):
        self.agent = agent #load control system
        self.scan = 4. * np.ones([512, 3]) #LIDAR information, store 3 frame
        self.uwb_pos = np.zeros(4) #robot pose state [x, y, z, theta]
        self.vel = np.zeros(2)   #robot velocity state [v, w]
        self.target_pos = np.zeros(3) #robot target pose 
        time.sleep(1)

    def run(self, scan_, goal_, vel_, pose_, action_, max_linear_speed, max_angular_speed):
        MAX_LINEAR = max_linear_speed 
        MAX_ANGULAR = max_angular_speed
        self.scan_callback(scan_)
        self.pos_callback(pose_)
        self.vel_callback(vel_)
        self.goal_callback(goal_)
        
        goal, terminated = self.preprocess_nav()
        act = self.agent.act([self.scan, goal, self.vel], terminated, self.target_pos, self.uwb_pos, action_)
        return act
       


    def send_command(self, act):
        command = Twist()
        command.linear.x = act[0]
        command.angular.z = act[1]
        self.command_pub.publish(command)

    def preprocess_scan(self, scan):
        scan = np.asarray(scan)
        scan_list = scan.tolist()
        #scan_list.reverse()
        scan = np.asarray(scan_list)
        scan -= 0.15
        scan = np.where(np.isinf(scan), 4., scan)
        scan = np.where(np.isnan(scan), 4., scan)
        scan = np.where(scan > 4., 4., scan)
        scan = np.where(scan <= 0., 4., scan)
        scan /= SCALE_PARAM
        return scan

    def preprocess_nav(self):
        goal = [self.compute_distance(), self.compute_angle()]
        if goal[0] < 0.0001:
            terminated = True
        else:
            terminated = False
        return goal, terminated

    def compute_distance(self):
        return np.hypot(self.uwb_pos[0] - self.target_pos[0], self.uwb_pos[1] - self.target_pos[1])

    def compute_angle(self):
        angle =  math.atan2(self.target_pos[1]-self.uwb_pos[1], self.target_pos[0]-self.uwb_pos[0]) - self.uwb_pos[3]
        if angle > np.pi:
            angle -= 2 * np.pi
        elif angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def scan_callback(self, data):

        scan = self.preprocess_scan(data)
        self.scan = np.concatenate((self.scan[:, 1:], np.asarray([scan]).transpose()), axis=1)

    def pos_callback(self, data):
        self.uwb_pos[0] = data[0]
        self.uwb_pos[1] = data[1]
        self.uwb_pos[3] = data[2]


    def vel_callback(self, data):
        self.vel[0] = data[0]
        self.vel[1] = data[1]


    def goal_callback(self, data):
        self.target_pos[0] = data[0]
        self.target_pos[1] = data[1]
        self.target_pos[2] = data[2]
    

    