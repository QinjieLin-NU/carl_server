#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import numpy as np
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_respawnGoal import Respawn
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import time
import copy
import os

class GazeboWorld():
    def __init__(self, beam_num, index, num_env,ros_port,mpi_rank,env_index):
        os.environ["ROS_MASTER_URI"]="http://localhost:%d"%ros_port
        self.mpi_rank =mpi_rank
        self.index = index#roboIndex
        self.num_env = num_env
        self.env_index = env_index
        node_name = 'GazeboEnv_' + str(index)
        print("rank: %d node name:%s"%(mpi_rank,node_name))
        rospy.init_node(node_name, anonymous=None)

        self.goal_x = 0
        self.goal_y = 0
        self.initGoal = True
        self.get_goalbox = False
        self.position = Point()
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
        self.respawn_goal = Respawn(self.env_index)
        self.resetX = rospy.get_param('/x_pos',0.0)
        self.resetY = rospy.get_param('/y_pos',0.0)
        self.resetZ = rospy.get_param('/z_pos',0.0)
        self.resetYaw = rospy.get_param('/yaw_angle',0.0)
        self.resetQua = quaternion_from_euler(0.0,0.0,self.resetYaw)
        #initialzie parameters
        self.position.x = self.resetX
        self.position.y = self.resetY
        self.init_pose = [self.position.x,self.position.y]
        self.goal_point = [0,0]
        self.current_pos = [0,0,0]
        self.speed = [0,0]
        self.goal_size = 0.5


    def getGoalDistace(self):
        goal_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)
        return goal_distance

    def getOdometry(self, odom):
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.current_pos = [self.position.x,self.position.y,yaw]
        self.speed = [odom.twist.twist.linear.x, odom.twist.twist.angular.z]

    def reset_world(self):
        self.reset_turtlebot3()
        return 

    def reset_turtlebot3(self):
        turState = ModelState()
        turState.model_name = "turtlebot3_burger"
        turState.pose.position.x = self.resetX
        turState.pose.position.y = self.resetY
        turState.pose.position.z = self.resetZ
        turState.pose.orientation.x = self.resetQua[0]
        turState.pose.orientation.y = self.resetQua[1]
        turState.pose.orientation.z = self.resetQua[2]
        turState.pose.orientation.w = self.resetQua[3]
        rospy.wait_for_service('/gazebo/set_model_state')
        set_model_prox = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        set_model_prox(turState)
        return
    
    def stop_turtlebot3(self):
        self.pub_cmd_vel.publish(Twist())

    def store_resetPose(self):
        return 
    
    def reset_pose(self):
        self.reset_turtlebot3()
    
    def generate_goal_point(self):
        if(self.initGoal):
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=False)
            self.initGoal = False
        else:           
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)
        self.goal_distance = self.getGoalDistace()
        self.distance = self.goal_distance#qinjielin
        self.get_goalbox = False
        self.goal_point = [self.goal_x,self.goal_y]

    def get_laser_observation(self):
        scan = None
        while scan is None:
            try:
                scan = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        min_range = 0.3
        done = False
        full_scan_range = []

        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                full_scan_range.append(3.5)
            elif scan.ranges[i] == 0.0:
                full_scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]):
                full_scan_range.append(0)
            else:
                full_scan_range.append(scan.ranges[i])

        full_scan_range = np.asarray(full_scan_range)
        return full_scan_range / 3.5 -0.5
                
    def get_local_goal(self):
        [x, y, theta] = self.current_pos
        [goal_x, goal_y] = self.goal_point
        local_x = (goal_x - x) * np.cos(theta) + (goal_y - y) * np.sin(theta)
        local_y = -(goal_x - x) * np.sin(theta) + (goal_y - y) * np.cos(theta)
        return [local_x, local_y]

    def get_self_speed(self):
        return self.speed
    
    def control_vel(self, action):
        move_cmd = Twist()
        move_cmd.linear.x = action[0]
        move_cmd.linear.y = 0.
        move_cmd.linear.z = 0.
        move_cmd.angular.x = 0.
        move_cmd.angular.y = 0.
        move_cmd.angular.z = -1 * action[1]#reverse because of gazebo
        self.pub_cmd_vel.publish(move_cmd)
    
    def get_self_stateGT(self):
        # can we get that from gazebo
        return self.current_pos

    def get_self_speedGT(self):
        #can we get that from gazebo
        return self.speed 
    
    def get_crash_state(self):
        scan = None
        while scan is None:
            try:
                scan = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        min_range = 0.3
        done = False
        full_scan_range = []

        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                full_scan_range.append(3.5)
            elif scan.ranges[i] == 0.0:
                full_scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]):
                full_scan_range.append(0)
            else:
                full_scan_range.append(scan.ranges[i])
        full_scan_range = np.asarray(full_scan_range)

        if min_range > min(full_scan_range) > 0:
            # print("creahed detected:",min(full_scan_range))
            done = True
        
        return done
    
    def get_reward_and_terminate(self,step):
        terminate = False
        # laser_scan = self.get_laser_observation()
        [x, y, theta] = self.get_self_stateGT()
        [v, w] = self.get_self_speedGT()
        self.pre_distance = copy.deepcopy(self.distance)
        self.distance = np.sqrt((self.goal_point[0] - x) ** 2 + (self.goal_point[1] - y) ** 2)
        reward_g = (self.pre_distance - self.distance) * 2.5
        reward_c = 0
        reward_w = 0
        result = 0
        is_crash = self.get_crash_state()

        if self.distance < self.goal_size:
            terminate = True
            reward_g = 15
            result = 'Reach Goal'

        if is_crash == 1:
            terminate = True
            reward_c = -15.
            result = 'Crashed'

        if np.abs(w) >  1.05:
            reward_w = -0.1 * np.abs(w)

        if step > 150:
            # terminate = True
            result = 'Time out'
        reward = reward_g + reward_c + reward_w

        print("reward:",reward)

        return reward, terminate, result