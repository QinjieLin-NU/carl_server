from numpy import array, rint, linspace, pi, cos, sin
import itertools
import random
import numpy 
from mpi4py import MPI
import logging

import tf
import os
import rospy
import sys
import time

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Empty
from std_msgs.msg import Int8


class DataRecorder(object):
    """A disk-shaped agent."""
    def __init__(self):
        self.x_lis =  [7.7,  8.0, 7.7, -7.7, -8.0, -7.7]
        self.y_list = [-2.0, 0.0, 2.0, 2.0,  0.0,  -2.0]
        self.goals =  [[7,-2],[8,0],[7,2],[-7,-2],[-8,0],[-7,2]]
        self.firsts = [False,False,False,False,False,False]

        odom0_topic = 'robot_' + str(0) + '/odom'
        self.odom_sub0 = rospy.Subscriber(odom0_topic, Odometry, self.odometry_callback0)

        odom1_topic = 'robot_' + str(1) + '/odom'
        self.odom_sub1 = rospy.Subscriber(odom1_topic, Odometry, self.odometry_callback1)

        odom2_topic = 'robot_' + str(2) + '/odom'
        self.odom_sub2 = rospy.Subscriber(odom2_topic, Odometry, self.odometry_callback2)

        odom3_topic = 'robot_' + str(3) + '/odom'
        self.odom_sub3 = rospy.Subscriber(odom3_topic, Odometry, self.odometry_callback3)

        odom4_topic = 'robot_' + str(4) + '/odom'
        self.odom_sub4 = rospy.Subscriber(odom4_topic, Odometry, self.odometry_callback4)

        odom5_topic = 'robot_' + str(5) + '/odom'
        self.odom_sub5 = rospy.Subscriber(odom5_topic, Odometry, self.odometry_callback5)
    
    def odometry_callback0(self,odometry):
        [goal_x,goal_y] = self.goals[5]
        x,y = odometry.pose.pose.position.x, odometry.pose.pose.position.y
        d = numpy.sqrt((goal_x-x)**2 + (goal_y - y)**2) 
        if(d>0.5 and (not self.firsts[0])):
            self.x_lis[0],self.y_list[0] = x,y
        else:
            self.firsts[0] = True

    def odometry_callback1(self,odometry):
        [goal_x,goal_y] = self.goals[4]
        x,y = odometry.pose.pose.position.x, odometry.pose.pose.position.y
        d = numpy.sqrt((goal_x-x)**2 + (goal_y - y)**2) 
        if(d>0.5 and (not self.firsts[1])):
            self.x_lis[1],self.y_list[1] = x,y
        else:
            self.firsts[1] = True

    def odometry_callback2(self,odometry):
        [goal_x,goal_y] = self.goals[3]
        x,y = odometry.pose.pose.position.x, odometry.pose.pose.position.y
        d = numpy.sqrt((goal_x-x)**2 + (goal_y - y)**2) 
        if(d>0.5 and (not self.firsts[2])):
            self.x_lis[2],self.y_list[2] = x,y
        else:
            self.firsts[2] = True

    def odometry_callback3(self,odometry):
        [goal_x,goal_y] = self.goals[2]
        x,y = odometry.pose.pose.position.x, odometry.pose.pose.position.y
        d = numpy.sqrt((goal_x-x)**2 + (goal_y - y)**2) 
        if(d>0.5 and (not self.firsts[3])):
            self.x_lis[3],self.y_list[3] = x,y
        else:
            self.firsts[3] = True

    def odometry_callback4(self,odometry):
        [goal_x,goal_y] = self.goals[1]
        x,y = odometry.pose.pose.position.x, odometry.pose.pose.position.y
        d = numpy.sqrt((goal_x-x)**2 + (goal_y - y)**2) 
        if(d>0.5 and (not self.firsts[4])):
            self.x_lis[4],self.y_list[4] = x,y
        else:
            self.firsts[4] = True

    def odometry_callback5(self,odometry):
        [goal_x,goal_y] = self.goals[0]
        x,y = odometry.pose.pose.position.x, odometry.pose.pose.position.y
        d = numpy.sqrt((goal_x-x)**2 + (goal_y - y)**2) 
        if(d>0.5 and (not self.firsts[5])):
            self.x_lis[5],self.y_list[5] = x,y
        else:
            self.firsts[5] = True


def record_trajectory(x_lis,y_list):
    x_lis = list(numpy.around(numpy.array(x_lis),2))
    y_list = list(numpy.around(numpy.array(y_list),2))
    with open('discrete.txt', 'a') as f:
        f.write(str(x_lis)+" ")
        f.write(str(y_list)+"\n")
    f.close()

if __name__ == '__main__':
    os.environ["ROS_MASTER_URI"]="http://localhost:%d"%11336
    rospy.init_node("record_data", anonymous=None)
    data_recorder = DataRecorder()
    while not rospy.is_shutdown():
        time.sleep(1.0)
        print("x:",data_recorder.x_lis)
        print("y:",data_recorder.y_list)
        record_trajectory(data_recorder.x_lis,data_recorder.y_list)

