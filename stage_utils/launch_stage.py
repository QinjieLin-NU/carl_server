#!/usr/bin/env python
import os
import time
import sys
import re
import matplotlib.pyplot as plt
import glob

def generate_roscoreCmd(rosport):
    cmd_rosocre = "roscore -p %d"%(rosport)
    cmd_bash = "exec bash"
    cmd = cmd_rosocre + ";"+ cmd_bash
    return cmd

def generate_stageCmd(rosport,world_path):
    cmd_exp = "export ROS_MASTER_URI=http://localhost:%s"%str(rosport)
    cmd_stage = "rosrun stage_ros_add_pose_and_crash stageros %s"%(world_path)
    cmd_bash = "exec bash"
    cmd = cmd_exp + ";" + cmd_stage + ";"+ cmd_bash
    return cmd


if __name__ == "__main__":
    rosport_start = 11333
    worlds_path = ["../worlds/stage_adMap2.world","../worlds/stage_adMap3.world","../worlds/stage_adMap4.world"]
    worlds_num = 3 #parallel stage nums
    rosport = rosport_start

    for i in range(worlds_num):
        for path in worlds_path:
            cmd_ros = generate_roscoreCmd(rosport)
            cmd_stage = generate_stageCmd(rosport,path)
            print(cmd_ros)
            print(cmd_stage)
            os.system("gnome-terminal --tab -e 'bash -c \"%s\" '"%cmd_ros)
            time.sleep(3)
            os.system("gnome-terminal --tab -e 'bash -c \"%s\" '"%cmd_stage)
            rosport +=1