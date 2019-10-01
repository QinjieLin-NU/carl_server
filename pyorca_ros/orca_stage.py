from __future__ import division

from pyorca import Agent, get_avoidance_velocity, orca, normalized, perp, AdversarialAgent
from numpy import array, rint, linspace, pi, cos, sin
import itertools
import random
import numpy as np
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

def init_logger():
    testLogName = '/test.log'

    # config log
    dirname = '/clever/saved_model_ppo/autoRL_99999' 
    logdir = dirname + "/log"
    if not os.path.exists(dirname):
        os.makedirs(dirname)
    if not os.path.exists(logdir):
        os.makedirs(logdir)
    cal_file = logdir + testLogName
    output_file =logdir + '/output.log'

    # config log
    logger = logging.getLogger('mylogger')
    logger.setLevel(logging.INFO)

    file_handler = logging.FileHandler(output_file, mode='a')
    file_handler.setLevel(logging.INFO)
    file_handler.setFormatter(logging.Formatter("%(asctime)s - %(levelname)s - %(message)s"))
    stdout_handler = logging.StreamHandler(sys.stdout)
    stdout_handler.setLevel(logging.INFO)
    logger.addHandler(file_handler)
    logger.addHandler(stdout_handler)

    logger_cal = logging.getLogger('loggercal')
    logger_cal.setLevel(logging.INFO)
    cal_f_handler = logging.FileHandler(cal_file, mode='a')
    file_handler.setLevel(logging.INFO)
    logger_cal.addHandler(cal_f_handler)

    return logger_cal


if __name__ == "__main__":
    ROSPORT = 11638
    log_cal = init_logger()
    comm = MPI.COMM_WORLD
    rank = comm.Get_rank()
    size = comm.Get_size()
    envIndex = rank

    #init orca node
    os.environ["ROS_MASTER_URI"]="http://localhost:%d"%(ROSPORT+rank*3)
    rospy.init_node("test_orca", anonymous=None)
    reset_stage = rospy.ServiceProxy('reset_positions', Empty)
    reset_stage()

    #init orca agents
    env= Agent(robo_index = 0, radius = 0.3,max_speed = 1,goal=[0,4],env_index = 2)
    ad0 = AdversarialAgent(robo_index = 1,radius = 0.3, max_speed = 1, goal = [-3,-3],env_index=rank)
    ad1 = AdversarialAgent(robo_index=2,radius = 0.3,max_speed = 1, goal = [0,-3],env_index = rank)
    ad2 = AdversarialAgent(robo_index=3,radius = 0.3,max_speed = 1, goal = [3,-3],env_index = rank)
    agents = [env,ad0,ad1,ad2]

    #init orca parameters
    time.sleep(5)
    running = True
    FPS = 10
    dt = 1/FPS
    tau = 2
    step = 0
    all_lines = [[]] * len(agents)

    for id in range(5000):
        reset_stage()
        env.generate_goal()
        result = ""
        terminate = False

        rospy.sleep(0.5)
        startPose = [env.state[0],env.state[1]]
        startTime = time.time()
        goalPose = env.goal
        deltaDistance = 0.0
        nowPos = [env.state[0],env.state[1]]
        lastPos = list(nowPos)
        reachFlag = 0
        step = 0

        while not terminate:
            #get state
            terminate,result = env.get_agentState()

            #predict action using orca
            new_vels = [None] * len(agents)
            for i, agent in enumerate(agents):
                candidates = agents[:i] + agents[i + 1:]
                new_vels[i], all_lines[i] = orca(agent, candidates, tau, dt)
            env.control_vel(new_vels[0])
            rospy.sleep(0.1)

            # calculate daltaDis
            nowPos = [env.state[0],env.state[1]]
            tempDeltaDis = np.sqrt((nowPos[0] - lastPos[0])**2 + (nowPos[1]-lastPos[1])**2)
            deltaDistance += tempDeltaDis
            step +=1
            lastPos = nowPos
            if(result == "Reach Goal"):
                reachFlag =1
            if(step > 100):
                reachFlag = 2
                break

        env.stop()
        endTime = time.time()
        deltaTime = (endTime - startTime)
        stateInfo = "scenarioId: %d, start:(%4f,%4f), goal: (%4f,%4f), state: %d, time: %4f, distance: %4f"\
            %(rank,startPose[0],startPose[1],goalPose[0],goalPose[1],reachFlag,deltaTime,deltaDistance)
        if((id>=10) and (id<60)):
            print(stateInfo)
            log_cal.info(stateInfo)
