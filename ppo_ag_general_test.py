import os
import logging
import sys
import socket
import numpy as np
import rospy
import torch
import torch.nn as nn
from mpi4py import MPI

from torch.optim import Adam
from collections import deque

from model.net import MLPPolicy, CNNPolicy
from model.ppo import ppo_update_stage1, generate_train_data
from model.ppo import generate_action, generate_action_no_sampling
from model.ppo import transform_buffer
from stage_ag_chase import StageWorld as StageWorld_chase
from stage_ag_push import StageWorld as StageWorld_push
from stage_ag_notPass import StageWorld as StageWorld_notPass

MAX_EPISODES = 5000
EP_LEN = 400
LASER_BEAM = 360
LASER_HIST = 3
HORIZON = 128
GAMMA = 0.99
LAMDA = 0.95
BATCH_SIZE = 1024
EPOCH = 2
COEFF_ENTROPY = 5e-4
CLIP_VALUE = 0.1
NUM_ENV = 3
OBS_SIZE = 360
ACT_SIZE = 2
LEARNING_RATE = 5e-5
LASER_NORM = True

def run(comm, env, policy, policy_path, action_bound, optimizer):

    # rate = rospy.Rate(5)
    buff = []
    global_update = 0
    global_step = 0


    if env.index == 0:
        env.reset_world()
    env.store_resetPose()


    for id in range(MAX_EPISODES):
        env.generate_goal_point()
        print("new goal:",env.goal_point,"and rnak:",env.mpi_rank)
        terminal = False
        next_ep = False
        ep_reward = 0
        step = 1

        env.reset_pose()

        obs = env.get_laser_observation()
        if(not LASER_NORM):
            obs = (obs + 0.5)*3.5
        obs_stack = deque([obs, obs, obs])
        goal = np.asarray(env.get_local_goal())
        speed = np.asarray(env.get_self_speed())
        state = [obs_stack, goal, speed]

        while not next_ep and not rospy.is_shutdown():
            state_list = comm.gather(state, root=0)


            # generate actions at rank==0
            mean, scaled_action=generate_action_no_sampling(env=env, state_list=state_list,policy=policy, action_bound=action_bound)

            # execute actions
            real_action = comm.scatter(scaled_action, root=0)
            env.control_vel(real_action)


            # rate.sleep()
            rospy.sleep(0.001)

            # get informtion
            r, terminal, result = env.get_reward_and_terminate(step)
            ep_reward += r
            global_step += 1

            if(result == "Reach Goal"):
                env.generate_goal_point()
                print("new goal:",env.goal_point)
            if (terminal):
                env.reset_pose()
                obs = env.get_laser_observation()
                if(not LASER_NORM):
                    obs = (obs + 0.5)*3.5
                obs_stack = deque([obs, obs, obs])


            # get next state
            s_next = env.get_laser_observation()
            if(not LASER_NORM):
                s_next = (s_next + 0.5) * 3.5
            left = obs_stack.popleft()
            obs_stack.append(s_next)
            goal_next = np.asarray(env.get_local_goal())
            speed_next = np.asarray(env.get_self_speed())
            state_next = [obs_stack, goal_next, speed_next]


            # add transitons in buff and update policy
            r_list = comm.gather(r, root=0)
            step += 1
            state = state_next





if __name__ == '__main__':
    ROS_PORT0 = 11323 #ros port starty from 11321
    ID = 25#21#25# 21 #policy saved directory
    NUM_ENV = 3 # number of robot
    POLICY_NAME = "/Stage1_260"#"/Stage1_9940"#"/Stage1_9940"
    LASER_NORM = False

    # config log
    # hostname = socket.gethostname()
    hostname = "autoRL_%d"%ID
    dirname = '/clever/saved_model_ppo/' + hostname
    logdir = dirname + "/log"
    policydir = dirname
    if not os.path.exists(dirname):
        os.makedirs(dirname)
    if not os.path.exists(logdir):
        os.makedirs(logdir)
    output_file =logdir + '/output.log'
    cal_file = logdir + '/cal.log'

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

    comm = MPI.COMM_WORLD
    rank = comm.Get_rank()
    size = comm.Get_size()
    robotIndex = 0
    rosPort = ROS_PORT0 + rank
    env = None
    if(rank == 0):
        envIndex = 8
        env = StageWorld_chase(beam_num=360, index=robotIndex, num_env=NUM_ENV,ros_port = rosPort,mpi_rank = rank,env_index = envIndex)
    elif (rank == 1):
        envIndex = 9
        env = StageWorld_push(beam_num=360, index=robotIndex, num_env=NUM_ENV,ros_port = rosPort,mpi_rank = rank,env_index = envIndex)
    elif (rank == 2):
        envIndex = 10
        env = StageWorld_notPass(beam_num=360, index=robotIndex, num_env=NUM_ENV,ros_port = rosPort,mpi_rank = rank,env_index = envIndex)

    logger.info('rosport: %d robotIndex: %d rank:%d' %(rosPort,robotIndex,rank))
    reward = None
    action_bound = [[0, -1], [1, 1]]

    # torch.manual_seed(1)
    # np.random.seed(1)
    if rank == 0:
        policy_path = policydir
        policy = CNNPolicy(frames=LASER_HIST, action_space=2)
        policy.cuda()
        opt = Adam(policy.parameters(), lr=LEARNING_RATE)
        mse = nn.MSELoss()

        if not os.path.exists(policy_path):
            os.makedirs(policy_path)

        file = policy_path + POLICY_NAME
        if os.path.exists(file):
            logger.info('####################################')
            logger.info('############Loading Model###########')
            logger.info('####################################')
            state_dict = torch.load(file)
            policy.load_state_dict(state_dict)
        else:
            logger.info('#####################################')
            logger.info('############Start Training###########')
            logger.info('#####################################')
    else:
        policy = None
        policy_path = None
        opt = None

    try:
        run(comm=comm, env=env, policy=policy, policy_path=policy_path, action_bound=action_bound, optimizer=opt)
    except KeyboardInterrupt:
        pass