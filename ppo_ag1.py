#!/usr/bin/env python
import os
import numpy as np
import rospy
import torch
import torch.nn as nn
#from mpi4py import MPI

from torch.optim import Adam
from collections import deque

from model.net import MLPPolicy, CNNPolicy
from stage_ag1 import StageWorld
from model.ppo import generate_action_no_sampling, transform_buffer
import copy




MAX_EPISODES = 5000
LASER_BEAM = 360
LASER_HIST = 3
HORIZON = 200
GAMMA = 0.99
LAMDA = 0.95
BATCH_SIZE = 512
EPOCH = 3
COEFF_ENTROPY = 5e-4
CLIP_VALUE = 0.1
NUM_ENV = 50
OBS_SIZE = 360
ACT_SIZE = 2
LEARNING_RATE = 5e-5


def enjoy(env, policy, action_bound):


    # if env.index == 0:
    #     env.reset_world()

    #env.reset_pose()

    env.generate_goal_point()
    step = 1
    terminal = False
    result = "begin"

    obs = env.get_laser_observation()
    obs_stack = deque([obs, obs, obs])
    env.generate_goal_point()
    goal = np.asarray(env.get_local_goal())
    speed = np.asarray(env.get_self_speed())
    state = [obs_stack, goal, speed]

    while not rospy.is_shutdown():
        # get next state
        s_next = env.get_laser_observation()
        left = obs_stack.popleft()
        obs_stack.append(s_next)
 
        # if terminal == True:
        #     real_action = [0,0]
        #     env.control_vel(real_action)  
        #     env.generate_goal_point()
        #     print("new goal:",env.goal_point)
        if result == "Crashed":
            real_action = [0,0]
            env.control_vel(real_action) 
            env.reset_pose()
        
        if result == 'Reach Goal':
            real_action = [0,0]
            env.control_vel(real_action)  
            env.generate_goal_point()
            print("new goal:",env.goal_point)
            
        goal_next = np.asarray(env.get_local_goal())
        speed_next = np.asarray(env.get_self_speed())
        state_next = [obs_stack, goal_next, speed_next]

        state = state_next

        #state_list = comm.gather(state, root=0)
        state_list=[]
        state_list.append(state)

        # generate actions at rank==0
        mean, scaled_action =generate_action_no_sampling(env=env, state_list=state_list,
                                            policy=policy, action_bound=action_bound)      
        # execute actions
        real_action = copy.deepcopy(scaled_action[0])

        env.control_vel(real_action)
        # rate.sleep()
        rospy.sleep(0.01)
        # get informtion
        r, terminal, result = env.get_reward_and_terminate(step)
        step += 1

        if(step>200):
            result = "Crashed"
            step = 0




if __name__ == '__main__':

    robotIndex = 0
    NUM_ENV = 1
    rosPort = 11321
    rank = 0
    #generate random goals
    envIndex = 1 

    filename = "autoRL_5"
    load_ep = 300#1760
    policy_path = os.path.dirname(os.path.abspath(__file__))+"/../../../../../../clever/saved_model_ppo/"
    file = policy_path + filename+'/Stage1_%d'%load_ep
    print("filename:",file)
    print("begin to load world")
    env = StageWorld(beam_num=360, index=robotIndex, num_env=NUM_ENV,ros_port = rosPort,mpi_rank = rank,env_index = envIndex)
    print("generate the world")
    reward = None
    action_bound = [[0, -3], [1, 3]]


    policy_path = 'policy'
    # policy = MLPPolicy(obs_size, act_size)
    policy = CNNPolicy(frames=LASER_HIST, action_space=2)
    policy.cuda()
    opt = Adam(policy.parameters(), lr=LEARNING_RATE)
    mse = nn.MSELoss()

    if not os.path.exists(policy_path):
        os.makedirs(policy_path)

    # file = policy_path + '/Stage1_800'
    if os.path.exists(file):
        print ('####################################')
        print ('############Loading Model###########')
        print ('####################################')
        state_dict = torch.load(file)
        policy.load_state_dict(state_dict)
    else:
        print ('Error: Policy File Cannot Find')
        exit()

    try:
        enjoy(env=env, policy=policy, action_bound=action_bound)
    except KeyboardInterrupt:
        import traceback
        traceback.print_exc()
