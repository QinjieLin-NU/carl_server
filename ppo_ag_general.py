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
from model.ppo import generate_action
from model.ppo import transform_buffer
from stage_ag_chase import StageWorld as StageWorld_chase
from stage_ag_push import StageWorld as StageWorld_push
from stage_ag_notPass import StageWorld as StageWorld_notPass
from stage_ag_general import StageWorld as StageWorld_general
from stage_utils.arg_parse import parse_args

MAX_EPISODES = 5000
EP_LEN = 200
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
LASER_NORM = False
ID_EP = 0

def run(comm, env, policy, policy_path, action_bound, optimizer):

    # rate = rospy.Rate(5)
    buff = []
    global_update = ID_EP
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
            v, a, logprob, scaled_action=generate_action(env=env, state_list=state_list,
                                                         policy=policy, action_bound=action_bound)

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
                # if(not LASER_NORM):
                #     obs = (obs + 0.5)*3.5
                obs_stack = deque([obs, obs, obs])
            if(step > EP_LEN):
                next_ep = True


            # get next state
            s_next = env.get_laser_observation()
            # if(not LASER_NORM):
            #     s_next = (s_next + 0.5) * 3.5
            left = obs_stack.popleft()
            obs_stack.append(s_next)
            goal_next = np.asarray(env.get_local_goal())
            speed_next = np.asarray(env.get_self_speed())
            state_next = [obs_stack, goal_next, speed_next]

            if global_step % HORIZON == 0:
                state_next_list = comm.gather(state_next, root=0)
                last_v, _, _, _ = generate_action(env=env, state_list=state_next_list, policy=policy,
                                                               action_bound=action_bound)
            # add transitons in buff and update policy
            r_list = comm.gather(r, root=0)
            terminal_list = comm.gather(terminal, root=0)

            if env.mpi_rank == 0:
                buff.append((state_list, a, r_list, terminal_list, logprob, v))
                if len(buff) > HORIZON - 1:
                    s_batch, goal_batch, speed_batch, a_batch, r_batch, d_batch, l_batch, v_batch = \
                        transform_buffer(buff=buff)
                    t_batch, advs_batch = generate_train_data(rewards=r_batch, gamma=GAMMA, values=v_batch,
                                                              last_value=last_v, dones=d_batch, lam=LAMDA)
                    memory = (s_batch, goal_batch, speed_batch, a_batch, l_batch, t_batch, v_batch, r_batch, advs_batch)
                    print("PPO update start")
                    ppo_update_stage1(policy=policy, optimizer=optimizer, batch_size=BATCH_SIZE, memory=memory,
                                            epoch=EPOCH, coeff_entropy=COEFF_ENTROPY, clip_value=CLIP_VALUE, num_step=HORIZON,
                                            num_env=NUM_ENV, frames=LASER_HIST,
                                            obs_size=OBS_SIZE, act_size=ACT_SIZE)

                    print("PPO update finish")
                    buff = []
                    global_update += 1

            step += 1
            state = state_next


        if env.mpi_rank == 0:
            if global_update != 0 and global_update % 20 == 0:
                torch.save(policy.state_dict(), policy_path + '/Stage1_{}'.format(global_update))
                logger.info('########################## model saved when update {} times#########'
                            '################'.format(global_update))
        # distance = np.sqrt((env.goal_point[0] - env.init_pose[0])**2 + (env.goal_point[1]-env.init_pose[1])**2)
        distance = np.sqrt((env.goal_point[0] - env.init_pose[0])**2 + (env.goal_point[1]-env.init_pose[1])**2)

        logger.info('Env %02d, Goal (%05.1f, %05.1f), Episode %05d, setp %03d, Reward %-5.1f, Distance %05.1f, %s' % \
                    (env.mpi_rank, env.goal_point[0], env.goal_point[1], id + 1, step, ep_reward, distance, result))
        print("befpre ep_reward")
        logger_cal.info(ep_reward)



def make_env(arg_list,rankId,size):
    scenarios = arg_list.scenarios
    rosports = arg_list.rosports
    robotIds = arg_list.robotIds
    if( not (size == len(robotIds))):
        return None
    robotIndex = robotIds[rankId]
    rosPort = rosports[rankId]
    envIndex = scenarios[rankId]
    env = None
    env = StageWorld_general(beam_num=360, index=robotIndex, num_env=size,ros_port = rosPort,mpi_rank = rankId,env_index = envIndex)
    return env

def make_env_v2(arg_list,rankId,size):

    scenarios = arg_list.scenarios
    rosports = arg_list.rosports
    robotIds = arg_list.robotIds

    num_scene = len(scenarios)
    rosport_start = rosports[0]

    robotIndex = robotIds[0]
    rosPort = rosport_start + rankId
    envIndex = scenarios[rankId%num_scene]
    env = None
    env = StageWorld_general(beam_num=360, index=robotIndex, num_env=size,ros_port = rosPort,mpi_rank = rankId,env_index = envIndex)
    print("rosport: %d, robotID: %d, envIndex: %d"%(rosPort,robotIndex,envIndex))
    return env

if __name__ == '__main__':
    comm = MPI.COMM_WORLD
    rank = comm.Get_rank()
    size = comm.Get_size()

    arg_list = parse_args()
    ID = arg_list.fileIds[0] #policy saved directory
    ID_EP = 0
    if(not (arg_list.modelEps==None)):
        ID_EP = arg_list.modelEps[0]
    NUM_ENV = size # number of robot
    LASER_NORM = True
    env = make_env_v2(arg_list,rank,size)
    if(env == None):
        print("making env fails")
        sys.exit()

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

    # logger.info('rosport: %d robotIndex: %d rank:%d' %(arg_list.rosports[rank],arg_list.robotIds[rank],rank))
    reward = None
    action_bound = [[0, -1], [1, 1]]

    # torch.manual_seed(1)
    # np.random.seed(1)
    if rank == 0:
        policy_path = policydir
        # policy_path = 'policy'
        # policy = MLPPolicy(obs_size, act_size)
        policy = CNNPolicy(frames=LASER_HIST, action_space=2)
        policy.cuda()
        opt = Adam(policy.parameters(), lr=LEARNING_RATE)
        mse = nn.MSELoss()

        if not os.path.exists(policy_path):
            os.makedirs(policy_path)

        file = policy_path + '/Stage1_%d'%ID_EP
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