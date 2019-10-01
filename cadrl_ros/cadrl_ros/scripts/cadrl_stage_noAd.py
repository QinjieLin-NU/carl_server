import agent
import network
import util
import numpy as np
from mpi4py import MPI
import logging

import time
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


class CadrlStage():
    def __init__(self,ros_port,envIndex,roboIndex):
        #load trained network
        self.possible_actions = network.Actions()
        self.num_actions = self.possible_actions.num_actions
        self.nn = network.NetworkVP_rnn(network.Config.DEVICE, 'network', self.num_actions)
        self.nn.simple_load('../checkpoints/network_01900000')
        
        os.environ["ROS_MASTER_URI"]="http://localhost:%d"%(ros_port)
        rospy.init_node("cadrl_stage_%d"%roboIndex, anonymous=None)
        
        rospy.sleep(1.)

        #-----------------subscriber and publisher----------#
        self.roboIndex = roboIndex
        self.state = [0.0,0.0,1.0]
        self.speed = [0.0,0.0]
        cmd_pose_topic = 'robot_' + str(roboIndex) + '/cmd_pose'
        self.cmd_pose = rospy.Publisher(cmd_pose_topic, Pose, queue_size=2)

        odom_topic = 'robot_' + str(roboIndex) + '/odom'
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odometry_callback)

        cmd_vel_topic = 'robot_' + str(roboIndex) + '/cmd_vel'
        self.cmd_vel = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        self.reach_flag = Int8()
        self.reach_flag.data = 8
        reach_topic = 'robot_' + str(roboIndex) + '/is_reached'
        self.reach_publisher = rospy.Publisher(reach_topic, Int8, queue_size=2)

        crash_topic = 'robot_' + str(roboIndex) + '/is_crashed'
        self.check_crash = rospy.Subscriber(crash_topic, Int8, self.crash_callback)

        goal_topic = 'robot_' + str(roboIndex) + '/goal_pose'
        self.goal_pub = rospy.Publisher(goal_topic, Pose, queue_size=10)

        self.state_ad0 = [0.0,0.0,1.0]
        self.speed_ad0 = [0.0,0.0]
        odom_topic_ad0 = 'robot_' + str(0) + '/odom'
        self.odom_sub_ad0 = rospy.Subscriber(odom_topic_ad0, Odometry, self.odometry_callback_ad0)

        self.state_ad1 = [0.0,0.0,1.0]
        self.speed_ad1 = [0.0,0.0]
        odom_topic_ad1 = 'robot_' + str(1) + '/odom'
        self.odom_sub_ad1 = rospy.Subscriber(odom_topic_ad1, Odometry, self.odometry_callback_ad1)

        self.state_ad2 = [0.0,0.0,1.0]
        self.speed_ad2 = [0.0,0.0]
        odom_topic_ad2 = 'robot_' + str(2) + '/odom'
        self.odom_sub_ad2 = rospy.Subscriber(odom_topic_ad2, Odometry, self.odometry_callback_ad2)

        self.state_ad3 = [0.0,0.0,1.0]
        self.speed_ad3 = [0.0,0.0]
        odom_topic_ad3 = 'robot_' + str(3) + '/odom'
        self.odom_sub_ad3 = rospy.Subscriber(odom_topic_ad3, Odometry, self.odometry_callback_ad3)

        self.state_ad4 = [0.0,0.0,1.0]
        self.speed_ad4 = [0.0,0.0]
        odom_topic_ad4 = 'robot_' + str(4) + '/odom'
        self.odom_sub_ad4 = rospy.Subscriber(odom_topic_ad4, Odometry, self.odometry_callback_ad4)

        self.state_ad5 = [0.0,0.0,1.0]
        self.speed_ad5 = [0.0,0.0]
        odom_topic_ad5 = 'robot_' + str(5) + '/odom'
        self.odom_sub_ad5 = rospy.Subscriber(odom_topic_ad5, Odometry, self.odometry_callback_ad5)

        self.reset_stage = rospy.ServiceProxy('reset_positions', Empty)
        self.reset_stage()

        self.first_pose = None
        while self.first_pose is None:
            try:
                self.first_pose = rospy.wait_for_message(odom_topic, Odometry, timeout=5).pose.pose
            except:
                pass

        rospy.sleep(1.)
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.envIndex = envIndex
        self.psi = 0
        self.is_crashed = 0
        # self.get_agentCMD()

    def crash_callback(self, flag):
        self.is_crashed = flag.data

    def odometry_callback(self,odometry):
        Quaternions = odometry.pose.pose.orientation
        Euler = tf.transformations.euler_from_quaternion([Quaternions.x, Quaternions.y, Quaternions.z, Quaternions.w])
        self.psi = np.arctan2(2.0*(Quaternions.w*Quaternions.z + Quaternions.x*Quaternions.y), 1-2*(Quaternions.y*Quaternions.y+Quaternions.z*Quaternions.z)) # bounded by [-pi, pi]
        self.state = [odometry.pose.pose.position.x, odometry.pose.pose.position.y, Euler[2]]
        self.speed = [odometry.twist.twist.linear.x, odometry.twist.twist.angular.z]

    def odometry_callback_ad0(self,odometry):
        Quaternions = odometry.pose.pose.orientation
        Euler = tf.transformations.euler_from_quaternion([Quaternions.x, Quaternions.y, Quaternions.z, Quaternions.w])
        self.psi_ad0 = np.arctan2(2.0*(Quaternions.w*Quaternions.z + Quaternions.x*Quaternions.y), 1-2*(Quaternions.y*Quaternions.y+Quaternions.z*Quaternions.z)) # bounded by [-pi, pi]
        self.state_ad0 = [odometry.pose.pose.position.x, odometry.pose.pose.position.y, Euler[2]]
        self.speed_ad0 = [odometry.twist.twist.linear.x, odometry.twist.twist.angular.z]

    def odometry_callback_ad1(self,odometry):
        Quaternions = odometry.pose.pose.orientation
        Euler = tf.transformations.euler_from_quaternion([Quaternions.x, Quaternions.y, Quaternions.z, Quaternions.w])
        self.psi_ad1 = np.arctan2(2.0*(Quaternions.w*Quaternions.z + Quaternions.x*Quaternions.y), 1-2*(Quaternions.y*Quaternions.y+Quaternions.z*Quaternions.z)) # bounded by [-pi, pi]
        self.state_ad1 = [odometry.pose.pose.position.x, odometry.pose.pose.position.y, Euler[2]]
        self.speed_ad1 = [odometry.twist.twist.linear.x, odometry.twist.twist.angular.z]

    def odometry_callback_ad2(self,odometry):
        Quaternions = odometry.pose.pose.orientation
        Euler = tf.transformations.euler_from_quaternion([Quaternions.x, Quaternions.y, Quaternions.z, Quaternions.w])
        self.psi_ad2 = np.arctan2(2.0*(Quaternions.w*Quaternions.z + Quaternions.x*Quaternions.y), 1-2*(Quaternions.y*Quaternions.y+Quaternions.z*Quaternions.z)) # bounded by [-pi, pi]
        self.state_ad2 = [odometry.pose.pose.position.x, odometry.pose.pose.position.y, Euler[2]]
        self.speed_ad2 = [odometry.twist.twist.linear.x, odometry.twist.twist.angular.z]

    def odometry_callback_ad3(self,odometry):
        Quaternions = odometry.pose.pose.orientation
        Euler = tf.transformations.euler_from_quaternion([Quaternions.x, Quaternions.y, Quaternions.z, Quaternions.w])
        self.psi_ad3 = np.arctan2(2.0*(Quaternions.w*Quaternions.z + Quaternions.x*Quaternions.y), 1-2*(Quaternions.y*Quaternions.y+Quaternions.z*Quaternions.z)) # bounded by [-pi, pi]
        self.state_ad3 = [odometry.pose.pose.position.x, odometry.pose.pose.position.y, Euler[2]]
        self.speed_ad3 = [odometry.twist.twist.linear.x, odometry.twist.twist.angular.z]

    def odometry_callback_ad4(self,odometry):
        Quaternions = odometry.pose.pose.orientation
        Euler = tf.transformations.euler_from_quaternion([Quaternions.x, Quaternions.y, Quaternions.z, Quaternions.w])
        self.psi_ad4 = np.arctan2(2.0*(Quaternions.w*Quaternions.z + Quaternions.x*Quaternions.y), 1-2*(Quaternions.y*Quaternions.y+Quaternions.z*Quaternions.z)) # bounded by [-pi, pi]
        self.state_ad4 = [odometry.pose.pose.position.x, odometry.pose.pose.position.y, Euler[2]]
        self.speed_ad4 = [odometry.twist.twist.linear.x, odometry.twist.twist.angular.z]

    def odometry_callback_ad5(self,odometry):
        Quaternions = odometry.pose.pose.orientation
        Euler = tf.transformations.euler_from_quaternion([Quaternions.x, Quaternions.y, Quaternions.z, Quaternions.w])
        self.psi_ad5 = np.arctan2(2.0*(Quaternions.w*Quaternions.z + Quaternions.x*Quaternions.y), 1-2*(Quaternions.y*Quaternions.y+Quaternions.z*Quaternions.z)) # bounded by [-pi, pi]
        self.state_ad5 = [odometry.pose.pose.position.x, odometry.pose.pose.position.y, Euler[2]]
        self.speed_ad5 = [odometry.twist.twist.linear.x, odometry.twist.twist.angular.z]

    def observe_others(self,goal_x,goal_y):
        # Sample observation data in a format easily generated from sensors
        agents_x = [self.state_ad0[0],self.state_ad1[0],self.state_ad2[0],self.state_ad3[0],self.state_ad4[0],self.state_ad5[0]]
        agents_y = [self.state_ad0[1],self.state_ad1[1],self.state_ad2[1],self.state_ad3[1],self.state_ad4[1],self.state_ad5[1]]
        agents_r = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
        agents_vx = [self.speed_ad0[0],self.speed_ad1[0], self.speed_ad2[0], self.speed_ad3[0],self.speed_ad4[0],self.speed_ad5[0]]
        agents_vy = [0.0, 0.0, 0.0,0.0, 0.0, 0.0]

        other_agents_x = agents_x[:self.roboIndex] + agents_x[self.roboIndex+1:]
        other_agents_y = agents_y[:self.roboIndex] + agents_y[self.roboIndex+1:]
        other_agents_r = agents_r[:self.roboIndex] + agents_r[self.roboIndex+1:]
        other_agents_vx = agents_vx[:self.roboIndex] + agents_vx[self.roboIndex+1:]
        other_agents_vy = agents_vy[:self.roboIndex] + agents_vy[self.roboIndex+1:]

        num_other_agents = len(other_agents_x)

        # Create Agent objects for each observed dynamic obstacle
        other_agents = []
        for i in range(num_other_agents):
            x = other_agents_x[i]; y = other_agents_y[i]
            v_x = other_agents_vx[i]; v_y = other_agents_vy[i]
            radius = other_agents_r[i]
            
            other_agent = agent.Agent(x, y, goal_x, goal_y, radius=radius, id=i+1)
            other_agent.vel_global_frame = np.array([v_x, v_y])
            other_agents.append(other_agent)
        
        return other_agents

    def generate_goal(self):
        # goals = raw_input("Press Enter to continue...")
        goals = self.generate_stage_goal()     
        self.goal_x = float(goals[0])
        self.goal_y = float(goals[1])
        
        goal = Pose()
        goal.position.x = self.goal_x
        goal.position.y = self.goal_y 
        self.goal_pub.publish(goal)

    def generate_fixedGoal(self):
        self.goal_x = 0.0
        self.goal_y = 4.0
        if(self.envIndex == 1):
            self.goal_x = 0.0
            self.goal_y = 0.0

        goal = Pose()
        goal.position.x = self.goal_x
        goal.position.y = self.goal_y 
        self.goal_pub.publish(goal)

    def get_agentState(self):
        #set current state of host agent
        [start_x,start_y,heading_angle] = [self.state[0],self.state[1],self.psi]
        # [v_x,v_y] = [self.speed[0]*np.cos(heading_angle),self.speed[0]*np.sin(heading_angle)]
        [v_x,v_y] = [self.speed[0],0]
        goal_x = self.goal_x
        goal_y = self.goal_y
        radius = 0.5
        pref_speed = 1.0
        index = 0
        terminate = False
        result = ''

        host_agent = agent.Agent(start_x, start_y, goal_x, goal_y, 
                                radius=radius, pref_speed=pref_speed, initial_heading=heading_angle,
                                id=index)
        host_agent.vel_global_frame = np.array([v_x, v_y])

        delta_distance = np.sqrt((start_x - goal_x)**2 + (start_y - goal_y)**2)
        if(delta_distance < 0.5):
            self.reach_publisher.publish(self.reach_flag)
            terminate = True
            result = 'reach'

        if self.is_crashed == 1:
            terminate = True
            result = 'crash'

        return host_agent,goal_x,goal_y,terminate,result

    def get_agentCMD(self):
        host_agent,goal_x,goal_y = self.get_agentState()

        other_agents = self.observe_others(goal_x,goal_y)

        #convert agent state into obs vector
        obs = host_agent.observe(other_agents)[1:]
        obs = np.expand_dims(obs, axis=0)

        #QUery the policy based on obs vector
        predictions = self.nn.predict_p(obs)[0]
        raw_action = self.possible_actions.actions[np.argmax(predictions)]
        action = np.array([host_agent.pref_speed*raw_action[0], util.wrap(raw_action[1] + host_agent.heading_global_frame)])
        print "action:", action
        # Action contains: [new forward speed, change in heading angle]
        self.control_vel(action)

    def predict_action(self,obs):
        #QUery the policy based on obs vector
        predictions = self.nn.predict_p(obs)[0]
        raw_action = self.possible_actions.actions[np.argmax(predictions)]
        action = np.array([host_agent.pref_speed*raw_action[0], util.wrap(raw_action[1] + host_agent.heading_global_frame)])
        # print "action:", action
        # Action contains: [new forward speed, change in heading angle]
        return action

    def control_vel(self, action):
        move_cmd = Twist()
        move_cmd.linear.x = action[0]
        move_cmd.linear.y = 0.
        move_cmd.linear.z = 0.
        
        move_cmd.angular.x = 0.
        move_cmd.angular.y = 0.
        yaw_error = action[1] - self.psi
        if yaw_error > np.pi:
            yaw_error -= 2*np.pi
        if yaw_error < -np.pi:
            yaw_error += 2*np.pi
        move_cmd.angular.z = 2*yaw_error

        self.cmd_vel.publish(move_cmd)

    def stop_agent(self):
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)

    def generate_random_goal_v2(self):
        self.init_pose = self.state
        x = np.random.uniform(-5.5, 5.5)
        y = np.random.uniform(-5.5, 5.5)
        dis_origin = np.sqrt(x ** 2 + y ** 2)
        dis_goal = np.sqrt((x - self.init_pose[0]) ** 2 + (y - self.init_pose[1]) ** 2)
        while (dis_origin > 5.5 or dis_goal > 5.5 or dis_goal < 4) and not rospy.is_shutdown():
            x = np.random.uniform(-5.5, 5.5)
            y = np.random.uniform(-5.5, 5.5)
            dis_origin = np.sqrt(x ** 2 + y ** 2)
            dis_goal = np.sqrt((x - self.init_pose[0]) ** 2 + (y - self.init_pose[1]) ** 2)

        return [x, y]

    def generate_random_goal_v3(self):
        self.init_pose = self.state
        x = np.random.uniform(-5.5, 5.5)
        y = np.random.uniform(0, 5.5)
        dis_origin = np.sqrt(x ** 2 + y ** 2)
        dis_goal = np.sqrt((x - self.init_pose[0]) ** 2 + (y - self.init_pose[1]) ** 2)
        while (dis_origin > 5.5 or dis_goal > 5.5 or dis_goal < 4) and not rospy.is_shutdown():
            x = np.random.uniform(-5.5, 5.5)
            y = np.random.uniform(0, 5.5)
            dis_origin = np.sqrt(x ** 2 + y ** 2)
            dis_goal = np.sqrt((x - self.init_pose[0]) ** 2 + (y - self.init_pose[1]) ** 2)
        return [x, y]

    def generate_random_goal_v4(self):
        # agent( pose [7.73 -2.07 0.00 525.00])
        # agent( pose [8.00 0.00 0.00 180.00])
        # agent( pose [7.73 2.07 0.00 195.00])
        # agent( pose [-7.73 2.07 0.00 345.00])
        # agent( pose [-8.00 -0.00 0.00 360.00])
        # agent( pose [-7.73 -2.07 0.00 375.00])
        goals = [[7,-2],[8,0],[7,2],[-7,-2],[-8,0],[-7,2]]
        goalIndex = 5 - self.roboIndex
        return goals[goalIndex]

    def generate_stage_goal(self):   
        if(self.envIndex == 0):
            return self.generate_random_goal_v2()
        elif (self.envIndex ==1):
            return self.generate_random_goal_v2()
        elif (self.envIndex ==2):
            return self.generate_random_goal_v3()
        elif (self.envIndex ==3):
            return self.generate_random_goal_v4()
    
    def reset_pose(self):
        self.cmd_pose.publish(self.first_pose)

def init_logger():
    testLogName = '/test_nothing.log'

    # config log
    dirname = '/clever/saved_model_ppo/autoRL_88888' 
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
    ROSPORT = 11336
    log_cal = init_logger()

    comm = MPI.COMM_WORLD
    rank = comm.Get_rank()
    size = comm.Get_size()

    #setting for init node
    envIndex = 3
    robotIndex = rank
    ros_port = ROSPORT 

    env = CadrlStage(ros_port,envIndex,robotIndex)
    for id in range(5000):
        env.reset_pose()
        # env.generate_fixedGoal()
        env.generate_goal()
        result = ""
        terminate = False

        rospy.sleep(0.5)
        startPose = [env.state[0],env.state[1]]
        startTime = time.time()
        goalPose = [env.goal_x,env.goal_y]
        deltaDistance = 0.0
        nowPos = [env.state[0],env.state[1]]
        lastPos = list(nowPos)
        reachFlag = 0
        step = 0

        while not terminate:
            host_agent,goal_x,goal_y,terminate,result = env.get_agentState()
            other_agents = env.observe_others(goal_x,goal_y)

            #convert agent state into obs vector
            obs = host_agent.observe(other_agents)[1:]
            obs = np.expand_dims(obs, axis=0)

            #predict action from learned model
            action = env.predict_action(obs)
            env.control_vel(action)
            rospy.sleep(0.001)

            # calculate daltaDis
            nowPos = [env.state[0],env.state[1]]
            tempDeltaDis = np.sqrt((nowPos[0] - lastPos[0])**2 + (nowPos[1]-lastPos[1])**2)
            deltaDistance += tempDeltaDis
            step +=1
            lastPos = nowPos
            if(result == "reach"):
                reachFlag =1
            if(step > 5000):
                reachFlag = 2
                break

        env.stop_agent()
        endTime = time.time()
        deltaTime = (endTime - startTime)
        stateInfo = "scenarioId: %d, start:(%4f,%4f), goal: (%4f,%4f), state: %d, time: %4f, distance: %4f"\
            %(rank,startPose[0],startPose[1],goalPose[0],goalPose[1],reachFlag,deltaTime,deltaDistance)
        if((id>=10) and (id<110)):
            print(stateInfo)
            log_cal.info(stateInfo)


