import agent
import network
import util
import numpy as np
from mpi4py import MPI

import time
import tf
import os
import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Empty
from std_msgs.msg import Int8


class CadrlStage():
    def __init__(self,ros_port,envIndex):
        #load trained network
        self.possible_actions = network.Actions()
        self.num_actions = self.possible_actions.num_actions
        self.nn = network.NetworkVP_rnn(network.Config.DEVICE, 'network', self.num_actions)
        self.nn.simple_load('../checkpoints/network_01900000')
        
        os.environ["ROS_MASTER_URI"]="http://localhost:%d"%(ros_port)
        rospy.init_node("cadrl_stage", anonymous=None)
        
        rospy.sleep(1.)

        #-----------------subscriber and publisher----------#
        roboIndex = 0
        self.state = [0.0,0.0,1.0]
        self.speed = [0.0,0.0]
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

        self.reset_stage = rospy.ServiceProxy('reset_positions', Empty)
        self.reset_stage()
    
        rospy.sleep(1.)
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.envIndex = envIndex
        # self.get_agentCMD()

    def crash_callback(self, flag):
        self.is_crashed = flag.data

    def odometry_callback(self,odometry):
        Quaternions = odometry.pose.pose.orientation
        Euler = tf.transformations.euler_from_quaternion([Quaternions.x, Quaternions.y, Quaternions.z, Quaternions.w])
        self.psi = np.arctan2(2.0*(Quaternions.w*Quaternions.z + Quaternions.x*Quaternions.y), 1-2*(Quaternions.y*Quaternions.y+Quaternions.z*Quaternions.z)) # bounded by [-pi, pi]
        self.state = [odometry.pose.pose.position.x, odometry.pose.pose.position.y, Euler[2]]
        self.speed = [odometry.twist.twist.linear.x, odometry.twist.twist.angular.z]

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

    def observe_others(self,goal_x,goal_y):
        # Sample observation data in a format easily generated from sensors
        other_agents_x = [self.state_ad1[0],self.state_ad2[0],self.state_ad3[0]]
        other_agents_y = [self.state_ad1[1],self.state_ad2[1],self.state_ad3[1]]
        other_agents_r = [0.2, 0.2, 0.2]
        other_agents_vx = [self.speed_ad1[0], self.speed_ad2[0], self.speed_ad3[0]]
        other_agents_vy = [0.0, 0.0, 0.0]
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
        goals = raw_input("Press Enter to continue...")
        goals = goals.split(" ")        
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

if __name__ == "__main__":
    ROSPORT = 11323

    comm = MPI.COMM_WORLD
    rank = comm.Get_rank()
    size = comm.Get_size()
    envIndex = rank

    env = CadrlStage(ROSPORT+rank,envIndex)
    for i in range(5000):
        env.reset_stage()
        # env.generate_goal()
        env.generate_fixedGoal()
        result = ""
        terminate = False
        while not terminate:
            host_agent,goal_x,goal_y,terminate,result = env.get_agentState()
            other_agents = env.observe_others(goal_x,goal_y)

            #convert agent state into obs vector
            obs = host_agent.observe(other_agents)[1:]
            obs = np.expand_dims(obs, axis=0)

            #predict action from learned model
            action = env.predict_action(obs)
            env.control_vel(action)
            rospy.sleep(0.1)
        print(result)


