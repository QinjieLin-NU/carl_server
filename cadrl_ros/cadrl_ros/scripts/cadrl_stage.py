import agent
import network
import util
import numpy as np

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
    def __init__(self):
        #load trained network
        self.possible_actions = network.Actions()
        self.num_actions = self.possible_actions.num_actions
        self.nn = network.NetworkVP_rnn(network.Config.DEVICE, 'network', self.num_actions)
        self.nn.simple_load('../checkpoints/network_01900000')

        os.environ["ROS_MASTER_URI"]="http://localhost:%d"%11323
        rospy.init_node("cadrl_stage", anonymous=None)
        
        roboIndex = 0
        self.state = [0.0,0.0,1.0]
        self.speed = [0.0,0.0]
        odom_topic = 'robot_' + str(roboIndex) + '/odom'
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odometry_callback)

        cmd_vel_topic = 'robot_' + str(roboIndex) + '/cmd_vel'
        self.cmd_vel = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        print("*************cmd_topic****************",cmd_vel_topic)

        # self.host_agent = None
        self.get_agentCMD()

    def odometry_callback(self,odometry):
        Quaternions = odometry.pose.pose.orientation
        Euler = tf.transformations.euler_from_quaternion([Quaternions.x, Quaternions.y, Quaternions.z, Quaternions.w])
        self.state = [odometry.pose.pose.position.x, odometry.pose.pose.position.y, Euler[2]]
        self.speed = [odometry.twist.twist.linear.x, odometry.twist.twist.angular.z]

    def observe_others(self,goal_x,goal_y):
        # Sample observation data in a format easily generated from sensors
        other_agents_x = [-1,-2,-3]
        other_agents_y = [2,3,4]
        other_agents_r = [0.5, 0.4, 0.3]
        other_agents_vx = [1.0, 0.6, 0.2]
        other_agents_vy = [0.0, 0.6, 0.8]
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
    
    def get_agentState(self):
        #set current state of host agent
        [start_x,start_y,heading_angle] = self.state
        [v_x,v_y] = [self.speed[0]*np.cos(heading_angle),self.speed[1]*np.sin(heading_angle)]
        print(start_x,start_y,v_x,v_y)
        goal_x = 3
        goal_y = 2
        radius = 0.5
        pref_speed = 0.9
        index = 0

        host_agent = agent.Agent(start_x, start_y, goal_x, goal_y, 
                                radius=radius, pref_speed=pref_speed, initial_heading=heading_angle,
                                id=index)
        host_agent.vel_global_frame = np.array([v_x, v_y])

        return host_agent,goal_x,goal_y

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

    def control_vel(self, action):
        move_cmd = Twist()
        move_cmd.linear.x = action[0]
        move_cmd.linear.y = 0.
        move_cmd.linear.z = 0.
        move_cmd.angular.x = 0.
        move_cmd.angular.y = 0.
        move_cmd.angular.z = action[1]
        self.cmd_vel.publish(move_cmd)

if __name__ == "__main__":
    test = CadrlStage()
    while(True):
        test.get_agentCMD()
        time.sleep(1.0)