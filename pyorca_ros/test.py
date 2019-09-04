# Copyright (c) 2013 Mak Nazecic-Andrlon 
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


from __future__ import division

from pyorca import Agent, get_avoidance_velocity, orca, normalized, perp
from numpy import array, rint, linspace, pi, cos, sin

import itertools
import random
import time
import rospy
import os
from std_srvs.srv import Empty


ros_port = 11325
os.environ["ROS_MASTER_URI"]="http://localhost:%d"%ros_port
rospy.init_node("test_orca", anonymous=None)

reset_stage = rospy.ServiceProxy('reset_positions', Empty)
reset_stage()

pro_agent= Agent(robo_index = 0, radius = 0.3,max_speed = 1,goal=[0,4])
ad0 = Agent (robo_index = 1,radius = 0.3, max_speed = 1, goal = [-3,-3])
ad1 = Agent(robo_index=2,radius = 0.3,max_speed = 1, goal = [0,-3])
ad2 = Agent(robo_index=3,radius = 0.3,max_speed = 1, goal = [3,-3])
agents = [pro_agent,ad0,ad1,ad2]
# agents = [pro_agent,ad2]
time.sleep(5)

running = True
FPS = 2
dt = 1/FPS
tau = 2
step = 0
all_lines = [[]] * len(agents)
while running:

    new_vels = [None] * len(agents)
    for i, agent in enumerate(agents):
        candidates = agents[:i] + agents[i + 1:]
        new_vels[i], all_lines[i] = orca(agent, candidates, tau, dt)

    # agents[0].control_vel(new_vels[i])

    for i, agent in enumerate(agents):
        agent.velocity = array(new_vels[i])
        agent.control_vel(new_vels[i])
        # if(i==0):
            # print("agent0:",new_vels[i])
    
    step += 1
    if(step>100):
        break

    time.sleep(0.5)

for agent in agents:
    agent.stop()
