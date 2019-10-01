import matplotlib.pyplot as plt
import numpy as np
np.set_printoptions(formatter={'float': '{: 0.3f}'.format})
# fileids = [21,1012,1014,1013]
# len_episodes = [30,40,40,40]#num of robot
fileids = [41,104,102]#[103,104,105]#[103,104,105]#[41,104,102]#[41,101,105]#[39,42,45][24,33,27]#[32,28,33,31]#[30,28,29,31]#[24,21,25,28]#23
len_episodes = [120,120,120,60,600,600]#[240,30,120,30]#[10,160,80]#num of robot
# labels = ["agent with multi-scenarios","agent with push adversary","agent without adversary","agent in complex environments"]
labels = ["agent in complex environments","agent with ad from scratch","multi agents","agent with learned ad"]
# labels = ["agent","adversary in predator-prey","adverary in shall not pass","adverary in keep-away"]
colors = ["yellowgreen","cornflowerblue","coral","chocolate"]
# labels = ["Chase","BLock","Crossing"]
labels = ["vel=0.5","vel=1.0","vel=1.5"]
# labels = ["Size=0.22","Size=0.44","Size=0.66"]
# labels = ["Protagonist"," Adversaries"]
labels = ["Discrete","Linear","Sigmoid","sin","max"]
# labels = ["Chase","Block","Crossing"]
fig, ax = plt.subplots()
for i in range(len(fileids)):
    fileid = fileids[i]
    len_episode = len_episodes[i]
    mylabel = labels[i]
    rewards=[]
    hostname = "autoRL_%d/"%fileid
    dirname = '/clever/saved_model_ppo/' + hostname + "log"
    with open(dirname+"/cal.log", 'r') as f:
        for line in f:
            rewards.append(line)

    rewards = map(lambda s: s.strip(), rewards)
    for r in range(len(rewards)):
        try:
            rewards[r] = float(rewards[r])
            if(rewards[r]>1000):
                rewards[r]=0
            if(fileid ==105):
                rewards[r] = rewards[r]*0.5
        except:
            rewards[r]=0
    num = len(rewards)
    num_episode = int(num / len_episode)
    num_episode = 50
    new_rewards=[]
    new_std = []
    for i in range(num_episode):
        temp_rewards = np.asarray(rewards[i*len_episode:(i+1)*len_episode])
        temp_var = np.round(np.var(temp_rewards),2) 
        averaged_reward = sum(rewards[i*len_episode:(i+1)*len_episode])/len_episode
        new_rewards.append(averaged_reward)
        new_std.append(temp_var)
    print(list(np.round(new_rewards,4)))
    print(new_std)

    x= range(len(new_rewards))
    x = [i*len_episodes[0] for i in x ]
    # print(len(new_rewards))
    ax = plt.axes()
    ax.yaxis.grid(True)
    ax.set_axisbelow(True)
    ax.yaxis.grid(color = "gainsboro")
    plt.locator_params(axis='y', nbins=5)
    plt.yticks(fontsize=13)
    plt.xticks(np.arange(min(x), max(x)+1, 2000),fontsize=13)
    plt.plot(x, new_rewards,label = mylabel,linewidth=2.0)
    # new_rewards = np.asarray(new_rewards)
    # new_std = np.asarray(new_std)
    # plt.fill_between(x,new_rewards+new_std,new_rewards-new_std)
plt.legend(fontsize=13, frameon=False)
plt.xlabel("episode",fontsize=13)
plt.ylabel("reward",fontsize=13)
ax.spines['right'].set_visible(False)
ax.spines['top'].set_visible(False)
ax.spines['left'].set_color('black')
ax.spines['bottom'].set_color('black')
ax.tick_params(axis='y', colors='black')
ax.tick_params(axis='x', colors='black')
# plt.ax.spines['left'].set_visible(False)
# plt.ylim(-200, 200)
plt.show()
