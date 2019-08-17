import matplotlib.pyplot as plt
# fileids = [21,1012,1014,1013]
# len_episodes = [30,40,40,40]#num of robot
fileids = [1020,21,23,24,25]
len_episodes = [15,20,20,20,40]#num of robot
labels = ["agent with multi-scenarios","agent with push adversary","agent without adversary","agent in complex environments"]
# labels = ["agent","adversary in predator-prey","adverary in shall not pass","adverary in keep-away"]
for i in range(len(fileids)-4):
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
    rewards = map(float,rewards)
    num = len(rewards)
    num_episode = int(num / len_episode)
    num_episode = 100
    new_rewards=[]
    for i in range(num_episode):
        averaged_reward = sum(rewards[i*len_episode:(i+1)*len_episode])/len_episode
        new_rewards.append(averaged_reward)

    x= range(len(new_rewards))
    print(len(new_rewards))
    if(fileid == 1013):
        new_rewards = [j/50 for j in new_rewards]
    plt.plot(x, new_rewards,label = mylabel)
plt.legend()
plt.xlabel("episode")
plt.ylabel("reward")
plt.show()
