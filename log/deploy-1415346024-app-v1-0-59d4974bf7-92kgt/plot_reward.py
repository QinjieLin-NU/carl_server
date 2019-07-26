import matplotlib.pyplot as plt
rewards=[]
with open("cal.log", 'r') as f:
    for line in f:
        rewards.append(line)

rewards = map(lambda s: s.strip(), rewards)
rewards = map(float,rewards)
print(len(rewards))
new_rewards=[]
i=0
while i < 37:
    
    averaged_reward = sum(rewards[i*1000:(i+1)*1000])/1000
    new_rewards.append(averaged_reward)
    i+=1

x= range(len(new_rewards))
print(len(new_rewards))
plt.plot(x, new_rewards)
plt.show()