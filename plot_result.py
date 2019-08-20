import matplotlib.pyplot as plt
import re
import numpy as np 
fileids = [21,25]
successRates = []
exeDistances = []
exeTimes = []
for i in range(len(fileids)):
    successRate = [0,0,0]
    exeDistance = [0,0,0]
    exeTime = [0,0,0]
    fileid = fileids[i]
    hostname = "autoRL_%d/"%fileid
    dirname = '/clever/saved_model_ppo/' + hostname + "log"
    with open(dirname+"/test.log", 'r') as f:
        for line in f:
            p = re.compile (r'[-+]?[0-9]*\.?[0-9]+')
            floats = [float(i) for i in p.findall(line)]  # Convert strings to float
            scenarioId = int(floats[0])
            reachFlag = floats[-3]
            reachTime = floats[-2]
            reachDis = floats[-1]
            if(not (reachFlag == 1.0)):
                reachFlag =0.0
            successRate[scenarioId] += reachFlag
            exeDistance[scenarioId] += reachDis
            exeTime[scenarioId] += reachTime
        
    successRate = [i/100 for i in successRate]
    exeDistance = [i/100 for i in exeDistance]
    exeTime = [i/100 for i in exeTime]
    successRates.append(successRate)
    exeDistances.append(exeDistance)
    exeTimes.append(exeTime)
    print(successRate)
    print(exeDistance)
    print(exeTime)

            
np.random.seed(19680801)

n_bins = 10
x = np.random.randn(1000, 3)

fig, axes = plt.subplots(nrows=2, ncols=2)
ax0, ax1, ax2, ax3 = axes.flatten()

colors = ['red', 'tan', 'lime']
ax0.hist(x, n_bins, density=True, histtype='bar', color=colors, label=colors)
ax0.legend(prop={'size': 10})
ax0.set_title('bars with legend')

ax1.hist(x, n_bins, density=True, histtype='bar', stacked=True)
ax1.set_title('stacked bar')

ax2.hist(x, n_bins, histtype='step', stacked=True, fill=False)
ax2.set_title('stack step (unfilled)')

# Make a multiple-histogram of data-sets with different length.
x_multi = [np.random.randn(n) for n in [10000, 5000, 2000]]
x_multi = successRates
n_bins = 3
ax3.hist(x_multi, n_bins, histtype='bar')
ax3.set_title('different sample sizes')

fig.tight_layout()
plt.show()

