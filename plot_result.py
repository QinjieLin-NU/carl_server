import matplotlib.pyplot as plt
import re
import numpy as np
from matplotlib.font_manager import FontProperties


def plotBars(datas,ax,y_label):
    N = 3
    ind = np.arange(N)  # the x locations for the groups
    width = 0.20  # the width of the bars
    fontP = FontProperties()
    fontP.set_size('small')

    yvals = datas[0]
    zvals = datas[1]
    kvals = datas[2]

    rects1 = ax.bar(ind, yvals, width, color='cornflowerblue')
    rects2 = ax.bar(ind+width, zvals, width, color='coral')
    rects3 = ax.bar(ind+width*2, kvals, width, color='yellowgreen')

    ax.set_ylabel(y_label)
    ax.set_xticks(ind+width)
    ax.set_xticklabels( ('Scenario1', 'Scenario2', 'Scenario3') )
    ax.legend( (rects1[0], rects2[0], rects3[0]), ('agent V adversaries', 'agents in complex environments', 'CADRL'),prop={'size':8} )
    # def autolabel(rects):
    #     for rect in rects:
    #         h = rect.get_height()
    #         ax.text(rect.get_x()+rect.get_width()/2., 1.005*h, '%1.2f'%float(h),
    #                 ha='center', va='bottom')
    #
    # autolabel(rects1)
    # autolabel(rects2)
    # autolabel(rects3)

def plotDatas(successRates,deltaDiss,deltaTimes):
    avSpeeds = [[1, 1, 1], [2, 2, 2], [1, 1, 1]]
    for i in range(3):
        for j in range(3):
            avSpeeds[i][j] = 0.1 * deltaDiss[i][j] / deltaTimes[i][j]

    fig, axes = plt.subplots(nrows=2, ncols=2)
    ax0, ax1, ax2, ax3 = axes.flatten()
    plotBars(successRates, ax0, "successRates")
    ax0.set_title("successRates")
    plotBars(deltaDiss, ax1, "diatance")
    ax1.set_title("diatance")
    plotBars(deltaTimes, ax2, "time")
    ax2.set_title("time")
    plotBars(avSpeeds, ax3, "average speed")
    ax3.set_title("average speed")

    fig.tight_layout()
    fig.set_figheight(15)
    fig.set_figwidth(15)
    plt.show()

fileids = [21,25,88888]
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
plotDatas(successRates,exeDistances,exeTimes)


