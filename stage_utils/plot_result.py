import matplotlib.pyplot as plt
import re
import numpy as np
from matplotlib.font_manager import FontProperties


def plotBars(datas,ax,y_label):
    N = 4
    ind = np.arange(3)  # the x locations for the groups
    width = 0.10  # the width of the bars
    fontP = FontProperties()
    fontP.set_size('small')

    yvals = datas[0]
    zvals = datas[1]
    kvals = datas[2]
    wvals = datas[3]

    rects1 = ax.bar(ind, yvals, width, color='cornflowerblue')
    rects2 = ax.bar(ind+width, zvals, width, color='coral')
    rects3 = ax.bar(ind+width*2, kvals, width, color='yellowgreen')
    rects4 = ax.bar(ind+width*3, wvals, width, color='chocolate')


    ax.set_ylabel(y_label)
    ax.set_xticks(ind+width)
    ax.set_xticklabels( ('Scenario1', 'Scenario2', 'Scenario3') )
    # ax.legend( (rects1[0], rects2[0], rects3[0]), ('our method', 'long', 'cadrl'),prop={'size':8} )
    ax.legend( (rects1[0], rects2[0], rects3[0],rects4[0]), ('our method', 'DRLMACA', 'CADRL','ORCA'),prop={'size':8} )
    # def autolabel(rects):
    #     for rect in rects:
    #         h = rect.get_height()
    #         ax.text(rect.get_x()+rect.get_width()/2., 1.005*h, '%1.2f'%float(h),
    #                 ha='center', va='bottom')
    #
    # autolabel(rects1)
    # autolabel(rects2)
    # autolabel(rects3)

def plotDatas(successRates,deltaDiss,deltaTimes,avSpeeds):
    # avSpeeds = [[1, 1, 1], [2, 2, 2], [1, 1, 1],[1,1,1]]
    # for i in range(4):
    #     for j in range(3):
    #         avSpeeds[i][j] = 0.1 * deltaDiss[i][j] / deltaTimes[i][j]
    # print(avSpeeds)

    fig, axes = plt.subplots(nrows=2, ncols=2)
    ax0, ax1, ax2, ax3 = axes.flatten()
    plotBars(successRates, ax0, "successRates")
    ax0.set_title("successRates")
    plotBars(deltaDiss, ax1, "diatance")
    ax1.set_title("extra diatance")
    plotBars(deltaTimes, ax2, "time")
    ax2.set_title("extra time")
    plotBars(avSpeeds, ax3, "average speed")
    ax3.set_title("average speed")

    fig.tight_layout()
    fig.set_figheight(15)
    fig.set_figwidth(15)
    plt.show()

# fileids = [66666,77777,88888,99999]#21:old ad,27 not normalized, 88888 cadrl
fileids = [10,77777,88888,99999]#21:old ad,27 not normalized, 88888 cadrl
fileNames = ["our method","DRLMACA","CADRL","ORCA"]
successRates = []
exeDistances = []
exeTimes = []
avSpeeds = []
extraDistances = []
extraTimes = []

for i in range(len(fileids)):
    successRate = [0,0,0]
    exeDistance = [0,0,0]
    exeTime = [0,0,0]
    num_test = [0,0,0]
    extraDistance = [0,0,0]
    extraTime = [0,0,0]
    fileid = fileids[i]
    hostname = "autoRL_%d/"%fileid
    dirname = '/clever/saved_model_ppo/' + hostname + "log"
    with open(dirname+"/test.log", 'r') as f:
        for line in f:
            p = re.compile (r'[-+]?[0-9]*\.?[0-9]+')
            floats = [float(i) for i in p.findall(line)]  # Convert strings to float
            scenarioId = int(floats[0])
            num_test[scenarioId] += 1
            reachFlag = floats[-3]                
            reachTime = floats[-2]
            # if(fileid==77777 or fileid==77777):
                # reachTime = reachTime * 10
            reachDis = floats[-1] + 0.5
            diaDistance = np.sqrt((floats[1]-floats[3])**2 + (floats[2]-floats[4])**2) 
            extraDis = reachDis - diaDistance + 0.5
            extraT = reachTime - (diaDistance/1)
            if(not (reachFlag == 1.0)):
                reachFlag =0.0
            else:
                extraDistance[scenarioId] += extraDis
                extraTime[scenarioId] += extraT
            successRate[scenarioId] += reachFlag
            exeDistance[scenarioId] += reachDis
            exeTime[scenarioId] += reachTime
            # extraDistance[scenarioId] += extraDis
            # extraTime[scenarioId] += extraT
        
    successRate = [successRate[i]/num_test[i] for i in range(3)]
    exeDistance = [exeDistance[i]/num_test[i] for i in range(3)]
    exeTime = [exeTime[i]/num_test[i] for i in range(3)]
    avSpeed = [exeDistance[i]/exeTime[i] for i in range(3)]
    extraDistance = [extraDistance[i]/num_test[i] for i in range(3)]
    extraTime = [extraTime[i]/num_test[i] for i in range(3)]
    successRates.append(successRate)
    exeDistances.append(exeDistance)
    exeTimes.append(exeTime)
    extraDistances.append(extraDistance)
    extraTimes.append(extraTime)
    avSpeeds.append(avSpeed)
    print(fileid)
    print(successRate)
    print(extraDistance)
    print(extraTime)
    print(avSpeed)
plotDatas(successRates,extraDistances,extraTimes,avSpeeds)


