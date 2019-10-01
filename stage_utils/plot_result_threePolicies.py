import matplotlib.pyplot as plt
import re
import numpy as np
from matplotlib.font_manager import FontProperties
import matplotlib as mpl

mpl.rcParams["font.size"] = 13

def plotBars(datas,ax,y_label):
    N = 6
    ind = np.arange(3)  # the x locations for the groups
    width = 0.10  # the width of the bars
    fontP = FontProperties()
    fontP.set_size('small')

    yvals = datas[0]
    zvals = datas[1]
    kvals = datas[2]
    wvals = datas[3]
    avals = datas[4]
    bvals = datas[5]

    # rects1 = ax.bar(ind, yvals, width, color='cornflowerblue')
    # rects2 = ax.bar(ind+width, zvals, width, color='coral')
    # rects3 = ax.bar(ind+width*2, kvals, width, color='yellowgreen')
    # rects4 = ax.bar(ind+width*3, wvals, width, color='chocolate')
    # rects5 = ax.bar(ind+width*4, avals, width, color='goldenrod')
    # rects6 = ax.bar(ind+width*5, bvals, width, color='plum')
    rects1 = ax.bar(ind, yvals, width, color='cornflowerblue')
    rects2 = ax.bar(ind+width, zvals, width, color='dodgerblue')
    rects3 = ax.bar(ind+width*2, kvals, width, color='steelblue')
    rects4 = ax.bar(ind+width*3, wvals, width, color='coral')
    rects5 = ax.bar(ind+width*4, avals, width, color='goldenrod')
    rects6 = ax.bar(ind+width*5, bvals, width, color='yellowgreen')


    ax.set_ylabel(y_label,style='italic')
    ax.set_xticks(ind+width)
    ax.set_xticklabels( ('Chase Scenario', 'Block Scenario', 'Cross Scenario') )
    horiz_offset = 1.0
    vert_offset = 1.1
    ax.legend((rects1[0], rects2[0], rects3[0],rects4[0],rects5[0],rects6[0]), ("Ladder","Linear","Sigmoid","DRLMACA","CADRL","ORCA"),prop={'size':10}, ncol=3, \
    bbox_to_anchor=(horiz_offset, vert_offset), frameon=False )
    ax.spines['right'].set_visible(False)
    ax.spines['top'].set_visible(False)
    ax.spines['left'].set_visible(False)
    start, end = ax.get_ylim()
    if(y_label == "Success Rate"):
        ax.yaxis.set_ticks(np.arange(start, end, 0.4))
    if(y_label == "Distance"):
        ax.yaxis.set_ticks(np.arange(start, end, 10))
    if(y_label == "Time"):
        end = end -5
        ax.yaxis.set_ticks(np.arange(start, end, 20.0))
    if(y_label == "Average Speed"):
        end = 0.9
        ax.yaxis.set_ticks(np.arange(start, end, 0.4))
    # ax.tick_params(axis='y', colors='gainsboro')
    ax.yaxis.grid(True)
    ax.set_axisbelow(True)
    ax.yaxis.grid(color = "gainsboro")

def plotDatas(successRates,deltaDiss,deltaTimes,avSpeeds):
    # avSpeeds = [[1, 1, 1], [2, 2, 2], [1, 1, 1],[1,1,1]]
    # for i in range(4):
    #     for j in range(3):
    #         avSpeeds[i][j] = 0.1 * deltaDiss[i][j] / deltaTimes[i][j]
    # print(avSpeeds)

    fig, axes = plt.subplots(nrows=2, ncols=2)
    ax0, ax1, ax2, ax3 = axes.flatten()
    plotBars(successRates, ax0, "Success Rate")
    ax0.set_title("Comparison of Policies on Success Rate",y=-.15,style='italic')
    plotBars(deltaDiss, ax3, "Distance")
    ax3.set_title("Comparison of Policies on  Extra Distance",y=-.15,style='italic')
    plotBars(deltaTimes, ax2, "Time")
    ax2.set_title("Comparison of Policies on  Extra Time",y=-0.15,style='italic')
    plotBars(avSpeeds, ax1, "Average Speed")
    ax1.set_title("Comparison of Policies on  Average Speed",y=-0.15,style='italic')

    plt.tight_layout()
    fig.set_figheight(0)
    fig.set_figwidth(0)
    # plt.rcParams.update({'font.size': 100})
    plt.show()

# fileids = [66666,77777,88888,99999]#21:old ad,27 not normalized, 88888 cadrl
# fileids = [9,10,11,77777,88888,99999]#21:old ad,27 not normalized, 88888 cadrl
fileids = [41,101,105,77777,88888,99999]#21:old ad,27 not normalized, 88888 cadrl
# fileids = [106,107,105,109,110,99999]#[41,104,105,77777,88888,99999]#21:old ad,27 not normalized, 88888 cadrl
fileNames = ["Ladder","Linear","Sigmoid","DRLMACA","CADRL","ORCA"]
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
print(successRates)
print(extraDistances)
print(extraTimes)
print(avSpeeds)
plotDatas(successRates,extraDistances,extraTimes,avSpeeds)


