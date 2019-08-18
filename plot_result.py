import matplotlib.pyplot as plt
import re
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
    print(successRate)
    print(exeDistance)
    print(exeTime)
    
            


