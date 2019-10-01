import numpy as np
import math

CONV_EPOSODE = 300
LADDER_STEP = 50

def sigmoid(x):
  return 1 / (1 + math.exp(-x))

def value_factor(bound,factor):
    return bound[0]+((bound[1]-bound[0])*factor)

def max_factor(ep_length,std_bound,speed_bound):
    return [std_bound[1],speed_bound[1]]

def min_factor(ep_length,std_bound,speed_bound):
    return [std_bound[0],speed_bound[0]]

def linear_factor(ep_length,std_bound,speed_bound):
    CONV_EPOSODE = 300
    std = 0
    speed = 0
    if(ep_length<=CONV_EPOSODE):
        factor = float(float(ep_length)/float(CONV_EPOSODE))
        std = value_factor(std_bound,factor)
        speed = value_factor(speed_bound,factor)
    else:
        std= std_bound[1]
        speed = speed_bound[1]
    return [std,speed]

def ladder_factor(ep_length,std_bound,speed_bound):
    LADDER_STEP = 100
    CONV_EPOSODE = 300
    std= 0
    speed = 0
    degree = int(ep_length/LADDER_STEP)
    total = int(CONV_EPOSODE/LADDER_STEP)
    factor = float(float(degree)/float(total))
    if(factor<=1):
        std = value_factor(std_bound,factor)
        speed = value_factor(speed_bound,factor)
    else:
        std = std_bound[1]
        speed = speed_bound[1]
    return [std,speed]

def sigmoid_facotr(ep_length,std_bound,speed_bound):
    CONV_EPOSODE = 300.0
    scale = CONV_EPOSODE/60.0
    scale_ep = (float(ep_length)/float(scale))-30.0
    factor = float(sigmoid(scale_ep))
    std = value_factor(std_bound,factor)
    speed = value_factor(speed_bound,factor)
    std,speed = np.round(std,2),np.round(speed,2)
    return [std, speed]

def sin_factor(ep_length,std_bound,speed_bound):
    ep = float(ep_length)*3.14/100.0
    factor = (np.sin(ep)+1.0)/2.0
    std = value_factor(std_bound,factor)
    speed = value_factor(speed_bound,factor)
    std,speed = np.round(std,2),np.round(speed,2)
    return [std, speed]

def get_factors(func_index, ep_lenth, std_bound, speed_bound):
    """
    :param func_index:
    :param ep_lenth:
    :param std_bound: [5,0], need to decrease acoording to the melicioud
    :param speed_bound: [0.2,1.2], need to increase
    :return:
    """
    FUNCS = [min_factor,max_factor,ladder_factor,linear_factor,sigmoid_facotr,sin_factor]
    [std,speed] = FUNCS[func_index](ep_lenth,std_bound,speed_bound)
    # print("return funindex:%d, ep:%d, std:%f, speed:%f"%(func_index,ep_lenth,std,speed))
    return std,speed

if __name__ == '__main__':
    eps= [0,50,80,100,150,200,250,300,400]
    speed_bound = [0.5,1.5]
    std_bound = [5,0]
    # for i in range(len(FUNCS)):
    for ep in eps:
        print(get_factors(5,ep,std_bound,speed_bound))
    print()