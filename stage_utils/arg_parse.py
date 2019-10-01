import argparse


def parse_args():
    parser = argparse.ArgumentParser("Reinforcement Learning experiments for multiagent environments")
    parser.add_argument("--fileIds", type=int, nargs='+')
    parser.add_argument("--modelEps", type=int, nargs='+')
    parser.add_argument("--scenarios", type=int, nargs='+',help="type of adversary: 0 represents chase, 1 represent push, 2 represents notPass")
    parser.add_argument('--rosports', type=int, nargs='+')
    parser.add_argument('--robotIds', type=int, nargs='+')
    parser.add_argument('--speedLimit', type=float, default=1.0)
    parser.add_argument('--train', type=int, default=1)
    parser.add_argument('--speedScale', type=float, default=1)
    parser.add_argument('--gaussianStd', type=float, default=0.0)
    parser.add_argument('--robotSize', type=float, default=0.2)
    parser.add_argument('--speedBound', type=float, nargs='+')
    parser.add_argument('--stdBound', type=float, nargs='+')
    parser.add_argument('--funIndex', type=int,default=0)
    # parser.add_argument('--', type=int, default=1)
    parser.add_argument('--perceptionAccuracy', type=float, default=0)
    arg_list = parser.parse_args()
    return arg_list
    # if(len(arg_list.rosports) == len(arg_list.robotIds)):
    #     return arg_list
    # else:
    #     print("****not same length****")
    #     return None



if __name__ == '__main__':
    arglist = parse_args()
    print arglist