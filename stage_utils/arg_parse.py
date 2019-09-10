import argparse


def parse_args():
    parser = argparse.ArgumentParser("Reinforcement Learning experiments for multiagent environments")
    parser.add_argument("--fileId", type=int, default=10000)
    parser.add_argument("--scenarios", type=int, nargs='+',help="type of adversary: 0 represents chase, 1 represent push, 2 represents notPass")
    parser.add_argument('--rosports', type=int, nargs='+')
    parser.add_argument('--robotIds', type=int, nargs='+')
    arg_list = parser.parse_args()
    if(len(arg_list.rosports) == len(arg_list.robotIds)):
        return arg_list
    else:
        print("****not same length****")
        return None



if __name__ == '__main__':
    arglist = parse_args()
    print arglist