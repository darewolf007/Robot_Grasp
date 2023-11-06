from Robot_Grasp.Process_grasp import Process_grasp

import argparse
def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--limb", type=str, default="right", help="which robot arm to process grasp")
    args = parser.parse_args()
    return args