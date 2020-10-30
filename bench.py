#!/bin/python3
import argparse
import os
import re

def str2bool(v):
    if isinstance(v, bool):
       return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

parser = argparse.ArgumentParser(description='Choose implementation to benchmark')
parser.add_argument('mode', choices = ["CPU", "GPU"],
                    help="Implementation to benchmark, `CPU` or `GPU`")
parser.add_argument("source", type=str, help="Choose source (shifted) cloud point")
parser.add_argument("target", type=str, help="Choose target (reference) cloud point")
parser.add_argument("--iterations", type=int, default=10, required=False,
                    help="Number of repetitions, default 10")
parser.add_argument("--make", type=str2bool, nargs="?", choices=[True, False], default="True",
                    help="Optional argument to precise if the script should make the binary or not. Defaults to True")
args = parser.parse_args()

if args.make:
    os.system("cd {0}; make; cd ..;".format(args.mode))
exec_cmd = "./" + args.mode + "/icp " + args.source + " " + args.target
time_cmd = "{ time " + exec_cmd + " 2> icp.stderr ; } 2>> time.txt"
for i in range(args.iterations):
    os.system(time_cmd)
    print("Finished timing execution {0} of {1} ({2}%)".format(i, args.iterations, i / args.iterations * 100))
with open("time.txt", "r") as f:
    lines = f.readlines()
wall_times = []
for elm in lines:
    if elm.startswith("real"):
        actual_time = elm.split()[-1]
        seconds = int(actual_time.split("m")[0]) * 60 + int(actual_time.split("m")[1].split(",")[0]) + float(actual_time.split("m")[1].split(",")[1][:-1]) / 1000
        wall_times.append(seconds)
mean = sum(wall_times) / len(wall_times)
print("Average execution time over {0} iterations was {1} seconds".format(args.iterations, round(mean, 5)))
os.system("rm time.txt icp.stderr")