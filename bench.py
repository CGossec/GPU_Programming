#!/bin/python3
import argparse
import os
import re
import pandas as pd
import matplotlib.pyplot as plt

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
parser.add_argument("--iterations", type=int, default=10, required=False,
                    help="Number of repetitions, default 10")
parser.add_argument("--make", type=str2bool, nargs="?", choices=[True, False], default="True",
                    help="Optional argument to precise if the script should make the binary or not. Defaults to True")
args = parser.parse_args()

#==================================================
data = ["./datatest/small_30"]
#for nb_pts in [100, 500, 1000, 10000, 20000, 30000, 40000,
#    50000, 60000, 70000, 80000, 90000]:
#    file = "./datatest/fixed/spiral_" + str(nb_pts)
#    data.append(file)
#==================================================


modes = ["CPU", "GPU", "GPU-opti"]
performances = {"CPU":[], "GPU":[], "GPU-opti":[], "nbPoints":[], "nbIterations":[]}
for ii, test in enumerate(data):
    source = test + "_src"
    target = test + "_tgt"
    with open(source, "r") as f:
        performances["nbPoints"].append(len(f.readlines()) - 1)
    performances["nbIterations"].append(-1)
    for jj, mode in enumerate(modes):
        if args.make:
            os.system("cd {0}; make; cd ..;".format(mode))
        exec_cmd = "./" + mode + "/icp ./datatest/" + source + " ./datatest/" + target
        time_cmd = "{ time " + exec_cmd + " 2> icp.stderr ; } 2>> time.txt"
        for i in range(args.iterations):
            os.system(time_cmd)
            print("Finished timing execution {0} of {1} ({2}%)".format(i + 1, args.iterations, (i + 1) / args.iterations * 100))
        with open("time.txt", "r") as f:
            lines = f.readlines()
        wall_times = []
        for elm in lines:
            if elm.startswith("real"):
                actual_time = elm.split()[-1]
                minutes_to_seconds = int(actual_time.split("m")[0]) * 60
                parse_seconds = re.split("\.|,", actual_time.split("m")[1])
                seconds = minutes_to_seconds + int(parse_seconds[0]) + float(parse_seconds[1][:-1]) / 1000
                wall_times.append(seconds)
        mean = round(sum(wall_times) / len(wall_times), 5)
        performances[mode].append(mean)
        os.system("rm time.txt icp.stderr")
        print("{0}% of ICPs completed. Please wait a little longer.".format(round((ii * len(modes) + jj + 1) / (len(data) * len(modes)) * 100), 2))
df = pd.DataFrame(data=performances)
print(df)
fig, ax = plt.subplots(1, 1, figsize=(15,9))
ax.set_xlabel("Number of points")
ax.set_ylabel("Speed in seconds")
ax.plot(df.nbPoints, df.CPU, "o", label="CPU implementation")
ax.plot(df.nbPoints, df.GPU, "o", label="GPU implementation")
ax.plot(df.nbPoints, df["GPU-opti"], "o", label="GPU-opti implementation")
ax.legend()
fig.savefig("benchmarks_plots.png")
plt.close()

fig, ax = plt.subplots(1, 1, figsize=(15,9))
ax.bar(x=df.nbPoints-.8, height=df.CPU, label="CPU implementation")
ax.bar(x=df.nbPoints, height=df.GPU, label="GPU implementation")
plt.show()
fig.savefig("benchmarks_bars.png")
plt.close()
