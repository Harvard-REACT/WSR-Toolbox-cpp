# '''
# (c) REACT LAB, Harvard University
# Author: Ninad Jadhav
# '''

#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import json
import glob, os
from mpl_toolkits.mplot3d import Axes3D
import argparse

SMALL_SIZE = 8
MEDIUM_SIZE = 10
BIGGER_SIZE = 12

plt.rc('font', size=MEDIUM_SIZE)         # controls default text sizes
plt.rc('axes', titlesize=BIGGER_SIZE)    # fontsize of the axes title
plt.rc('axes', labelsize=BIGGER_SIZE)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=MEDIUM_SIZE)   # fontsize of the tick labels
plt.rc('ytick', labelsize=MEDIUM_SIZE)   # fontsize of the tick labels
plt.rc('legend', fontsize=MEDIUM_SIZE)   # legend fontsize
plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title

def main():

    parser = argparse.ArgumentParser()
    parser.add_argument("--file", help="Interpolated Trajectory data file")
    args = parser.parse_args()
        
    f = open(args.file,"r") 
    data_json = json.loads(f.read())
    d = data_json["pose_list"]
    data_json_sorted = json.dumps({int(x):d[x] for x in d.keys()}, sort_keys=True)
    data_json = json.loads(data_json_sorted)

    traj = pd.DataFrame.from_dict(data_json, orient="index")

    fig = plt.figure()
    ax = fig.add_subplot(111, projection = '3d')

    x = traj['x']
    y = traj['y']
    z = traj['z']

    ax.set_xlabel("X-axis(m)")
    ax.set_ylabel("Y-axis(m)")
    ax.set_zlabel("Z-axis(m)")

    ax.scatter(x, y, z,marker='x',s=8)
    plt.show()


if __name__ == "__main__":
    main()
