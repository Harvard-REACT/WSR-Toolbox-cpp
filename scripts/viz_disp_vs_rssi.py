# '''
# (c) REACT LAB, Harvard University
# Author: Ninad Jadhav
# '''

#!/usr/bin/env python3

import pandas as pd
import json
import argparse
import plotly.graph_objects as go
import numpy as np
import os

def main():

    parser = argparse.ArgumentParser()
    parser.add_argument("--file_traj2", help="Interpolated Trajectory data file")
    parser.add_argument("--file_rssi2", help="RSSI data file")
    parser.add_argument("--file_traj3", help="Interpolated Trajectory data file")
    parser.add_argument("--file_rssi3", help="RSSI data file")
    parser.add_argument("--file_traj4", help="Interpolated Trajectory data file")
    parser.add_argument("--file_rssi4", help="RSSI data file")
    args = parser.parse_args()


    #File for TX2
    f = open(args.file_traj2,"r") 
    data_json = json.loads(f.read())
    d = data_json["pose_list"]
    data_json_sorted = json.dumps({int(x):d[x] for x in d.keys()}, sort_keys=True)
    data_traj_final = json.loads(data_json_sorted)

    f = open(args.file_rssi2,"r") 
    data_json_rssi = json.loads(f.read())
    d = data_json_rssi["rssi_value"]
    data_json_sorted = json.dumps({int(x):d[x] for x in d.keys()}, sort_keys=True)
    data_rssi_final = json.loads(data_json_sorted)

    traj2 = pd.DataFrame.from_dict(data_traj_final, orient="index")
    rssi2 = pd.DataFrame.from_dict(data_rssi_final, orient="index")


    cumulative_disp2 = np.zeros((len(traj2)))
    rssi_a2 = np.zeros((len(traj2)))
    disp_val = 0
    
    for i in range(1,len(traj2)):
        rssi_a2[i] = rssi2["rssi_a"][i]
        val = pow((traj2["x"][i]-traj2["x"][i-1]),2) + pow((traj2["y"][i]-traj2["y"][i-1]),2) + pow((traj2["z"][i]-traj2["z"][i-1]),2)
        disp_val+=pow((val),0.5)
        cumulative_disp2[i] = disp_val

    
    #File for TX3
    f = open(args.file_traj3,"r") 
    data_json = json.loads(f.read())
    d = data_json["pose_list"]
    data_json_sorted = json.dumps({int(x):d[x] for x in d.keys()}, sort_keys=True)
    data_traj_final = json.loads(data_json_sorted)

    f = open(args.file_rssi3,"r") 
    data_json_rssi = json.loads(f.read())
    d = data_json_rssi["rssi_value"]
    data_json_sorted = json.dumps({int(x):d[x] for x in d.keys()}, sort_keys=True)
    data_rssi_final = json.loads(data_json_sorted)

    traj3 = pd.DataFrame.from_dict(data_traj_final, orient="index")
    rssi3 = pd.DataFrame.from_dict(data_rssi_final, orient="index")

    cumulative_disp3 = np.zeros((len(traj3)))
    rssi_a3 = np.zeros((len(traj3)))
    disp_val = 0
    
    for i in range(1,len(traj3)):
        rssi_a3[i] = rssi3["rssi_a"][i]
        val = pow((traj3["x"][i]-traj3["x"][i-1]),2) + pow((traj3["y"][i]-traj3["y"][i-1]),2) + pow((traj3["z"][i]-traj3["z"][i-1]),2)
        disp_val+=pow((val),0.5)
        cumulative_disp3[i] = disp_val
    
    
    #File for TX4
    f = open(args.file_traj4,"r") 
    data_json = json.loads(f.read())
    d = data_json["pose_list"]
    data_json_sorted = json.dumps({int(x):d[x] for x in d.keys()}, sort_keys=True)
    data_traj_final = json.loads(data_json_sorted)

    f = open(args.file_rssi4,"r") 
    data_json_rssi = json.loads(f.read())
    d = data_json_rssi["rssi_value"]
    data_json_sorted = json.dumps({int(x):d[x] for x in d.keys()}, sort_keys=True)
    data_rssi_final = json.loads(data_json_sorted)

    traj4 = pd.DataFrame.from_dict(data_traj_final, orient="index")
    rssi4 = pd.DataFrame.from_dict(data_rssi_final, orient="index")

    cumulative_disp4 = np.zeros((len(traj4)))
    rssi_a4 = np.zeros((len(traj4)))
    disp_val = 0
    
    for i in range(1,len(traj4)):
        rssi_a4[i] = rssi4["rssi_a"][i]
        val = pow((traj4["x"][i]-traj4["x"][i-1]),2) + pow((traj4["y"][i]-traj4["y"][i-1]),2) + pow((traj4["z"][i]-traj4["z"][i-1]),2)
        disp_val+=pow((val),0.5)
        cumulative_disp4[i] = disp_val


    fig = go.Figure()
    fig.add_trace(go.Scatter(x=cumulative_disp2, y=rssi_a2,
                        mode='lines',
                        name='rssi_tx2'))

    fig.add_trace(go.Scatter(x=cumulative_disp3, y=rssi_a3,
                        mode='lines',
                        name='rssi_tx3'))
    
    fig.add_trace(go.Scatter(x=cumulative_disp4, y=rssi_a4,
                        mode='lines',
                        name='rssi_tx3'))

    fig.show()


if __name__ == "__main__":
    main()


