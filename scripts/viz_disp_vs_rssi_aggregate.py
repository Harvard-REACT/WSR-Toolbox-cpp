# '''
# (c) REACT LAB, Harvard University
# Author: Ninad Jadhav
# '''

#!/usr/bin/env python3

import pandas as pd
import json
import argparse
import seaborn as sns
import plotly.graph_objects as go
import matplotlib.pyplot as plt
import plotly.express as px
import numpy as np
import os


# convert plotly hex colors to rgba to enable transparency adjustments
def hex_rgba(hex, transparency):
    col_hex = hex.lstrip('#')
    col_rgb = list(int(col_hex[i:i+2], 16) for i in (0, 2, 4))
    col_rgb.extend([transparency])
    areacol = tuple(col_rgb)
    return areacol

# Make sure the colors run in cycles if there are more lines than colors
def next_col(cols):
    while True:
        for col in cols:
            yield col



def main():

    parser = argparse.ArgumentParser()
    parser.add_argument("--traj_dir", help="Interpolated Trajectory data file")
    parser.add_argument("--rssi_dir", help="RSSI data file")
    parser.add_argument("--traj_dir_nlos", help="Interpolated Trajectory data file")
    parser.add_argument("--rssi_dir_nlos", help="RSSI data file")
    args = parser.parse_args()

    traj_files = [name for name in os.listdir(args.traj_dir) if os.path.isfile(os.path.join(args.traj_dir, name))]
    rssi_files = [name for name in os.listdir(args.rssi_dir) if os.path.isfile(os.path.join(args.rssi_dir, name))]
    fig = go.Figure()
    
    final_disp = np.empty((0))
    final_rssi = np.empty((0))

    print(final_disp.shape)
    print(final_rssi.shape)

    # define colors as a list 
    colors = px.colors.qualitative.Plotly

    layout = dict(xaxis = dict(title = 'Cumulative displacement', showgrid=False, ticks='inside', mirror=True,showline=True,linecolor='black'),
                yaxis = dict(title = 'Signal Strength', showgrid=False, ticks='inside',mirror=True,showline=True,linecolor='black'),
                font=dict(size=28),
                plot_bgcolor='rgba(0,0,0,0)'
                )
    

    fig = go.Figure(layout=layout)

    '''
    ++++++++++++++++++++++++ LOS ++++++++++++++++++++++++++
    '''
    plot_val_disp = pd.DataFrame()
    plot_val_rssi = pd.DataFrame()

    for i in range(len(traj_files)):
    # for i in range(5):
        fn = os.path.join(args.traj_dir, traj_files[i])
        print(fn)
        f = open(fn,"r") 
        data_json = json.loads(f.read())
        d = data_json["pose_list"]
        data_json_sorted = json.dumps({int(x):d[x] for x in d.keys()}, sort_keys=True)
        data_traj_final = json.loads(data_json_sorted)

        fn = os.path.join(args.rssi_dir, rssi_files[i])
        print(fn)
        f = open(fn,"r") 
        data_json_rssi = json.loads(f.read())
        d = data_json_rssi["rssi_value"]
        data_json_sorted = json.dumps({int(x):d[x] for x in d.keys()}, sort_keys=True)
        data_rssi_final = json.loads(data_json_sorted)

        traj2 = pd.DataFrame.from_dict(data_traj_final, orient="index")
        rssi2 = pd.DataFrame.from_dict(data_rssi_final, orient="index")


        cumulative_disp2 = np.zeros((825))
        rssi_a2 = np.zeros((825))
        disp_val = 0
        
        #Bound displacement to first 825 positions
        for z in range(1,825):
            rssi_a2[z] = rssi2["rssi_a"][z]
            val = pow((traj2["x"][z]-traj2["x"][z-1]),2) + pow((traj2["y"][z]-traj2["y"][z-1]),2) + pow((traj2["z"][z]-traj2["z"][z-1]),2)
            disp_val+=pow((val),0.5)
            cumulative_disp2[z] = disp_val
    

        plot_val_disp["disp_val"+str(i)] = cumulative_disp2
        plot_val_rssi["rssi_val"+str(i)] = rssi_a2
        


    '''
    ++++++++++++++++++++++++ NLOS ++++++++++++++++++++++++++
    '''
    traj_files_nlos = [name for name in os.listdir(args.traj_dir_nlos) if os.path.isfile(os.path.join(args.traj_dir_nlos, name))]
    rssi_files_nlos = [name for name in os.listdir(args.rssi_dir_nlos) if os.path.isfile(os.path.join(args.rssi_dir_nlos, name))]
    plot_val_disp_nlos = pd.DataFrame()
    plot_val_rssi_nlos = pd.DataFrame()

    for i in range(len(traj_files_nlos)):
    # for i in range(5):
        fn = os.path.join(args.traj_dir_nlos, traj_files_nlos[i])
        print(fn)
        f = open(fn,"r") 
        data_json = json.loads(f.read())
        d = data_json["pose_list"]
        data_json_sorted = json.dumps({int(x):d[x] for x in d.keys()}, sort_keys=True)
        data_traj_final = json.loads(data_json_sorted)

        fn = os.path.join(args.rssi_dir_nlos, rssi_files_nlos[i])
        print(fn)
        f = open(fn,"r") 
        data_json_rssi = json.loads(f.read())
        d = data_json_rssi["rssi_value"]
        data_json_sorted = json.dumps({int(x):d[x] for x in d.keys()}, sort_keys=True)
        data_rssi_final = json.loads(data_json_sorted)

        traj2 = pd.DataFrame.from_dict(data_traj_final, orient="index")
        rssi2 = pd.DataFrame.from_dict(data_rssi_final, orient="index")


        cumulative_disp2 = np.zeros((825))
        rssi_a2 = np.zeros((825))
        disp_val = 0
        
        #Bound displacement to first 825 positions
        for z in range(1,825):
            rssi_a2[z] = rssi2["rssi_a"][z]
            val = pow((traj2["x"][z]-traj2["x"][z-1]),2) + pow((traj2["y"][z]-traj2["y"][z-1]),2) + pow((traj2["z"][z]-traj2["z"][z-1]),2)
            disp_val+=pow((val),0.5)
            cumulative_disp2[z] = disp_val
    

        plot_val_disp_nlos["disp_val"+str(i)] = cumulative_disp2
        plot_val_rssi_nlos["rssi_val"+str(i)] = rssi_a2
        


    rgba = [hex_rgba(c, transparency=0.2) for c in colors]
    colCycle = ['rgba'+str(elem) for elem in rgba]
    
    plot_val_disp['mean'] = plot_val_disp.mean(axis=1)
    plot_val_rssi['mean'] = plot_val_rssi.mean(axis=1)
    plot_val_rssi['stdev'] = plot_val_rssi.std(axis=1)
    plot_val_rssi['upper'] = plot_val_rssi['mean'] + plot_val_rssi['stdev']
    plot_val_rssi['lower'] = plot_val_rssi['mean'] - plot_val_rssi['stdev'] 


    plot_val_disp_nlos['mean'] = plot_val_disp_nlos.mean(axis=1)
    plot_val_rssi_nlos['mean'] = plot_val_rssi_nlos.mean(axis=1)
    plot_val_rssi_nlos['stdev'] = plot_val_rssi_nlos.std(axis=1)
    plot_val_rssi_nlos['upper'] = plot_val_rssi_nlos['mean'] + plot_val_rssi_nlos['stdev']
    plot_val_rssi_nlos['lower'] = plot_val_rssi_nlos['mean'] - plot_val_rssi_nlos['stdev'] 




    line_color=next_col(cols=colCycle)
    new_col = next(line_color)
    #================LOS==================
    fig.add_trace(go.Scatter(x=plot_val_disp['mean'], y=plot_val_rssi['upper'],
                            mode='lines',
                            line=dict(color='rgba(255,255,255,0)'),
                            showlegend=False))

    
    fig.add_trace(go.Scatter(x=plot_val_disp['mean'], y=plot_val_rssi['lower'],
                        mode='lines',
                        fill='tonexty',
                        fillcolor=new_col,
                        line=dict(color='rgba(255,255,255,0)'),
                        showlegend=False,))
    

    fig.add_trace(go.Scatter(x=plot_val_disp['mean'], y=plot_val_rssi['mean'],
                        mode='lines',
                        line_color='blueviolet',
                        name='LOS'))
                        


    #================NLOS==================
    new_col = next(line_color)
    fig.add_trace(go.Scatter(x=plot_val_disp_nlos['mean'], y=plot_val_rssi_nlos['upper'],
                            mode='lines',
                            line=dict(color='rgba(255,255,255,0)'),
                            showlegend=False))


    fig.add_trace(go.Scatter(x=plot_val_disp_nlos['mean'], y=plot_val_rssi_nlos['lower'],
                        mode='lines',
                        fill='tonexty',
                        fillcolor=new_col,
                        line=dict(color='rgba(255,255,255,0)'),
                        showlegend=False,))
    

    fig.add_trace(go.Scatter(x=plot_val_disp_nlos['mean'], y=plot_val_rssi_nlos['mean'],
                        mode='lines',
                        line_color='orangered',
                        name='NLOS'))
                        



    fig.update_yaxes(range=[25, 45])
    fig.show()

if __name__ == "__main__":
    main()


