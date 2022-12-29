# '''
# (c) REACT LAB, Harvard University
# Author: Ninad Jadhav
# '''

# #!/usr/bin/env python3

from mpl_toolkits.mplot3d import Axes3D
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import pandas as pd

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", help="AOA profile csv file")
    parser.add_argument("--nphi", type = int, help="Resolution of azimuth angle")
    parser.add_argument("--ntheta", type = int, help="Resolution of elevation angle")
    args = parser.parse_args()

    x_range = 2*args.nphi
    x_val, y_val = np.linspace(-args.nphi, args.nphi, x_range), np.linspace(0, args.ntheta, args.ntheta)
    z_data = pd.read_csv(args.file).T

    #Reference: https://plotly.com/python/mixed-subplots/
    fig = make_subplots(
                    rows=1, cols=2,
                    specs=[[{"type": "surface"},{"type": "heatmap"}]])


    fig.add_trace(go.Surface(z=z_data.values,x=x_val,y=y_val),row=1, col=1)
    fig.add_trace(go.Heatmap(z=z_data.values,x=x_val,y=y_val),row=1, col=2)
    fig.update_layout(title='AOA profile', xaxis_title="Azimuth angle(Degrees)", yaxis_title="Elevation angle (degrees)",)
    fig.show()

    # X = np.arange(0, 2*args.nphi-1, 1).flatten()
    # Y = np.arange(0, args.ntheta-1, 1).flatten()
    # X, Y = np.meshgrid(X, Y)
    
    # aoa_profile = np.genfromtxt(args.file, delimiter=',')
    # print(aoa_profile.shape)
    
    # max_peak = np.unravel_index(np.argmax(aoa_profile, axis=None), aoa_profile.shape)
    # profile = aoa_profile[:,0:args.ntheta]
    # Z = profile[X,Y]

    # fig = plt.figure(figsize=(12,6))
    # ax = fig.add_subplot(111, projection='3d')

    # surf = ax.plot_surface(X, Y, Z, rstride=8, cstride=8, alpha=0.8, cmap=cm.ocean)
    # cset = ax.contourf(X, Y, Z, zdir='z', offset=np.min(Z), cmap=cm.ocean)
    # cset = ax.contourf(X, Y, Z, zdir='x', offset=-5, cmap=cm.ocean)
    # cset = ax.contourf(X, Y, Z, zdir='y', offset=5, cmap=cm.ocean)


    # fig.colorbar(surf, ax=ax, shrink=0.5, aspect=5)
    # ax.set_xlim(0,2*args.nphi-1)
    # ax.set_xticklabels([x-180 for x in ax.get_xticks()])
    # ax.set_xlabel('Azimuth (Degree)')
    # ax.set_ylabel('Elevation (Degree)')
    # ax.set_zlabel('Magnitude')
    # ax.set_title('AOA profile')

    # plt.show()

if __name__ == "__main__":
    main()