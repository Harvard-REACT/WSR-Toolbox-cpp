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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", help="AOA profile csv file")
    args = parser.parse_args()
    fig_size = plt.rcParams["figure.figsize"]
    fig_size[0] = 12
    fig_size[1] = 10
    plt.rcParams["figure.figsize"] = fig_size
    
    X = np.arange(-180, 180, 1).flatten()
    Y = np.arange(0, 180, 1).flatten()
    X, Y = np.meshgrid(X, Y)
    
    aoa_profile = np.genfromtxt(args.file, delimiter=',')
    profile = aoa_profile[:,0:180]
    Z = profile[X+180,Y]

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    surf = ax.plot_surface(X, Y , Z, cmap=plt.cm.viridis,linewidth=0.01, antialiased=False)
    ax.set_xlabel('Azimuth (Phi)')
    ax.set_ylabel('Elevation(Theta)')
    ax.set_zlabel('Magnitude')
    locs = ax.get_xticks()

    fig.colorbar(surf, shrink=0.5, aspect=12)

    plt.show()

if __name__ == "__main__":
    main()