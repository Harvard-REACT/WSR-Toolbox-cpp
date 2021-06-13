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
    
    X = np.arange(0, 359, 1).flatten()
    Y = np.arange(0, 179, 1).flatten()
    X, Y = np.meshgrid(X, Y)
    
    aoa_profile = np.genfromtxt(args.file, delimiter=',')

    max_peak = np.unravel_index(np.argmax(aoa_profile, axis=None), aoa_profile.shape)
    profile = aoa_profile[:,0:180]
    Z = profile[X,Y]

    fig = plt.figure(figsize=(12,6))
    ax = fig.add_subplot(111, projection='3d')

    surf = ax.plot_surface(X, Y, Z, rstride=8, cstride=8, alpha=0.8, cmap=cm.ocean)
    cset = ax.contourf(X, Y, Z, zdir='z', offset=np.min(Z), cmap=cm.ocean)
    cset = ax.contourf(X, Y, Z, zdir='x', offset=-5, cmap=cm.ocean)
    cset = ax.contourf(X, Y, Z, zdir='y', offset=5, cmap=cm.ocean)


    fig.colorbar(surf, ax=ax, shrink=0.5, aspect=5)


    ax.set_xlabel('Azimuth (Degree)')
    ax.set_ylabel('Elevation (Degree)')
    ax.set_zlabel('Magnitude')
    ax.set_title('AOA profile')

    plt.show()

if __name__ == "__main__":
    main()