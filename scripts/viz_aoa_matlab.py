# '''
# (c) REACT LAB, Harvard University
# Author: Ninad Jadhav
# '''

# #!/usr/bin/env python3

import matlab.engine
import argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", help="AOA profile csv file")
    parser.add_argument("--nphi", type = int, help="Resolution of azimuth angle")
    parser.add_argument("--ntheta", type = int, help="Resolution of elevation angle")
    parser.add_argument("--phi_min", type = float, help="min phi angle")
    parser.add_argument("--phi_max", type = float, help="max phi angle")
    parser.add_argument("--theta_min", type = float, help="min_theta angle")
    parser.add_argument("--theta_max", type = float, help="max theta angle")
    args = parser.parse_args()
    
    eng = matlab.engine.start_matlab()
    filename = args.file
    nphi = args.nphi
    ntheta = args.ntheta
    phi_min = args.phi_min
    phi_max = args.phi_max
    theta_min = args.theta_min
    theta_max = args.theta_max

    eng.viz_aoa(filename,nphi,
                ntheta,
                phi_min, 
                phi_max, 
                theta_min,
                theta_max,
                nargout=0)
    val = input("Finished visualizing?")
    eng.quit()

if __name__ == "__main__":
    main()