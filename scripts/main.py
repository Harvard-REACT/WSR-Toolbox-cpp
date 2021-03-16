'''
(c) REACT LAB, Harvard University
Author: Ninad Jadhav, Weiying Wang
'''

#!/usr/bin/env python

from libs import wsr_module
from RobotTrajectory import RobotTrajectory
import argparse
from os.path import expanduser


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", help="Input data file")
    parser.add_argument("--parser_type", help="Source of input trajectory")
    parser.add_argument("--mocap_id", help="ID of rigid body") #TODO: read from config file when multiple IDs are given
    args = parser.parse_args()
    
    homedir = expanduser("~")    
    tx_csi_fn = homedir + "/catkin_ws/src/csitoolbox/data/3D_Helix_101_mocap_2020-03-04_110659/csi_a.dat"
    rx_csi_fn = homedir + "/catkin_ws/src/csitoolbox/data/3D_Helix_101_mocap_2020-03-04_110659/csi_b.dat"
    config_fn = homedir + "/catkin_ws/src/csitoolbox/config/config_3D_SAR.json"
    
    ptypes = ['optitrack','vicon','t265']
    if args.parser_type not in ptypes:
        print("Incorrect parser type")
        exit(1)
    
    robot_ts = RobotTrajectory()

    if args.parser_type == "optitrack":
        #Single trajecotry from optitrack
        parsed_trajectory = robot_ts.parse_trajectory(traj_type = "solo", 
                        parser_type = "optitrack", 
                        rx_robot_traj_fn = args.file,
                        rx_mocap_id = args.mocap_id)
    elif args.parser_type == "vicon":
        #Single trajecotry from vicon
        parsed_trajectory = robot_ts.parse_trajectory(traj_type = "solo", 
                        parser_type = "vicon", 
                        rx_robot_traj_fn = args.file)
    elif args.parser_type == "t265":
        #Single trajecotry from t265
        parsed_trajectory = robot_ts.parse_trajectory(traj_type = "solo", 
                        parser_type = "t265", 
                        rx_robot_traj_fn = args.file)

    # print(robot_ts.parsed_trajectory['pose_list'][0])

    #Make C++ function calls to calculate AOA profile
    wsr_obj = wsr_module.PyWSR_Module(config_fn)
    wsr_obj.AOA_profile(tx_csi_fn,rx_csi_fn,parsed_trajectory)



if __name__ == "__main__":
    main()

