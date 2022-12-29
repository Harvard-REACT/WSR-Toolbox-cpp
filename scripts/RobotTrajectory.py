'''
(c) REACT LAB, Harvard University
Author: Ninad Jadhav, Weiying Wang
'''

#!/usr/bin/env python

import math
#from tf.transformations import euler_from_quaternion
from transformations import euler_from_quaternion
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
import matplotlib.pyplot as plt

class RobotTrajectory:
    def __init__(self, plot=True):
        self.defaul_robot_id = 000
        self.plot_traj = plot


    def parse_trajectory(self, traj_type, parser_type, rx_robot_traj_fn, tx_robot_traj_fn=None, rx_mocap_id=None, tx_mocap_id=None):
        
        parsed_rx_robot_trajectory, parsed_tx_robot_trajectory, parsed_relative_traj = {},{},{}
        
        if parser_type == "optitrack":
            parsed_rx_robot_trajectory = self.optitrack_mocap_data_parser(rx_robot_traj_fn, rx_mocap_id)
        elif parser_type == "t265":
            parsed_rx_robot_trajectory = self.T265_camera_data_parser(rx_robot_traj_fn)
        elif parser_type == "vicon":
            parsed_rx_robot_trajectory = self.vicon_mocap_ros_data_parser(rx_robot_traj_fn)

        # For moving ends, need relative trajectory
        if traj_type == "relative":
            if parser_type == "optitrack":
                parsed_tx_robot_trajectory = self.optitrack_mocap_data_parser(rx_robot_traj_fn, tx_mocap_id) #For optitrack, the trajectory file has traj for all robots
            elif parser_type == "t265":
                parsed_tx_robot_trajectory = self.T265_camera_data_parser(tx_robot_traj_fn)
            elif parser_type == "vicon":
                parsed_tx_robot_trajectory = self.vicon_mocap_ros_data_parser(tx_robot_traj_fn)

            parsed_relative_traj = self.get_relative_trajectory(parsed_rx_robot_trajectory, parsed_tx_robot_trajectory)
            return parsed_relative_traj

        return parsed_rx_robot_trajectory


    '''
    #TODO
    Return relative trajectory
    '''
    def get_relative_trajectory(self, parsed_rx_robot_trajectory, parsed_tx_robot_trajectory):
        parse_relative_traj = {}

        #How to get relative traj for t265 and vicon, if there is mismatch between number of poses? (match based on timestamp?)
        return parse_relative_traj


    '''
    Read data from the mocap file
    Return: dictonary with position (meters) and orientation data (degrees) for a specific robot
    '''

    def optitrack_mocap_data_parser(self, robot_traj, robot_id):

        self.file = open(robot_traj, "r")
        parsed_robot_trajectory = {}
        pos_x = []
        pos_y = []
        pos_z = []
        time_nan_secs = []
        latency = []
        pitch = []
        yaw = []
        m = self.file.readlines()
        i = 0

        while "Client is connected to server and listening for data..." not in m[i] and i < len(m): i+=1
        #print("Found at ",i-1," Parsing now")

        len_m = len(m)
        #print(len_m)

        #ignore the last data packet from the end of file.
        for l in range(len(m)-1,0,-1):
            #print m[l]
            if "header" in m[l]:
                len_m = l - 1
                #print("new eof", len_m)
                break

        count=0
        while i < len_m-3:
            if "nsecs" in m[i]:
                latency_val = int(m[i + 2].split(':', 1)[1].strip())
                if latency_val < 0:
                    i = i + 13
                    continue
                val = int(m[i].split(':', 1)[1].strip())
                time_nan_secs.append(val)
                i = i  + 1
            elif "orientation" in m[i] and i+4<len(m):
                check_id = int(m[i - 5 ].split(':', 1)[1].strip())
                if check_id == int(robot_id):
                    #print "count = ", count
                    count+=1
                    ori_x = float(m[i + 1].split(':', 1)[1].strip())
                    ori_y = float(m[i + 2].split(':', 1)[1].strip())
                    ori_z = float(m[i + 3].split(':', 1)[1].strip())
                    ori_w = float(m[i + 4].split(':', 1)[1].strip())
                    tmp_angle = euler_from_quaternion([ori_x, ori_y, ori_z, ori_w])
                    pitch.append(tmp_angle[1])
                    yaw.append(tmp_angle[2])                
                    i = i + 4
                else:
                    i = i + 1
            elif "latency" in m[i]:
                latency.append(int(m[i].split(':', 1)[1].strip()))
                i = i + 1
            elif "position" in m[i]:
                check_id = int(m[i - 1 ].split(':', 1)[1].strip())
                if check_id == int(robot_id):
                    pos_x.append(float(m[i + 1].split(':', 1)[1].strip()))
                    pos_y.append(float(m[i + 2].split(':', 1)[1].strip()))
                    pos_z.append(float(m[i + 3].split(':', 1)[1].strip()))
                    i= i + 3
                else:
                    i = i + 1
            else:
                i = i + 1

        print("frame count = ", count)
        print("len of nsecs = ", len(time_nan_secs))
        print("len of latency = ", len(latency))
        print("len of orientation = ", len(pitch))
        print("len of pos_x = ", len(pos_x))
        print("len of pos_y = ", len(pos_y))
        min_len = min(len(latency), count)
        
        parsed_robot_trajectory['robot_id'] = robot_id
        parsed_robot_trajectory['pose_list']= []

        for pose_count in range(min_len):
            corrected_time = str(time_nan_secs[pose_count] - latency[pose_count]) #Latency is in nanoseconds as well

            pose_data = {
                'pose_num' : pose_count,
                'x' : pos_x[pose_count],
                'y' : pos_y[pose_count],
                'z' : pos_z[pose_count],
                'pitch' : math.degrees(pitch[pose_count]),
                'yaw' : math.degrees(yaw[pose_count]),
                'time_sec' : int(corrected_time[:10]),
                # 'time_sec' : int(corrected_time)
                'time_nsec' : int(corrected_time[10:])
            }
            parsed_robot_trajectory['pose_list'].append(pose_data)

        print("Completed extracting pose information for robot ID: ", robot_id)
        
        #Plot the trajectory
        # self.visualize_trajectory(pos_x, pos_y, pos_z)

        return parsed_robot_trajectory


    '''
    Read data from the T265 trajectory file
    Return: dictonary with position (meters) obtained from tracking camera. Latency and orientation are not obtained
    ** Note: The coordinate system is different for the T265 tracking camera compared to the mocap systems. Modify the parser accordingly
    ** refer : https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md
    ** The coordinate axis convention should be :
    +x => move forward
    +y => move left
    +z => move up 
    '''    

    def T265_camera_data_parser(self, robot_traj_file):

        self.file = open(robot_traj_file, "r")
        parsed_robot_trajectory = {}
        pos_x = []
        pos_y = []
        pos_z = []
        time_nan_secs = []
        latency = []
        pitch = []
        yaw = []
        m = self.file.readlines()
        i = 0

        #print("Found at ",i-1," Parsing now")
        len_m = len(m)
        #print(len_m)

        #ignore the last data packet from the end of file.
        for l in range(len_m-1,0,-1):
            #print m[l]
            if "header" in m[l]:
                len_m = l - 1
                #print("new eof", len_m)
                break
        count=0
        while i < len_m-4:
            if "nsecs" in m[i]:
                val = int(m[i].split(':', 1)[1].strip())
                time_nan_secs.append(val)
                i = i  + 1
            elif "orientation" in m[i] and i+4<len_m:
                count+=1
                ori_x = float(m[i + 1].split(':', 1)[1].strip())
                ori_y = float(m[i + 2].split(':', 1)[1].strip())
                ori_z = float(m[i + 3].split(':', 1)[1].strip())
                ori_w = float(m[i + 4].split(':', 1)[1].strip())
                tmp_angle = euler_from_quaternion([ori_x, ori_y, ori_z, ori_w])
                pitch.append(tmp_angle[1])
                yaw.append(tmp_angle[2])                
                i = i + 4
            elif "position" in m[i]:
                    #Modified for use with the raw camera api. Align the mocap x,y,z with camera x,y,z
                    #mocap (x,y,z) = TrackingCamera(z,x,y)
                    pos_x.append(-float(m[i + 3].split(':', 1)[1].strip()))
                    pos_y.append(-float(m[i + 1].split(':', 1)[1].strip())) 
                    pos_z.append(float(m[i + 2].split(':', 1)[1].strip()))
                    i= i + 3
            else:
                i = i + 1

        print("frame count = ", count)
        print("len of nsecs = ", len(time_nan_secs))
        print("len of pos_x = ", len(pos_x))
        print("len of pos_y = ", len(pos_y))
        print("len of pos_z = ", len(pos_z))
        min_len = min(len(pos_x), count)
        
        parsed_robot_trajectory['robot_id'] = self.defaul_robot_id
        parsed_robot_trajectory['pose_list']= []

        for pose_count in range(min_len):
            corrected_time = str(time_nan_secs[pose_count])
            pose_data = {
                'pose_num' : pose_count,
                'x' : pos_x[pose_count],
                'y' : pos_y[pose_count],
                'z' : pos_z[pose_count],
                'pitch' : math.degrees(pitch[pose_count]),
                'yaw' : math.degrees(yaw[pose_count]),
                'time_sec' : int(corrected_time[:10]),
                'time_nsec' : int(corrected_time[10:])
            }
            parsed_robot_trajectory['pose_list'].append(pose_data)

        print("Completed extracting pose information for")

        #Plot the trajectory
        if self.plot_traj:
            self.visualize_trajectory(pos_x, pos_y, pos_z)

        return parsed_robot_trajectory


    '''
    parse trajectory obtained from vicon motion capture system
    '''
    def vicon_mocap_ros_data_parser(self, robot_traj):
        self.file = open(robot_traj, "r")
        parsed_robot_trajectory = {}
        pos_x = []
        pos_y = []
        pos_z = []
        time_secs = []
        time_nsecs = []
        latency = []
        pitch = []
        yaw = []
        m = self.file.readlines()
        i = 0

        len_m = len(m)
        #print(len_m)

        #ignore the last data packet from the end of file.
        for l in range(len(m)-1,0,-1):
            #print m[l]
            if "header" in m[l]:
                len_m = l - 1
                #print("new eof", len_m)
                break

        count=0
        while i < len_m-3:
            if "secs" in m[i] and "nsecs" not in m[i]:
                #Latency values are already incorporated in the timestamp of ros message
                time_secs.append(int(m[i].split(':', 1)[1].strip()))
                print("pp ",m[i+1])
                time_nsecs.append(int(m[i+1].split(':', 1)[1].strip()))
                i = i  + 1
            elif "orientation" in m[i] and i+4<len_m:
                count+=1
                ori_x = float(m[i + 1].split(':', 1)[1].strip())
                ori_y = float(m[i + 2].split(':', 1)[1].strip())
                ori_z = float(m[i + 3].split(':', 1)[1].strip())
                ori_w = float(m[i + 4].split(':', 1)[1].strip())
                tmp_angle = euler_from_quaternion([ori_x, ori_y, ori_z, ori_w])
                pitch.append(tmp_angle[1])
                yaw.append(tmp_angle[2])                
                i = i + 4
            elif "position" in m[i] and "orientation" in m[i+4]:
                pos_x.append(float(m[i + 1].split(':', 1)[1].strip()))
                pos_y.append(float(m[i + 2].split(':', 1)[1].strip()))
                pos_z.append(float(m[i + 3].split(':', 1)[1].strip()))
                i= i + 3
            else:
                i = i + 1

        print("frame count = ", count)
        print("len of nsecs = ", len(time_nsecs))
        print("len of pos_x = ", len(pos_x))
        print("len of pos_y = ", len(pos_y))
        print("len of pos_z = ", len(pos_z))
        print("len of pitch = ", len(pitch))
        min_len = min(len(pos_x), count)
        
        parsed_robot_trajectory['robot_id'] = self.defaul_robot_id
        parsed_robot_trajectory['pose_list']= []

        for pose_count in range(min_len):
            pose_data = {
                'pose_num' : pose_count,
                'x' : pos_x[pose_count],
                'y' : pos_y[pose_count],
                'z' : pos_z[pose_count],
                'pitch' : math.degrees(pitch[pose_count]),
                'yaw' : math.degrees(yaw[pose_count]),
                'time_sec' : time_secs[pose_count],
                'time_nsec' : time_nsecs[pose_count]
            }
            parsed_robot_trajectory['pose_list'].append(pose_data)

        print("Completed extracting pose information")
        
        #Plot the trajectory
        self.visualize_trajectory(pos_x, pos_y, pos_z)
        
        return parsed_robot_trajectory

    
    def visualize_trajectory(self,pos_x, pos_y, pos_z):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(pos_x, pos_y, pos_z, marker='o')
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')
        plt.show()

    '''
    Currently not used. TODO Functionality for 2 antennas
    '''
    def imu_data_parser(self, input_file):
        file = open(input_file, "r")
        time_secs = []
        time_nan_secs = []
        latency = []
        ang_x = []
        ang_y = []
        ang_z = []
        lin_x = []
        lin_y = []
        lin_z = []
        m = file.readlines()
        i = 0
        len_m = len(m)
        #print(len_m)

        #ignore the last data packet from the end of file.
        for l in range(len(m)-1,0,-1):
            #print m[l]
            if "header" in m[l]:
                len_m = l - 1
                print("new eof", len_m)
                break

        count=0

        print("Collecting data")

        while i < len_m-3:
            if "stamp" in m[i]:
                time_secs.append(int(m[i+1].split(':', 1)[1].strip()))
                time_nan_secs.append(int(m[i + 2].split(':', 1)[1].strip()))
                i = i  + 3
                # if time_secs[len(time_secs)-1] - time_secs[len(time_secs)-2] < 0:
                #     print "line is " , i
                #     print time_secs[len(time_secs)-1]
                #     print time_secs[len(time_secs)-2]
                    #a= input()

            elif "angular_velocity" in m[i]:
                count+=1
                ang_x.append(float(m[i + 1].split(':', 1)[1].strip()))
                ang_y.append(float(m[i + 2].split(':', 1)[1].strip()))
                ang_z.append(float(m[i + 3].split(':', 1)[1].strip()))
                lin_x.append(float(m[i + 6].split(':', 1)[1].strip()))
                lin_y.append(float(m[i + 7].split(':', 1)[1].strip()))
                lin_z.append(float(m[i + 8].split(':', 1)[1].strip()))
                i = i + 9

            i = i+1

        print("frame count = ", count)
        print("len of nsecs = ", len(time_nan_secs))
        print("len of imu angluar velocity = ", len(ang_z))
        min_len = min(len(time_nan_secs), count)
        print("min_len = ", min_len)

        yaw = 0.0
        #Yaw Angle from Angular velocity
        with open(pwd+'/deg_'+fname, 'w') as f:
            f.write("%s\n" % str(min_len-1))
            for i in range(min_len-1):
                time_a = time_secs[i] + time_nan_secs[i]*0.000000001
                time_b = time_secs[i+1] + time_nan_secs[i+1]*0.000000001
                yaw = yaw + ((ang_z[i] + ang_z[i+1]) * (time_b-time_a)/2)
                if yaw > 2*math.pi:
                    yaw = yaw - 2*math.pi

                deg_yaw = yaw * 180/math.pi

                if deg_yaw > 180:
                    deg_yaw = deg_yaw - 360

                f.write("%s\n" % str(deg_yaw))


        with open(pwd+'/deg_'+fname, 'a') as f:
            for i in range(min_len-1):
                f.write("%s\n" % str(time_secs[i]))

        with open(pwd+'/deg_'+fname, 'a') as f:
            for i in range(min_len-1):
                f.write("%s\n" % str(time_nan_secs[i]))

        print(pwd+'deg_'+fname)
        print("Completed")
