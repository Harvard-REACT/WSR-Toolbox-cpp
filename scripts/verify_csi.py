#!/usr/bin/env python
'''
(c) REACT LAB, Harvard University
Author: Ninad Jadhav, Weiying Wang
'''

import wsr_module
import argparse
from os.path import expanduser
import rospy
import time
from std_msgs.msg import Bool
import numpy as np
import os
import re
import time
import json
import pandas as pd
import matplotlib.pyplot as plt

def get_plot(filename, tx):
    print(tx)
    f = open(filename,"r") 
    data_json = json.loads(f.read())
    f.close()

    #Sort the keys numerically to preserve order.
    d = data_json["channel_packets"]
    data_json_sorted = json.dumps({int(x):d[x] for x in d.keys()}, sort_keys=True)
    data_json = json.loads(data_json_sorted)
    
    traj = pd.DataFrame.from_dict(data_json, orient="index")


    plt.scatter(np.arange(0,len(data_json),1),
                traj["center_subcarrier_phase"],s=1)
    plt.title('Channel Phase for '+ str(tx))
    plt.xlabel("WiFi Packets collected")
    plt.ylabel("WiFi channel phase")
    plt.show(block=False)
    plt.pause(10)
    plt.close()

def main():
    rospy.init_node('wsr_py_node', anonymous=True)
    homedir = expanduser("~")    
    d_type = rospy.get_param('~d_type', 'gt')
    catkin_ws_name = rospy.get_param('~ws_name', 'catkin_ws')
    config_fn = rospy.get_param('~config_fn', homedir+'/'+catkin_ws_name+'/src/WSR-Toolbox-cpp/config/config_3D_SAR.json') 
    rootdir = homedir+'/'+catkin_ws_name+"/src/WSR-Toolbox-cpp/debug/" 
   
    #Ros publisher
    rate = rospy.Rate(1) # 10hz

    # #Make C++ function calls to calculate AOA profile
    wsr_obj = wsr_module.PyWSR_Module(config_fn, d_type)
    check_csi = Bool()
    
    while not rospy.is_shutdown():
        #Wait till new a request generated
        rospy.loginfo("Waiting for request...")
        check_csi = rospy.wait_for_message('/verify_csi', Bool)
        
        if(check_csi.data):
            rospy.loginfo("Verifying CSI Data")
            tx_vals = wsr_obj.Verify_csi()
            for tx in tx_vals:
                regex = re.compile(tx)
                for root, dirs, files in os.walk(rootdir):
                    for file in files:
                        if regex.match(file):
                            fn = rootdir+file
                            get_plot(fn,tx)

            rospy.loginfo("Request completed")
         

if __name__ == "__main__":
    main()
