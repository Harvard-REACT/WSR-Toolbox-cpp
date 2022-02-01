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
from wsr_toolbox_cpp.msg import wsr_aoa, wsr_aoa_array
import numpy as np

def main():
    rospy.init_node('wsr_py_node', anonymous=True)
    d_type = rospy.get_param('~d_type', 'gt')
    config_fn = rospy.get_param('~config_fn', '/home/wsr-ros-test/catkin_ws/src/WSR-Toolbox-cpp/config/config_3D_SAR.json') 
    
    #Ros publisher
    pub = rospy.Publisher('wsr_aoa_topic', wsr_aoa_array, queue_size=10)
    rate = rospy.Rate(100) # 10hz

    #publish dummy data
    tx_aoa_array = wsr_aoa_array()
    for i in range(5):
	pub.publish(tx_aoa_array)
	rate.sleep()
 
    # #Make C++ function calls to calculate AOA profile
    wsr_obj = wsr_module.PyWSR_Module(config_fn, d_type)
    get_new_aoa = Bool()
    
    while not rospy.is_shutdown():
        #Wait till new ao request generated
        rospy.loginfo("Waiting for aoa request...")
        get_new_aoa = rospy.wait_for_message('/get_aoa', Bool)
        
        if(get_new_aoa.data):
            rospy.loginfo("Calculating AOA")
            aoa_toolbox = wsr_obj.AOA_profile()
            tx_aoa_array = wsr_aoa_array()
            tx_aoa_array.header.stamp = rospy.Time.now() 
            tx_aoa_array.header.frame_id = "wsr_angle"

            for key, val in aoa_toolbox.items():
                tx_aoa = wsr_aoa()
                tx_aoa.id = key
                tx_aoa.azimuth_dim = int(val[0][0])
                tx_aoa.elevation_dim = int(val[0][1])
                tx_aoa.profile_variance = val[1][0]
                tx_aoa.aoa_azimuth = val[2]
                tx_aoa.aoa_elevation = val[3]
                tx_aoa.aoa_profile = val[4]
                #print(tx_aoa)
                tx_aoa_array.aoa_array.append(tx_aoa)
                
            # time.sleep(0.05)
            pub.publish(tx_aoa_array)
            rospy.loginfo("AOA published")
         

if __name__ == "__main__":
    main()
