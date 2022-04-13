#!/usr/bin/env python

'''
(c) REACT LAB, Harvard University
Author: Ninad Jadhav, Weiying Wang
'''

import rospy
from wsr_toolbox_cpp.msg import wsr_aoa_array
import numpy as np
from os.path import expanduser

def wsr_cb(msg):
    print("######################### Got message ######################")
    for tx in msg.aoa_array:
        print("=========== ID: "+ tx.id +" =============")
        print("TOP N AOA azimuth peak: "+ str(tx.aoa_azimuth))
        print("TOp N AOA elevation peak: "+ str(tx.aoa_elevation))
        print("Profile variance: "+ str(tx.profile_variance))
        print("Profile saved as profile_"+tx.id+".csv")

        homedir = expanduser("~")
        catkin_ws_name = rospy.get_param('~ws_name', 'catkin_ws')
        rootdir = homedir+'/'+catkin_ws_name+"/src/WSR-Toolbox-cpp/debug/"
        aoa_profile = np.asarray(tx.aoa_profile).reshape((tx.azimuth_dim, tx.elevation_dim))
        np.savetxt(rootdir+'/profile_'+tx.id+'.csv', aoa_profile, delimiter=',')

def main():
    
    #ROS subscriber
    pub = rospy.Subscriber('wsr_aoa_topic', wsr_aoa_array, wsr_cb)
    rospy.init_node('wsr_py_sub_node', anonymous=True)
    rospy.spin()

if __name__ == "__main__":
    main()
