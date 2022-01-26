'''
(c) REACT LAB, Harvard University
Author: Ninad Jadhav, Weiying Wang
'''

#!/usr/bin/env python

#from libs import wsr_module
import wsr_module
import argparse
from os.path import expanduser
import rospy
from std_msgs.msg import String

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", help="Input data file")
    parser.add_argument("--d_type", help="Source of robot displacement: gt, t265, odom")
    args = parser.parse_args()
    
    homedir = expanduser("~")    
    config_fn = homedir + "/WSR_Project/WSR-Toolbox-cpp/config/config_3D_SAR.json"
    
    #Ros publisher
    pub = rospy.Publisher('chatter', String, queue_size=0)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    

    # #Make C++ function calls to calculate AOA profile
    wsr_obj = wsr_module.PyWSR_Module(config_fn, args.d_type)    
    while not rospy.is_shutdown():
        aoa = wsr_obj.AOA_profile()
        tx_id = aoa[0]
        aoa_angles = aoa[1]
        hello_str = "TX: " + tx_id[0] + ", AOA (azimuth): "+ str(aoa_angles[0]) + ", ts : " + str(rospy.get_time()) 
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        print("pub 1")	
        pub.publish(hello_str)
        print("pub 2")
        pub.publish(hello_str)
        print("pub 3")
        a=input("next")
        #rate.sleep()           

if __name__ == "__main__":
    main()
