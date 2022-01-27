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
import time
from std_msgs.msg import Bool
from wsr_toolbox_cpp.msg import wsr_aoa, wsr_aoa_array

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--d_type", help="Source of robot displacement: gt, t265, odom")
    args = parser.parse_args()
    
    homedir = expanduser("~")    
    config_fn = homedir + "/catkin_ws/src/WSR-Toolbox-cpp/config/config_3D_SAR.json"
    
    #Ros publisher
    pub = rospy.Publisher('wsr_aoa_topic', wsr_aoa_array, queue_size=10)
    rospy.init_node('wsr_py_node', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    count=0

    #publish dummy data
    tx_aoa_array = wsr_aoa_array()
    for i in range(5):
	pub.publish(tx_aoa_array)
	rate.sleep()
 
    # #Make C++ function calls to calculate AOA profile
    wsr_obj = wsr_module.PyWSR_Module(config_fn, args.d_type)
    get_new_aoa = Bool()
    
    while not rospy.is_shutdown():
	
	#Wait till new ao request generated
	get_new_aoa = rospy.wait_for_message('/get_aoa', Bool)
	
	if(get_new_aoa.data):
            aoa_toolbox = wsr_obj.AOA_profile()
            tx_aoa_array = wsr_aoa_array()
	    tx_aoa_array.header.stamp = rospy.Time.now() 
	    tx_aoa_array.header.frame_id = "wsr_angle"

	    for i in range(len(aoa_toolbox[0])):
                tx_aoa = wsr_aoa()
	        tx_aoa.id = aoa_toolbox[0][i]
	        tx_aoa.aoa_azimuth = aoa_toolbox[1][i]
		#print(tx_aoa)
	        tx_aoa_array.aoa_array.append(tx_aoa)
	    
	    time.sleep(0.1)
            pub.publish(tx_aoa_array)
            print("pub 1")	
         

if __name__ == "__main__":
    main()
