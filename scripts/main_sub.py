'''
(c) REACT LAB, Harvard University
Author: Ninad Jadhav, Weiying Wang
'''

#!/usr/bin/env python

#from libs import wsr_module
import rospy
from wsr_toolbox_cpp.msg import wsr_aoa_array

def wsr_cb(msg):
    print(msg)

def main():
    
    #Ros subscriber
    pub = rospy.Subscriber('wsr_aoa_topic', wsr_aoa_array, wsr_cb)
    rospy.init_node('wsr_py_sub_node', anonymous=True)
    rospy.spin()

if __name__ == "__main__":
    main()
