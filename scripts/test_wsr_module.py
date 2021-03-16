'''
(c) REACT LAB, Harvard University
	Author: Ninad Jadhav, Weiying Wang
'''

#!/usr/bin/env python

from libs import wsr_module

def main():
    wsr_obj = wsr_module.PyWSR_Module()
    # wsr_obj.test_CSIReader()
    
    #Need to pass strings as bytes
    tx_csi = b"/home/jadhav/catkin_ws/src/csitoolbox/data/csi_a_2020-02-29_171359.dat"
    rx_csi = b"/home/jadhav/catkin_ws/src/csitoolbox/data/csi_b_2020-02-29_171359.dat"
    wsr_obj.getInputs(tx_csi, rx_csi)

if __name__ == "__main__":
    main()