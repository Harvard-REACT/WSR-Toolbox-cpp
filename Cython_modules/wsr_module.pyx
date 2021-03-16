from WSR_Module cimport WSR_Module, WIFI_Agent
from libcpp.string cimport string
from libcpp.vector cimport vector

cdef class PyWSR_Module:
    cdef WSR_Module run_module  # Hold a C++ instance which we're wrapping 

    def __cinit__(self,config_file):
        cdef string config_fn = config_file.encode('utf-8')
        self.run_module = WSR_Module(config_fn)


    def AOA_profile(self,tx_csi_file, rx_csi_file,trajectory):
        cdef string tx_csi = tx_csi_file.encode('utf-8')
        cdef string rx_csi = rx_csi_file.encode('utf-8')
        cdef vector[vector[double]] rx_trajectory
        cdef vector[double] tmp_traj
        
        for i in range(len(trajectory['pose_list'])):
            # print(trajectory['pose_list'][i])
            tmp_traj = [
                trajectory['pose_list'][i]['time_sec'],
                trajectory['pose_list'][i]['time_nsec'],
                trajectory['pose_list'][i]['x'], 
                trajectory['pose_list'][i]['y'], 
                trajectory['pose_list'][i]['z'], 
            ]
            rx_trajectory.push_back(tmp_traj)
        
        # for i in range(rx_trajectory.size()):
        #     print(rx_trajectory[i])

        return self.run_module.calculate_AOA_profile(tx_csi, rx_csi, rx_trajectory)


cdef class PyWIFI_Agent:
    cdef WIFI_Agent run_module  # Hold a C++ instance which we're wrapping