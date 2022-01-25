from WSR_Module cimport WIFI_Agent, WSR_Main
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp.pair cimport pair


cdef class PyWSR_Module:
    cdef WSR_Main main_module # Hold a C++ instance which we're wrapping

    def __cinit__(self,config_file, displacement_type):
        cdef string config_fn = config_file.encode('utf-8')
        cdef string d_type = displacement_type.encode('utf-8')
        self.main_module = WSR_Main(config_fn, d_type)

    def AOA_profile(self):
        cdef pair[vector[string], vector[double]] aoa_vals = self.main_module.generate_aoa()
        return aoa_vals


cdef class PyWIFI_Agent:
    cdef WIFI_Agent run_module  # Hold a C++ instance which we're wrapping