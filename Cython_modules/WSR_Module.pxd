from libcpp.vector cimport vector
from libcpp.string cimport string

cdef extern from "WSR_Module.cpp":
    pass

cdef extern from "WIFI_Agent.cpp":
    pass

cdef extern from "WSR_Util.cpp":
    pass

cdef extern from "WSR_Module.h":
    cdef cppclass WSR_Module:
        WSR_Module() except +
        WSR_Module(string) except +
        int calculate_AOA_profile(string, string, vector[vector[double]])

cdef extern from "WIFI_Agent.h":
    cdef cppclass WIFI_Agent:
        WIFI_Agent() except +


cdef extern from "WSR_Util.h":
    cdef cppclass WSR_Util:
        WSR_Util() except +
