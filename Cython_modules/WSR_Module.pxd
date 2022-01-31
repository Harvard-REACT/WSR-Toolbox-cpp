from libcpp.vector cimport vector
from libcpp.string cimport string
#from libcpp.pair cimport pair
from libcpp.unordered_map cimport unordered_map

cdef extern from "WSR_Module.cpp":
    pass

cdef extern from "WIFI_Agent.cpp":
    pass

cdef extern from "WSR_Util.cpp":
    pass

cdef extern from "WSR_Main.cpp":
    pass


cdef extern from "WSR_Main.h":
    cdef cppclass WSR_Main:
        WSR_Main() except +
        WSR_Main(string, string) except +
        unordered_map[string, vector[double]] generate_aoa()
        

cdef extern from "WSR_Module.h":
    cdef cppclass WSR_Module:
        WSR_Module() except +
        WSR_Module(string) except +
        int calculate_AOA_profile(string, string, vector[double],vector[double])

cdef extern from "WIFI_Agent.h":
    cdef cppclass WIFI_Agent:
        WIFI_Agent() except +


cdef extern from "WSR_Util.h":
    cdef cppclass WSR_Util:
        WSR_Util() except +
        vector[vector[double]] loadTrajFromCSV(string displacement_filename)
