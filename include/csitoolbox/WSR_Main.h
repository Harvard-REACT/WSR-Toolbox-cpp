/*
 * (c) REACT LAB, Harvard University
	Author: Weiying Wang, Ninad Jadhav
 */

#ifndef CSITOOLBOX_WSR_MAIN_H
#define CSITOOLBOX_WSR_MAIN_H

#include "csitoolbox/WSR_Module.h"
#include <unistd.h>
#include <sys/types.h>
#include <unordered_map>
#include <chrono>

class WSR_Main
{
    public:

        std::string __d_type, __config_fn;

        WSR_Main();
        ~WSR_Main();
        
        WSR_Main(std::string config_fn,
                std::string displacement_type);

        std::pair<std::vector<std::string>, std::vector<vector<double>>>  generate_aoa();
};      



#endif //CSITOOLBOX_WSR_MAIN_H