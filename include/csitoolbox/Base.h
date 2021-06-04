#ifndef CSITOOLBOX_BASE_H
#define CSITOOLBOX_BASE_H

#include <string>
#include <iostream>
#include <fstream>
#include <array>
#include <vector>
#include <complex>
#include <math.h>
#include <cmath>
#include "Eigen/Dense"
// #include <Eigen/unsupported/Eigen/MatrixFunctions>
#include <utility>
#include "nlohmann/json.hpp"
#include <NumCpp.hpp> 
#include <sstream>
#include <pwd.h>
#include <thread>
#include <unordered_map>

/*Data packets sturcture from log_to_file*/
struct DataPacket{
    unsigned long timestamp_low;
    unsigned short bfee_count;
    unsigned int Nrx;
    unsigned int Ntx;
    unsigned int rssi_a;
    unsigned int rssi_b;
    unsigned int rssi_c;
    char noise;
    unsigned int agc;
    unsigned int fake_rate_n_flags;
    unsigned long tv_sec;
    unsigned long tv_usec;
    unsigned long frame_count;
    double ts;
    std::string mac_real;
    // std::vector<double> mac_real;
    // std::pair <double, double> csi[30][3];
    std::complex<double> csi[30][3];
    double perm[3];
};

#endif //CSITOOLBOX_BASE_H