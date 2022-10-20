/*
 * (c) REACT LAB, Harvard University
	Author: Ninad Jadhav, Weiying Wang
 */

#ifndef CSITOOLBOX_WSR_UTIL_H
#define CSITOOLBOX_WSR_UTIL_H

using namespace std;

#include "Base.h"
#include "WIFI_Agent.h"


class WSR_Util{
    
    public:
        void displayDataPacket(DataPacket wifi_data_packet, int packet_number);
        Eigen::MatrixXcd  getForwardReverseChannel(std::vector<DataPacket> transmitter, 
                                                    std::vector<DataPacket> receiver);
        std::pair<nc::NdArray<std::complex<double>>,nc::NdArray<double>> getForwardReverseChannel_v2(
                                                                std::vector<DataPacket> rx_robot,
                                                                std::vector<DataPacket> tx_robot,
                                                                double predefined_offset,
                                                                double threshold,
                                                                double& calculated_ts_offset,
                                                                bool interpolate_phase,
                                                                bool sub_sample);
        void read_bfee_timestamp_mac(uint8_t *inBytes, WIFI_Agent& robot);
        int readCsiData(std::string fn, WIFI_Agent& robot, bool __FLAG_debug);
        std::pair<nc::NdArray<double>,std::vector<size_t>> interpolate(const nc::NdArray<double>& inX,
                                                                       const nc::NdArray<double>& inXp,
                                                                       const nc::NdArray<double>& inFp);
        int formatTrajectory(std::vector<std::vector<double>>& rx_trajectory, 
                            Eigen::MatrixXd& displacement, 
                            Eigen::MatrixXd& trajectory_timestamp);
        std::pair<nc::NdArray<double>, nc::NdArray<double>> formatTrajectory_v2(std::vector<std::vector<double>>& rx_trajectory,
                                                                                std::vector<double>& antenna_offset,
                                                                                nc::NdArray<double>& mean_pos,string& traj_type,
                                                                                bool __Flag_get_mean_pos,bool __Flag_offset);
        std::pair<nc::NdArray<double>, nc::NdArray<double>> getRelativeTrajectory(std::vector<std::vector<double>>& trajectory_tx,
                                                                                  std::vector<std::vector<double>>& trajectory_rx,
                                                                                  std::vector<double>& antenna_offset,string& traj_type,
                                                                                  bool __Flag_get_mean_pos, bool __Flag_offset);
        std::pair<int,int> returnClosestIndices(const nc::NdArray<double>& csi_timestamp,
                                                const nc::NdArray<double>& trajectory_timestamp);

        void writeToFile(nc::NdArray<double>& nd_array, 
                        std::string fn);
        void writeCSIToFile(nc::NdArray<std::complex<double>>& nd_array,
                            nc::NdArray<double>&timestamp,  
                            string fn);
        void writeCSIToJsonFile(nc::NdArray<std::complex<double>>& nd_array, 
                                nc::NdArray<double>&timestamp, 
                                const std::string& fn,
                                bool __FLAG_interpolate_phase); 
        void writeTrajToFile(std::vector<std::vector<double>>& rx_trajectory, 
                            std::string fn);
        void writeTrajToFile(nc::NdArray<double>& rx_trajectory, 
                            const std::string& fn);
        std::vector<std::vector<double>> loadTrajFromCSV(std::string traj_fn);
        double __offset =  pow(10,-6), __threshold = 600*pow(10,-6), __threshold_traj = 100*pow(10,-6);
        std::pair<nc::NdArray<double>, nc::NdArray<double>> match_trajectory_timestamps(
                                                    nc::NdArray<double> timestamp_tx,
                                                    nc::NdArray<double> displacement_tx,
                                                    nc::NdArray<double> timestamp_rx,
                                                    nc::NdArray<double> displacement_rx);
        double mem_usage();
        std::unordered_map<std::string, std::pair<double,double>> get_true_aoa(std::vector<std::vector<double>>& rx_trajectory,
                                                                               nlohmann::json true_positions_tx);

        nc::NdArray<double> unwrap(const nc::NdArray<double>& phase_list);
        double anglediff(double a, double b);
        nc::NdArray<int> unravel_index(int ind, int row, int col);
        struct passwd *pw = getpwuid(getuid());
        std::string __homedir = pw->pw_dir;
        std::string bool_to_string(bool value);
        std::string format_mac(std::string const& s);
        void writePacketDistributionToJsonFile(const nc::NdArray<double>& csi_timestamp, 
                                                const nc::NdArray<double>& trajectory_timestamp, 
                                                const nc::NdArray<double>& displacement,
                                                std::string fn);
        std::pair<nc::NdArray<std::complex<double>>,nc::NdArray<double>> getConjugateProductChannel(
                                                                std::vector<DataPacket> rx_robot,
                                                                bool sub_sample,
                                                                int __snum_start,
                                                                int __snum_end);
        std::string dec2hex(unsigned int i);
        std::pair<nc::NdArray<std::complex<double>>,nc::NdArray<double>> getForwardReverseChannelCounter(
                                                                std::vector<DataPacket> rx_robot,
                                                                std::vector<DataPacket> tx_robot,
                                                                bool interpolate_phase,
                                                                bool sub_sample);
        void get_phase_diff_metrics(nc::NdArray<std::complex<double>>& channel,
                                    double &channel_ang_diff_mean,
                                    double &channel_ang_diff_stdev,
                                    bool& interpolate_phase,
                                    bool& moving);
        double static diff_360(double a, double b);
        std::unordered_map<std::string, std::pair<double,double>> get_true_aoa_v2(nc::NdArray<double>& mean_pos,
                                                                                 nlohmann::json true_positions_tx);
        float wrapNegPitoPi(float val);
        float get_yaw(double& x, double& y, double& z, double& w);
        Eigen::Vector3f rotationMatrixToEulerAngles(Eigen::Matrix3f &R);
        std::vector<std::vector<int>> get_signal_strength(std::vector<DataPacket>& rx_robot_data,
                                                            int start_index,
                                                            int end_index);
        void writeRssiToFile(std::vector<std::vector<int>>& rssi_rx_robot, std::string fn) ;
        std::pair<nc::NdArray<std::complex<double>>,nc::NdArray<std::complex<double>>> getRawCSIData(
                                                        std::vector<DataPacket> rx_robot);
        double wrap0to2Pi(double val); 
        std::pair<nc::NdArray<double>, nc::NdArray<double>> formatDisplacementTwoAntenna(
                                                            std::vector<std::vector<double>>& rx_displacement,
                                                            string& displacement_type);                                                                                                               
};


#endif //CSITOOLBOX_WSR_UTIL_H


