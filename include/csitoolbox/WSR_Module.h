/*
 * (c) REACT LAB, Harvard University
	Author: Weiying Wang, Ninad Jadhav
 */

#ifndef CSITOOLBOX_WSR_MODULE_H
#define CSITOOLBOX_WSR_MODULE_H

#include "WIFI_Agent.h"
#include "WSR_Util.h"

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> EigenDoubleMatrix;
typedef Eigen::Map<EigenDoubleMatrix> EigenDoubleMatrixMap;
typedef Eigen::Matrix<std::complex<double>, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> EigencdMatrix;
typedef Eigen::Map<EigencdMatrix> EigencdMatrixMap;


class WSR_Module
{   
    private:
        double __lambda,__time_offset,__time_threshold,__peak_radius;
        size_t __nphi, __ntheta, __snum_start, __snum_end;
        std::unordered_map<std::string, double> __perf_aoa_profile_cal_time,__memory_used, __calculated_ts_offset,
                                                __channel_phase_diff_mean,__channel_phase_diff_stdev,
                                                __static_channel_phase_mean,__static_channel_phase_stdev,
                                                __closest_peak_confidence, __top_peak_confidence;
        std::unordered_map<std::string, int> __paired_pkt_count,__tx_pkt_size,__rx_pkt_size;
        int _topN_count = 1, __max_packets_to_process=500, __min_packets_to_process=10;
        int _phi_min=-180, _phi_max=180, _theta_min = 0,_theta_max=180, __relative_magnitude_threshold=40;
        std::vector<double> __aoa_profile_variance;
        std::string __RX_SAR_robot_MAC_ID, __trajType;
        nc::NdArray<double> compute_profile_bartlett_multithread(
                            const nc::NdArray<std::complex<double>>& h_list, 
                            const nc::NdArray<double>& pose_list);
        
        nc::NdArray<double> compute_profile_bartlett_singlethread(
                    const nc::NdArray<std::complex<double>>& h_list, 
                    const nc::NdArray<double>& pose_list);
        
        nc::NdArray<double> compute_profile_bartlett_offboard(
                            const nc::NdArray<std::complex<double>>& h_list, 
                            const nc::NdArray<double>& pose_list);

        nc::NdArray<double> compute_profile_music_offboard(
                    const nc::NdArray<std::complex<double>>& h_list, 
                    const nc::NdArray<double>& pose_list);

        nc::NdArray<double> phi_list,precomp_rep_phi,theta_list, precomp_rep_theta;
        bool __FLAG_normalize_profile = true, __FLAG_packet_threshold = false, __FLAG_debug = true, __FLAG_threading=false,
            __FLAG_interpolate_phase = true, __FLAG_sub_sample = false, __FLag_use_packet_id = true, __FLAG_offboard=false,
            __FLAG_openmp=false, __FLAG_use_relative_displacement=false;
        nc::NdArray<double> __aoa_profile;
        std::unordered_map<std::string, nc::NdArray<double>> __all_aoa_profiles;
        std::unordered_map<std::string, std::vector<double>> __all_topN_confidence;
        std::unordered_map<std::string, std::vector<double>> __all_topN_magnitudes;
        std::unordered_map<std::string, std::pair<std::vector<double>,std::vector<double>>> __TX_top_N_angles;
        EigenDoubleMatrix __eigen_lambda_list, __eigen_precomp_rep_phi, __eigen_precomp_rep_theta,__eigen_rep_theta_sine,
                        __eigen_rep_theta_co, __precomp__eigen_rep_lambda, __precomp__eigen_rep_phi, __precomp__eigen_rep_theta;
        std::vector<double> __peak_magnitudes;
        
    public:
        nlohmann::json __precompute_config; 
        std::unordered_map<std::string,std::string> data_sample_ts;
        std::unordered_map<std::string,std::string> tx_name_list;
        int calculate_AOA_profile(std::string tx_csi_file, 
                                    std::unordered_map<std::string, std::string> rx_csi_file, 
                                    nc::NdArray<double> rx_displacement,
                                    nc::NdArray<double> displacement_timestamp);
        WSR_Module();
        ~WSR_Module();
        WSR_Module(std::string config_fn);
        int get_tx_pkt_count(const std::string& tx_mac_id);
        int get_rx_pkt_count(const std::string& tx_mac_id);
        int get_paired_pkt_count(const std::string& tx_mac_id);
        double get_memory_used(const std::string& tx_mac_id);
        double get_calculated_ts_offset(const std::string& tx_mac_id);
        double get_cpdm (const std::string& tx_mac_id);
        double get_cpd_stdev (const std::string& tx_mac_id);
        double get_scpm (const std::string& tx_mac_id);
        double get_scd_stdev (const std::string& tx_mac_id);
        double get_top_confidence (const std::string& tx_mac_id);
//        std::vector<double> find_topN_phi(nc::NdArray<double> profile);
//        std::vector<double> find_topN_theta(nc::NdArray<double> profile);
        std::pair<std::vector<double>,std::vector<double>> find_topN();
//        std::vector<double> __top_N_phi, __top_N_theta;
        nc::NdArray<double> get_aoa_profile();
        std::pair<std::vector<double>,std::vector<double>> get_topN_AOA();
        std::pair<double,double> get_closest_AOA();
        std::unordered_map<std::string, nc::NdArray<double>> get_all_aoa_profile();
        std::unordered_map<std::string, std::pair<std::vector<double>,std::vector<double>>> get_TX_topN_angles();
        std::pair<double,double> get_phi_theta();
        void get_eigen_rep_angle_trig(EigenDoubleMatrix& output, 
                                      EigenDoubleMatrix& input,
                                      std::string trig_operation);
        void get_eigen_rep_angle_trig_openmp(EigenDoubleMatrix& output, 
                                      EigenDoubleMatrix& input,
                                      std::string trig_operation);
        void get_cwiseProduct_openmp(EigenDoubleMatrix& output, 
                                  EigenDoubleMatrix& input1,
                                  EigenDoubleMatrix& input2);
        void get_repmat(EigenDoubleMatrix& output, 
                        EigenDoubleMatrix& input,
                        int rows, int cols);
        void get_cwiseProduct(EigenDoubleMatrix& output, 
                              EigenDoubleMatrix& input1,
                              EigenDoubleMatrix& input2);

        void get_cwiseProduct_cd(EigencdMatrix& output, 
                              EigenDoubleMatrix& input1,
                              EigencdMatrix& input2);

        void get_matrix_block(EigenDoubleMatrix& output, 
                              EigenDoubleMatrix& input1,
                              int rows, int cols);

        void get_eterm_3DAdjustment(EigencdMatrix& output, 
                                    EigenDoubleMatrix& input);
        void get_bterm_all(EigencdMatrix& e_term_exp, 
                            EigenDoubleMatrix &e_rep_pitch,EigenDoubleMatrix &diff_phi_yaw,EigenDoubleMatrix &rep_rho);
        void get_block_exp(EigencdMatrix& output, 
                            EigencdMatrix& input,
                            int start, int end, int rows, int cols);  

        double  __vm_before=0, __vm_after=0, __rss_before=0, __rss_after=0;
        float get_profile_variance(double phi_ind, double theta_ind);
        double get_processing_time(const std::string& tx_mac_id);
        std::vector<double> top_aoa_error(double phi, double theta,
                                          std::pair<double,double> groundtruth_angles,
                                          const string& traj_type);
        std::vector<std::vector<float>> get_aoa_error(const std::pair<std::vector<double>,std::vector<double>>& topN_AOA,
                                          std::pair<double,double> groundtruth_angles,
                                          const string& traj_type);
        nlohmann::json get_stats(double true_phi,
                                double true_theta,std::vector<vector<float>>& aoa_error,
                                const std::string& tx_mac_id,
                                const std::string& tx_name,
                                const nc::NdArray<double>& rx_pos_true, 
                                const nc::NdArray<double>& rx_pos_est,
                                nlohmann::json true_positions_tx,
                                const int pos_idx);
        nlohmann::json get_stats_old_json(double true_phi,
                                double true_theta,std::vector<vector<float>>& aoa_error,
                                const std::string& tx_mac_id,
                                const std::string& tx_name,
                                const nc::NdArray<double>& rx_pos_true, 
                                const nc::NdArray<double>& rx_pos_est,
                                nlohmann::json true_positions_tx,
                                const int pos_idx);                                
        int test_csi_data(std::string rx_csi_file, 
                        std::unordered_map<std::string, std::string> tx_csi_file);
        nlohmann::json get_performance_stats(const std::string& tx_mac_id,
                                            const std::string& tx_name);
        std::unordered_map<std::string, int> get_paired_pkt_count();
        std::unordered_map<std::string, std::vector<double>>get_all_confidence();

        int calculate_spoofed_AOA_profile(std::string rx_csi_file,
                                            std::unordered_map<std::string, std::string> tx_csi_file,
                                            nc::NdArray<double> displacement,
                                            nc::NdArray<double> displacement_timestamp);
        std::vector<double> get_top_magnitudes(const std::string &tx_mac_id);

};


#endif //CSITOOLBOX_WSR_MODULE_H