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
        double __lambda;
        double __time_offset;
        double __time_threshold;
        double __peak_radius;
        double __centerfreq;
        float __antenna_separation;
        size_t __nphi;
        size_t __ntheta;
        size_t __snum_start;
        size_t __snum_end;
        int _topN_count = 1;
        int __max_packets_to_process=500;
        int __min_packets_to_process=10;
        int _phi_min=-180;
        int _phi_max=180;
        int _theta_min = 0;
        int _theta_max=180;
        int __relative_magnitude_threshold=40;
        int __num_peaks_above_threshold=5;
        bool __FLAG_normalize_profile = true;
        bool __FLAG_packet_threshold = false; 
        bool __FLAG_threading=false;
        bool __FLAG_interpolate_phase = true;
        bool __FLAG_sub_sample = false; 
        bool __FLag_use_packet_id = true; 
        bool __FLAG_offboard=false;
        bool __FLAG_openmp=false; 
        bool __FLAG_use_relative_displacement=false;
        bool __FLAG_slice=false;
        bool __FLAG_slice_first=false;
        bool __FLAG_slice_second=false;
        bool __FLAG_two_antenna=false;
        bool __FLAG_debug = false; 
        bool __FLAG_info = true;
        std::string __RX_SAR_robot_MAC_ID;
        std::string __trajType;
        std::string __debug_dir;
        std::string __estimator;
        std::unordered_map<std::string, double> __perf_aoa_profile_cal_time;
        std::unordered_map<std::string, double> __memory_used, __calculated_ts_offset;
        std::unordered_map<std::string, double> __channel_phase_diff_mean;
        std::unordered_map<std::string, double> __channel_phase_diff_stdev;
        std::unordered_map<std::string, double> __static_channel_phase_mean;
        std::unordered_map<std::string, double> __static_channel_phase_stdev;
        std::unordered_map<std::string, double> __closest_peak_confidence;
        std::unordered_map<std::string, double> __top_peak_confidence;
        std::unordered_map<std::string, int> __paired_pkt_count;
        std::unordered_map<std::string, int> __tx_pkt_size;
        std::unordered_map<std::string, int> __rx_pkt_size;
        std::vector<double> __aoa_profile_variance;
        std::vector<double> __peak_magnitudes;
        std::vector<std::string> __RX_SAR_robot_MAC_ID_List;
        
        nc::NdArray<double> compute_profile_bartlett(
            const nc::NdArray<std::complex<double>>& h_list, 
            const nc::NdArray<double>& pose_list
        );
        nc::NdArray<double> compute_profile_music(
            const nc::NdArray<std::complex<double>>& h_list, 
            const nc::NdArray<double>& pose_list
        );
        nc::NdArray<double> phi_list;
        nc::NdArray<double> precomp_rep_phi;
        nc::NdArray<double> theta_list;
        nc::NdArray<double> precomp_rep_theta; 
        nc::NdArray<double> __aoa_profile;
        std::unordered_map<std::string, nc::NdArray<double>> __all_aoa_profiles;
        std::unordered_map<std::string, std::vector<double>> __all_topN_confidence;
        std::unordered_map<std::string, std::vector<double>> __all_topN_magnitudes;
        std::unordered_map<std::string, std::pair<std::vector<double>,std::vector<double>>> __TX_top_N_angles;
        EigenDoubleMatrix __eigen_lambda_list;
        EigenDoubleMatrix __eigen_precomp_rep_phi;
        EigenDoubleMatrix __eigen_precomp_rep_theta;
        EigenDoubleMatrix __eigen_rep_theta_sine;
        EigenDoubleMatrix __eigen_rep_theta_co;
        EigenDoubleMatrix __precomp__eigen_rep_lambda;
        EigenDoubleMatrix __precomp__eigen_rep_phi;
        EigenDoubleMatrix __precomp__eigen_rep_theta;
        std::unordered_map<std::string, int>  __all_topN_above_threshold;
        std::unordered_map<std::string, std::string> __channel_data_output_file;

    public:
        std::string __rx_name;
        double  __vm_before=0;
        double __vm_after=0;
        double __rss_before=0;
        double __rss_after=0;

        WSR_Module();
        WSR_Module(std::string config_fn);
        ~WSR_Module();
        
        int calculate_AOA_profile(
            std::string tx_csi_file, 
            std::unordered_map<std::string, std::string> rx_csi_file, 
            nc::NdArray<double> rx_displacement,
            nc::NdArray<double> displacement_timestamp
        );
        int calculate_AOA_profile_multi(
            std::string rx_csi_file,
            std::unordered_map<std::string, std::string> tx_csi_file,
            nc::NdArray<double> displacement,
            nc::NdArray<double> displacement_timestamp
        );
        int get_tx_pkt_count(const std::string& tx_mac_id);
        int get_rx_pkt_count(const std::string& tx_mac_id);
        int get_paired_pkt_count(const std::string& tx_mac_id);
        int test_csi_data(
            std::string rx_csi_file, 
            std::unordered_map<std::string,std::string> tx_csi_file
        );
        int calculate_AOA_using_csi_conjugate_multiple(
            std::string rx_csi_file,
            nc::NdArray<double> displacement,
            nc::NdArray<double> displacement_timestamp
        ); 
        int calculate_spoofed_AOA_profile(
            std::string rx_csi_file,
            std::unordered_map<std::string, std::string> tx_csi_file,
            nc::NdArray<double> displacement,
            nc::NdArray<double> displacement_timestamp
        );
        int get_peak_num_above_threshold(const std::string &tx_mac_id);
        int test_csi_data_conjugate(std::string rx_csi_file);
        int calculate_AOA_using_csi_conjugate(
            std::string rx_csi_file,
            nc::NdArray<double> displacement,
            nc::NdArray<double> displacement_timestamp
        );
        double get_memory_used(const std::string& tx_mac_id);
        double get_calculated_ts_offset(const std::string& tx_mac_id);
        double get_cpdm (const std::string& tx_mac_id);
        double get_cpd_stdev (const std::string& tx_mac_id);
        double get_scpm (const std::string& tx_mac_id);
        double get_scd_stdev (const std::string& tx_mac_id);
        double get_top_confidence (const std::string& tx_mac_id);
        double get_processing_time(const std::string& tx_mac_id);
        float get_profile_variance(double phi_ind, double theta_ind);
//        std::vector<double> find_topN_phi(nc::NdArray<double> profile);
//        std::vector<double> find_topN_theta(nc::NdArray<double> profile);
//        std::vector<double> __top_N_phi, __top_N_theta;
        
        void get_eigen_rep_angle_trig(
            EigenDoubleMatrix& output, 
            EigenDoubleMatrix& input,
            std::string trig_operation
        );
        void get_eigen_rep_angle_trig_openmp(
            EigenDoubleMatrix& output, 
            EigenDoubleMatrix& input,
            std::string trig_operation
        );
        void get_cwiseProduct_openmp(
            EigenDoubleMatrix& output, 
            EigenDoubleMatrix& input1,
            EigenDoubleMatrix& input2
        );
        void get_repmat(
            EigenDoubleMatrix& output, 
            EigenDoubleMatrix& input,
            int rows, 
            int cols
        );
        void get_cwiseProduct(
            EigenDoubleMatrix& output, 
            EigenDoubleMatrix& input1,
            EigenDoubleMatrix& input2
        );
        void get_cwiseProduct_cd(
            EigencdMatrix& output, 
            EigenDoubleMatrix& input1,
            EigencdMatrix& input2
        );
        void get_matrix_block(
            EigenDoubleMatrix& output, 
            EigenDoubleMatrix& input1,
            int rows,
            int cols
        );
        void get_eterm_3DAdjustment(
            EigencdMatrix& output, 
            EigenDoubleMatrix& input
        );
        void get_bterm_all(
            EigencdMatrix& e_term_exp, 
            EigenDoubleMatrix &e_rep_pitch,
            EigenDoubleMatrix &diff_phi_yaw,
            EigenDoubleMatrix &rep_rho
        );
        void get_block_exp(
            EigencdMatrix& output, 
            EigencdMatrix& input,
            int start,
            int end, 
            int rows, 
            int cols
        );  
        void get_two_antenna_bterm_all(
            EigencdMatrix &e_term_exp,
            EigenDoubleMatrix &eigen_yaw_list
        );
        void get_bterm_all_subcarrier_conjugate(
            EigenDoubleMatrix &e_term,
            EigenDoubleMatrix &eigen_yaw_list
        );
        void get_bterm_all_subcarrier(
            EigenDoubleMatrix &e_term_exp,
            EigenDoubleMatrix &eigen_pitch_list, 
            EigenDoubleMatrix &eigen_yaw_list, 
            EigenDoubleMatrix &rep_rho
        );
        void getExponential(
            EigencdMatrix &out, 
            EigencdMatrix &in
        );
        
        std::vector<double> get_top_magnitudes(const std::string &tx_mac_id);
        std::string get_channel_data_output_filename(const std::string &tx_mac_id);
        std::pair<std::vector<double>,std::vector<double>> find_topN();
        std::pair<std::vector<double>,std::vector<double>> get_topN_AOA();
        std::pair<double,double> get_closest_AOA();
        std::pair<double,double> get_phi_theta();
        std::pair<std::vector<double>, std::vector<double>> find_topN_azimuth(); 

        std::unordered_map<std::string,std::string> data_sample_ts;
        std::unordered_map<std::string,std::string> tx_name_list;
        std::unordered_map<std::string, nc::NdArray<double>> get_all_aoa_profile();
        std::unordered_map<std::string, std::pair<std::vector<double>,std::vector<double>>> get_TX_topN_angles();
        std::unordered_map<std::string, int> get_paired_pkt_count();
        std::unordered_map<std::string, std::vector<double>>get_all_confidence();

        std::vector<double> top_aoa_error(
            double phi, 
            double theta,
            std::pair<double,double> groundtruth_angles,
            const string& traj_type
        );
        std::vector<std::vector<float>> get_aoa_error(
            const std::pair<std::vector<double>,
            std::vector<double>>& topN_AOA,
            std::pair<double,double> groundtruth_angles,
            const string& traj_type
        );   

        nlohmann::json __precompute_config; 
        nlohmann::json get_stats(
            double true_phi,
            double true_theta,
            std::vector<vector<float>>& aoa_error,
            const std::string& tx_mac_id,
            const std::string& tx_name,
            const nc::NdArray<double>& rx_pos_true, 
            const nc::NdArray<double>& rx_pos_est,
            nlohmann::json true_positions_tx,
            const int pos_idx
        );
        nlohmann::json get_stats_old_json(
            double true_phi,
            double true_theta,
            std::vector<vector<float>>& aoa_error,
            const std::string& tx_mac_id,
            const std::string& tx_name,
            const nc::NdArray<double>& rx_pos_true, 
            const nc::NdArray<double>& rx_pos_est,
            nlohmann::json true_positions_tx,
            const int pos_idx
        );                                
        nlohmann::json get_performance_stats(
            const std::string& tx_mac_id,
            const std::string& tx_name
        );

        nc::NdArray<double> get_aoa_profile();
        nc::NdArray<double> compute_conjugate_profile_bartlett_multithread(
            const nc::NdArray<std::complex<double>> &input_h_list,
            const nc::NdArray<double> &input_pose_list
        );
        nc::NdArray<double> compute_conjuate_profile_music_offboard(
            const nc::NdArray<std::complex<double>> &input_h_list,
            const nc::NdArray<double> &input_pose_list
        );
        
        bool get__FLAG_two_antenna();
                                           
};


#endif //CSITOOLBOX_WSR_MODULE_H