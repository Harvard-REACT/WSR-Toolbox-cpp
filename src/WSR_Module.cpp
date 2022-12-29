/**
 * (c) REACT LAB, Harvard University
	Author: Ninad Jadhav, Weiying Wang
*/

#include "csitoolbox/WSR_Module.h"
#include <omp.h>

WSR_Util utils;
constexpr bool __takeOwnership = true;

//======================================================================================================================
/**
 * Description: 
 * Input:
 * Output:
 * */
WSR_Module::WSR_Module(void) {}
WSR_Module::~WSR_Module(void) {}

//======================================================================================================================
/**
 * Description: 
 * Input:
 * Output:
 * */
WSR_Module::WSR_Module(std::string config_fn)
{
    std::cout << "log-info  [Initialize]: ----- Started Initializations -----" << std::endl;
    std::ifstream input(config_fn);
    if (input.fail())
    {
        throw std::runtime_error("log-info  [Initialize]: Unable to open file " + config_fn);
    }

    input >> __precompute_config;

    //Set flags
    __FLAG_packet_threshold   = bool(__precompute_config["use_max_packets_threshold"]["value"]);
    __FLAG_debug              = bool(__precompute_config["debug"]["value"]);
    __FLAG_interpolate_phase  = bool(__precompute_config["interpolate_phase"]["value"]);
    __FLAG_sub_sample         = bool(__precompute_config["sub_sample_channel_data"]["value"]);
    __FLAG_normalize_profile  = bool(__precompute_config["normalize_profile"]["value"]);
    __FLag_use_packet_id      = bool(__precompute_config["use_packet_id"]["value"]);
    bool __FLAG_use_multiple_sub_carriers = bool(__precompute_config["multiple_sub_carriers"]["value"]);
    __FLAG_use_relative_displacement = bool(__precompute_config["use_relative_trajectory"]["value"]);
    __FLAG_two_antenna        = bool(__precompute_config["use_two_antennas"]["value"]);
    __FLAG_slice              = bool(__precompute_config["slice_displacement"]["value"]);
    __FLAG_slice_first        = bool(__precompute_config["slice_displacement_first"]["value"]);
    __FLAG_slice_second       = bool(__precompute_config["slice_displacement_second"]["value"]);

    //Set variables
    __centerfreq              = (5000 + double(__precompute_config["channel"]["value"]) * 5) * 1e6;
    __time_offset             = double(__precompute_config["time_offset"]["value"]);
    __time_threshold          = double(__precompute_config["time_threshold"]["value"]);
    __lambda                  = double(__precompute_config["c"]["value"]) / __centerfreq;
    __nphi                    = int(__precompute_config["nphi"]["value"]);
    __ntheta                  = int(__precompute_config["ntheta"]["value"]);
    _phi_min                  = int(__precompute_config["phi_min"]["value"]);
    _phi_max                  = int(__precompute_config["phi_max"]["value"]);
    _theta_min                = int(__precompute_config["theta_min"]["value"]);
    _theta_max                = int(__precompute_config["theta_max"]["value"]);
    __snum_start              = int(__precompute_config["scnum_start"]["value"]);
    _topN_count               = int(__precompute_config["topN_count"]["value"]);
    __max_packets_to_process  = int(__precompute_config["max_packets_to_process"]["value"]);
    __min_packets_to_process  = int(__precompute_config["min_packets_to_process"]["value"]);
    __peak_radius             = int(__precompute_config["peak_radius"]["value"]);
    __snum_end                = int(__precompute_config["scnum_end"]["value"]);
    __displacement_type        = __precompute_config["displacement_type"]["value"];
    __relative_magnitude_threshold = int(__precompute_config["top_N_magnitude"]["value"]);
    __antenna_separation      = float(__precompute_config["antenna_separation"]["value"]);
    __estimator               = __precompute_config["aoa_estimator"]["value"];
    __debug_dir               = __precompute_config["debug_dir"]["value"].dump();
    __debug_dir.erase(remove(__debug_dir.begin(), __debug_dir.end(), '\"'), __debug_dir.end());


    //=================== Check variables =======================
    if(__estimator!="bartlett" && __estimator!="music")
    {
        THROW_INVALID_ARGUMENT_ERROR("Invalid AOA estimator selected. Valid options: bartlett, music.\n");
    }
    if(__displacement_type!="2D" && __displacement_type!="3D")
    {
        THROW_INVALID_ARGUMENT_ERROR("Invalid displacement type. Valid options: 2D, 3D.\n");
    }
    //=================== Check variables =======================


    if(__FLAG_two_antenna)
    {
         __RX_SAR_robot_MAC_ID     = __precompute_config["input_RX_channel_csi_fn"]["value"]["mac_id"];       
    }
    else
    {
        __RX_SAR_robot_MAC_ID      = __precompute_config["input_RX_channel_csi_fn"]["value"]["mac_id"];
        __RX_SAR_robot_MAC_ID_List = __precompute_config["RX_mac_ID_list"]["value"].get<std::vector<std::string>>();
    }

    theta_list = nc::linspace(_theta_min * M_PI / 180, _theta_max * M_PI / 180, __ntheta);
    phi_list = nc::linspace(_phi_min * M_PI / 180, _phi_max * M_PI / 180, __nphi);
    precomp_rep_phi = nc::repeat(phi_list.transpose(), __ntheta, 1);
    precomp_rep_theta = nc::repeat(theta_list.transpose(), 1, __nphi);
    
    // std::cout << "Precomp rep phi =" << precomp_rep_phi.shape() << std::endl;
    // std::cout << "Precomp rep theta =" << precomp_rep_theta.shape() << std::endl;
    
    precomp_rep_theta = nc::reshape(precomp_rep_theta, __nphi * __ntheta, 1);
    // std::cout << "Precomp rep theta =" << precomp_rep_theta.shape() << std::endl;

    nc::NdArray<double> lambda_list = {__lambda};

    __eigen_precomp_rep_phi = EigenDoubleMatrixMap(precomp_rep_phi.data(),
                                                   precomp_rep_phi.numRows(),
                                                   precomp_rep_phi.numCols());
    __eigen_precomp_rep_theta = EigenDoubleMatrixMap(precomp_rep_theta.data(),
                                                     precomp_rep_theta.numRows(),
                                                     precomp_rep_theta.numCols());
    
    std::thread t2(&WSR_Module::get_repmat, this,
                    std::ref(__precomp__eigen_rep_phi),
                    std::ref(__eigen_precomp_rep_phi), 1, __max_packets_to_process);

    std::thread t3(&WSR_Module::get_repmat, this,
                    std::ref(__precomp__eigen_rep_theta),
                    std::ref(__eigen_precomp_rep_theta), 1, __max_packets_to_process);

    t2.join();
    t3.join();


    if (__FLAG_debug)
    {
        //List of bool flags that impact calculations
        std::cout << "  Displacement Type = " << __displacement_type << std::endl;
        std::cout << "  WiFi Channel = " << double(__precompute_config["channel"]["value"]) << std::endl;
        std::cout << "  Channel center frequency (GHz) = " << __centerfreq << std::endl;
        std::cout << "  WiFi signal wavelength = " << __lambda << std::endl;
        std::cout << "  __FLAG_packet_threshold = " << utils.bool_to_string(__FLAG_packet_threshold) << std::endl;
        std::cout << "  __FLAG_debug = " << utils.bool_to_string(__FLAG_debug) << std::endl;
        std::cout << "  __FLAG_interpolate_phase = " << utils.bool_to_string(__FLAG_interpolate_phase) << std::endl;
        std::cout << "  __FLAG_sub_sample = " << utils.bool_to_string(__FLAG_sub_sample) << std::endl;
        std::cout << "  __FLAG_normalize_profile = " << utils.bool_to_string(__FLAG_normalize_profile) << std::endl;
        std::cout << "  __FLAG_use_multiple_sub_carriers = " << utils.bool_to_string(__FLAG_use_multiple_sub_carriers) << std::endl;
        std::cout << "  __FLAG_use_relative_displacement = " << utils.bool_to_string(__FLAG_use_relative_displacement) << std::endl;
    }

    std::cout << "log-info  [Initialize]: ----- Finished Initializations ----- " << std::endl;
}


//======================================================================================================================
/**
 * Description: 
 * Input:
 * Output:
 * */
int WSR_Module::calculate_AOA_profile(std::string rx_csi_file,
                                      std::unordered_map<std::string, std::string> tx_csi_file,
                                      nc::NdArray<double> displacement,
                                      nc::NdArray<double> displacement_timestamp)
{

    std::cout << "log-info  [calculate_AOA_profile] ============ Starting WSR module ==============" << std::endl;

    WIFI_Agent RX_SAR_robot;
    nc::NdArray<std::complex<double>> h_list_all;
    nc::NdArray<std::complex<double>> h_list_static;
    nc::NdArray<std::complex<double>> h_list;
    nc::NdArray<double> csi_timestamp_all;
    nc::NdArray<double> csi_timestamp;
    double cal_ts_offset;
    int ret_val = 0;
    std::vector<std::string> mac_id_tx;
    std::vector<DataPacket> data_packets_RX, data_packets_TX;
    
    std::cout << "log-info  [calculate_AOA_profile]: Parsing CSI Data " << std::endl;
    utils.readCsiData(rx_csi_file, RX_SAR_robot, __FLAG_debug);

    std::cout << "log-info  [calculate_AOA_profile]: Neighbouring TX robot IDs count = " << RX_SAR_robot.unique_mac_ids_packets.size() << std::endl;

    for (auto key : RX_SAR_robot.unique_mac_ids_packets)
    {
        if (__FLAG_debug)
            std::cout << "log-debug [calculate_AOA_profile]: Detected MAC ID = " << key.first
                      << ", Packet count: = " << key.second << std::endl;
        mac_id_tx.push_back(key.first);
    }

    //Get AOA profile for each of the RX neighboring robots
    std::cout << "log-info  [calculate_AOA_profile]: Getting AOA profiles" << std::endl;
    
    for (int num_tx = 0; num_tx < mac_id_tx.size(); num_tx++)
    {
        WIFI_Agent TX_Neighbor_robot;
        std::pair<nc::NdArray<std::complex<double>>, nc::NdArray<double>> csi_data;

        if (tx_csi_file.find(mac_id_tx[num_tx]) == tx_csi_file.end())
        {
            std::cout << "log-info  [calculate_AOA_profile]: No CSI data available for TX Neighbor MAC-ID: "
                          << mac_id_tx[num_tx] << ". Skipping" << std::endl;
            continue;
        }

        std::cout << "log-info  [calculate_AOA_profile]: =========================" << std::endl;
        std::cout << "log-info  [calculate_AOA_profile]: Profile for RX_SAR_robot MAC-ID: " << __RX_SAR_robot_MAC_ID
                  << ", TX_Neighbor_robot MAC-ID: " << mac_id_tx[num_tx] << std::endl;

        utils.readCsiData(tx_csi_file[mac_id_tx[num_tx]], TX_Neighbor_robot, __FLAG_debug);

        for (auto key : TX_Neighbor_robot.unique_mac_ids_packets)
        {
            if (__FLAG_debug)
                std::cout << "log-debug [calculate_AOA_profile]: Detected RX MAC IDs = " << key.first
                          << ", Packet count: = " << key.second << std::endl;
        }

        data_packets_RX = RX_SAR_robot.get_wifi_data(mac_id_tx[num_tx]);          //Packets for a TX_Neigbor_robot in RX_SAR_robot's csi file
        data_packets_TX = TX_Neighbor_robot.get_wifi_data(__RX_SAR_robot_MAC_ID); //Packets only of RX_SAR_robot in a TX_Neighbor_robot's csi file

        if (__FLAG_debug)
        {
            std::cout << "log-debug [calculate_AOA_profile]: Packets for TX_Neighbor_robot collected by RX_SAR_robot : "
                      << data_packets_RX.size() << std::endl;
            std::cout << "log-debug [calculate_AOA_profile]: Packets for RX_SAR_robot collected by TX_Neighbor_robot : "
                      << data_packets_TX.size() << std::endl;
        }

        if (__FLag_use_packet_id)
        {
            std::cout << "log-info  [calculate_AOA_profile]: Calculating forward-reverse channel product using counter " << std::endl;
            csi_data = utils.getForwardReverseChannelCounter(data_packets_RX,
                                                             data_packets_TX,
                                                             __FLAG_interpolate_phase,
                                                             __FLAG_sub_sample);
        }
        else
        {
            std::cout << "log-info  [calculate_AOA_profile]: Calculating forward-reverse channel product using Timestamps " << std::endl;
            csi_data = utils.getForwardReverseChannel_v2(data_packets_RX,
                                                         data_packets_TX,
                                                         __time_offset,
                                                         __time_threshold,
                                                         cal_ts_offset,
                                                         __FLAG_interpolate_phase,
                                                         __FLAG_sub_sample);
        }

        if (__FLAG_debug) std::cout << "log-debug [calculate_AOA_profile]: Corrected CFO " << std::endl;
        h_list_all = csi_data.first;
        csi_timestamp_all = csi_data.second;

        if (csi_timestamp_all.size() < __min_packets_to_process)
        {
            std::cout << csi_timestamp_all.size() << std::endl;
            if (__FLAG_debug)
                std::cout << "log-debug [calculate_AOA_profile]: Very few CSI data packets left after forward-backward product" << std::endl;
            break;
        }

        /*Get the shifted version of the pose timestamps such that they match exactly with csi timestamps.*/
        if (__FLAG_debug)
            std::cout << "log-debug [calculate_AOA_profile]: Removing unused csi data" << std::endl;
        
        std::pair<int, int> csi_timestamp_range = utils.returnClosestIndices(csi_timestamp_all, displacement_timestamp);
        int start_index = csi_timestamp_range.first, end_index = csi_timestamp_range.second;

        /*Slice the csi_timestamp using the indices to remove unused CSI values
        * Note: Make sure that the packet transmission freqency is high enough when random_packets is used, 
        * such that about 400 left after slicing even if the trajectory duration is small. 
        * */
        if (__FLAG_debug)
            std::cout << "log-debug [calculate_AOA_profile]: Slicing CSI timestamps" << std::endl;
        
        csi_timestamp = csi_timestamp_all({start_index, end_index}, csi_timestamp_all.cSlice());
        h_list = h_list_all({start_index, end_index}, h_list_all.cSlice());
        h_list_static = h_list_all({0, start_index}, h_list_all.cSlice());

        if (h_list.shape().rows < __min_packets_to_process)
        {
            std::cout << "log-info  [calculate_AOA_profile]: Not enough CSI packets." << std::endl;
            std::cout << "log-info  [calculate_AOA_profile]: Return Empty AOA profile" << std::endl;
            //Return empty dummpy AOA profile
            __aoa_profile = nc::zeros<double>(1, 1);
        }
        else
        {
            /*Interpolate the trajectory using the csi data timestamps*/
            if (__FLAG_debug)
                std::cout << "log-debug [calculate_AOA_profile]: interpolating the displacement and csi forward-reverse product" << std::endl;
            
            auto interpolated_data = utils.interpolate(csi_timestamp, displacement_timestamp, displacement);
            nc::NdArray<double> pose_list = interpolated_data.first;

            if (__FLAG_debug)
            {
                std::cout << "log-debug [calculate_AOA_profile]: CSI_packets_used = " << csi_timestamp.shape() << std::endl;
                std::cout << "log-debug [calculate_AOA_profile]: pose_list size  = " << pose_list.shape() << std::endl;
                std::cout << "log-debug [calculate_AOA_profile]: h_list size  = " << h_list.shape() << std::endl;

                //Store phase and timestamp of the channel for debugging
                std::string channel_data_sliced = __debug_dir + "/" + tx_name_list[mac_id_tx[num_tx]] + "_" + data_sample_ts[mac_id_tx[num_tx]] + "_sliced_channel_data.json";
                utils.writeCSIToJsonFile(h_list, csi_timestamp, channel_data_sliced, __FLAG_interpolate_phase);

                std::string channel_data_all = __debug_dir + "/" + tx_name_list[mac_id_tx[num_tx]] + "_" + data_sample_ts[mac_id_tx[num_tx]] + "_all_channel_data.json";
                utils.writeCSIToJsonFile(h_list_all, csi_timestamp_all, channel_data_all, __FLAG_interpolate_phase);

                //Store the packet distribution to check for spotty packets
                std::string packet_dist = __debug_dir + "/" + tx_name_list[mac_id_tx[num_tx]] + "_" + data_sample_ts[mac_id_tx[num_tx]] + "_packet_dist.json";
                utils.writePacketDistributionToJsonFile(csi_timestamp, displacement_timestamp, displacement, packet_dist);

                //Store interpolated trajectory for debugging
                std::string interpl_trajectory = __debug_dir + "/" + tx_name_list[mac_id_tx[num_tx]] + "_" + data_sample_ts[mac_id_tx[num_tx]] + "_interpl_trajectory.json";
                utils.writeTrajToFile(pose_list, interpl_trajectory);
            }

            /*Interpolate the trajectory and csi data*/
            std::cout << "log-info  [calculate_AOA_profile]: Calculating AOA profile..." << std::endl;
            auto starttime = std::chrono::high_resolution_clock::now();

            if(__estimator == "bartlett")
                __aoa_profile = compute_profile_bartlett(h_list, pose_list);
            else if(__estimator == "music")
                __aoa_profile = compute_profile_music(h_list, pose_list);

            auto endtime = std::chrono::high_resolution_clock::now();
            float processtime = std::chrono::duration<float, std::milli>(endtime - starttime).count();

            //Stats
            std::pair<std::vector<double>, std::vector<double>> top_N = __displacement_type == "2D" ? find_topN_azimuth():find_topN();
            __TX_top_N_angles[mac_id_tx[num_tx]] = top_N;
            __paired_pkt_count[mac_id_tx[num_tx]] = csi_timestamp.shape().rows;
            __perf_aoa_profile_cal_time[mac_id_tx[num_tx]] = processtime / 1000;
            __memory_used[mac_id_tx[num_tx]] = utils.mem_usage() / 1000000;
            __calculated_ts_offset[mac_id_tx[num_tx]] = cal_ts_offset;
            __rx_pkt_size[mac_id_tx[num_tx]] = data_packets_RX.size();
            __tx_pkt_size[mac_id_tx[num_tx]] = data_packets_TX.size();
            __top_peak_confidence[mac_id_tx[num_tx]] = __aoa_profile_variance[0];
            __all_topN_magnitudes[mac_id_tx[num_tx]] = __peak_magnitudes;
            __all_topN_above_threshold[mac_id_tx[num_tx]] = __num_peaks_above_threshold;
        }

        /*Store the aoa_profile*/
        __all_aoa_profiles[mac_id_tx[num_tx]] = __aoa_profile;
        __all_topN_confidence[mac_id_tx[num_tx]] = __aoa_profile_variance;

        //TODO: get azimuth and elevation from beta_profile
        std::cout << "log-info  [calculate_AOA_profile]: Completed AOA calculation." << std::endl;
        TX_Neighbor_robot.reset();
    }

    std::cout << "============ WSR module end ==============" << std::endl;
    RX_SAR_robot.reset();

    return ret_val;
}
//=============================================================================================================================
/**
 *
 *
 * */
nc::NdArray<double> WSR_Module::compute_profile_bartlett(
    const nc::NdArray<std::complex<double>> &input_h_list,
    const nc::NdArray<double> &input_pose_list)
{
    auto total_start  = std::chrono::high_resolution_clock::now();
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    auto total_end = std::chrono::high_resolution_clock::now();
    nc::NdArray<std::complex<double>> h_list_single_channel;
    EigencdMatrix eigen_eterm_3DAdjustment, e_term_prod;
    EigenDoubleMatrix eigen_rep_lambda, eigen_rep_phi, eigen_rep_theta, diff_phi_yaw, eigen_rep_yaw, eigen_rep_pitch, eigen_rep_rho;

    int total_packets = input_h_list.shape().rows;
    if (total_packets != input_pose_list.shape().rows)
    {
        THROW_INVALID_ARGUMENT_ERROR("Number of CSI and poses are different.\n");
    }

    int max_packets = total_packets;
    /*Use max packets*/
    if (__FLAG_packet_threshold)
    {
        max_packets = input_h_list.shape().rows > __max_packets_to_process ? __max_packets_to_process : input_h_list.shape().rows;
    }

    nc::NdArray<std::complex<double>> h_list = input_h_list(nc::Slice(0, max_packets), input_h_list.cSlice());
    nc::NdArray<double> pose_list = input_pose_list(nc::Slice(0, max_packets), input_pose_list.cSlice());

    if (__FLAG_debug)
        std::cout << "log-debug [Bartlett estimator] Total packets: " << total_packets << ", Max packets used: " << max_packets << std::endl;

    std::cout.precision(15);

    if (__FLAG_interpolate_phase)
        h_list_single_channel = h_list(h_list.rSlice(), 30);
    else
        h_list_single_channel = h_list(h_list.rSlice(), 15);

    auto num_poses = nc::shape(pose_list).rows;
    if (h_list_single_channel.shape().cols == 0)
    {
        THROW_INVALID_ARGUMENT_ERROR("No subcarrier selected for CSI data.\n");
    }

    //========== Matrix repmat operation ====================
    if (__FLAG_debug)
    {
        std::cout << "log-debug [Bartlett estimator] : Matrix repmat for yaw, pitch and rho matrices" << std::endl;
        start = std::chrono::high_resolution_clock::now();
    }
    auto pose_x = pose_list(pose_list.rSlice(), 0);
    auto pose_y = pose_list(pose_list.rSlice(), 1);
    auto pose_z = pose_list(pose_list.rSlice(), 2);

    auto yaw_list = nc::arctan2(pose_y, pose_x);
    yaw_list = nc::angle(-nc::exp(nc::multiply(yaw_list, std::complex<double>(0, 1))));
    EigenDoubleMatrix eigen_yaw_list_tmp = EigenDoubleMatrixMap(yaw_list.data(),
                                                                yaw_list.numRows(),
                                                                yaw_list.numCols());
    EigenDoubleMatrix eigen_yaw_list = eigen_yaw_list_tmp.transpose();
 

    auto pitch_list = nc::arctan2(pose_z, nc::sqrt(nc::square(pose_x) + nc::square(pose_y)));
    pitch_list = nc::angle(-nc::exp(nc::multiply(pitch_list, std::complex<double>(0, 1))));
    EigenDoubleMatrix eigen_pitch_list_tmp = EigenDoubleMatrixMap(pitch_list.data(),
                                                                  pitch_list.numRows(),
                                                                  pitch_list.numCols());
    EigenDoubleMatrix eigen_pitch_list = eigen_pitch_list_tmp.transpose();

    assert(pitch_list.shape() == yaw_list.shape());

    auto rho_list = nc::sqrt(nc::square(pose_x) + nc::square(pose_y) + nc::square(pose_z));
    EigenDoubleMatrix eigen_rho_list_tmp = EigenDoubleMatrixMap(rho_list.data(),
                                                                rho_list.numRows(),
                                                                rho_list.numCols());
    EigenDoubleMatrix eigen_rho_list = eigen_rho_list_tmp.transpose();
    
    if (__FLAG_debug)
    {
        end = std::chrono::high_resolution_clock::now();
        std::cout << "log-debug [Bartlett estimator] : Time elapsed for repmat operation: " << (end - start) / std::chrono::milliseconds(1) << std::endl;

        start = std::chrono::high_resolution_clock::now();
    }

    //========== Steering vector computation ===============
    std::cout << "log-info  [Bartlett estimator] : Computing steering vector" << std::endl;
    
    EigencdMatrix e_term_exp(__nphi * __ntheta, num_poses);
    //====Openmp implementation =============.
    if(__displacement_type == "2D")
        get_bterm_all_2D(std::ref(e_term_exp), std::ref(eigen_pitch_list), std::ref(eigen_yaw_list), std::ref(eigen_rho_list));
    else
        get_bterm_all_3D(std::ref(e_term_exp), std::ref(eigen_pitch_list), std::ref(eigen_yaw_list), std::ref(eigen_rho_list));
    
    //====Openmp implementation=============.
    
    std::complex<double> *cddataPtr = new std::complex<double>[e_term_exp.rows() * e_term_exp.cols()];
    EigencdMatrixMap(cddataPtr, e_term_exp.rows(), e_term_exp.cols()) = e_term_exp;
    auto e_term = nc::NdArray<std::complex<double>>(cddataPtr, e_term_exp.rows(), e_term_exp.cols(), __takeOwnership);
    
    if (__FLAG_debug)
    {
        end = std::chrono::high_resolution_clock::now();
        std::cout << "log-debug [Bartlett estimator] : Time elapsed for Steering vector (eterm_prod and eterm_exp) :  " 
                  << (end - start) / std::chrono::milliseconds(1) << "ms" << std::endl;
    }
    //========== Steering vector computation ===============

    //========== AOA profile computation ===============
    if (__FLAG_debug) std::cout << "log-debug  [Bartlett estimator] : Getting transpose and returning profile" << std::endl;
    auto result_mat = nc::matmul(e_term, h_list_single_channel);
    auto betaProfileProd = nc::power(nc::abs(result_mat), 2);
    auto angle_of_arrival_profile = nc::reshape(betaProfileProd, __ntheta, __nphi);

    if (__FLAG_normalize_profile)
    {
        auto sum_val = nc::sum(nc::sum(angle_of_arrival_profile));
        angle_of_arrival_profile = angle_of_arrival_profile / sum_val(0, 0);
    }

    angle_of_arrival_profile = nc::transpose((angle_of_arrival_profile));
    std::cout << "log-info  [Bartlett estimator] : Got AOA profile" << std::endl;
    //========== AOA profile computation ===============

    if(__FLAG_debug)
    {
        total_end = std::chrono::high_resolution_clock::now();
        std::cout << "log-debug [Bartlett estimator] : Profile computation time " << (total_end- total_start)/std::chrono::milliseconds(1)<< " ms"<< std::endl;
    }

    return angle_of_arrival_profile;
}
//=============================================================================================================================
/**
 *
 *
 * */
nc::NdArray<double> WSR_Module::compute_profile_music(
    const nc::NdArray<std::complex<double>> &input_h_list,
    const nc::NdArray<double> &input_pose_list)
{
    auto total_start  = std::chrono::high_resolution_clock::now();
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    auto total_end = std::chrono::high_resolution_clock::now();
    
    EigencdMatrix eigen_eterm_3DAdjustment;
    EigencdMatrix e_term_prod;
    EigenDoubleMatrix eigen_rep_lambda; 
    EigenDoubleMatrix eigen_rep_phi;
    EigenDoubleMatrix eigen_rep_theta;
    EigenDoubleMatrix diff_phi_yaw;
    EigenDoubleMatrix eigen_rep_yaw;
    EigenDoubleMatrix eigen_rep_pitch;
    EigenDoubleMatrix eigen_rep_rho;
    EigenDoubleMatrix eigen_betaProfile_final;
    bool first = true;
    
    WSR_Util util_obj;

    int total_packets = input_h_list.shape().rows;
    if (total_packets != input_pose_list.shape().rows)
    {
        THROW_INVALID_ARGUMENT_ERROR("number of CSI and poses are different.\n");
    }

    int max_packets = total_packets;
    /*Use max packets*/
    if (__FLAG_packet_threshold)
    {
        max_packets = input_h_list.shape().rows > __max_packets_to_process ? __max_packets_to_process : input_h_list.shape().rows;
    }

    nc::NdArray<std::complex<double>> h_list = input_h_list(nc::Slice(0, max_packets), input_h_list.cSlice());
    nc::NdArray<double> pose_list = input_pose_list(nc::Slice(0, max_packets), input_pose_list.cSlice());
    auto num_poses = nc::shape(pose_list).rows;

    if (__FLAG_debug) std::cout << "log-debug  [MUSIC estimator] Total packets: " << total_packets << ", Max packets used: " << max_packets << std::endl;

    std::cout.precision(15);
    //========== Matrix repmat operation ====================
    if (__FLAG_debug)
    {
        std::cout << "log-debug [MUSIC estimator] : Matrix repmat for yaw, pitch and rho matrices" << std::endl;
        start = std::chrono::high_resolution_clock::now();
    }
    auto pose_x = pose_list(pose_list.rSlice(), 0);
    auto pose_y = pose_list(pose_list.rSlice(), 1);
    auto pose_z = pose_list(pose_list.rSlice(), 2);

    auto yaw_list = nc::arctan2(pose_y, pose_x);
    yaw_list = nc::angle(-nc::exp(nc::multiply(yaw_list, std::complex<double>(0, 1))));
    EigenDoubleMatrix eigen_yaw_list_tmp = EigenDoubleMatrixMap(yaw_list.data(),
                                                                yaw_list.numRows(),
                                                                yaw_list.numCols());
    EigenDoubleMatrix eigen_yaw_list = eigen_yaw_list_tmp.transpose();
    auto pitch_list = nc::arctan2(pose_z, nc::sqrt(nc::square(pose_x) + nc::square(pose_y)));
    pitch_list = nc::angle(-nc::exp(nc::multiply(pitch_list, std::complex<double>(0, 1))));
    EigenDoubleMatrix eigen_pitch_list_tmp = EigenDoubleMatrixMap(pitch_list.data(),
                                                                  pitch_list.numRows(),
                                                                  pitch_list.numCols());
    EigenDoubleMatrix eigen_pitch_list = eigen_pitch_list_tmp.transpose();


    assert(pitch_list.shape() == yaw_list.shape());
    auto rho_list = nc::sqrt(nc::square(pose_x) + nc::square(pose_y) + nc::square(pose_z));
    EigenDoubleMatrix eigen_rho_list_tmp = EigenDoubleMatrixMap(rho_list.data(),
                                                                rho_list.numRows(),
                                                                rho_list.numCols());
    EigenDoubleMatrix eigen_rho_list = eigen_rho_list_tmp.transpose();
    //========== Matrix repmat operation ====================    

    if (__FLAG_debug)
    {
        end = std::chrono::high_resolution_clock::now();
        std::cout << "log-debug [MUSIC estimator] : Time elapsed for repmat operation: " << (end - start) / std::chrono::milliseconds(1) << std::endl;

        start = std::chrono::high_resolution_clock::now();
    }

    std::cout << "log-info  [MUSIC estimator] : Computing steering vector" << std::endl;
    //========== Steering vector computation ================
    EigenDoubleMatrix e_term(__nphi * __ntheta, num_poses);

    //====Openmp implementation 0.2 sec faster =============.
    if(__displacement_type == "2D")
        get_bterm_all_subcarrier_2D(std::ref(e_term), std::ref(eigen_pitch_list), std::ref(eigen_yaw_list), std::ref(eigen_rho_list));
    else
        get_bterm_all_subcarrier_3D(std::ref(e_term), std::ref(eigen_pitch_list), std::ref(eigen_yaw_list), std::ref(eigen_rho_list));
    //====Openmp implementation=============
    //========== Steering vector computation ================

    if (__FLAG_debug)
    {
        end = std::chrono::high_resolution_clock::now();
        std::cout << "log-debug [MUSIC estimator] : Time elapsed for Steering vector (eterm_prod and eterm_exp) :  " 
                  << (end - start) / std::chrono::milliseconds(1) << std::endl;
    }

    //========== AOA profile computation ===============
    std::cout << "log-info  [MUSIC estimator] : Getting profile using multiple subcarriers" << std::endl;
    for(int h_i=__snum_start; h_i<__snum_end; h_i++)
    {
        std::cout << "log-info  [MUSIC estimator] Subcarrier : " << h_i << std::endl;
 
        double carrierfreq = (5000 + double(__precompute_config["channel"]["value"]) * 5) * 1e6 +
                            (15.5 - h_i) * 20e6 / 30;
        double lambda_inv =  carrierfreq/double(__precompute_config["c"]["value"]);
        EigencdMatrix temp1 = e_term * (-4.0 * std::complex<double>(0, 1) * M_PI * lambda_inv);
        EigencdMatrix e_term_exp(__nphi * __ntheta, num_poses);
        getExponential(e_term_exp, temp1);
        
        nc::NdArray<std::complex<double>> h_list_single_channel;
        h_list_single_channel = h_list(h_list.rSlice(), h_i);
        
        if (h_list_single_channel.shape().cols == 0)
        {
            THROW_INVALID_ARGUMENT_ERROR("No subcarrier selected for CSI data.\n");
        }

        //Get complex conjugate of the channel  
        auto h_list_eigen = EigencdMatrixMap(h_list_single_channel.data(), h_list_single_channel.numRows(), h_list_single_channel.numCols());
        EigenDoubleMatrix eigen_betaProfileProd = (e_term_exp * h_list_eigen).cwiseAbs2();    
        EigenDoubleMatrixMap eigen_betaProfile(eigen_betaProfileProd.data(),__ntheta, __nphi);
        
        if(__FLAG_debug)
        {
            std::cout << "log-debug [MUSIC estimator] h_list rows = " << h_list_eigen.rows() << ", h_list cols = " << h_list_eigen.cols() << std::endl;
            std::cout << "log-debug [MUSIC estimator] e_term rows = " << e_term_exp.rows() << ", e_term cols = " << e_term_exp.cols() << std::endl;
            std::cout << "log-debug [MUSIC estimator] beta profile rows = " << eigen_betaProfile.rows() << ",  beta profile cols = " << eigen_betaProfile.cols() << std::endl;
        }
        
        //Multiply the subcarrier profiles
        if(first)
        {
            eigen_betaProfile_final = eigen_betaProfile;
            first = false;
        }
        else
        {
            eigen_betaProfile_final = eigen_betaProfile_final.cwiseProduct(eigen_betaProfile);
        }
    }
    
    double *cddataPtr2 = new double[eigen_betaProfile_final.rows() * eigen_betaProfile_final.cols()];
    EigenDoubleMatrixMap(cddataPtr2, eigen_betaProfile_final.rows(), eigen_betaProfile_final.cols()) = eigen_betaProfile_final;
    auto angle_of_arrival_profile = nc::NdArray<double>(cddataPtr2, eigen_betaProfile_final.rows(), eigen_betaProfile_final.cols(), __takeOwnership);
    //========== AOA profile computation ===============

    if (__FLAG_debug) std::cout << "log-debug  [MUSIC estimator] : Getting transpose and returning profile" << std::endl;
    if (__FLAG_normalize_profile || (__snum_end -__snum_start > 1)) //Always normalize if using multiple subcarriers.
    {
        auto sum_val = nc::sum(nc::sum(angle_of_arrival_profile));
        angle_of_arrival_profile = angle_of_arrival_profile / sum_val(0, 0);
    }
    angle_of_arrival_profile = nc::transpose((angle_of_arrival_profile));
    
    std::cout << "log-info  [MUSIC estimator] : Got AOA profile" << std::endl;
    if (__FLAG_debug)
    {
        std::cout << "log-debug [MUSIC estimator] AOA profile shape:" << angle_of_arrival_profile.shape() << std::endl;
        total_end = std::chrono::high_resolution_clock::now();
        std::cout << "log-debug [MUSIC estimator] : Total Computation time : " << (total_end- total_start)/std::chrono::milliseconds(1)<< " ms"<< std::endl;
    }
    
    return angle_of_arrival_profile;
}
//======================================================================================================================
/**
 * Description: Checks the signal phase CFO correction using forward-reverse method (ref WSR IJRR 2022)
 * Input: RX_SAR_ROBOT csi file name (reverse channel data) and TX_SAR_Robot(s) file(s) name(s)
 * Output: none
 * */
int WSR_Module::test_csi_data(
    std::string rx_csi_file,
    std::unordered_map<std::string, std::string> tx_csi_file)
{

    std::cout << "log-info  [test_csi_data (forward-reverse channel)] ============ Testing CSI data ==============" << std::endl;

    WIFI_Agent RX_SAR_robot; //Broardcasts the csi packets and does SAR
    nc::NdArray<std::complex<double>> h_list_all, h_list_static, h_list;
    nc::NdArray<double> csi_timestamp_all, csi_timestamp;
    double cal_ts_offset, moving_channel_ang_diff_mean, moving_channel_ang_diff_stdev,
    static_channel_ang_mean, static_channel_ang_stdev;
    std::string debug_dir = __precompute_config["debug_dir"]["value"].dump();
    debug_dir.erase(remove(debug_dir.begin(), debug_dir.end(), '\"'), debug_dir.end());
    int ret_val = 0;

    std::cout << "log-info  [test_csi_data (forward-reverse channel)]: Parsing CSI Data " << std::endl;

    auto temp1 = utils.readCsiData(rx_csi_file, RX_SAR_robot, __FLAG_debug);

    // std::vector<std::string> mac_id_tx;
    std::vector<std::string> mac_id_tx;

    std::cout << "log-info [test_csi_data (forward-reverse channel)]: Neighbouring TX robot IDs count = " << RX_SAR_robot.unique_mac_ids_packets.size() << std::endl;

    for (auto key : RX_SAR_robot.unique_mac_ids_packets)
    {
        if (__FLAG_debug)
            std::cout << "log-debug [test_csi_data (forward-reverse channel)]: Detected MAC ID = " << key.first
                      << ", Packet count: = " << key.second << std::endl;
        mac_id_tx.push_back(key.first);
    }

    //Get AOA profile for each of the RX neighboring robots
    std::cout << "log-info  [test_csi_data (forward-reverse channel)]: Getting AOA profiles" << std::endl;
    
    std::vector<DataPacket> data_packets_RX, data_packets_TX;

    for (int num_tx = 0; num_tx < mac_id_tx.size(); num_tx++)
    {
        WIFI_Agent TX_Neighbor_robot; // Neighbouring robots who reply back
        std::pair<nc::NdArray<std::complex<double>>, nc::NdArray<double>> csi_data;

        if (tx_csi_file.find(mac_id_tx[num_tx]) == tx_csi_file.end())
        {
            std::cout << "log-info  [test_csi_data (forward-reverse channel)]: No CSI data available for TX Neighbor MAC-ID: "
                          << mac_id_tx[num_tx] << ". Skipping" << std::endl;
            continue;
        }
        std::cout << "log-info  [test_csi_data (forward-reverse channel)]: =========================" << std::endl;
        std::cout << "log-info  [test_csi_data (forward-reverse channel)]: Profile for RX_SAR_robot MAC-ID: " << __RX_SAR_robot_MAC_ID
                  << ", TX_Neighbor_robot MAC-ID: " << mac_id_tx[num_tx] << std::endl;

        auto temp2 = utils.readCsiData(tx_csi_file[mac_id_tx[num_tx]], TX_Neighbor_robot, __FLAG_debug);

        if(__FLAG_debug)
        {
            for (auto key : TX_Neighbor_robot.unique_mac_ids_packets)
            {
                std::cout << "log-debug [test_csi_data (forward-reverse channel)]: Detected RX MAC IDs = " << key.first
                            << ", Packet count: = " << key.second << std::endl;
            }
        }

        data_packets_RX = RX_SAR_robot.get_wifi_data(mac_id_tx[num_tx]);          //Packets for a TX_Neigbor_robot in RX_SAR_robot's csi file
        data_packets_TX = TX_Neighbor_robot.get_wifi_data(__RX_SAR_robot_MAC_ID); //Packets only of RX_SAR_robot in a TX_Neighbor_robot's csi file

        if (__FLAG_debug)
        {
            std:: cout << "log-debug [test_csi_data (forward-reverse channel)]----------Checking packet frame counter-------------------------------" << std::endl;
            for(int i=0; i<100; i++)
            {
                printf("%d, ", data_packets_TX[i].frame_count );
            }
            std:: cout << "\nlog-debug [test_csi_data (forward-reverse channel)]----------Checking packet frame counter-------------------------------" << std::endl;
        }
        

        std::cout << "log-info  [test_csi_data (forward-reverse channel)]: Packets for TX_Neighbor_robot collected by RX_SAR_robot : "
                    << data_packets_RX.size() << std::endl;
        std::cout << "log-info  [test_csi_data (forward-reverse channel)]: Packets for RX_SAR_robot collected by TX_Neighbor_robot : "
                    << data_packets_TX.size() << std::endl;


        if (__FLag_use_packet_id)
        {
            std::cout << "log-info  [test_csi_data (forward-reverse channel)]: Calculating forward-reverse channel product using Counter " << std::endl;
            csi_data = utils.getForwardReverseChannelCounter(data_packets_RX,
                                                             data_packets_TX,
                                                             __FLAG_interpolate_phase,
                                                             __FLAG_sub_sample);
        }
        else
        {
            std::cout << "log-info  [test_csi_data (forward-reverse channel)]: Calculating forward-reverse channel product using Timestamps " << std::endl;
            csi_data = utils.getForwardReverseChannel_v2(data_packets_RX,
                                                         data_packets_TX,
                                                         __time_offset,
                                                         __time_threshold,
                                                         cal_ts_offset,
                                                         __FLAG_interpolate_phase,
                                                         __FLAG_sub_sample);
        }

        std::cout << "log-info  [test_csi_data (forward-reverse channel)]: corrected CFO " << std::endl;
        h_list_all = csi_data.first;
        csi_timestamp_all = csi_data.second;

        bool moving = true;
        utils.get_phase_diff_metrics(h_list_all,
                                     moving_channel_ang_diff_mean,
                                     moving_channel_ang_diff_stdev,
                                     __FLAG_interpolate_phase,
                                     moving);

        if (h_list_all.shape().rows < __min_packets_to_process)
        {
            std::cout << "log-info  [test_csi_data (forward-reverse channel)]: Not enough CSI packets." << std::endl;
            std::cout << "log-info  [test_csi_data (forward-reverse channel)]: Return Empty AOA profile" << std::endl;
            //Return empty dummpy AOA profile
            __aoa_profile = nc::zeros<double>(1, 1);
        }
        else
        {

            std::cout << "log-info  [test_csi_data (forward-reverse channel)]: CSI_packets_used = " << csi_timestamp_all.shape() << std::endl;
            std::cout << "log-info  [test_csi_data (forward-reverse channel)]: h_list size  = " << h_list_all.shape() << std::endl;

            std::string debug_dir = __precompute_config["debug_dir"]["value"].dump();
            debug_dir.erase(remove(debug_dir.begin(), debug_dir.end(), '\"'), debug_dir.end());

            //Store phase and timestamp of the channel for debugging
            std::string channel_data_all = debug_dir + tx_name_list[mac_id_tx[num_tx]] + "_" + data_sample_ts[mac_id_tx[num_tx]] + "_all_channel_data.json";
            std::cout << "log [test_csi_data (forward-reverse channel)]: Output file: " << channel_data_all << std::endl;
            utils.writeCSIToJsonFile(h_list_all, csi_timestamp_all, channel_data_all, __FLAG_interpolate_phase);

            auto starttime = std::chrono::high_resolution_clock::now();
            auto endtime = std::chrono::high_resolution_clock::now();
            float processtime = std::chrono::duration<float, std::milli>(endtime - starttime).count();
            //Interpolate the trajectory and csi data
            __paired_pkt_count[mac_id_tx[num_tx]] = csi_timestamp_all.shape().rows;
            __calculated_ts_offset[mac_id_tx[num_tx]] = cal_ts_offset;
            __rx_pkt_size[mac_id_tx[num_tx]] = data_packets_RX.size();
            __tx_pkt_size[mac_id_tx[num_tx]] = data_packets_TX.size();
            __perf_aoa_profile_cal_time[mac_id_tx[num_tx]] = processtime / 1000;
            __channel_phase_diff_mean[mac_id_tx[num_tx]] = moving_channel_ang_diff_mean;
            __channel_phase_diff_stdev[mac_id_tx[num_tx]] = moving_channel_ang_diff_stdev;
            __channel_data_output_file[mac_id_tx[num_tx]] = channel_data_all;
        }

        //TODO: get azimuth and elevation from beta_profile
        std::cout << "log-info  [test_csi_data (forward-reverse channel)]: Completed Testing CSI data for TX_Neighbor_robot MAC-ID: " << mac_id_tx[num_tx] << std::endl;
        TX_Neighbor_robot.reset();
    }

    std::cout << "log-info  [test_csi_data (forward-reverse channel)] ============ WSR module end ==============" << std::endl;
    RX_SAR_robot.reset();

    return 0;
    
}
//=============================================================================================================================
/**
 * Description: Single antenna steering vector for pure 2D motion. Ignores elevation (theta) angles, thus speeding up computation
 * Input:
 * Output:
 */
void WSR_Module::get_bterm_all_2D(EigencdMatrix &e_term_exp,
                               EigenDoubleMatrix &eigen_pitch_list, EigenDoubleMatrix &eigen_yaw_list, EigenDoubleMatrix &rep_rho)
{

    int i = 0;
    int j = 0;
#pragma omp parallel for shared(e_term_exp, __eigen_precomp_rep_theta, eigen_pitch_list, eigen_yaw_list, rep_rho) private(i, j) collapse(2)
    for (i = 0; i < e_term_exp.rows(); i++)
    {
        for (j = 0; j < e_term_exp.cols(); j++)
        {
            //Ignore elevation (theta) if motion is 2D. Uses Theta = 90 deg
            e_term_exp(i, j) = exp((cos(eigen_pitch_list(0,j)) * 
                                    cos(__eigen_precomp_rep_phi(i, 0)-eigen_yaw_list(0,j))) *
                                    rep_rho(0, j) * 
                                    (-4.0 * std::complex<double>(0, 1) * M_PI / __lambda));
        }
    }
}
//=============================================================================================================================
/**
 * Description: Single antenna steering vector for 3D motion.
 * Input:
 * Output:
 */
void WSR_Module::get_bterm_all_3D(EigencdMatrix &e_term_exp,
                               EigenDoubleMatrix &eigen_pitch_list, EigenDoubleMatrix &eigen_yaw_list, EigenDoubleMatrix &rep_rho)
{

    int i = 0;
    int j = 0;
#pragma omp parallel for shared(e_term_exp, __eigen_precomp_rep_theta, eigen_pitch_list, eigen_yaw_list, rep_rho) private(i, j) collapse(2)
    for (i = 0; i < e_term_exp.rows(); i++)
    {
        for (j = 0; j < e_term_exp.cols(); j++)
        {
            e_term_exp(i, j) = exp((sin(__eigen_precomp_rep_theta(i, 0)) * 
                                    cos(eigen_pitch_list(0,j)) * 
                                    cos(__eigen_precomp_rep_phi(i, 0)-eigen_yaw_list(0,j)) +
                                    sin(eigen_pitch_list(0, j)) * 
                                    cos(__eigen_precomp_rep_theta(i, 0))) *
                                    rep_rho(0, j) * 
                                    (-4.0 * std::complex<double>(0, 1) * M_PI / __lambda));
        }
    }
}
//=============================================================================================================================
/**
 * Description: Does not include division by lambda in the exponential term; can be used for different subcarriers. Motion is 2D 
 * Input:
 * Output:
 */
void WSR_Module::get_bterm_all_subcarrier_2D(
                EigenDoubleMatrix &e_term,
                EigenDoubleMatrix &eigen_pitch_list, 
                EigenDoubleMatrix &eigen_yaw_list, 
                EigenDoubleMatrix &rep_rho)
{
    int i = 0;
    int j = 0;
#pragma omp parallel for shared(e_term, __eigen_precomp_rep_theta, eigen_pitch_list, eigen_yaw_list, rep_rho) private(i, j) collapse(2)
    for (i = 0; i < e_term.rows(); i++)
    {
        for (j = 0; j < e_term.cols(); j++)
        {
            //Ignore elevation (theta) if motion is 2D. Uses Theta = 90 deg
            e_term(i, j) = (cos(eigen_pitch_list(0,j)) * 
                            cos(__eigen_precomp_rep_phi(i, 0)-eigen_yaw_list(0,j))) *
                            rep_rho(0, j) ;
        }
    }
}
//=============================================================================================================================
/**
 * Description: Does not include division by lambda in the exponential term; can be used for different subcarriers 
 * Input:
 * Output:
 */
void WSR_Module::get_bterm_all_subcarrier_3D(
                EigenDoubleMatrix &e_term,
                EigenDoubleMatrix &eigen_pitch_list, 
                EigenDoubleMatrix &eigen_yaw_list, 
                EigenDoubleMatrix &rep_rho)
{
    int i = 0;
    int j = 0;
#pragma omp parallel for shared(e_term, __eigen_precomp_rep_theta, eigen_pitch_list, eigen_yaw_list, rep_rho) private(i, j) collapse(2)
    for (i = 0; i < e_term.rows(); i++)
    {
        for (j = 0; j < e_term.cols(); j++)
        {
            e_term(i, j) = (sin(__eigen_precomp_rep_theta(i, 0)) * 
                            cos(eigen_pitch_list(0,j)) * 
                            cos(__eigen_precomp_rep_phi(i, 0)-eigen_yaw_list(0,j)) +
                            sin(eigen_pitch_list(0, j)) * 
                            cos(__eigen_precomp_rep_theta(i, 0))) * 
                            rep_rho(0, j);
        }
    }
}
//=============================================================================================================================
/**
 * Description:
 * Input:
 * Output:
 * */
std::pair<std::vector<double>, std::vector<double>> WSR_Module::find_topN()
{
    std::cout << "log-info  [find_top_N] Getting top N angles from the AOA Profile" << std::endl;
    std::vector<double> ret_phi, ret_theta;
    nc::NdArray<double> max_peak = nc::amax(__aoa_profile);
    nc::NdArray<double> phi_max = nc::amax(__aoa_profile, nc::Axis::COL);
    __aoa_profile_variance.clear();
    __peak_magnitudes.clear();
    __num_peaks_above_threshold = 0;
    int peak_ind = 1, phi_idx = 0, theta_idx = 0;
    bool check_peak = false;

    // radius to define local maxima as per the config file
    const int radius = __peak_radius;
    auto all_idx_flat = nc::flip(nc::argsort((__aoa_profile.flatten())));
    
    //n*2 array save coordinates for sorted value in the whole profile
    nc::NdArray<int> sorted_inds(all_idx_flat.size(), 2);
    for (int i = 0; i < all_idx_flat.size(); i++)
    {
        sorted_inds.put(i, sorted_inds.cSlice(), utils.unravel_index(all_idx_flat(0, i), __nphi, __ntheta));
    }
    phi_idx = sorted_inds(0, 0);
    theta_idx = sorted_inds(0, 1);

    //Get the peak AOA
    if (__FLAG_debug)
    {
        std::cout << "log-debug [find_top_N]: Top azimuth angle " << peak_ind
                    << " : " << phi_list(0, phi_idx) * 180 / M_PI << std::endl;
    
        std::cout << "log-debug [find_top_N]: Top elevation angle " << peak_ind << " : "
                    << theta_list(0, theta_idx) * 180 / M_PI << std::endl;

        std::cout << "log-debug [find_top_N]: Phi idx = " << phi_idx << ", theta idx = " << theta_idx << std::endl;
    }

    ret_phi.push_back(phi_list(0, phi_idx) * 180 / M_PI);
    ret_theta.push_back(theta_list(0, theta_idx) * 180 / M_PI);
    __peak_magnitudes.push_back(__aoa_profile(phi_idx, theta_idx));
    __num_peaks_above_threshold+=1;
    float variance = get_profile_variance(phi_idx, theta_idx);
    __aoa_profile_variance.push_back(variance);

    if (__FLAG_debug)
        std::cout << "log-debug [find_top_N] profile variance " << peak_ind << " : " << variance << std::endl;

    // 2d matrix to label the searched index
    auto ind_profile = nc::zeros<bool>(__nphi, __ntheta);

    // Assign true to all neighbor region
    for (int i = -radius; i < radius; i++)
    {
        int tmp_row = 0, tmp_col = 0;
        if (phi_idx + i < 0)
            tmp_row = __nphi + i;
        else if (phi_idx + i >= __nphi)
            tmp_row = (phi_idx + i) % __nphi;
        else
            tmp_row = i + phi_idx;
        
        for (int j = -radius; j < radius; j++)
        {
            if (theta_idx + j < 0)
                tmp_col = __ntheta + j;
            else if (theta_idx + j >= __ntheta)
                tmp_col = (__ntheta + j) % __ntheta;
            else
                tmp_col = theta_idx + j;
            ind_profile(tmp_row, tmp_col) = true;
        }
    }

    //Iterate all entries in the 2d matrix
    for (int itr = 1; itr < all_idx_flat.size() && peak_ind < _topN_count; itr++)
    {

        //        phi_idx = sortedIdxs_phi(0,arr_idx_phi-i_phi);
        //        theta_idx = sortedIdxs_theta(0,arr_idx_theta-i_theta);
        phi_idx = sorted_inds(itr, 0);
        theta_idx = sorted_inds(itr, 1);
        
        //Don't square since the entire profile itself is already squared.
        //float relative_peak_magnitude = 100 * std::pow(__aoa_profile(phi_idx, theta_idx), 2) / std::pow(max_peak(0, 0), 2);
        
        float relative_peak_magnitude = 100 * __aoa_profile(phi_idx, theta_idx) / max_peak(0, 0);

        //Default threshold so that we always return a good number of peaks.
        if (relative_peak_magnitude >= 0.0000005)
            check_peak = true;
        else
            check_peak = false;
        
        
        for (int i = -radius; i < radius && check_peak; i++)
        {
            int tmp_row = 0, tmp_col = 0;
            if (phi_idx + i < 0)
                tmp_row = __nphi + i;
            else if (phi_idx + i >= __nphi)
                tmp_row = (phi_idx + i) % __nphi;
            else
                tmp_row = phi_idx + i;
            
            for (int j = -radius; j < radius && check_peak; j++)
            {
                if (theta_idx + j < 0)
                    tmp_col = __ntheta + j;
                else if (theta_idx + j >= __ntheta)
                    tmp_col = (__ntheta + j) % __ntheta;
                else
                    tmp_col = theta_idx + j;
                //if the nearby pos is checked before or value is greater than current peak value
                
                if (ind_profile(tmp_row, tmp_col) ||
                    __aoa_profile(tmp_col, tmp_row) > __aoa_profile(phi_idx, theta_idx))
                {
                    check_peak = false;
                    break;
                }
            }
        }
        //
        if (check_peak)
        {
            peak_ind++;
            if (__FLAG_debug)
            {
                std::cout << "log-debug [find_top_N] Top azimuth angle " << peak_ind
                            << " : " << phi_list(0, phi_idx) * 180 / M_PI << std::endl;
                std::cout << "log-debug [find_top_N] Top elevation angle " << peak_ind << " : "
                            << theta_list(0, theta_idx) * 180 / M_PI << std::endl;
                std::cout << "log-debug [find_top_N] Phi idx = " << phi_idx << ", theta idx = " << theta_idx << std::endl;
            }

            ret_phi.push_back(phi_list(0, phi_idx) * 180 / M_PI);
            ret_theta.push_back(theta_list(0, theta_idx) * 180 / M_PI);
            //            phi_indexes_stored(0,phi_idx) = 1;
            //            theta_indexes_stored(0,theta_idx) = 1;
            __peak_magnitudes.push_back(__aoa_profile(phi_idx, theta_idx));
            

            if (relative_peak_magnitude >= __relative_magnitude_threshold)
                __num_peaks_above_threshold+=1;

            for (int i = -radius; i < radius; i++)
            {
                int tmp_row = 0, tmp_col = 0;
                if (phi_idx + i < 0)
                    tmp_row = __nphi + i;
                else if (phi_idx + i >= __nphi)
                    tmp_row = (phi_idx + i) % __nphi;
                else
                    tmp_row = phi_idx + i;
                
                for (int j = -radius; j < radius; j++)
                {
                    if (theta_idx + j < 0)
                        tmp_col = __ntheta + j;
                    else if (theta_idx + j >= __ntheta)
                        tmp_col = (__ntheta + j) % __ntheta;
                    else
                        tmp_col = theta_idx + j;
                    
                    //if the nearby pos is checked before or value is greater than current peak value
                    // std::cout << "temp_row:" << phi_list(0, tmp_row) * 180 / M_PI << " tmp_col:" << theta_list(0, tmp_col) * 180 / M_PI << std::endl; 
                    ind_profile(tmp_row, tmp_col) = true;
                }
            }
        }
    }

    std::cout << "log-info  [find_top_N] Got top N angles from the AOA Profile" << std::endl;
    return std::make_pair(ret_phi, ret_theta);
}
//=============================================================================================================================
/**
 * Description:
 * Input:
 * Output:
 * */
std::pair<std::vector<double>, std::vector<double>> WSR_Module::find_topN_azimuth()
{

    std::vector<double> ret_phi, ret_theta;
    nc::NdArray<double> max_peak = nc::amax(__aoa_profile);
    nc::NdArray<double> phi_max = nc::amax(__aoa_profile, nc::Axis::COL);
    __aoa_profile_variance.clear();
    __peak_magnitudes.clear();
    __num_peaks_above_threshold = 0;
    int peak_ind = 1, phi_idx = 0, theta_idx = 0;
    bool check_peak = false;

    // radius to define local maxima as per the config file
    const int radius = __peak_radius;

    auto all_idx_flat = nc::flip(nc::argsort((__aoa_profile.flatten())));
    //n*2 array save coordinates for sorted value in the whole profile
    nc::NdArray<int> sorted_inds(all_idx_flat.size(), 2);
    for (int i = 0; i < all_idx_flat.size(); i++)
    {
        sorted_inds.put(i, sorted_inds.cSlice(), utils.unravel_index(all_idx_flat(0, i), __nphi, __ntheta));
    }
    phi_idx = sorted_inds(0, 0);
    theta_idx = sorted_inds(0, 1);

    //Get the peak AOA
    if (__FLAG_debug)
        std::cout << "log [calculate_AOA_profile] Top azimuth angle " << peak_ind
                    << " : " << phi_list(0, phi_idx) * 180 / M_PI << std::endl;
    ret_phi.push_back(phi_list(0, phi_idx) * 180 / M_PI);
    ret_theta.push_back(0 * 180 / M_PI);

    __peak_magnitudes.push_back(__aoa_profile(phi_idx, theta_idx));
    std::cout << "Phi idx = " << phi_idx << ", theta idx = " << theta_idx << std::endl;

    __num_peaks_above_threshold+=1;
    float variance = get_profile_variance(phi_idx, theta_idx);
    __aoa_profile_variance.push_back(variance);

    if (__FLAG_debug)
        std::cout << "log [calculate_AOA_profile] profile variance " << peak_ind << " : " << variance << std::endl;

    auto ind_profile = nc::zeros<bool>(__nphi, __ntheta);

    // Assign true to all neighbor region
    for (int i = -radius; i < radius; i++)
    {
        int tmp_row = 0, tmp_col = 0;
        if (phi_idx + i < 0)
            tmp_row = __nphi + i;
        else if (phi_idx + i >= __nphi)
            tmp_row = (phi_idx + i) % __nphi;
        else
            tmp_row = i + phi_idx;
        
        for (int j = 0; j <__ntheta ; j++)
        {
            ind_profile(tmp_row, j) = true;
        }
    }

    //Iterate all entries in the 2d matrix
    for (int itr = 1; itr < all_idx_flat.size() && peak_ind < _topN_count; itr++)
    {

        phi_idx = sorted_inds(itr, 0);
        theta_idx = 0;
        
        //Don't square since the entire profile itself is already squared.
        //float relative_peak_magnitude = 100 * std::pow(__aoa_profile(phi_idx, theta_idx), 2) / std::pow(max_peak(0, 0), 2);
        
        float relative_peak_magnitude = 100 * __aoa_profile(phi_idx, theta_idx) / max_peak(0, 0);

        //Default threshold so that we always return a good number of peaks.
        if (relative_peak_magnitude >= 0.0000005)
            check_peak = true;
        else
            check_peak = false;
        
        
        for (int i = -radius; i < radius && check_peak; i++)
        {
            int tmp_row = 0, tmp_col = 0;
            if (phi_idx + i < 0)
                tmp_row = __nphi + i;
            else if (phi_idx + i >= __nphi)
                tmp_row = (phi_idx + i) % __nphi;
            else
                tmp_row = phi_idx + i;
            
            for (int j = 0; j < __ntheta && check_peak; j++)
            {
                //if the nearby pos is checked before or value is greater than current peak value
                if (ind_profile(tmp_row, j) || __aoa_profile(tmp_row, 0) > __aoa_profile(phi_idx, 0))
                { 
                    check_peak = false;
                    break;
                }
            }
        }

        //
        if (check_peak)
        {
            peak_ind++;
            if (__FLAG_debug)
                std::cout << "log [calculate_AOA_profile] Top azimuth angle " << peak_ind
                            << " : " << phi_list(0, phi_idx) * 180 / M_PI << std::endl;
            ret_phi.push_back(phi_list(0, phi_idx) * 180 / M_PI);
            ret_theta.push_back(0 * 180 / M_PI);

            __peak_magnitudes.push_back(__aoa_profile(phi_idx, 0));
            std::cout << "Phi idx = " << phi_idx << ", theta idx = " << 0 << std::endl;

            if (relative_peak_magnitude >= __relative_magnitude_threshold)
                __num_peaks_above_threshold+=1;

            for (int i = -radius; i < radius; i++)
            {
                int tmp_row = 0, tmp_col = 0;
                if (phi_idx + i < 0)
                    tmp_row = __nphi + i;
                else if (phi_idx + i >= __nphi)
                    tmp_row = (phi_idx + i) % __nphi;
                else
                    tmp_row = phi_idx + i;
                
                for (int j = 0; j < __ntheta; j++)
                {
                  ind_profile(tmp_row, j) = true;
                }
            }
        }
    }

    return std::make_pair(ret_phi, ret_theta);
}
//======================================================================================================================
/**
 * Description: Get the confidence of the AOA profile by calculating its variance 
 * Input:
 * Output:
 * */
float WSR_Module::get_profile_variance(double phi_ind, double theta_ind)
{

    float sumf = __aoa_profile.sum()(0, 0);
    auto twod_profile = nc::sum(__aoa_profile, nc::Axis::COL);
    
    // std::cout << twod_profile.shape() << std::endl;
    float sigma_f = 0, sigma_n = 0, temp1 = 0, temp2;

    for (size_t ind_r = 0; ind_r < __nphi; ind_r++)
    {
        for (size_t ind_c = 0; ind_c < __ntheta; ind_c++)
        {
            temp1 = pow((WSR_Util::diff_360(ind_r, phi_ind)), 2);
            temp2 = pow((ind_c - theta_ind), 2);
            sigma_f += ((temp1 + temp2) * __aoa_profile(ind_r, ind_c) / sumf);
            sigma_n += ((temp1 + temp2) * sumf / (__ntheta * __nphi));
        }
    }

    return sigma_f / sigma_n;
}
//======================================================================================================================
/**
 * Description: Test the channel phase and CFO correction for two antenna SAR CSI data 
 * Input:
 * Output:
 * */
int WSR_Module::test_csi_data_conjugate(std::string rx_csi_file)
{
    std::cout << "log-info  [test_csi_data_conjugate (relative channel)] ============ Testing CSI data ==============" << std::endl;

    WIFI_Agent RX_SAR_robot; //Broardcasts the csi packets and does SAR
    nc::NdArray<std::complex<double>> h_list_all;
    nc::NdArray<std::complex<double>> h_list_static;
    nc::NdArray<std::complex<double>> h_list;
    nc::NdArray<double> csi_timestamp_all;
    nc::NdArray<double> csi_timestamp;
    double cal_ts_offset;
    std::string debug_dir = __precompute_config["debug_dir"]["value"].dump();
    debug_dir.erase(remove(debug_dir.begin(), debug_dir.end(), '\"'), debug_dir.end());
    int ret_val = 0;

    std::cout << "log-info  [test_csi_data_conjugate (relative channel)]: Parsing CSI Data " << std::endl;

    auto temp1 = utils.readCsiData(rx_csi_file, RX_SAR_robot, __FLAG_debug);

    std::vector<std::string> mac_id_tx;
    std::cout << "log-info [test_csi_data_conjugate (relative channel)]: Neighbouring TX robot IDs count = " << RX_SAR_robot.unique_mac_ids_packets.size() << std::endl;

    for (auto key : RX_SAR_robot.unique_mac_ids_packets)
    {
        if (__FLAG_debug)
            std::cout << "log-debug [test_csi_data_conjugate (relative channel)] " << key.first
                      << ", Packet count: = " << key.second << std::endl;
        mac_id_tx.push_back(key.first);
    }

    //Get AOA profile for each of the RX neighboring robots
    std::cout << "log-info  [test_csi_data_conjugate (relative channel)]: Getting AOA profiles" << std::endl;
    
    std::vector<DataPacket> data_packets_RX, data_packets_TX;

    for (int num_tx = 0; num_tx < mac_id_tx.size(); num_tx++)
    {
        WIFI_Agent TX_Neighbor_robot; // Neighbouring robots who reply back
        std::pair<nc::NdArray<std::complex<double>>, nc::NdArray<double>> csi_data;

        std::cout << "log-info  [test_csi_data_conjugate (relative channel)]: =========================" << std::endl;
        std::cout << "log-info  [test_csi_data_conjugate (relative channel)]: Profile for RX_SAR_robot MAC-ID: " << __RX_SAR_robot_MAC_ID
                  << ", TX_Neighbor_robot MAC-ID: " << mac_id_tx[num_tx] << std::endl;

        data_packets_RX = RX_SAR_robot.get_wifi_data(mac_id_tx[num_tx]);          //Packets for a TX_Neigbor_robot in RX_SAR_robot's csi file


        std::cout << "log-info  [test_csi_data_conjugate (relative channel)]: Packets for TX_Neighbor_robot collected by RX_SAR_robot : "
                    << data_packets_RX.size() << std::endl;

        std::cout << "log-info  [test_csi_data_conjugate (relative channel)]: Calculating Complex conjugate product" << std::endl;
        
        //Complex conjugate to correct for channel phase
        csi_data = utils.getConjugateProductChannel(data_packets_RX,
                                                    __FLAG_sub_sample,
                                                    __snum_start,
                                                    __snum_end);


        std::cout << "log-info  [test_csi_data_conjugate (relative channel)]: corrected CFO " << std::endl;
        h_list_all = csi_data.first;
        csi_timestamp_all = csi_data.second;

        if (h_list_all.shape().rows < __min_packets_to_process)
        {
            std::cout << "log-info  [test_csi_data_conjugate (relative channel)]: Not enough CSI packets." << std::endl;
            std::cout << "log-info  [test_csi_data_conjugate (relative channel)]: Return Empty AOA profile" << std::endl;
            //Return empty dummpy AOA profile
            __aoa_profile = nc::zeros<double>(1, 1);
        }
        else
        {
            std::cout << "log-info  [test_csi_data_conjugate (relative channel)]: CSI_packets_used = " << csi_timestamp_all.shape() << std::endl;
            std::cout << "log-info  [test_csi_data_conjugate (relative channel)]: h_list size  = " << h_list_all.shape() << std::endl;

            std::string debug_dir = __precompute_config["debug_dir"]["value"].dump();
            debug_dir.erase(remove(debug_dir.begin(), debug_dir.end(), '\"'), debug_dir.end());

            //Store phase and timestamp of the channel for debugging
            std::string channel_data_all = debug_dir + "/" + tx_name_list[mac_id_tx[num_tx]] + "_" + data_sample_ts[mac_id_tx[num_tx]] + "_all_channel_data.json";
            std::string channel_sub_15_csv = debug_dir + "/" + tx_name_list[mac_id_tx[num_tx]] + "_" + data_sample_ts[mac_id_tx[num_tx]] + "_channel_sub15_data.csv";

            std::cout << "log [test_csi_data (forward-reverse channel)]: Output file: " << channel_data_all << std::endl;
            utils.writeCSIToJsonFile(h_list_all, csi_timestamp_all, channel_data_all, __FLAG_interpolate_phase);
            utils.writeCSIToFile(h_list_all, csi_timestamp_all, channel_sub_15_csv);

            auto starttime = std::chrono::high_resolution_clock::now();
            auto endtime = std::chrono::high_resolution_clock::now();
            float processtime = std::chrono::duration<float, std::milli>(endtime - starttime).count();
            /*Interpolate the trajectory and csi data*/
            __paired_pkt_count[mac_id_tx[num_tx]] = csi_timestamp_all.shape().rows;
            __calculated_ts_offset[mac_id_tx[num_tx]] = cal_ts_offset;
            __rx_pkt_size[mac_id_tx[num_tx]] = data_packets_RX.size();
            __tx_pkt_size[mac_id_tx[num_tx]] = data_packets_TX.size();
            __perf_aoa_profile_cal_time[mac_id_tx[num_tx]] = processtime / 1000;
            // __channel_phase_diff_mean[mac_id_tx[num_tx]] = moving_channel_ang_diff_mean;
            // __channel_phase_diff_stdev[mac_id_tx[num_tx]] = moving_channel_ang_diff_stdev;
            __channel_data_output_file[mac_id_tx[num_tx]] = channel_data_all;

        }

        //TODO: get azimuth and elevation from beta_profile
        std::cout << "log-info  [test_csi_data_conjugate (relative channel)]: Completed Testing CSI data" << std::endl;
        TX_Neighbor_robot.reset();
    }

    std::cout << "log-info  [test_csi_data_conjugate (relative channel)]: ============ Test complete ==============" << std::endl;
    RX_SAR_robot.reset();

    return 0;
    
}

//=============================================================================================================================
/**
 *Calculates the AOA profiles for TX robots using only the CSI data received by the RX robot
 * It uses the complex conjugate of the CSI data and using the orientation of the robots.
 * */
int WSR_Module::calculate_AOA_using_csi_conjugate(std::string rx_csi_file,
                                        nc::NdArray<double> displacement,
                                        nc::NdArray<double> displacement_timestamp)
{
    std::cout << "============ Testing WSR module ==============" << std::endl;

    WIFI_Agent RX_SAR_robot; //Receives the broadcasted packets from the Neighboring robots
    nc::NdArray<std::complex<double>> h_list_all, h_list_static, h_list, csi_ant_1, csi_ant_2;
    nc::NdArray<double> csi_timestamp_all, csi_timestamp;
    double cal_ts_offset;
    int ret_val = 0;
    
    /**
     * ------------------------------------------------------------------------------------------
     * Parse CSI data collected on the receiver robot and extract the mac-ids of the transmitters.
     * ------------------------------------------------------------------------------------------
     * */
    std::cout << "log [calculate_AOA_using_csi_conjugate]: Parsing CSI Data " << std::endl;
    auto temp1 = utils.readCsiData(rx_csi_file, RX_SAR_robot, __FLAG_debug);
    std::vector<std::string> mac_id_tx;

    std::cout << "log [calculate_AOA_using_csi_conjugate]: Neighbouring TX robot IDs count = " << RX_SAR_robot.unique_mac_ids_packets.size() << std::endl;

    for (auto key : RX_SAR_robot.unique_mac_ids_packets)
    {
        if (__FLAG_debug)
            std::cout << "log [calculate_AOA_using_csi_conjugate]: Detected MAC ID = " << key.first
                      << ", Packet count: = " << key.second << std::endl;
        mac_id_tx.push_back(key.first);
    }

    /**
     * -----------------------------------------------------
     * Get AOA profile for each of the RX neighboring robots
     * -----------------------------------------------------
     * */
    if (__FLAG_debug) std::cout << "log [calculate_AOA_using_csi_conjugate]: Getting AOA profiles" << std::endl;
    std::vector<DataPacket> data_packets_RX, data_packets_TX;
    for (int num_tx = 0; num_tx < mac_id_tx.size(); num_tx++)
    {
        WIFI_Agent TX_Neighbor_robot; // Neighbouring robots who reply back
        std::pair<nc::NdArray<std::complex<double>>, nc::NdArray<double>> csi_data;
        std::pair<nc::NdArray<std::complex<double>>, nc::NdArray<std::complex<double>>> raw_csi_data;

        std::cout << "log [calculate_AOA_using_csi_conjugate]: =========================" << std::endl;
        std::cout << "log [calculate_AOA_using_csi_conjugate]: Profile for RX_SAR_robot MAC-ID: " << __RX_SAR_robot_MAC_ID
                  << ", TX_Neighbor_robot MAC-ID: " << mac_id_tx[num_tx] << std::endl;

        data_packets_RX = RX_SAR_robot.get_wifi_data(mac_id_tx[num_tx]); //Packets for a TX_Neigbor_robot in RX_SAR_robot's csi file
        std::cout << "log [calculate_AOA_using_csi_conjugate]: Packets for TX_Neighbor_robot collected by RX_SAR_robot : "
                    << data_packets_RX.size() << std::endl;

        std::cout << "log [calculate_AOA_using_csi_conjugate]: Getting Raw CSI data" << std::endl;
        
        //Complex conjugate to correct for channel phase
        std::cout << "log [calculate_AOA_using_csi_conjugate]: Calculating Complex conjugate product" << std::endl;
        csi_data = utils.getConjugateProductChannel(data_packets_RX,
                                                    __FLAG_sub_sample,
                                                    __snum_start,
                                                    __snum_end);


        std::cout << "log [calculate_AOA_using_csi_conjugate]: corrected CFO " << std::endl;
        h_list_all = csi_data.first;
        csi_timestamp_all = csi_data.second;

        // raw_csi_data = utils.getRawCSIData(data_packets_RX);
        // std::string csi_antenna_1 = __debug_dir + "/" + tx_name_list[mac_id_tx[num_tx]] + "_" + data_sample_ts[mac_id_tx[num_tx]] + "_csi_antenna_1.csv";
        // std::cout << "log [calculate_AOA_using_csi_conjugate]: Saving Raw CSI data antenna 1" << std::endl;
        // csi_ant_1 = raw_csi_data.first;
        // utils.writeCSIToFile(csi_ant_1, csi_timestamp_all, csi_antenna_1);
        
        // std::cout << "log [calculate_AOA_using_csi_conjugate]: Saving Raw CSI data antenna 2" << std::endl;
        // std::string csi_antenna_2 = __debug_dir + "/" + tx_name_list[mac_id_tx[num_tx]] + "_" + data_sample_ts[mac_id_tx[num_tx]] + "_csi_antenna_2.csv";
        // csi_ant_2 = raw_csi_data.second;
        // utils.writeCSIToFile(csi_ant_2, csi_timestamp_all, csi_antenna_2);


        if (csi_timestamp_all.size() < __min_packets_to_process)
        {
            std::cout << csi_timestamp_all.size() << std::endl;
            if (__FLAG_debug)
                std::cout << "log [calculate_AOA_using_csi_conjugate]: Very few CSI data packets left after forward-backward product" << std::endl;
            break;
        }

        if (__FLAG_debug) std::cout << "log [calculate_AOA_using_csi_conjugate]: Removing unused csi data" << std::endl;
        std::pair<int, int> csi_timestamp_range = utils.returnClosestIndices(csi_timestamp_all, displacement_timestamp);
        int start_index = csi_timestamp_range.first, end_index = csi_timestamp_range.second;

        /*Slice the csi_timestamp using the indices to remove unused CSI values
        * Note: Make sure that the packet transmission freqency is high enough when random_packets is used, 
        * such that about 400 left after slicing even if the trajectory duration is small. 
        * */
        if (__FLAG_debug) std::cout << "log [calculate_AOA_using_csi_conjugate]: Slicing CSI timestamps" << std::endl;
        csi_timestamp = csi_timestamp_all({start_index, end_index}, csi_timestamp_all.cSlice());
        h_list = h_list_all({start_index, end_index}, h_list_all.cSlice());

        if (h_list.shape().rows < __min_packets_to_process)
        {
            std::cout << "log [calculate_AOA_using_csi_conjugate]: Not enough CSI packets." << std::endl;
            std::cout << "log [calculate_AOA_using_csi_conjugate]: Return Empty AOA profile" << std::endl;
            //Return empty dummpy AOA profile
            __aoa_profile = nc::zeros<double>(1, 1);
        }
        else
        {
            /*Interpolate the trajectory using the csi data timestamps*/
            if (__FLAG_debug) std::cout << "log [calculate_AOA_using_csi_conjugate]: interpolating the displacement and csi " << std::endl;
            
            std::cout << "Size of displacement cols:" << nc::shape(displacement) << std::endl;

            //Convert the angles from -PitoPi to 0toPI to enable correct interpolation
            for(int i=0; i<nc::shape(displacement).rows; i++)
            {
                // std::cout << displacement(i,0) <<"," << displacement(i,1) <<"," << displacement(i,3) << std::endl; 
                displacement(i,3) = utils.wrap0to2Pi(displacement(i,3));
            }
            
            auto interpolated_data = utils.interpolate(csi_timestamp, displacement_timestamp, displacement);
            nc::NdArray<double> pose_list = interpolated_data.first;
            std::cout << "log [calculate_AOA_using_csi_conjugate]: PoseList after interpolation  = " << pose_list.shape() << std::endl;
            
            if(__FLAG_slice)
            {
                int begin =0, last = 0;
                if (__FLAG_slice_first)
                {
                    begin = 0, last=pose_list.shape().rows*1/3;
                }
                else if (__FLAG_slice_second)
                {
                    begin = pose_list.shape().rows*1/3 + 1, last=pose_list.shape().rows*2/3;
                }
                else
                {
                    begin = pose_list.shape().rows*2/3 + 1, last=pose_list.shape().rows;
                }

                auto h_list_slicing = h_list({begin, last}, h_list.cSlice());
                auto pose_list_slicing = pose_list({begin, last}, pose_list.cSlice());
                auto csi_timestamp_slicing = csi_timestamp({begin, last}, csi_timestamp.cSlice());

                h_list = h_list_slicing;
                pose_list = pose_list_slicing;
                csi_timestamp = csi_timestamp_slicing;
            }

            if (__FLAG_debug)
            {
                std::cout << "log [calculate_AOA_using_csi_conjugate]: CSI_packets_used = " << csi_timestamp.shape() << std::endl;
                std::cout << "log [calculate_AOA_using_csi_conjugate]: pose_list size  = " << pose_list.shape() << std::endl;
                std::cout << "log [calculate_AOA_using_csi_conjugate]: h_list size  = " << h_list.shape() << std::endl;

                //Store phase and timestamp of the channel for debugging
                std::string channel_data_sliced = __debug_dir + "/" + tx_name_list[mac_id_tx[num_tx]] + "_" + data_sample_ts[mac_id_tx[num_tx]] + "_sliced_channel_data.json";
                utils.writeCSIToJsonFile(h_list, csi_timestamp, channel_data_sliced, __FLAG_interpolate_phase);

                std::string channel_data_all = __debug_dir + "/" + tx_name_list[mac_id_tx[num_tx]] + "_" + data_sample_ts[mac_id_tx[num_tx]] + "_all_channel_data.json";
                utils.writeCSIToJsonFile(h_list_all, csi_timestamp_all, channel_data_all, __FLAG_interpolate_phase);

                //Store the packet distribution to check for spotty packets
                std::string packet_dist = __debug_dir + "/" + tx_name_list[mac_id_tx[num_tx]] + "_" + data_sample_ts[mac_id_tx[num_tx]] + "_packet_dist.json";
                utils.writePacketDistributionToJsonFile(csi_timestamp, displacement_timestamp, displacement, packet_dist);

                //Store interpolated trajectory for debugging
                std::string interpl_trajectory = __debug_dir + "/" + tx_name_list[mac_id_tx[num_tx]] + "_" + data_sample_ts[mac_id_tx[num_tx]] + "_interpl_trajectory.json";
                utils.writeTrajToFile(pose_list, interpl_trajectory);
            }

            /*Interpolate the trajectory and csi data*/
            std::cout << "log [calculate_AOA_profile]: Calculating AOA profile..." << std::endl;
            auto starttime = std::chrono::high_resolution_clock::now();

            __aoa_profile = compute_conjugate_profile_bartlett_multithread(h_list, pose_list);
            // __aoa_profile = compute_conjuate_profile_music_offboard(h_list, pose_list);

            auto endtime = std::chrono::high_resolution_clock::now();
            float processtime = std::chrono::duration<float, std::milli>(endtime - starttime).count();
            
            //Stats
            std::pair<std::vector<double>, std::vector<double>> top_N = find_topN_azimuth();
            __TX_top_N_angles[mac_id_tx[num_tx]] = top_N;
            __paired_pkt_count[mac_id_tx[num_tx]] = csi_timestamp.shape().rows;
            __perf_aoa_profile_cal_time[mac_id_tx[num_tx]] = processtime / 1000;
            __memory_used[mac_id_tx[num_tx]] = utils.mem_usage() / 1000000;
            __calculated_ts_offset[mac_id_tx[num_tx]] = cal_ts_offset;
            __rx_pkt_size[mac_id_tx[num_tx]] = data_packets_RX.size();
            __tx_pkt_size[mac_id_tx[num_tx]] = data_packets_TX.size();
            __top_peak_confidence[mac_id_tx[num_tx]] = __aoa_profile_variance[0];
            __all_topN_magnitudes[mac_id_tx[num_tx]] = __peak_magnitudes;
            __all_topN_above_threshold[mac_id_tx[num_tx]] = __num_peaks_above_threshold;
        }

        /*Store the aoa_profile*/
        __all_aoa_profiles[mac_id_tx[num_tx]] = __aoa_profile;
        __all_topN_confidence[mac_id_tx[num_tx]] = __aoa_profile_variance;

        std::cout << "log [test_csi_data]: Completed Testing CSI data" << std::endl;
    }

    std::cout << "============ Test complete ==============" << std::endl;
    RX_SAR_robot.reset();

    return 0;
    
}


/**=======================================================================================================
 * Description: Uses the complex conjugate steering vector formulation for SAR to compute AOA profile
 * Input: Relative channel and robot displacement
 * Output: AOA profile
 * */
//=======================================================================================================
nc::NdArray<double> WSR_Module::compute_conjugate_profile_bartlett_multithread(
                                const nc::NdArray<std::complex<double>> &input_h_list,
                                const nc::NdArray<double> &input_pose_list)
{
    auto total_start  = std::chrono::high_resolution_clock::now();
    EigencdMatrix eigen_eterm_3DAdjustment, e_term_prod;
    EigenDoubleMatrix eigen_rep_lambda, eigen_rep_phi, eigen_rep_theta, diff_phi_yaw, eigen_rep_yaw, eigen_rep_pitch, eigen_rep_rho;

    int total_packets = input_h_list.shape().rows;
    if (total_packets != input_pose_list.shape().rows)
    {
        THROW_INVALID_ARGUMENT_ERROR("number of CSI and poses are different.\n");
    }

    int max_packets = total_packets;
    /*Use max packets*/
    if (__FLAG_packet_threshold)
    {
        max_packets = input_h_list.shape().rows > __max_packets_to_process ? __max_packets_to_process : input_h_list.shape().rows;
    }

    nc::NdArray<std::complex<double>> h_list = input_h_list(nc::Slice(0, max_packets), input_h_list.cSlice());
    nc::NdArray<double> pose_list = input_pose_list(nc::Slice(0, max_packets), input_pose_list.cSlice());

    if (__FLAG_debug)
        std::cout << "log [compute_conjugate_profile_bartlett_multithread] Total packets: " << total_packets << ", Max packets used: " << max_packets << std::endl;

    std::cout.precision(15);
    auto num_poses = nc::shape(pose_list).rows;

    if (__FLAG_debug)
        std::cout << "log [compute_conjugate_profile_bartlett_multithread] : get lambda and phi" << std::endl;
    
    auto start = std::chrono::high_resolution_clock::now();
    if (__FLAG_debug)
        std::cout << "log [compute_conjugate_profile_bartlett_multithread] : get yaw, pitch and rho values" << std::endl;

    nc::NdArray<double> orientation_list = pose_list(pose_list.rSlice(), 3);
    // std::cout << orientation_list*(180/M_PI) << std::endl;

    //Wrap the angles again. The -ve sign for exp flips the angles
    // std::cout << "------------------------------------------" << std::endl;
    orientation_list = nc::angle(-nc::exp(nc::multiply(orientation_list, std::complex<double>(0, 1))));
    // std::cout << orientation_list*(180/M_PI) << std::endl;

    EigenDoubleMatrix eigen_yaw_list = EigenDoubleMatrixMap(orientation_list.data(),
                                                                orientation_list.numRows(),
                                                                orientation_list.numCols());

    auto end = std::chrono::high_resolution_clock::now();
    std::cout << " Time elapsed for repmat operation: " << (end - start) / std::chrono::milliseconds(1) << std::endl;

    if (__FLAG_debug)
        std::cout << "log [compute_conjugate_profile_bartlett_multithread] : Element-wise sin and cos for theta,pitch" << std::endl;
 

    //===========Openmp implementation 0.2 sec faster =============.
    start = std::chrono::high_resolution_clock::now();
    std::cout << "Computing e_term_prod...." << std::endl;
    EigenDoubleMatrix e_term(__nphi * __ntheta, num_poses);
    get_bterm_all_subcarrier_conjugate(std::ref(e_term), std::ref(eigen_yaw_list));
    end = std::chrono::high_resolution_clock::now();
    std::cout << " Time elapsed for eterm_prod and eterm_exp:  " << (end - start) / std::chrono::milliseconds(1) << std::endl;
    //===========Openmp implementation=============.
    
    EigenDoubleMatrix eigen_betaProfile_final;
    bool first = true;
    for(int h_i=__snum_start; h_i<=__snum_end; h_i++)
    {
        std::cout << "Subcarrier : " << h_i << std::endl;
        double carrierfreq = (5000 + double(__precompute_config["channel"]["value"]) * 5) * 1e6 +
                            (15.5 - h_i) * 20e6 / 30;
        double lambda_inv =  carrierfreq/double(__precompute_config["c"]["value"]);
        EigencdMatrix temp1 = e_term * (2.0 * std::complex<double>(0, 1) * M_PI * lambda_inv);
        EigencdMatrix e_term_exp(__nphi * __ntheta, num_poses);
        getExponential(e_term_exp, temp1);
        
        nc::NdArray<std::complex<double>> h_list_single_channel;
        h_list_single_channel = h_list(h_list.rSlice(), h_i);
        
        if (h_list_single_channel.shape().cols == 0)
        {
            THROW_INVALID_ARGUMENT_ERROR("No subcarrier selected for CSI data.\n");
        }

        if (__FLAG_debug)
            std::cout << "log [compute_conjugate_profile_bartlett_multithread] : getting profile using matmul" << std::endl;
        
        auto h_list_eigen = EigencdMatrixMap(h_list_single_channel.data(), h_list_single_channel.numRows(), h_list_single_channel.numCols());
        std::cout << "rows = " << h_list_eigen.rows() << ",  cols = " << h_list_eigen.cols() << std::endl;
        std::cout << "rows = " << e_term_exp.rows() << ",  cols = " << e_term_exp.cols() << std::endl;

        EigenDoubleMatrix eigen_betaProfileProd = (e_term_exp * h_list_eigen).cwiseAbs2();    
        EigenDoubleMatrixMap eigen_betaProfile(eigen_betaProfileProd.data(),__ntheta, __nphi);
        std::cout << "beta profile rows = " << eigen_betaProfile.rows() << ",  beta profile cols = " << eigen_betaProfile.cols() << std::endl;

        if(first)
        {
            eigen_betaProfile_final = eigen_betaProfile;
            first = false;
        }
        else
        {
            eigen_betaProfile_final = eigen_betaProfile_final.cwiseProduct(eigen_betaProfile);
        }
    }

    double *cddataPtr2 = new double[eigen_betaProfile_final.rows() * eigen_betaProfile_final.cols()];
    EigenDoubleMatrixMap(cddataPtr2, eigen_betaProfile_final.rows(), eigen_betaProfile_final.cols()) = eigen_betaProfile_final;
    auto beta_profile = nc::NdArray<double>(cddataPtr2, eigen_betaProfile_final.rows(), eigen_betaProfile_final.cols(), __takeOwnership);

       
    if (__FLAG_normalize_profile)
    {
        auto sum_val = nc::sum(nc::sum(beta_profile));
        beta_profile = beta_profile / sum_val(0, 0);
    }

    if (__FLAG_debug)
        std::cout << "log [compute_conjugate_profile_bartlett_multithread] : Getting transpose and returning profile" << std::endl;
    
    beta_profile = nc::transpose((beta_profile));
    auto total_end = std::chrono::high_resolution_clock::now();
    std::cout << "log [compute_conjugate_profile_bartlett_multithread] Computation spent " << (total_end- total_start)/std::chrono::milliseconds(1)<< " ms"<< std::endl;

    return beta_profile;
}

//=============================================================================================================================
/**
 * Description: 
 * Input:
 * Output:
 */
void WSR_Module::get_two_antenna_bterm_all(EigencdMatrix &e_term_exp,
                                           EigenDoubleMatrix &eigen_yaw_list)
{

    std::cout << "Antenna Separation = " << __antenna_separation << std::endl;
    int i = 0;
    int j = 0;
#pragma omp parallel for shared(e_term_exp,eigen_yaw_list) private(i, j) collapse(2)
    for (i = 0; i < e_term_exp.rows(); i++)
    {
        for (j = 0; j < e_term_exp.cols(); j++)
        {                                                                                                      //Trig identity simplifies and also cancels out the -ve sign in -2j
            e_term_exp(i, j) = exp(__antenna_separation*cos(__eigen_precomp_rep_phi(i, 0)-eigen_yaw_list(j,0))*cos(__eigen_precomp_rep_theta(i, 0))*(2.0 * std::complex<double>(0, 1) * M_PI / __lambda));
            //e_term_exp(i, j) = exp((2.0 * std::complex<double>(0, 1) * M_PI * double(__antenna_separation) / __lambda)*cos(__eigen_precomp_rep_phi(i, 0)-eigen_yaw_list(j,0)));
        }
    }
}

//=============================================================================================================================
/**
 *
 *
 * */
nc::NdArray<double> WSR_Module::compute_conjuate_profile_music_offboard(
                                const nc::NdArray<std::complex<double>> &input_h_list,
                                const nc::NdArray<double> &input_pose_list)
{
    auto total_start  = std::chrono::high_resolution_clock::now();
    EigencdMatrix eigen_eterm_3DAdjustment, e_term_prod;
    EigenDoubleMatrix eigen_rep_lambda, eigen_rep_phi, eigen_rep_theta, diff_phi_yaw, eigen_rep_yaw, eigen_rep_pitch, eigen_rep_rho;
    WSR_Util util_obj;

    int total_packets = input_h_list.shape().rows;
    if (total_packets != input_pose_list.shape().rows)
    {
        THROW_INVALID_ARGUMENT_ERROR("number of CSI and poses are different.\n");
    }

    int max_packets = total_packets;
    /*Use max packets*/
    if (__FLAG_packet_threshold)
    {
        max_packets = input_h_list.shape().rows > __max_packets_to_process ? __max_packets_to_process : input_h_list.shape().rows;
    }

    nc::NdArray<std::complex<double>> h_list = input_h_list(nc::Slice(0, max_packets), input_h_list.cSlice());
    nc::NdArray<double> pose_list = input_pose_list(nc::Slice(0, max_packets), input_pose_list.cSlice());
        auto num_poses = nc::shape(pose_list).rows;

    if (__FLAG_debug)
        std::cout << "log [compute_AOA] Total packets: " << total_packets << ", Max packets used: " << max_packets << std::endl;

    std::cout.precision(15);

    if (__FLAG_debug)
        std::cout << "log [compute_AOA] : get lambda, phi and theta values" << std::endl;

    
    auto start = std::chrono::high_resolution_clock::now();
    if (__FLAG_debug)
        std::cout << "log [compute_AOA] : get yaw, pitch and rho values" << std::endl;
    auto pose_x = pose_list(pose_list.rSlice(), 0);
    auto pose_y = pose_list(pose_list.rSlice(), 1);
    auto pose_z = pose_list(pose_list.rSlice(), 2);
    auto yaw_list = pose_list(pose_list.rSlice(), 3);

    //std::cout << yaw_list*(180/M_PI) << std::endl;

    //This flips the angles
    // yaw_list = nc::angle(-nc::exp(nc::multiply(yaw_list, std::complex<double>(0, 1))));

    EigenDoubleMatrix eigen_yaw_list = EigenDoubleMatrixMap(yaw_list.data(),
                                                                yaw_list.numRows(),
                                                                yaw_list.numCols());
    
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << " Time elapsed for repmat operation: " << (end - start) / std::chrono::milliseconds(1) << std::endl;

    if (__FLAG_debug)
        std::cout << "log [compute_AOA] : Element-wise sin and cos for theta,pitch" << std::endl;
 

    start = std::chrono::high_resolution_clock::now();
    std::cout << "Computing e_term_prod...." << std::endl;
    EigenDoubleMatrix e_term(__nphi * __ntheta, num_poses);

    //===========Openmp implementation 0.2 sec faster =============.
    get_bterm_all_subcarrier_conjugate(std::ref(e_term), std::ref(eigen_yaw_list));
    std::cout << " Time elapsed for eterm_prod and eterm_exp:  " << (end - start) / std::chrono::milliseconds(1) << std::endl;
    //===========Openmp implementation=============.

    
    if (__FLAG_debug)
        std::cout << "log [compute_AOA] : getting profile using matmul" << std::endl;
    
    EigenDoubleMatrix eigen_betaProfile_final;
    bool first = true;
    for(int h_i=__snum_start; h_i<__snum_end; h_i++)
    {
        std::cout << "Subcarrier : " << h_i << std::endl;
        int d = std::rand();
        std::cout << d << std::endl;

        double carrierfreq = (5000 + double(__precompute_config["channel"]["value"]) * 5) * 1e6 +
                            (15.5 - h_i) * 20e6 / 30;
        double lambda_inv =  carrierfreq/double(__precompute_config["c"]["value"]);
        EigencdMatrix temp1 = e_term * (-2.0 * std::complex<double>(0, 1) * M_PI * lambda_inv);
        EigencdMatrix e_term_exp(__nphi * __ntheta, num_poses);
        getExponential(e_term_exp, temp1);
        
        nc::NdArray<std::complex<double>> h_list_single_channel;
        h_list_single_channel = h_list(h_list.rSlice(), h_i);
        
        if (h_list_single_channel.shape().cols == 0)
        {
            THROW_INVALID_ARGUMENT_ERROR("No subcarrier selected for CSI data.\n");
        }

        //Get complex conjugate of the channel  
        auto h_list_eigen = EigencdMatrixMap(h_list_single_channel.data(), h_list_single_channel.numRows(), h_list_single_channel.numCols());

        std::cout << "rows = " << h_list_eigen.rows() << ",  cols = " << h_list_eigen.cols() << std::endl;
        std::cout << "rows = " << e_term_exp.rows() << ",  cols = " << e_term_exp.cols() << std::endl;

        EigenDoubleMatrix eigen_betaProfileProd = (e_term_exp * h_list_eigen).cwiseAbs2();    
        EigenDoubleMatrixMap eigen_betaProfile(eigen_betaProfileProd.data(),__ntheta, __nphi);
        std::cout << "beta profile rows = " << eigen_betaProfile.rows() << ",  beta profile cols = " << eigen_betaProfile.cols() << std::endl;

        if(first)
        {
            eigen_betaProfile_final = eigen_betaProfile;
            first = false;
        }
        else
        {
            eigen_betaProfile_final = eigen_betaProfile_final.cwiseProduct(eigen_betaProfile);
        }
    }
    
    double *cddataPtr2 = new double[eigen_betaProfile_final.rows() * eigen_betaProfile_final.cols()];
    EigenDoubleMatrixMap(cddataPtr2, eigen_betaProfile_final.rows(), eigen_betaProfile_final.cols()) = eigen_betaProfile_final;
    auto beta_profile = nc::NdArray<double>(cddataPtr2, eigen_betaProfile_final.rows(), eigen_betaProfile_final.cols(), __takeOwnership);

    std::cout << beta_profile.shape() << std::endl;

    if (__FLAG_normalize_profile || (__snum_end -__snum_start > 1)) //Always normalize if using multiple subcarriers.
    {
        auto sum_val = nc::sum(nc::sum(beta_profile));
        beta_profile = beta_profile / sum_val(0, 0);
    }

    if (__FLAG_debug)
        std::cout << "log [compute_AOA] : Getting transpose and returning profile" << std::endl;

    beta_profile = nc::transpose((beta_profile));
    auto total_end = std::chrono::high_resolution_clock::now();
    std::cout << "Offboard Computation spent " << (total_end- total_start)/std::chrono::milliseconds(1)<< " ms"<< std::endl;

    return beta_profile;
}

//=============================================================================================================================
/**
 * Description: Does not include division by lambda in the exponential term; can be used for different subcarriers 
 * Input:
 * Output:
 */
void WSR_Module::get_bterm_all_subcarrier_conjugate(EigenDoubleMatrix &e_term,
                                                    EigenDoubleMatrix &eigen_yaw_list)
{
    std::cout << "Yaw list rows: " << eigen_yaw_list.rows() << std::endl;
    std::cout << "Yaw list cols: " << eigen_yaw_list.cols() << std::endl;
    int i = 0;
    int j = 0;
#pragma omp parallel for shared(e_term, eigen_yaw_list) private(i, j) collapse(2)
    for (i = 0; i < e_term.rows(); i++)
    {
        for (j = 0; j < e_term.cols(); j++)
        {                                                                                                      //Trig identity simplifies and also cancels out the -ve sign in -2j
            // e_term(i, j) = __antenna_separation*cos(__eigen_precomp_rep_phi(i, 0)-eigen_yaw_list(j,0))*cos(__eigen_precomp_rep_theta(i, 0));
            e_term(i, j) = __antenna_separation*cos(__eigen_precomp_rep_phi(i, 0)-eigen_yaw_list(j,0));
        }
    }
}

//=============================================================================================================================
/**
 *Calculates the AOA profiles for TX robots using only the CSI data received by the RX robot
 * Divide the trajectory into 3 parts
 * */
int WSR_Module::calculate_AOA_using_csi_conjugate_multiple(std::string rx_csi_file,
                                        nc::NdArray<double> displacement,
                                        nc::NdArray<double> displacement_timestamp)
{
    std::cout << "============ Testing WSR module ==============" << std::endl;

    WIFI_Agent RX_SAR_robot; //Receives the broadcasted packets from the Neighboring robots
    nc::NdArray<std::complex<double>> h_list_all, h_list_static, h_list, csi_ant_1, csi_ant_2;
    nc::NdArray<double> csi_timestamp_all, csi_timestamp;
    double cal_ts_offset;
    int ret_val = 0;
    
    /**
     * ------------------------------------------------------------------------------------------
     * Parse CSI data collected on the receiver robot and extract the mac-ids of the transmitters.
     * ------------------------------------------------------------------------------------------
     * */
    std::cout << "log [calculate_AOA_using_csi_conjugate]: Parsing CSI Data " << std::endl;
    auto temp1 = utils.readCsiData(rx_csi_file, RX_SAR_robot, __FLAG_debug);
    std::vector<std::string> mac_id_tx;

    std::cout << "log [calculate_AOA_using_csi_conjugate]: Neighbouring TX robot IDs count = " << RX_SAR_robot.unique_mac_ids_packets.size() << std::endl;

    for (auto key : RX_SAR_robot.unique_mac_ids_packets)
    {
        if (__FLAG_debug)
            std::cout << "log [calculate_AOA_using_csi_conjugate]: Detected MAC ID = " << key.first
                      << ", Packet count: = " << key.second << std::endl;
        mac_id_tx.push_back(key.first);
    }

    /**
     * -----------------------------------------------------
     * Get AOA profile for each of the RX neighboring robots
     * -----------------------------------------------------
     * */
    if (__FLAG_debug) std::cout << "log [calculate_AOA_using_csi_conjugate]: Getting AOA profiles" << std::endl;
    std::vector<DataPacket> data_packets_RX, data_packets_TX;
    for (int num_tx = 0; num_tx < mac_id_tx.size(); num_tx++)
    {
        WIFI_Agent TX_Neighbor_robot; // Neighbouring robots who reply back
        std::pair<nc::NdArray<std::complex<double>>, nc::NdArray<double>> csi_data;
        std::pair<nc::NdArray<std::complex<double>>, nc::NdArray<std::complex<double>>> raw_csi_data;

        std::cout << "log [calculate_AOA_using_csi_conjugate]: =========================" << std::endl;
        std::cout << "log [calculate_AOA_using_csi_conjugate]: Profile for RX_SAR_robot MAC-ID: " << __RX_SAR_robot_MAC_ID
                  << ", TX_Neighbor_robot MAC-ID: " << mac_id_tx[num_tx] << std::endl;

        data_packets_RX = RX_SAR_robot.get_wifi_data(mac_id_tx[num_tx]); //Packets for a TX_Neigbor_robot in RX_SAR_robot's csi file
        std::cout << "log [calculate_AOA_using_csi_conjugate]: Packets for TX_Neighbor_robot collected by RX_SAR_robot : "
                    << data_packets_RX.size() << std::endl;

        std::cout << "log [calculate_AOA_using_csi_conjugate]: Getting Raw CSI data" << std::endl;
        
        //Complex conjugate to correct for channel phase
        std::cout << "log [calculate_AOA_using_csi_conjugate]: Calculating Complex conjugate product" << std::endl;
        csi_data = utils.getConjugateProductChannel(data_packets_RX,
                                                    __FLAG_sub_sample,
                                                    __snum_start,
                                                    __snum_end);


        std::cout << "log [calculate_AOA_using_csi_conjugate]: corrected CFO " << std::endl;
        h_list_all = csi_data.first;
        csi_timestamp_all = csi_data.second;

        // raw_csi_data = utils.getRawCSIData(data_packets_RX);
        // std::string csi_antenna_1 = __debug_dir + "/" + tx_name_list[mac_id_tx[num_tx]] + "_" + data_sample_ts[mac_id_tx[num_tx]] + "_csi_antenna_1.csv";
        // std::cout << "log [calculate_AOA_using_csi_conjugate]: Saving Raw CSI data antenna 1" << std::endl;
        // csi_ant_1 = raw_csi_data.first;
        // utils.writeCSIToFile(csi_ant_1, csi_timestamp_all, csi_antenna_1);
        
        // std::cout << "log [calculate_AOA_using_csi_conjugate]: Saving Raw CSI data antenna 2" << std::endl;
        // std::string csi_antenna_2 = __debug_dir + "/" + tx_name_list[mac_id_tx[num_tx]] + "_" + data_sample_ts[mac_id_tx[num_tx]] + "_csi_antenna_2.csv";
        // csi_ant_2 = raw_csi_data.second;
        // utils.writeCSIToFile(csi_ant_2, csi_timestamp_all, csi_antenna_2);


        if (csi_timestamp_all.size() < __min_packets_to_process)
        {
            std::cout << csi_timestamp_all.size() << std::endl;
            if (__FLAG_debug)
                std::cout << "log [calculate_AOA_using_csi_conjugate]: Very few CSI data packets left after forward-backward product" << std::endl;
            break;
        }

        if (__FLAG_debug) std::cout << "log [calculate_AOA_using_csi_conjugate]: Removing unused csi data" << std::endl;
        std::pair<int, int> csi_timestamp_range = utils.returnClosestIndices(csi_timestamp_all, displacement_timestamp);
        int start_index = csi_timestamp_range.first, end_index = csi_timestamp_range.second;

        /*Slice the csi_timestamp using the indices to remove unused CSI values
        * Note: Make sure that the packet transmission freqency is high enough when random_packets is used, 
        * such that about 400 left after slicing even if the trajectory duration is small. 
        * */
        if (__FLAG_debug) std::cout << "log [calculate_AOA_using_csi_conjugate]: Slicing CSI timestamps" << std::endl;
        csi_timestamp = csi_timestamp_all({start_index, end_index}, csi_timestamp_all.cSlice());
        h_list = h_list_all({start_index, end_index}, h_list_all.cSlice());

        if (h_list.shape().rows < __min_packets_to_process)
        {
            std::cout << "log [calculate_AOA_using_csi_conjugate]: Not enough CSI packets." << std::endl;
            std::cout << "log [calculate_AOA_using_csi_conjugate]: Return Empty AOA profile" << std::endl;
            //Return empty dummpy AOA profile
            __aoa_profile = nc::zeros<double>(1, 1);
        }
        else
        {
            /*Interpolate the trajectory using the csi data timestamps*/
            if (__FLAG_debug) std::cout << "log [calculate_AOA_using_csi_conjugate]: interpolating the displacement and csi " << std::endl;
            
            std::cout << "Size of displacement cols:" << nc::shape(displacement) << std::endl;

            //Convert the angles from -PitoPi to 0toPI to enable correct interpolation
            for(int i=0; i<nc::shape(displacement).rows; i++)
            {
                // std::cout << displacement(i,0) <<"," << displacement(i,1) <<"," << displacement(i,3) << std::endl; 
                displacement(i,3) = utils.wrap0to2Pi(displacement(i,3));
            }
            
            auto interpolated_data = utils.interpolate(csi_timestamp, displacement_timestamp, displacement);
            nc::NdArray<double> pose_list = interpolated_data.first;
            std::cout << "log [calculate_AOA_using_csi_conjugate]: PoseList after interpolation  = " << pose_list.shape() << std::endl;
            
            int begin =0, last = 0;
            std::vector<double> all_topN_azimuth_peaks, dummy_elevation_peaks;
            float processtime;
            
            for(int disp_slice = 0; disp_slice < 3; disp_slice++)
            {
                std::cout << "Generating Profile for " << disp_slice << std::endl;
                if (disp_slice==0)
                {
                    begin = 0, last=pose_list.shape().rows*1/3;
                }
                else if (disp_slice==1)
                {
                    begin = pose_list.shape().rows*1/3 + 1, last=pose_list.shape().rows*2/3;
                }
                else
                {
                    begin = pose_list.shape().rows*2/3 + 1, last=pose_list.shape().rows;
                }

                auto h_list_slicing = h_list({begin, last}, h_list.cSlice());
                auto pose_list_slicing = pose_list({begin, last}, pose_list.cSlice());
                auto csi_timestamp_slicing = csi_timestamp({begin, last}, csi_timestamp.cSlice());

                if (__FLAG_debug)
                {
                    std::cout << "log [calculate_AOA_using_csi_conjugate]: CSI_packets_used = " << csi_timestamp_slicing.shape() << std::endl;
                    std::cout << "log [calculate_AOA_using_csi_conjugate]: pose_list size  = " << pose_list_slicing.shape() << std::endl;
                    std::cout << "log [calculate_AOA_using_csi_conjugate]: h_list size  = " << h_list_slicing.shape() << std::endl;

                    //Store phase and timestamp of the channel for debugging
                    std::string channel_data_sliced = __debug_dir + "/" + tx_name_list[mac_id_tx[num_tx]] + "_" + data_sample_ts[mac_id_tx[num_tx]] + "_sliced_channel_data.json";
                    utils.writeCSIToJsonFile(h_list_slicing, csi_timestamp_slicing, channel_data_sliced, __FLAG_interpolate_phase);

                    std::string channel_data_all = __debug_dir + "/" + tx_name_list[mac_id_tx[num_tx]] + "_" + data_sample_ts[mac_id_tx[num_tx]] + "_all_channel_data.json";
                    utils.writeCSIToJsonFile(h_list_all, csi_timestamp_all, channel_data_all, __FLAG_interpolate_phase);

                    //Store the packet distribution to check for spotty packets
                    std::string packet_dist = __debug_dir + "/" + tx_name_list[mac_id_tx[num_tx]] + "_" + data_sample_ts[mac_id_tx[num_tx]] + "_packet_dist.json";
                    utils.writePacketDistributionToJsonFile(csi_timestamp_slicing, displacement_timestamp, displacement, packet_dist);

                    //Store interpolated trajectory for debugging
                    std::string interpl_trajectory = __debug_dir + "/" + tx_name_list[mac_id_tx[num_tx]] + "_" + data_sample_ts[mac_id_tx[num_tx]] + "_interpl_trajectory.json";
                    utils.writeTrajToFile(pose_list_slicing, interpl_trajectory);
                }

                /*Interpolate the trajectory and csi data*/
                std::cout << "log [calculate_AOA_profile]: Calculating AOA profile..." << std::endl;
                auto starttime = std::chrono::high_resolution_clock::now();

                __aoa_profile = compute_conjugate_profile_bartlett_multithread(h_list_slicing, pose_list_slicing);
                // __aoa_profile = compute_conjuate_profile_music_offboard(h_list, pose_list);

                auto endtime = std::chrono::high_resolution_clock::now();
                processtime = std::chrono::duration<float, std::milli>(endtime - starttime).count();
                
                //Stats
                std::pair<std::vector<double>, std::vector<double>> top_N_temp = find_topN_azimuth();
                auto temp_azi_angles = top_N_temp.first;
                
                //Collect top_N_angles across sub-profiles
                std::cout << "Peaks above threshold = " << __num_peaks_above_threshold << std::endl;
                for(int az_itr=0;az_itr<__num_peaks_above_threshold;az_itr++)
                {
                    all_topN_azimuth_peaks.push_back(temp_azi_angles[az_itr]);
                    dummy_elevation_peaks.push_back(0);
                }
                
            }

            std::cout << "Total Peaks above threshold = " << all_topN_azimuth_peaks.size() << std::endl;
            __TX_top_N_angles[mac_id_tx[num_tx]] = std::make_pair(all_topN_azimuth_peaks,dummy_elevation_peaks);
            __paired_pkt_count[mac_id_tx[num_tx]] = csi_timestamp.shape().rows;
            __perf_aoa_profile_cal_time[mac_id_tx[num_tx]] = processtime / 1000;
            __memory_used[mac_id_tx[num_tx]] = utils.mem_usage() / 1000000;
            __calculated_ts_offset[mac_id_tx[num_tx]] = cal_ts_offset;
            __rx_pkt_size[mac_id_tx[num_tx]] = data_packets_RX.size();
            __tx_pkt_size[mac_id_tx[num_tx]] = data_packets_TX.size();
            __top_peak_confidence[mac_id_tx[num_tx]] = __aoa_profile_variance[0];
            __all_topN_magnitudes[mac_id_tx[num_tx]] = __peak_magnitudes;
            __all_topN_above_threshold[mac_id_tx[num_tx]] = all_topN_azimuth_peaks.size();
            __num_peaks_above_threshold = all_topN_azimuth_peaks.size();

            /*Store the aoa_profile*/
            __all_aoa_profiles[mac_id_tx[num_tx]] = __aoa_profile;
            __all_topN_confidence[mac_id_tx[num_tx]] = __aoa_profile_variance;

            std::cout << "log [test_csi_data]: Completed Testing CSI data" << std::endl;
        }
    }

    std::cout << "============ Test complete ==============" << std::endl;
    RX_SAR_robot.reset();

    return 0;
    
}
//=============================================================================================================================
/**
 *
 *
 * */
std::string WSR_Module::get_channel_data_output_filename(const std::string &tx_mac_id)
{
    return __channel_data_output_file[tx_mac_id];
}

//=============================================================================================================================
/**
 *
 *
 * */
bool WSR_Module::get__FLAG_two_antenna()
{
    return __FLAG_two_antenna;
}
//=============================================================================================================================
/**
 * Description: Returns all the AOA profiles calculated for multiple RX
 * Input:
 * Output:
 * */
std::unordered_map<std::string, nc::NdArray<double>> WSR_Module::get_all_aoa_profile()
{
    return __all_aoa_profiles;
}
//=============================================================================================================================
/**
 * Description: Returns all the AOA profiles calculated for multiple RX
 * Input:
 * Output:
 * */
std::unordered_map<std::string, std::vector<double>> WSR_Module::get_all_confidence()
{
    return __all_topN_confidence;
}
/**
 * Description: Returns all the AOA profiles calculated for multiple RX
 * Input:
 * Output:
 * */
std::unordered_map<std::string, std::pair<std::vector<double>, std::vector<double>>> WSR_Module::get_TX_topN_angles()
{
    return __TX_top_N_angles;
}

//=============================================================================================================================
/**
 * Description: 
 * Input:
 * Output:
 * */
void WSR_Module::get_block_exp(EigencdMatrix &output,
                               EigencdMatrix &input,
                               int start, int end, int rows, int cols)
{
    output = input.block(start, end, rows, cols).array().exp();
}
//=============================================================================================================================
/**
 * Description: 
 * Input:
 * Output:
 * */
void WSR_Module::get_eterm_3DAdjustment(EigencdMatrix &output,
                                        EigenDoubleMatrix &input)
{
    output = -4.0 * std::complex<double>(0, 1) * M_PI / input.array();
}
//=============================================================================================================================
/**
 * Description: 
 * Input:
 * Output:
 * */
void WSR_Module::get_matrix_block(EigenDoubleMatrix &output,
                                  EigenDoubleMatrix &input,
                                  int rows, int cols)
{
    output = input.block(0, 0, rows, cols);
}
//=============================================================================================================================
/**
 * Description: 
 * Input:
 * Output:
 * */
void WSR_Module::get_cwiseProduct(EigenDoubleMatrix &output,
                                  EigenDoubleMatrix &input1,
                                  EigenDoubleMatrix &input2)
{
    output = input1.cwiseProduct(input2);
}
//======================================================================================================================
/**
 * Description: 
 * Input:
 * Output:
 * */
void WSR_Module::get_cwiseProduct_openmp(EigenDoubleMatrix &output,
                                         EigenDoubleMatrix &input1,
                                         EigenDoubleMatrix &input2)
{
    int i, j;
    output = EigenDoubleMatrix::Zero(input1.rows(), input1.cols());
#pragma omp parallel for shared(input1, input2, output) private(i, j) collapse(2)
    for (i = 0; i < input1.rows(); i++)
        for (j = 0; j < input1.cols(); j++)
        {
            output(i, j) = input1(i, j) * input2(i, j);
            //   printf("Thread %d works on elemets %d", omp_get_thread_num(),j);
        }
}
//=============================================================================================================================
/**
 * Description: 
 * Input:
 * Output:
 * */
void WSR_Module::get_cwiseProduct_cd(EigencdMatrix &output,
                                     EigenDoubleMatrix &input1,
                                     EigencdMatrix &input2)
{
    // output = input1.cwiseProduct(input2);
    output = input1 * (-4.0 * std::complex<double>(0, 1) * M_PI / __lambda);
}
//=============================================================================================================================
/**
 * Description: 
 * Input:
 * Output:
 * */
void WSR_Module::get_repmat(EigenDoubleMatrix &output,
                            EigenDoubleMatrix &input,
                            int rows, int cols)
{
    output = input.replicate(rows, cols);
}

//=============================================================================================================================
/**
 * Description: 
 * Input:
 * Output:
 * */
void WSR_Module::get_eigen_rep_angle_trig(EigenDoubleMatrix &output,
                                          EigenDoubleMatrix &input,
                                          std::string trig_operation)
{
    if (trig_operation == "sin")
        output = input.array().sin();
    else
        output = input.array().cos();
}
//=============================================================================================================================
/**
 * Description: 
 * Input:
 * Output:
 * */
void WSR_Module::get_eigen_rep_angle_trig_openmp(EigenDoubleMatrix &output,
                                                 EigenDoubleMatrix &input,
                                                 std::string trig_operation)
{
    //   int max_threads = 64;
    int j;
    // int n_per = input.rows()/max_threads;
    if (trig_operation == "sin")
    {
#pragma omp parallel for shared(input, output) private(j) collapse(2)
        for (j = 0; j < input.rows(); j++)
        {
            for (int k = 0; k < input.cols(); k++)
            {
                output(j, k) = sin(input(j, k));
                //   printf("Thread %d works on elemets %d", omp_get_thread_num(),j);
            }
        }
    }
    else
    {
#pragma omp parallel for shared(input, output) private(j) collapse(2)
        for (j = 0; j < input.rows(); j++)
        {
            for (int k = 0; k < input.cols(); k++)
            {
                output(j, k) = cos(input(j, k));
            }
        }
    }
}


//=============================================================================================================================
/**
 *
 *
 * */
std::vector<std::vector<float>> WSR_Module::get_aoa_error(const std::pair<std::vector<double>, std::vector<double>> &topN_AOA,
                                                          std::pair<double, double> groundtruth_angles,
                                                          const string &traj_type)
{
    std::vector<std::pair<float, float>> true_aoa_angles;
    std::vector<double> topN_phi = topN_AOA.first;
    std::vector<double> topN_theta = topN_AOA.second;
    std::vector<std::vector<float>> aoa_error_metrics;

    float true_phi = groundtruth_angles.first;
    float true_theta = groundtruth_angles.second;
    float min_aoa_error = 1000, err = 0;
    float min_phi_error = 0, min_theta_error = 0;
    float closest_phi = 0, closest_theta = 0;

    for (int i = 0; i < topN_phi.size(); i++)
    {
        std::vector<float> temp;
        float phi_error = 0, theta_error = 0, theta_angle = 0;

        phi_error = utils.anglediff(true_phi, topN_phi[i]);

        //Squared error (as per WSR paper)
        theta_angle = topN_theta[i];
        theta_error = utils.anglediff(true_theta, theta_angle);
        err = std::sqrt(std::pow(phi_error, 2) + std::pow(theta_error, 2));


        temp.push_back(topN_phi[i]);
        temp.push_back(theta_angle);
        temp.push_back(err); //Total Error
        temp.push_back(phi_error);
        temp.push_back(theta_error);

        aoa_error_metrics.push_back(temp);
        temp.clear();
    }

    return aoa_error_metrics;
}
//=============================================================================================================================
/**
 *@Deprecated
 *
 * */
std::vector<double> WSR_Module::top_aoa_error(double phi, double theta,
                                              std::pair<double, double> groundtruth_angles,
                                              const string &traj_type)
{
    std::vector<double> ret;
    double phi_error = utils.anglediff(groundtruth_angles.first, phi);
    double theta_error = utils.anglediff(groundtruth_angles.second, theta);
    double err = 0;
    err = std::sqrt(std::pow(phi_error, 2) + std::pow(theta_error, 2));

    ret.push_back(phi);
    ret.push_back(theta);
    ret.push_back(err);
    ret.push_back(phi_error);
    ret.push_back(theta_error);

    return ret;
}
//=============================================================================================================================
/**
 *
 *
 * */
int WSR_Module::get_tx_pkt_count(const std::string &tx_mac_id)
{
    return __tx_pkt_size[tx_mac_id];
}
//=============================================================================================================================
/**
 *
 *
 * */
int WSR_Module::get_rx_pkt_count(const std::string &tx_mac_id)
{
    return __rx_pkt_size[tx_mac_id];
}
//=============================================================================================================================
/**
 *
 *
 * */
double WSR_Module::get_processing_time(const std::string &tx_mac_id)
{
    return __perf_aoa_profile_cal_time[tx_mac_id];
}
//=============================================================================================================================
/**
 *
 *
 * */
double WSR_Module::get_memory_used(const std::string &tx_mac_id)
{
    return __memory_used[tx_mac_id];
}
//=============================================================================================================================
/**
 *
 *
 * */
double WSR_Module::get_calculated_ts_offset(const std::string &tx_mac_id)
{
    return __calculated_ts_offset[tx_mac_id];
}
//=============================================================================================================================
/**
 *
 *
 * */
double WSR_Module::get_cpdm(const std::string &tx_mac_id)
{
    return __channel_phase_diff_mean[tx_mac_id];
}
//=============================================================================================================================
/**
 *
 *
 * */
double WSR_Module::get_cpd_stdev(const std::string &tx_mac_id)
{
    return __channel_phase_diff_stdev[tx_mac_id];
}
//=============================================================================================================================
/**
 *
 *
 * */
double WSR_Module::get_scpm(const std::string &tx_mac_id)
{
    return __static_channel_phase_mean[tx_mac_id];
}
//=============================================================================================================================
/**
 *
 *
 * */
double WSR_Module::get_scd_stdev(const std::string &tx_mac_id)
{
    return __static_channel_phase_stdev[tx_mac_id];
}
//=============================================================================================================================
/**
 *
 *
 * */
int WSR_Module::get_paired_pkt_count(const std::string &tx_mac_id)
{
    return __paired_pkt_count[tx_mac_id];
}
//=============================================================================================================================
/**
 *
 *
 * */
double WSR_Module::get_top_confidence(const std::string &tx_mac_id)
{
    return __top_peak_confidence[tx_mac_id];
}
//=============================================================================================================================
/**
 *
 *
 * */
nlohmann::json WSR_Module::get_stats(double true_phi,
                                     double true_theta, 
                                     std::vector<vector<float>> &aoa_error,
                                     const std::string &tx_mac_id,
                                     const std::string &tx_name,
                                     const nc::NdArray<double> &rx_pos_est,
                                     const nc::NdArray<double> &rx_pos_true,
                                     nlohmann::json true_positions_tx,
                                     const int pos_idx)
{
    std::vector<double> mag = get_top_magnitudes(tx_mac_id);
    nlohmann::json position = true_positions_tx["value"][tx_name];
    nlohmann::json output_stats =
        {
            {"a_INFO_Transmitting_robot", {{"Name", tx_name}, {"MAC_ID", tx_mac_id}, 
                                            {"groundtruth_position", 
                                            {{"x", float(position["position"]["x"])}, 
                                             {"y", float(position["position"]["y"])}, 
                                             {"z", float(position["position"]["z"])}}}, 
                                            {"groundtruth_azimuth", true_phi}, 
                                            {"groundtruth_elevation", true_theta}}},
            {"b_INFO_Receiving_robot", {{"id", pos_idx}, 
                                        {"displacement_type", __displacement_type}, 
                                        {"estimated_start_position", 
                                        {{"x", rx_pos_est(0, 0)}, //Will be same as true positin when using gt flag
                                         {"y", rx_pos_est(0, 1)},
                                         {"z", rx_pos_est(0, 2)},
                                         {"yaw", rx_pos_est(0, 3)}}},
                                        {"groundtruth_start_position", 
                                        {{"x", rx_pos_true(0, 0)}, 
                                        {"y", rx_pos_true(0, 1)}, 
                                        {"z", rx_pos_true(0, 2)}, 
                                        {"yaw", rx_pos_true(0, 3)}}}}},
            {"c_INFO_Performance", {{"azimuth_profile_resolution", __nphi}, 
                                    {"elevation_profile_resolution", __ntheta}, 
                                    {"Forward_channel_packets", get_tx_pkt_count(tx_mac_id)}, 
                                    {"Reverse_channel_packets", get_rx_pkt_count(tx_mac_id)}, 
                                    {"Packets_Used", get_paired_pkt_count(tx_mac_id)}, 
                                    {"time(sec)", get_processing_time(tx_mac_id)}, 
                                    {"memory(GB)", get_memory_used(tx_mac_id)}}},
            {"d_INFO_AOA_profile", {{"Profile_variance", get_top_confidence(tx_mac_id)},
            {"Top_N_threshold",__relative_magnitude_threshold},
                                    {"Peaks_above_threshold", get_peak_num_above_threshold(tx_mac_id)}, 
                                    {"Top_N_peaks", 
                                    {{"1", {{"estimated_azimuth", aoa_error[0][0]}, 
                                    {"estimated_elevation", aoa_error[0][1]}, 
                                    {"Total_AOA_Error", aoa_error[0][2]}, 
                                    {"azimuth_error", aoa_error[0][3]}, 
                                    {"elevation_error", aoa_error[0][4]},
                                    {"magnitude",mag[0]}}}}}
                                    }
            }
        };

    for (int i = 1; i < aoa_error.size(); i++)
    {
        std::string itr = std::to_string(i + 1);
        output_stats["d_INFO_AOA_profile"]["Top_N_peaks"][itr]["estimated_azimuth"] = aoa_error[i][0];
        output_stats["d_INFO_AOA_profile"]["Top_N_peaks"][itr]["estimated_elevation"] = aoa_error[i][1];
        output_stats["d_INFO_AOA_profile"]["Top_N_peaks"][itr]["Total_AOA_Error"] = aoa_error[i][2];
        output_stats["d_INFO_AOA_profile"]["Top_N_peaks"][itr]["azimuth_error"] = aoa_error[i][3];
        output_stats["d_INFO_AOA_profile"]["Top_N_peaks"][itr]["elevation_error"] = aoa_error[i][4];
        output_stats["d_INFO_AOA_profile"]["Top_N_peaks"][itr]["magnitude"] = mag[i]; 
    }
    //TODO loop through the vector of top N peak stats and append to the final json output.
    //Delete get_stats_top_peaks() after this.

    return output_stats;
}
//=============================================================================================================================
/**
 *
 *
 * */
nlohmann::json WSR_Module::get_performance_stats(const std::string &tx_mac_id,
                                                 const std::string &tx_name)
{
    nlohmann::json perf_stats = {
        {"a_Info_TX", {{"TX_Name", tx_name}, {"TX_MAC_ID", tx_mac_id}}},
        {"b_INFO_Performance",
         {{"Forward_channel_packets", get_tx_pkt_count(tx_mac_id)},
          {"Reverse_channel_packets", get_rx_pkt_count(tx_mac_id)},
          {"Packets_Used", get_paired_pkt_count(tx_mac_id)},
          {"time(sec)", get_processing_time(tx_mac_id)},
          {"memory(GB)", get_memory_used(tx_mac_id)},
          {"first_forward-reverse_ts_offset(sec)", get_calculated_ts_offset(tx_mac_id)}}}};

    return perf_stats;
}
//=============================================================================================================================
/**
 *
 *
 * */
std::unordered_map<std::string, int> WSR_Module::get_paired_pkt_count()
{
    return __paired_pkt_count;
}
//=============================================================================================================================
/**
 *
 *
 * */
nlohmann::json WSR_Module::get_stats_old_json(double true_phi,
                                              double true_theta, std::vector<vector<float>> &aoa_error,
                                              const std::string &tx_mac_id,
                                              const std::string &tx_name,
                                              const nc::NdArray<double> &rx_pos_est,
                                              const nc::NdArray<double> &rx_pos_true,
                                              nlohmann::json true_positions_tx,
                                              const int pos_idx)
{

    std::cout << "Getting json" << std::endl;
    nlohmann::json output_stats = {
        {"a_Info_TX", {{"TruePhi", true_phi}, {"Truetheta", true_theta}, {"TX_Name", tx_name}, {"TX_MAC_ID", tx_mac_id}}},
        {"b_INFO_Profile_Resolution", {
                                          {"nphi", __nphi},
                                          {"ntheta", __ntheta},
                                      }},
        {"e_INFO_Performance", {{"Forward_channel_packets", get_tx_pkt_count(tx_mac_id)}, {"Reverse_channel_packets", get_rx_pkt_count(tx_mac_id)}, {"Packets_Used", get_paired_pkt_count(tx_mac_id)}, {"time(sec)", get_processing_time(tx_mac_id)}, {"memory(GB)", get_memory_used(tx_mac_id)}, {"first_forward-reverse_ts_offset(sec)", get_calculated_ts_offset(tx_mac_id)}, {"Channel_moving_phase_diff_mean", get_cpdm(tx_mac_id)}, {"Channel_moving_phase_diff_stdev", get_cpd_stdev(tx_mac_id)}, {"Channel_static_phase_mean", get_scpm(tx_mac_id)}, {"Channel_static_phase_stdev", get_scd_stdev(tx_mac_id)}}},
        {"c_Info_AOA_Top", {{"Phi(deg)", aoa_error[0][0]}, {"Theta(deg)", aoa_error[0][1]}, {"Confidence", get_top_confidence(tx_mac_id)}, {"Total_AOA_Error(deg)", aoa_error[0][2]}, {"Phi_Error(deg)", aoa_error[0][3]}, {"Theta_Error(deg)", aoa_error[0][4]}}},
        {"d_Info_AOA_Closest", {{"Phi(deg)", 0}, {"Theta(deg)", 0}, {"Total_AOA_Error(deg)", 0}, {"Phi_Error(deg)", 0}, {"Theta_Error(deg)", 0}}},
        {"RX_idx", pos_idx},
        {"RX_displacement", __displacement_type},
        {"RX_position", {{"x", rx_pos_true(0, 0)}, {"y", rx_pos_true(0, 1)}, {"z", rx_pos_true(0, 2)}, {"yaw", rx_pos_true(0, 3)}}}};

    std::cout << "Got json" << std::endl;
    return output_stats;
}
//======================================================================================================================
/**
 * Description: 
 * Input:
 * Output:
 * */
int WSR_Module::calculate_AOA_profile_multi(std::string rx_csi_file,
                                      std::unordered_map<std::string, std::string> tx_csi_file,
                                      nc::NdArray<double> displacement,
                                      nc::NdArray<double> displacement_timestamp)
{

    std::cout << "============ Starting WSR module ==============" << std::endl;

    WIFI_Agent RX_SAR_robot; //Broardcasts the csi packets and does SAR
    nc::NdArray<std::complex<double>> h_list_all, h_list_static, h_list;
    nc::NdArray<double> csi_timestamp_all, csi_timestamp;
    std::vector<std::vector<int>> rssi_value;
    double cal_ts_offset, moving_channel_ang_diff_mean, moving_channel_ang_diff_stdev,
        static_channel_ang_mean, static_channel_ang_stdev;
    std::string debug_dir = __precompute_config["debug_dir"]["value"].dump();
    debug_dir.erase(remove(debug_dir.begin(), debug_dir.end(), '\"'), debug_dir.end());
    int ret_val = 0;

    std::cout << "log [calculate_AOA_profile]: Parsing CSI Data " << std::endl;

    auto temp1 = utils.readCsiData(rx_csi_file, RX_SAR_robot, __FLAG_debug);

    // std::vector<std::string> mac_id_tx;
    std::vector<std::string> mac_id_tx;

    std::cout << "log [calculate_AOA_profile]: Neighbouring TX robot IDs count = " << RX_SAR_robot.unique_mac_ids_packets.size() << std::endl;

    for (auto key : RX_SAR_robot.unique_mac_ids_packets)
    {
        if (__FLAG_debug)
            std::cout << "log [calculate_AOA_profile]: Detected MAC ID = " << key.first
                      << ", Packet count: = " << key.second << std::endl;
        
        mac_id_tx.push_back(key.first);
        
    }

    //With this, get the mac_id of the robot that is RX for a given round
    std::set<string> s1(__RX_SAR_robot_MAC_ID_List.begin(), __RX_SAR_robot_MAC_ID_List.end());
    std::set<string> s2(mac_id_tx.begin(), mac_id_tx.end());
    std::vector<string> v3;
    std::set_difference(s1.begin(), s1.end(), s2.begin(), s2.end(), std::back_inserter(v3));
    __RX_SAR_robot_MAC_ID = v3[0];

    for (auto it = __precompute_config["input_TX_channel_csi_fn"]["value"].begin(); 
        it != __precompute_config["input_TX_channel_csi_fn"]["value"].end(); ++it)
    {
        std::cout << it.key() << std::endl;
        std::cout << it.value()["mac_id"] << std::endl;
        std::cout << __RX_SAR_robot_MAC_ID << std::endl;
        if(it.value()["mac_id"] == __RX_SAR_robot_MAC_ID)
        {
            __rx_name =  it.key();
            break;
        }
    }
    


    //Get AOA profile for each of the RX neighboring robots
    if (__FLAG_debug)
        std::cout << "log [calculate_AOA_profile]: Getting AOA profiles" << std::endl;
    std::vector<DataPacket> data_packets_RX, data_packets_TX;

    int transmission_id=1; //Temp FIX to handle forward-backward packet incorrect counter mismatch.

    for (int num_tx = 0; num_tx < mac_id_tx.size(); num_tx++)
    {
        WIFI_Agent TX_Neighbor_robot; // Neighbouring robots who reply back
        // TX_Neighbor_robot.__transmission_id = transmission_id;
        // transmission_id+=1;
        std::pair<nc::NdArray<std::complex<double>>, nc::NdArray<double>> csi_data;

        if (tx_csi_file.find(mac_id_tx[num_tx]) == tx_csi_file.end())
        {
            if (__FLAG_debug)
                std::cout << "log [calculate_AOA_profile]: No CSI data available for TX Neighbor MAC-ID: "
                          << mac_id_tx[num_tx] << ". Skipping" << std::endl;
            continue;
        }
        std::cout << "log [calculate_AOA_profile]: =========================" << std::endl;
        std::cout << "log [calculate_AOA_profile]: Profile for RX_SAR_robot MAC-ID: " << __RX_SAR_robot_MAC_ID
                  << ", TX_Neighbor_robot MAC-ID: " << mac_id_tx[num_tx] << std::endl;

        auto temp2 = utils.readCsiData(tx_csi_file[mac_id_tx[num_tx]], TX_Neighbor_robot, __FLAG_debug);

        for (auto key : TX_Neighbor_robot.unique_mac_ids_packets)
        {
            if (__FLAG_debug)
                std::cout << "log [calculate_AOA_profile]: Detected RX MAC IDs = " << key.first
                          << ", Packet count: = " << key.second << std::endl;
            //mac_id_tx.push_back(key.first);
        }

        data_packets_RX = RX_SAR_robot.get_wifi_data(mac_id_tx[num_tx]);          //Packets for a TX_Neigbor_robot in RX_SAR_robot's csi file
        data_packets_TX = TX_Neighbor_robot.get_wifi_data(__RX_SAR_robot_MAC_ID); //Packets only of RX_SAR_robot in a TX_Neighbor_robot's csi file

        if (__FLAG_debug)
        {
            std::cout << "log [calculate_AOA_profile]: Packets for TX_Neighbor_robot collected by RX_SAR_robot : "
                      << data_packets_RX.size() << std::endl;
            std::cout << "log [calculate_AOA_profile]: Packets for RX_SAR_robot collected by TX_Neighbor_robot : "
                      << data_packets_TX.size() << std::endl;
        }

        if (__FLag_use_packet_id)
        {
            std::cout << "log [calculate_AOA_profile]: Calculating forward-reverse channel product using Counter " << std::endl;
            csi_data = utils.getForwardReverseChannelCounter(data_packets_RX,
                                                             data_packets_TX,
                                                             __FLAG_interpolate_phase,
                                                             __FLAG_sub_sample);
        }
        else
        {
            std::cout << "log [calculate_AOA_profile]: Calculating forward-reverse channel product using Timestamps " << std::endl;
            csi_data = utils.getForwardReverseChannel_v2(data_packets_RX,
                                                         data_packets_TX,
                                                         __time_offset,
                                                         __time_threshold,
                                                         cal_ts_offset,
                                                         __FLAG_interpolate_phase,
                                                         __FLAG_sub_sample);
        }

        std::cout << "log [calculate_AOA_profile]: corrected CFO " << std::endl;
        h_list_all = csi_data.first;
        csi_timestamp_all = csi_data.second;

        if (csi_timestamp_all.size() < __min_packets_to_process)
        {
            std::cout << csi_timestamp_all.size() << std::endl;
            if (__FLAG_debug)
                std::cout << "log [calculate_AOA_profile]: Very few CSI data packets left after forward-backward product" << std::endl;
            break;
        }

        /*Get the shifted version of the pose timestamps such that they match exactly with csi timestamps.*/
        if (__FLAG_debug)
            std::cout << "log [calculate_AOA_profile]: Removing unused csi data" << std::endl;
        std::pair<int, int> csi_timestamp_range = utils.returnClosestIndices(csi_timestamp_all, displacement_timestamp);
        int start_index = csi_timestamp_range.first, end_index = csi_timestamp_range.second;

        /*Slice the csi_timestamp using the indices to remove unused CSI values
        * Note: Make sure that the packet transmission freqency is high enough when random_packets is used, 
        * such that about 400 left after slicing even if the trajectory duration is small. 
        * */
        if (__FLAG_debug)
            std::cout << "log [calculate_AOA_profile]: Slicing CSI timestamps" << std::endl;
        
        csi_timestamp = csi_timestamp_all({start_index, end_index}, csi_timestamp_all.cSlice());
        h_list = h_list_all({start_index, end_index}, h_list_all.cSlice());
        h_list_static = h_list_all({0, start_index}, h_list_all.cSlice());

        if (__FLAG_debug)
            std::cout << "log [calculate_AOA_profile]: Slicing RSSI data" << std::endl;
        rssi_value = utils.get_signal_strength(data_packets_RX,start_index,end_index);
        

        bool moving = true;
        utils.get_phase_diff_metrics(h_list,
                                     moving_channel_ang_diff_mean,
                                     moving_channel_ang_diff_stdev,
                                     __FLAG_interpolate_phase,
                                     moving);

        moving = false;
        utils.get_phase_diff_metrics(h_list_static,
                                     static_channel_ang_mean,
                                     static_channel_ang_stdev,
                                     __FLAG_interpolate_phase,
                                     moving);

        if (h_list.shape().rows < __min_packets_to_process)
        {
            std::cout << "log [calculate_AOA_profile]: Not enough CSI packets." << std::endl;
            std::cout << "log [calculate_AOA_profile]: Return Empty AOA profile" << std::endl;
            //Return empty dummpy AOA profile
            __aoa_profile = nc::zeros<double>(1, 1);
        }
        else
        {

            /*Interpolate the trajectory using the csi data timestamps*/
            if (__FLAG_debug)
                std::cout << "log [calculate_AOA_profile]: interpolating the trajectory and csi forward-reverse product" << std::endl;
            auto interpolated_data = utils.interpolate(csi_timestamp, displacement_timestamp, displacement);
            nc::NdArray<double> pose_list = interpolated_data.first;

            if (__FLAG_debug)
            {
                std::cout << "log [calculate_AOA_profile]: CSI_packets_used = " << csi_timestamp.shape() << std::endl;
                std::cout << "log [calculate_AOA_profile]: pose_list size  = " << pose_list.shape() << std::endl;
                std::cout << "log [calculate_AOA_profile]: h_list size  = " << h_list.shape() << std::endl;

                std::string debug_dir = __precompute_config["debug_dir"]["value"].dump();
                debug_dir.erase(remove(debug_dir.begin(), debug_dir.end(), '\"'), debug_dir.end());

                //Store phase and timestamp of the channel for debugging
                std::string channel_data_sliced = debug_dir + "/" + tx_name_list[mac_id_tx[num_tx]] + "_" + data_sample_ts[mac_id_tx[num_tx]] + "_sliced_channel_data.json";
                utils.writeCSIToJsonFile(h_list, csi_timestamp, channel_data_sliced, __FLAG_interpolate_phase);

                std::string channel_data_all = debug_dir + "/" + tx_name_list[mac_id_tx[num_tx]] + "_" + data_sample_ts[mac_id_tx[num_tx]] + "_all_channel_data.json";
                utils.writeCSIToJsonFile(h_list_all, csi_timestamp_all, channel_data_all, __FLAG_interpolate_phase);

                //Store the packet distribution to check for spotty packets
                std::string packet_dist = debug_dir + "/" + tx_name_list[mac_id_tx[num_tx]] + "_" + data_sample_ts[mac_id_tx[num_tx]] + "_packet_dist.json";
                utils.writePacketDistributionToJsonFile(csi_timestamp, displacement_timestamp, displacement, packet_dist);

                //Store interpolated trajectory for debugging
                std::string interpl_trajectory = debug_dir + "/" + tx_name_list[mac_id_tx[num_tx]] + "_" + data_sample_ts[mac_id_tx[num_tx]] + "_interpl_trajectory.json";
                utils.writeTrajToFile(pose_list, interpl_trajectory);

                //Write RSSI value to Json file
                std::string rssi_val_fn = debug_dir + "/" + tx_name_list[mac_id_tx[num_tx]] + "_" + data_sample_ts[mac_id_tx[num_tx]] + "_rssi.json";
                utils.writeRssiToFile(rssi_value, rssi_val_fn);
            }

            /*Interpolate the trajectory and csi data*/
            std::cout << "log [calculate_AOA_profile]: Calculating AOA profile..." << std::endl;
            auto starttime = std::chrono::high_resolution_clock::now();

            __aoa_profile = compute_profile_music(h_list, pose_list);

            auto endtime = std::chrono::high_resolution_clock::now();
            float processtime = std::chrono::duration<float, std::milli>(endtime - starttime).count();

            //Stats
            std::pair<std::vector<double>, std::vector<double>> top_N = find_topN();
            __TX_top_N_angles[mac_id_tx[num_tx]] = top_N;
            __paired_pkt_count[mac_id_tx[num_tx]] = csi_timestamp.shape().rows;
            __perf_aoa_profile_cal_time[mac_id_tx[num_tx]] = processtime / 1000;
            __memory_used[mac_id_tx[num_tx]] = utils.mem_usage() / 1000000;
            __calculated_ts_offset[mac_id_tx[num_tx]] = cal_ts_offset;
            __rx_pkt_size[mac_id_tx[num_tx]] = data_packets_RX.size();
            __tx_pkt_size[mac_id_tx[num_tx]] = data_packets_TX.size();
            __channel_phase_diff_mean[mac_id_tx[num_tx]] = moving_channel_ang_diff_mean;
            __channel_phase_diff_stdev[mac_id_tx[num_tx]] = moving_channel_ang_diff_stdev;
            __static_channel_phase_mean[mac_id_tx[num_tx]] = static_channel_ang_mean;
            __static_channel_phase_stdev[mac_id_tx[num_tx]] = static_channel_ang_stdev;
            __top_peak_confidence[mac_id_tx[num_tx]] = __aoa_profile_variance[0];
            __all_topN_magnitudes[mac_id_tx[num_tx]] = __peak_magnitudes;
            __all_topN_above_threshold[mac_id_tx[num_tx]] = __num_peaks_above_threshold;
        }

        /*Store the aoa_profile*/
        __all_aoa_profiles[mac_id_tx[num_tx]] = __aoa_profile;
        __all_topN_confidence[mac_id_tx[num_tx]] = __aoa_profile_variance;

        //TODO: get azimuth and elevation from beta_profile
        std::cout << "log [calculate_AOA_profile]: Completed AOA calculation." << std::endl;
        TX_Neighbor_robot.reset();
    }

    std::cout << "============ WSR module end ==============" << std::endl;
    RX_SAR_robot.reset();

    return ret_val;
}
//======================================================================================================================
/**
 * Description: 
 * Input:
 * Output:
 * */
int WSR_Module::calculate_spoofed_AOA_profile(std::string rx_csi_file,
                                            std::unordered_map<std::string, std::string> tx_csi_file,
                                            nc::NdArray<double> displacement,
                                            nc::NdArray<double> displacement_timestamp)
{

    std::cout << "============ Starting WSR module Spoofing Simulation ==============" << std::endl;

    WIFI_Agent RX_SAR_robot; //Broardcasts the csi packets and does SAR
    nc::NdArray<std::complex<double>> h_list_all, h_list_static, h_list;
    nc::NdArray<double> csi_timestamp_all, csi_timestamp;
    std::vector<std::vector<int>> rssi_value;
    double cal_ts_offset, moving_channel_ang_diff_mean, moving_channel_ang_diff_stdev,
        static_channel_ang_mean, static_channel_ang_stdev;
    std::string debug_dir = __precompute_config["debug_dir"]["value"].dump();
    debug_dir.erase(remove(debug_dir.begin(), debug_dir.end(), '\"'), debug_dir.end());
    int ret_val = 0;

    std::cout << "log [calculate_AOA_profile]: Parsing CSI Data " << std::endl;

    auto temp1 = utils.readCsiData(rx_csi_file, RX_SAR_robot, __FLAG_debug);

    //Check the actual number of packets collected
    std::cout << "log [calculate_AOA_profile]: Neighbouring TX robot IDs count = " << RX_SAR_robot.unique_mac_ids_packets.size() << std::endl;
    for (auto key : RX_SAR_robot.unique_mac_ids_packets)
    {
        if (__FLAG_debug)
            std::cout << "log [calculate_AOA_profile]: Detected MAC ID = " << key.first
                      << ", Packet count: = " << key.second << std::endl;
    }


    //Simulated spoofed data by changing the mac-id of alternate packets on the RX for a specific "illegitimate client e.g.tx2"
    std::string illegit_mac_id = "";
    for(auto key: tx_name_list)
    {
        if(key.second == "tx3") illegit_mac_id = key.first;
    }
    RX_SAR_robot.simulate_spoofed_data_multiple(1,illegit_mac_id);


    // //Spoof the second MAC-ID
    for(auto key: tx_name_list)
    {
        if(key.second == "tx4") illegit_mac_id = key.first;
    }
    RX_SAR_robot.simulate_spoofed_data_second(1,illegit_mac_id);


    //Check the spoofed number of packets simulated
    std::vector<std::string> mac_id_tx;
    std::cout << "log [calculate_AOA_profile]: Neighbouring Spoofed TX robot IDs count = " << RX_SAR_robot.unique_mac_ids_packets_spoofed.size() << std::endl;
    for (auto key : RX_SAR_robot.unique_mac_ids_packets_spoofed)
    {
        if (__FLAG_debug)
            std::cout << "log [calculate_AOA_profile]: Detected MAC ID = " << key.first
                      << ", Packet count: = " << key.second << std::endl;
        mac_id_tx.push_back(key.first);
    }

    //With this, get the mac_id of the robot that is RX for a given round
    std::set<string> s1(__RX_SAR_robot_MAC_ID_List.begin(), __RX_SAR_robot_MAC_ID_List.end());
    std::set<string> s2(mac_id_tx.begin(), mac_id_tx.end());
    std::vector<string> v3;
    std::set_difference(s1.begin(), s1.end(), s2.begin(), s2.end(), std::back_inserter(v3));
    __RX_SAR_robot_MAC_ID = v3[0];


    for (auto it = __precompute_config["input_TX_channel_csi_fn"]["value"].begin(); 
        it != __precompute_config["input_TX_channel_csi_fn"]["value"].end(); ++it)
    {
        std::cout << it.key() << std::endl;
        std::cout << it.value()["mac_id"] << std::endl;
        std::cout << __RX_SAR_robot_MAC_ID << std::endl;
        if(it.value()["mac_id"] == __RX_SAR_robot_MAC_ID)
        {
            __rx_name =  it.key();
            break;
        }
    }


    //Get AOA profile for each of the Spoofed TX neighboring robots
    if (__FLAG_debug)
        std::cout << "log [calculate_AOA_profile]: Getting AOA profiles" << std::endl;
    std::vector<DataPacket> data_packets_RX, data_packets_TX;

    // int transmission_id=2; //Temp FIX to handle forward-backward packet incorrect counter mismatch.

    for (int num_tx = 0; num_tx < mac_id_tx.size(); num_tx++)
    {
        WIFI_Agent TX_Neighbor_robot; // Neighbouring robots who reply back
        // TX_Neighbor_robot.__transmission_id = transmission_id;
        // transmission_id+=1;
        std::pair<nc::NdArray<std::complex<double>>, nc::NdArray<double>> csi_data;

        if (tx_csi_file.find(mac_id_tx[num_tx]) == tx_csi_file.end())
        {
            if (__FLAG_debug)
                std::cout << "log [calculate_AOA_profile]: No CSI data available for TX Neighbor MAC-ID: "
                          << mac_id_tx[num_tx] << ". Skipping" << std::endl;
            continue;
        }
        std::cout << "log [calculate_AOA_profile]: =========================" << std::endl;
        std::cout << "log [calculate_AOA_profile]: Profile for RX_SAR_robot MAC-ID: " << __RX_SAR_robot_MAC_ID
                  << ", TX_Neighbor_robot MAC-ID: " << mac_id_tx[num_tx] << std::endl;

        auto temp2 = utils.readCsiData(tx_csi_file[mac_id_tx[num_tx]], TX_Neighbor_robot, __FLAG_debug);

        for (auto key : TX_Neighbor_robot.unique_mac_ids_packets)
        {
            if (__FLAG_debug)
                std::cout << "log [calculate_AOA_profile]: Detected RX MAC IDs = " << key.first
                          << ", Packet count: = " << key.second << std::endl;
            //mac_id_tx.push_back(key.first);
        }

        data_packets_RX = RX_SAR_robot.get_wifi_data_spoofed(mac_id_tx[num_tx]);  //Spoofed Packets for a TX_Neigbor_robot in RX_SAR_robot's csi file
        data_packets_TX = TX_Neighbor_robot.get_wifi_data(__RX_SAR_robot_MAC_ID); //Packets only of RX_SAR_robot in a TX_Neighbor_robot's csi file

        if (__FLAG_debug)
        {
            std::cout << "log [calculate_AOA_profile]: Packets for TX_Neighbor_robot collected by RX_SAR_robot : "
                      << data_packets_RX.size() << std::endl;
            std::cout << "log [calculate_AOA_profile]: Packets for RX_SAR_robot collected by TX_Neighbor_robot : "
                      << data_packets_TX.size() << std::endl;
        }


        std::cout << "log [calculate_AOA_profile]: Calculating forward-reverse channel product using Counter " << std::endl;
        
        if( mac_id_tx[num_tx] == "00:21:6A:3F:16:DA" || mac_id_tx[num_tx] == "00:21:6A:3E:F5:7E") 
            csi_data = utils.getForwardReverseChannelCounter(data_packets_RX,
                                                            data_packets_TX,
                                                            __FLAG_interpolate_phase,
                                                            true);
        else
            csi_data = utils.getForwardReverseChannelCounter(data_packets_RX,      //Do not subsample for illegitimate or spoofed clients.
                                                            data_packets_TX,
                                                            __FLAG_interpolate_phase,
                                                            false);


        std::cout << "log [calculate_AOA_profile]: corrected CFO " << std::endl;
        h_list_all = csi_data.first;
        csi_timestamp_all = csi_data.second;

        if (csi_timestamp_all.size() < __min_packets_to_process)
        {
            std::cout << csi_timestamp_all.size() << std::endl;
            if (__FLAG_debug)
                std::cout << "log [calculate_AOA_profile]: Very few CSI data packets left after forward-backward product" << std::endl;
            break;
        }

        /*Get the shifted version of the pose timestamps such that they match exactly with csi timestamps.*/
        if (__FLAG_debug)
            std::cout << "log [calculate_AOA_profile]: Removing unused csi data" << std::endl;
        std::pair<int, int> csi_timestamp_range = utils.returnClosestIndices(csi_timestamp_all, displacement_timestamp);
        int start_index = csi_timestamp_range.first, end_index = csi_timestamp_range.second;

        /*Slice the csi_timestamp using the indices to remove unused CSI values
        * Note: Make sure that the packet transmission freqency is high enough when random_packets is used, 
        * such that about 400 left after slicing even if the trajectory duration is small. 
        * */
        if (__FLAG_debug)
            std::cout << "log [calculate_AOA_profile]: Slicing CSI timestamps" << std::endl;
        
        csi_timestamp = csi_timestamp_all({start_index, end_index}, csi_timestamp_all.cSlice());
        h_list = h_list_all({start_index, end_index}, h_list_all.cSlice());
        h_list_static = h_list_all({0, start_index}, h_list_all.cSlice());

    
        rssi_value = utils.get_signal_strength(data_packets_RX,start_index,end_index);

        if (h_list.shape().rows < __min_packets_to_process)
        {
            std::cout << "log [calculate_AOA_profile]: Not enough CSI packets." << std::endl;
            std::cout << "log [calculate_AOA_profile]: Return Empty AOA profile" << std::endl;
            //Return empty dummpy AOA profile
            __aoa_profile = nc::zeros<double>(1, 1);
        }
        else
        {
            /*Interpolate the trajectory using the csi data timestamps*/
            if (__FLAG_debug)
                std::cout << "log [calculate_AOA_profile]: interpolating the trajectory and csi forward-reverse product" << std::endl;
            auto interpolated_data = utils.interpolate(csi_timestamp, displacement_timestamp, displacement);
            nc::NdArray<double> pose_list = interpolated_data.first;

            if (__FLAG_debug)
            {
                std::cout << "log [calculate_AOA_profile]: CSI_packets_used = " << csi_timestamp.shape() << std::endl;
                std::cout << "log [calculate_AOA_profile]: pose_list size  = " << pose_list.shape() << std::endl;
                std::cout << "log [calculate_AOA_profile]: h_list size  = " << h_list.shape() << std::endl;

                std::string debug_dir = __precompute_config["debug_dir"]["value"].dump();
                debug_dir.erase(remove(debug_dir.begin(), debug_dir.end(), '\"'), debug_dir.end());

                //Store phase and timestamp of the channel for debugging
                std::string channel_data_sliced = debug_dir + "/" + tx_name_list[mac_id_tx[num_tx]] + "_" + data_sample_ts[mac_id_tx[num_tx]] + "_sliced_channel_data.json";
                utils.writeCSIToJsonFile(h_list, csi_timestamp, channel_data_sliced, __FLAG_interpolate_phase);

                std::string channel_data_all = debug_dir + "/" + tx_name_list[mac_id_tx[num_tx]] + "_" + data_sample_ts[mac_id_tx[num_tx]] + "_all_channel_data.json";
                utils.writeCSIToJsonFile(h_list_all, csi_timestamp_all, channel_data_all, __FLAG_interpolate_phase);

                //Store the packet distribution to check for spotty packets
                std::string packet_dist = debug_dir + "/" + tx_name_list[mac_id_tx[num_tx]] + "_" + data_sample_ts[mac_id_tx[num_tx]] + "_packet_dist.json";
                utils.writePacketDistributionToJsonFile(csi_timestamp, displacement_timestamp, displacement, packet_dist);

                //Store interpolated trajectory for debugging
                std::string interpl_trajectory = debug_dir + "/" + tx_name_list[mac_id_tx[num_tx]] + "_" + data_sample_ts[mac_id_tx[num_tx]] + "_interpl_trajectory.json";
                utils.writeTrajToFile(pose_list, interpl_trajectory);

                //Write RSSI value to Json file
                std::string rssi_val_fn = debug_dir + "/" + tx_name_list[mac_id_tx[num_tx]] + "_" + data_sample_ts[mac_id_tx[num_tx]] + "_rssi.json";
                utils.writeRssiToFile(rssi_value, rssi_val_fn);
            }

            /*Interpolate the trajectory and csi data*/
            std::cout << "log [calculate_AOA_profile]: Calculating AOA profile..." << std::endl;
            auto starttime = std::chrono::high_resolution_clock::now();

            // __aoa_profile = compute_profile_bartlett(h_list, pose_list);
            __aoa_profile = compute_profile_music(h_list, pose_list);

            auto endtime = std::chrono::high_resolution_clock::now();
            float processtime = std::chrono::duration<float, std::milli>(endtime - starttime).count();

            //Stats
            std::pair<std::vector<double>, std::vector<double>> top_N = find_topN();
            __TX_top_N_angles[mac_id_tx[num_tx]] = top_N;
            __paired_pkt_count[mac_id_tx[num_tx]] = csi_timestamp.shape().rows;
            __perf_aoa_profile_cal_time[mac_id_tx[num_tx]] = processtime / 1000;
            __memory_used[mac_id_tx[num_tx]] = utils.mem_usage() / 1000000;
            __calculated_ts_offset[mac_id_tx[num_tx]] = cal_ts_offset;
            __rx_pkt_size[mac_id_tx[num_tx]] = data_packets_RX.size();
            __tx_pkt_size[mac_id_tx[num_tx]] = data_packets_TX.size();
            __top_peak_confidence[mac_id_tx[num_tx]] = __aoa_profile_variance[0];
            __all_topN_magnitudes[mac_id_tx[num_tx]] = __peak_magnitudes;
            __all_topN_above_threshold[mac_id_tx[num_tx]] = __num_peaks_above_threshold;
        }

        /*Store the aoa_profile*/
        __all_aoa_profiles[mac_id_tx[num_tx]] = __aoa_profile;
        __all_topN_confidence[mac_id_tx[num_tx]] = __aoa_profile_variance;

        //TODO: get azimuth and elevation from beta_profile
        std::cout << "log [calculate_AOA_profile]: Completed AOA calculation." << std::endl;
        TX_Neighbor_robot.reset();
    }

    std::cout << "============ WSR module end ==============" << std::endl;
    RX_SAR_robot.reset();

    return ret_val;
}
//=============================================================================================================================
/**
 *
 *
 * */
std::vector<double> WSR_Module::get_top_magnitudes(const std::string &tx_mac_id)
{
    return __all_topN_magnitudes[tx_mac_id];
}
//=============================================================================================================================
/**
 * Description: 
 * Input:
 * Output:
 */
void WSR_Module::getExponential(EigencdMatrix &out,
                                EigencdMatrix &in)
{

    int i = 0;
    int j = 0;
#pragma omp parallel for shared(out, in) private(i, j) collapse(2)
    for (i = 0; i < in.rows(); i++)
    {
        for (j = 0; j < in.cols(); j++)
        {
            out(i, j) = exp(in(i, j));
        }
    }
}
//=============================================================================================================================
/**
 *
 *
 * */
int WSR_Module::get_peak_num_above_threshold(const std::string &tx_mac_id)
{
    return __all_topN_above_threshold[tx_mac_id];
}

