/**
 * (c) REACT LAB, Harvard University
	Author: Ninad Jadhav, Weiying Wang
*/

#include "csitoolbox/WSR_Module.h"

WSR_Util utils;
constexpr bool __takeOwnership = true;

//======================================================================================================================
/**
 * Description: 
 * Input:
 * Output:
 * */
WSR_Module::WSR_Module(void){}
WSR_Module::~WSR_Module(void){}


//======================================================================================================================
/**
 * Description: 
 * Input:
 * Output:
 * */
WSR_Module::WSR_Module(std::string config_fn)
{  
    std::cout << "log [Precomp]: Started Initializations" << std::endl;
    std::ifstream input(config_fn);
    if(input.fail())
    {
        throw std::runtime_error("Unable to open file " + config_fn);
    }

    input >> __precompute_config;

    __FLAG_packet_threshold = bool(__precompute_config["use_max_packets_threshold"]["value"]);
    __FLAG_debug = bool(__precompute_config["debug"]["value"]);
    __FLAG_threading = bool(__precompute_config["multi_threading"]["value"]);
    __FLAG_interpolate_phase = bool(__precompute_config["interpolate_phase"]["value"]);
    __FLAG_sub_sample = bool(__precompute_config["sub_sample_channel_data"]["value"]);
    __FLAG_normalize_profile = bool(__precompute_config["normalize_profile"]["value"]);
    bool __FLAG_use_multiple_sub_carriers =  bool(__precompute_config["multiple_sub_carriers"]["value"]);
    bool __FLAG_use_magic_mac = bool(__precompute_config["use_magic_mac"]["value"]);
    
    //calculate channel freqeuency based on channel and subcarrier number
    double centerfreq = (5000 + double(__precompute_config["channel"]["value"])*5)*1e6 + 
                        (double(__precompute_config["subCarrier"]["value"]) - 15.5)*20e6/30;    
    __time_offset = double(__precompute_config["time_offset"]["value"]);
    __time_threshold = double(__precompute_config["time_threshold"]["value"]);
    __lambda = double(__precompute_config["c"]["value"]) / centerfreq;
    __nphi = int(__precompute_config["nphi"]["value"]);
    __ntheta = int(__precompute_config["ntheta"]["value"]);
    _phi_min = int(__precompute_config["phi_min"]["value"]);
    _phi_max = int(__precompute_config["phi_max"]["value"]);
    _theta_min = int(__precompute_config["theta_min"]["value"]);
    _theta_max = int(__precompute_config["theta_max"]["value"]);
    __snum_start = int(__precompute_config["scnum_start"]["value"]);   
    _topN_count = int(__precompute_config["topN_count"]["value"]);
    __max_packets_to_process = int(__precompute_config["max_packets_to_process"]["value"]);
    __min_packets_to_process = int(__precompute_config["min_packets_to_process"]["value"]);
    __peak_radius = int(__precompute_config["peak_radius"]["value"]);
    __snum_end = __FLAG_use_multiple_sub_carriers ? int(__precompute_config["scnum_end"]["value"]) : 
                int(__precompute_config["scnum_start"]["value"])+1; 
    
    if(__FLAG_use_magic_mac)
    {
        __RX_SAR_robot_MAC_ID = __precompute_config["Magic_MAC_ID"]["value"];
    }
    else
    {
        __RX_SAR_robot_MAC_ID = __precompute_config["MAC_ID"]["value"];
    }

    theta_list = nc::linspace(_theta_min*M_PI/180,_theta_max*M_PI/180,__ntheta);
    phi_list = nc::linspace(_phi_min*M_PI/180,_phi_max*M_PI/180,__nphi);
    precomp_rep_phi = nc::repeat(phi_list.transpose(),__ntheta,1);
    precomp_rep_theta = nc::repeat(theta_list.transpose(),1,__nphi);
    precomp_rep_theta = nc::reshape(precomp_rep_theta, __nphi*__ntheta,1);
    nc::NdArray<double> lambda_list = {__lambda};
    __eigen_lambda_list = EigenDoubleMatrixMap(lambda_list.data(), 
                                                lambda_list.numRows(), 
                                                lambda_list.numCols());
    __eigen_precomp_rep_phi = EigenDoubleMatrixMap(precomp_rep_phi.data(), 
                                                precomp_rep_phi.numRows(), 
                                                precomp_rep_phi.numCols());
    __eigen_precomp_rep_theta = EigenDoubleMatrixMap(precomp_rep_theta.data(), 
                                                    precomp_rep_theta.numRows(), 
                                                    precomp_rep_theta.numCols());


  if(__FLAG_threading)
    {
        std::thread t1 (&WSR_Module::get_repmat, this, 
                                    std::ref(__precomp__eigen_rep_lambda), 
                                    std::ref(__eigen_lambda_list), __nphi*__ntheta, __max_packets_to_process);

        std::thread t2 (&WSR_Module::get_repmat, this, 
                                std::ref(__precomp__eigen_rep_phi), 
                                std::ref(__eigen_precomp_rep_phi), 1, __max_packets_to_process);

        std::thread t3 (&WSR_Module::get_repmat, this, 
                                    std::ref(__precomp__eigen_rep_theta), 
                                    std::ref(__eigen_precomp_rep_theta), 1, __max_packets_to_process);                                                            

        t1.join();
        t2.join();
        t3.join();
    }
    else
    {
        get_repmat(__precomp__eigen_rep_lambda, __eigen_lambda_list,__nphi*__ntheta, __max_packets_to_process);
        get_repmat(__precomp__eigen_rep_phi, __eigen_precomp_rep_phi,1, __max_packets_to_process);
        get_repmat(__precomp__eigen_rep_theta, __eigen_precomp_rep_theta,1, __max_packets_to_process);
    }
    
    if(__FLAG_debug)
    {
        //List of bool flags that impact calculations
        std::cout << "log [Precomp]: Important FLAGS status" << std::endl;
        std::cout << "  Trajectory Type = " << __precompute_config["trajectory_type"]["value"] << std::endl;
        std::cout << "  __FLAG_packet_threshold = " << utils.bool_to_string(__FLAG_packet_threshold) << std::endl;
        std::cout << "  __FLAG_debug = " << utils.bool_to_string(__FLAG_debug) << std::endl;
        std::cout << "  __FLAG_threading = " << utils.bool_to_string(__FLAG_threading) << std::endl;
        std::cout << "  __FLAG_interpolate_phase = " << utils.bool_to_string(__FLAG_interpolate_phase) << std::endl;
        std::cout << "  __FLAG_sub_sample = " << utils.bool_to_string(__FLAG_sub_sample) << std::endl;
        std::cout << "  __FLAG_normalize_profile = " << utils.bool_to_string(__FLAG_normalize_profile) << std::endl;
        std::cout << "  __FLAG_use_multiple_sub_carriers = " << utils.bool_to_string(__FLAG_use_multiple_sub_carriers) << std::endl;
        std::cout << "  __FLAG_use_magic_mac = " << utils.bool_to_string(__FLAG_use_magic_mac) << std::endl;
    }

    std::cout << "log [Precomp]: Finished Initializations" << std::endl;

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
                                    nc::NdArray<double> trajectory_timestamp
                                    )
{

    std::cout << "============ Starting WSR module ==============" << std::endl;

    WIFI_Agent RX_SAR_robot; //Broardcasts the csi packets and does SAR 
    nc::NdArray<std::complex<double>> h_list_all;
    nc::NdArray<double> csi_timestamp_all;
    double cal_ts_offset;

    std::cout << "log [calculate_AOA_profile]: Parsing CSI Data " << std::endl;

    auto temp1  = utils.readCsiData(rx_csi_file, RX_SAR_robot,__FLAG_debug);

    // std::vector<std::string> mac_id_tx;
    std::vector<std::string> mac_id_tx; 

    std::cout << "log [calculate_AOA_profile]: Neighbouring TX robot IDs = " <<
                RX_SAR_robot.unique_mac_ids_packets.size() << std::endl;

    for(auto key : RX_SAR_robot.unique_mac_ids_packets)
    {
        if(__FLAG_debug) std::cout << "log [calculate_AOA_profile]: Detected MAC ID = " << key.first 
                                   << ", Packet count: = " << key.second << std::endl;
        mac_id_tx.push_back(key.first);
    }

    // std::cout << "log [calculate_AOA_profile]: Receiving robot IDs = "
    //           << receiving_robot.unique_mac_ids_packets.size() << std::endl;
    // for(auto key : receiving_robot.unique_mac_ids_packets)
    // {
    //     if(__FLAG_debug) std::cout << "log [calculate_AOA_profile]: MAC ID = "
    //                                << key.first << ", Packet count: = " << key.second << std::endl;
    //     mac_id_rx.push_back(key.first);
    // }

    //Get AOA profile for each of the RX neighboring robots
    if(__FLAG_debug) std::cout << "log [calculate_AOA_profile]: Getting AOA profiles" << std::endl;
    std::vector<DataPacket> data_packets_RX, data_packets_TX;

    for (int num_tx=0; num_tx<mac_id_tx.size(); num_tx++)
    {
        WIFI_Agent TX_Neighbor_robot; // Neighbouring robots who reply back

        if(tx_csi_file.find (mac_id_tx[num_tx]) == tx_csi_file.end()) 
        {
            if(__FLAG_debug) std::cout << "log [calculate_AOA_profile]: No CSI data available for TX Neighbor MAC-ID: "
                                       << mac_id_tx[num_tx] << ". Skipping" << std::endl;
            continue;
        }
        std::cout << "log [calculate_AOA_profile]: =========================" << std::endl;
        std::cout << "log [calculate_AOA_profile]: Profile for RX_SAR_robot MAC-ID: "<< __RX_SAR_robot_MAC_ID
                  << ", TX_Neighbor_robot MAC-ID: " << mac_id_tx[num_tx] << std::endl;

        auto temp2 = utils.readCsiData(tx_csi_file[mac_id_tx[num_tx]], 
                                                            TX_Neighbor_robot,__FLAG_debug);

        data_packets_RX = RX_SAR_robot.get_wifi_data(mac_id_tx[num_tx]); //Packets for a TX_Neigbor_robot in RX_SAR_robot's csi file
        data_packets_TX = TX_Neighbor_robot.get_wifi_data(__RX_SAR_robot_MAC_ID); //Packets only of RX_SAR_robot in a TX_Neighbor_robot's csi file
        
        if(__FLAG_debug)
        {
            std::cout << "log [calculate_AOA_profile]: Packets for TX_Neighbor_robot collected by RX_SAR_robot : "
                      << data_packets_RX.size() << std::endl;
            std::cout << "log [calculate_AOA_profile]: Packets for RX_SAR_robot collected by TX_Neighbor_robot : "
                      << data_packets_TX.size() << std::endl;
            std::cout << "log [calculate_AOA_profile]: Calculating forward-reverse channel product " << std::endl;
        }
        
        //Potential Bug #28
        auto csi_data = utils.getForwardReverseChannel_v2(data_packets_RX,
                                                          data_packets_TX,
                                                          __time_offset,
                                                          __time_threshold,
                                                          cal_ts_offset,
                                                          __FLAG_interpolate_phase,
                                                          __FLAG_sub_sample);
        nc::NdArray<std::complex<double>> h_list = csi_data.first;
        nc::NdArray<double> csi_timestamp = csi_data.second;

        if (__FLAG_debug)
        {
            h_list_all = h_list;
            csi_timestamp_all = csi_timestamp;
        }

        
        if(h_list.shape().rows < __min_packets_to_process)
        {
            std::cout << "log [calculate_AOA_profile]: Not enough CSI packets." << std::endl;    
            std::cout << "log [calculate_AOA_profile]: Return Empty AOA profile" << std::endl;            
            //Return empty dummpy AOA profile
            __aoa_profile = nc::zeros<double>(1,1);  
        }   
        else
        {
            /*Get the shifted version of the pose timestamps such that they match exactly with csi timestamps.*/
            if(__FLAG_debug) std::cout << "log [calculate_AOA_profile]: Removing unused csi data" << std::endl;
            std::pair<int,int> csi_timestamp_range = utils.returnClosestIndices(csi_timestamp, trajectory_timestamp);
            int start_index = csi_timestamp_range.first, end_index = csi_timestamp_range.second;

            /*Slice the csi_timestamp using the indices to remove unused CSI values
            * Note: Make sure that the packet transmission freqency is high enough when random_packets is used, 
            * such that about 400 left after slicing even if the trajectory duration is small. 
            * */
            if(__FLAG_debug) std::cout << "log [calculate_AOA_profile]: Slicing CSI timestamps" << std::endl;
            csi_timestamp = csi_timestamp({start_index,end_index},csi_timestamp.cSlice());
            h_list = h_list({start_index,end_index},h_list.cSlice());
            
            /*Interpolate the trajectory using the csi data timestamps*/
            if(__FLAG_debug) std::cout << "log [calculate_AOA_profile]: interpolating the trajectory and csi forward-reverse product" << std::endl;
            auto interpolated_data = utils.interpolate(csi_timestamp, trajectory_timestamp, displacement);
            nc::NdArray<double> pose_list = interpolated_data.first;
    
            if(__FLAG_debug)
            {
                std::cout << "log [calculate_AOA_profile]: CSI_packets_used = " << csi_timestamp.shape() << std::endl;
                std::cout << "log [calculate_AOA_profile]: pose_list size  = " << pose_list.shape() << std::endl;
                std::cout << "log [calculate_AOA_profile]: h_list size  = " << h_list.shape() << std::endl;
                
                std::string debug_dir = __precompute_config["debug_dir"]["value"].dump();
                debug_dir.erase(remove( debug_dir.begin(), debug_dir.end(), '\"' ),debug_dir.end());
                
                //Store phase and timestamp of the channel for debugging
                std::string channel_data_sliced =  debug_dir+"/tx_"+mac_id_tx[num_tx]+"_sliced_channel_data.json";
                utils.writeCSIToJsonFile(h_list, csi_timestamp, channel_data_sliced);

                std::string channel_data_all =  debug_dir+"/tx_"+mac_id_tx[num_tx]+"_all_channel_data.json";
                utils.writeCSIToJsonFile(h_list_all, csi_timestamp_all, channel_data_all);

                //Store interpolated trajectory for debugging
                std::string interpl_trajectory =  debug_dir+"/tx_"+mac_id_tx[num_tx]+"_interpl_trajectory.json";
                utils.writeTrajToFile(pose_list,interpl_trajectory);
            }

            /*Interpolate the trajectory and csi data*/
            std::cout << "log [calculate_AOA_profile]: Calculating AOA profile..." << std::endl;
            auto starttime = std::chrono::high_resolution_clock::now();
            
            if(__FLAG_threading)
            {
                __aoa_profile = compute_profile_bartlett_multithread(h_list,pose_list);
            }
            else
            {
                __aoa_profile = compute_profile_bartlett_singlethread(h_list,pose_list);
            }        
            auto endtime = std::chrono::high_resolution_clock::now();
            float processtime = std::chrono::duration<float, std::milli>(endtime - starttime).count();

            //Stats
            std::pair<std::vector<double>,std::vector<double>> top_N = find_topN();
            __TX_top_N_angles[mac_id_tx[num_tx]] = top_N;
            __paired_pkt_count[mac_id_tx[num_tx]] = csi_timestamp.shape().rows;
            __perf_aoa_profile_cal_time[mac_id_tx[num_tx]] = processtime/1000;
            __memory_used[mac_id_tx[num_tx]] = utils.mem_usage()/1000000;
            __calculated_ts_offset[mac_id_tx[num_tx]] = cal_ts_offset ;
            __rx_pkt_size[mac_id_tx[num_tx]] = data_packets_RX.size();
            __tx_pkt_size[mac_id_tx[num_tx]] = data_packets_TX.size();
        }

        /*Store the aoa_profile*/
        __all_aoa_profiles[mac_id_tx[num_tx]] = __aoa_profile;

        //TODO: get azimuth and elevation from beta_profile
        std::cout << "log [calculate_AOA_profile]: Completed AOA calculation." << std::endl; 
        TX_Neighbor_robot.reset();       
    }
    
    std::cout << "============ WSR module end ==============" << std::endl;
    RX_SAR_robot.reset();
    

    return 0;
}
//=============================================================================================================================
/**
 * Description: Deprecated
 * Input:
 * Output:
nc::NdArray<double>WSR_Module::get_aoa_profile()
{
    return __aoa_profile;
}
**/
//=============================================================================================================================
/**
 * Description: Returns all the AOA profiles calculated for multiple RX
 * Input:
 * Output:
 * */
std::unordered_map<std::string, nc::NdArray<double>>WSR_Module::get_all_aoa_profile()
{
    return __all_aoa_profiles;
}
/**
 * Description: Returns all the AOA profiles calculated for multiple RX
 * Input:
 * Output:
 * */
std::unordered_map<std::string, std::pair<std::vector<double>,std::vector<double>>>WSR_Module::get_TX_topN_angles()
{
    return __TX_top_N_angles;
}

//=============================================================================================================================
/**
 * Description: Deprecated. Use find_topN_phi and find_topN_theta directly.
 * Input:
 * Output:
//=============================================================================================================================
std::pair<double,double> WSR_Module::get_phi_theta()
{
    return std::make_pair(__top_N_phi[0], __top_N_theta[0]);
}
 * */

/**
 * Description: 
 * Input:
 * Output:
 * */
nc::NdArray<double> WSR_Module::compute_profile_bartlett_multithread(
    const nc::NdArray<std::complex<double>>& input_h_list, 
    const nc::NdArray<double>& input_pose_list)
{   
    EigencdMatrix eigen_eterm_3DAdjustment,e_term_prod;
    EigenDoubleMatrix eigen_rep_lambda, eigen_rep_phi, eigen_rep_theta, 
                      e_sin_rep_theta, e_cos_rep_pitch, e_sin_rep_pitch, 
                      e_cos_rep_theta, diff_phi_yaw, e_cos_rep_phi_rep_yaw,
                      eigen_rep_yaw, eigen_rep_pitch, eigen_rep_rho;
    
    int total_packets = input_h_list.shape().rows;
    if(total_packets != input_pose_list.shape().rows){
        THROW_CSI_INVALID_ARGUMENT_ERROR("number of CSI and poses are different.\n");
    }

    int max_packets = total_packets;
    /*Use max packets*/
    if(__FLAG_packet_threshold)
    {
        max_packets = input_h_list.shape().rows > __max_packets_to_process ? __max_packets_to_process : input_h_list.shape().rows;  
    }
    
    nc::NdArray<std::complex<double>> h_list = input_h_list(nc::Slice(0,max_packets), input_h_list.cSlice()); 
    nc::NdArray<double> pose_list = input_pose_list(nc::Slice(0,max_packets), input_pose_list.cSlice());
    
    if(__FLAG_debug) std::cout << "log [compute_AOA] Total packets: "<< total_packets << ", Max packets used: " << max_packets << std::endl;

    std::cout.precision(15);
    nc::NdArray<std::complex<double>> h_list_single_channel;
    
    if(__FLAG_interpolate_phase)
        h_list_single_channel = h_list;
    else
        h_list_single_channel = h_list(h_list.rSlice(),nc::Slice(__snum_start, __snum_end));
    
    auto num_poses = nc::shape(pose_list).rows;
    if(h_list_single_channel.shape().cols == 0)
    {
        THROW_CSI_INVALID_ARGUMENT_ERROR("No subcarrier selected for CSI data.\n");
    }
    
    if(__FLAG_debug) std::cout << "log [compute_AOA] : get lambda, phi and theta values" << std::endl;
        
    std::thread lambda_repmat (&WSR_Module::get_matrix_block, this, 
                                std::ref(eigen_rep_lambda), 
                                std::ref(__precomp__eigen_rep_lambda), __nphi*__ntheta, num_poses);

    std::thread phi_repmat (&WSR_Module::get_matrix_block, this, 
                                std::ref(eigen_rep_phi), 
                                std::ref(__precomp__eigen_rep_phi), __nphi*__ntheta, num_poses);

    std::thread theta_repmat (&WSR_Module::get_matrix_block, this, 
                                std::ref(eigen_rep_theta), 
                                std::ref(__precomp__eigen_rep_theta), __nphi*__ntheta, num_poses);


    if(__FLAG_debug)  std::cout << "log [compute_AOA] : get yaw, pitch and rho values" << std::endl;
    auto pose_x = pose_list(pose_list.rSlice(),0);
    auto pose_y = pose_list(pose_list.rSlice(),1);
    auto pose_z = pose_list(pose_list.rSlice(),2);

    auto yaw_list = nc::arctan2(pose_y,pose_x);
    yaw_list  = nc::angle(-nc::exp(nc::multiply(yaw_list,std::complex<double>(0,1))));
    EigenDoubleMatrix eigen_yaw_list_tmp = EigenDoubleMatrixMap(yaw_list.data(), 
                                                yaw_list.numRows(), 
                                                yaw_list.numCols());
    EigenDoubleMatrix eigen_yaw_list = eigen_yaw_list_tmp.transpose();
    std::thread yaw_repmat (&WSR_Module::get_repmat, this, 
                            std::ref(eigen_rep_yaw), 
                            std::ref(eigen_yaw_list), __nphi*__ntheta, 1);
    // dataPtr = new double[eigen_rep_yaw.rows() * eigen_rep_yaw.cols()];
    // EigenDoubleMatrixMap(dataPtr, eigen_rep_yaw.rows(), eigen_rep_yaw.cols()) = eigen_rep_yaw;
    // auto rep_yaw= nc::NdArray<double>(dataPtr, eigen_rep_yaw.rows(), eigen_rep_yaw.cols(), __takeOwnership);
    // auto rep_yaw = nc::repeat(yaw_list.transpose(),__nphi*__ntheta, 1);
   
    lambda_repmat.join();
 
    auto pitch_list = nc::arctan2(pose_z, nc::sqrt(nc::square(pose_x) + nc::square(pose_y)));
    pitch_list = nc::angle(-nc::exp(nc::multiply(pitch_list,std::complex<double>(0,1))));
    EigenDoubleMatrix eigen_pitch_list_tmp = EigenDoubleMatrixMap(pitch_list.data(), 
                                            pitch_list.numRows(), 
                                            pitch_list.numCols());
    EigenDoubleMatrix eigen_pitch_list = eigen_pitch_list_tmp.transpose();
    std::thread pitch_repmat (&WSR_Module::get_repmat, this, 
                        std::ref(eigen_rep_pitch), 
                        std::ref(eigen_pitch_list), __nphi*__ntheta, 1);
    
    assert(pitch_list.shape()==yaw_list.shape());
    
    phi_repmat.join();

    auto rho_list = nc::sqrt(nc::square(pose_x)+nc::square(pose_y)+nc::square(pose_z));
    EigenDoubleMatrix eigen_rho_list_tmp = EigenDoubleMatrixMap(rho_list.data(), 
                                            rho_list.numRows(), 
                                            rho_list.numCols());
    EigenDoubleMatrix eigen_rho_list = eigen_rho_list_tmp.transpose();
    std::thread rho_repmat (&WSR_Module::get_repmat, this, 
                        std::ref(eigen_rep_rho), 
                        std::ref(eigen_rho_list), __nphi*__ntheta, 1);

    
    //lambda_repmat.join();
    //phi_repmat.join();
    theta_repmat.join();
    yaw_repmat.join();
    pitch_repmat.join();
    rho_repmat.join();

    if(__FLAG_debug) std::cout << "log [compute_AOA] : Element-wise sin and cos for theta,pitch" << std::endl;
    diff_phi_yaw = eigen_rep_phi-eigen_rep_yaw;

    std::thread phi_yaw_cos (&WSR_Module::get_eigen_rep_angle_trig, this, 
                            std::ref(e_cos_rep_phi_rep_yaw), std::ref(diff_phi_yaw), "cos");

    std::thread theta_sin (&WSR_Module::get_eigen_rep_angle_trig, this, 
                            std::ref(e_sin_rep_theta), std::ref(eigen_rep_theta), "sin");
    
    std::thread pitch_cos (&WSR_Module::get_eigen_rep_angle_trig, this, 
                            std::ref(e_cos_rep_pitch), std::ref(eigen_rep_pitch), "cos");

    phi_yaw_cos.join();
    std::thread pitch_sin (&WSR_Module::get_eigen_rep_angle_trig, this, 
                            std::ref(e_sin_rep_pitch), std::ref(eigen_rep_pitch), "sin");

    theta_sin.join();
    std::thread theta_cos (&WSR_Module::get_eigen_rep_angle_trig, this, 
                            std::ref(e_cos_rep_theta), std::ref(eigen_rep_theta), "cos");

    pitch_cos.join();
    if(__FLAG_debug) std::cout << "log [compute_AOA] : calculating eterm_3DAdjustment" << std::endl;
    
    std::thread eterm3D (&WSR_Module::get_eterm_3DAdjustment, this, 
                            std::ref(eigen_eterm_3DAdjustment), std::ref(eigen_rep_lambda));

    //phi_yaw_cos.join();
    //theta_sin.join();
    //pitch_cos.join();
    pitch_sin.join();
    theta_cos.join();
    
    if(__FLAG_debug) std::cout << "log [compute_AOA] : calculating steering vector" << std::endl;
    
    // auto temp1 = e_sin_rep_theta.cwiseProduct(e_cos_rep_pitch);
    EigenDoubleMatrix temp1, temp2, temp3, eigen_bterm_rep2;
    std::thread temp1_cwp (&WSR_Module::get_cwiseProduct, this, std::ref(temp1),
                            std::ref(e_sin_rep_theta), std::ref(e_cos_rep_pitch));
    
    // auto temp2 = e_sin_rep_pitch.cwiseProduct(e_cos_rep_theta);
    std::thread temp2_cwp (&WSR_Module::get_cwiseProduct, this, std::ref(temp2),
                            std::ref(e_sin_rep_pitch), std::ref(e_cos_rep_theta));
    
    temp1_cwp.join();
    temp2_cwp.join();

    auto temp_prod = temp1.cwiseProduct(e_cos_rep_phi_rep_yaw);
    temp3 = temp_prod + temp2;
    eigen_bterm_rep2 = eigen_rep_rho.cwiseProduct(temp3);
    
    if(__FLAG_debug) std::cout << "log [compute_AOA] : calculating e_term product" << std::endl;
    // auto e_term_exp = bterm_rep2*eterm_3DAdjustment;
    eterm3D.join();
    get_cwiseProduct_cd(e_term_prod,eigen_bterm_rep2,eigen_eterm_3DAdjustment); 
        
    if(__FLAG_debug) std::cout << "log [compute_AOA] : calculating eterm elementwise exp" << std::endl;
    // auto e_term = nc::exp(e_term_exp);
    // std::cout << e_term_prod.rows() << "," << e_term_prod.cols() << std::endl; 
    
    EigencdMatrix e_term_exp(__nphi*__ntheta, num_poses);
    
    /* //Does not work on the UP board
    int max_threads = 1;
    int block_rows = __nphi*__ntheta/max_threads;
    std::thread e_term_blk_threads[max_threads];
    std::vector<EigencdMatrix> emat(max_threads);

    for (int itr=0; itr<max_threads;itr++)
    {
       e_term_blk_threads[itr] = std::thread(&WSR_Module::get_block_exp, this, std::ref(emat[itr]),
                           std::ref(e_term_prod), itr*block_rows, 0, block_rows, num_poses);
    }

    for (int itr=0; itr<max_threads;itr++)
    {
       e_term_blk_threads[itr].join();
    }
    
    e_term_exp << emat[0];
    */
    e_term_exp = e_term_prod.array().exp();


    std::complex<double>* cddataPtr = new std::complex<double>[e_term_exp.rows() * e_term_exp.cols()];
    EigencdMatrixMap(cddataPtr, e_term_exp.rows(), e_term_exp.cols()) = e_term_exp;
    auto e_term = nc::NdArray<std::complex<double>>(cddataPtr, e_term_exp.rows(), e_term_exp.cols(), __takeOwnership); 

    if(__FLAG_debug) std::cout << "log [compute_AOA] : getting profile using matmul" << std::endl;

    auto result_mat = nc::matmul(e_term, h_list_single_channel);

    auto final_result_mat = nc::prod(result_mat,nc::Axis::COL);

    auto betaProfileProd = nc::power(nc::abs(final_result_mat),2);
    auto beta_profile = nc::reshape(betaProfileProd,__ntheta,__nphi);
    
    if(__FLAG_normalize_profile)
    {
        auto sum_val = nc::sum(nc::sum(beta_profile)); 
        beta_profile = beta_profile / sum_val(0,0);
    }

    if(__FLAG_debug) std::cout << "log [compute_AOA] : Getting transpose and returning profile" << std::endl;
    beta_profile = nc::transpose((beta_profile));

    return beta_profile;
}
/**
 * Description: 
 * Input:
 * Output:
 * */
nc::NdArray<double> WSR_Module::compute_profile_bartlett_singlethread(
    const nc::NdArray<std::complex<double>>& input_h_list, 
    const nc::NdArray<double>& input_pose_list)
{   
    

    EigencdMatrix eigen_eterm_3DAdjustment, e_term_prod, e_term_exp;
    EigenDoubleMatrix eigen_rep_lambda, eigen_rep_phi, eigen_rep_theta, 
                      e_sin_rep_theta, e_cos_rep_pitch, e_sin_rep_pitch, 
                      e_cos_rep_theta, diff_phi_yaw, e_cos_rep_phi_rep_yaw,
                      eigen_rep_yaw, eigen_rep_pitch, eigen_rep_rho;


    int total_packets = input_h_list.shape().rows;
    if(total_packets != input_pose_list.shape().rows){
        THROW_CSI_INVALID_ARGUMENT_ERROR("number of CSI and poses are different.\n");
    }

    /*Use max packets*/
    int max_packets = total_packets;
    if(__FLAG_packet_threshold)
    {
        max_packets = input_h_list.shape().rows > __max_packets_to_process ? __max_packets_to_process : input_h_list.shape().rows;  
    }
    
    nc::NdArray<std::complex<double>> h_list = input_h_list(nc::Slice(0,max_packets), input_h_list.cSlice()); 
    nc::NdArray<double> pose_list = input_pose_list(nc::Slice(0,max_packets), input_pose_list.cSlice());
    
    if(__FLAG_debug) std::cout << "log [compute_AOA] Total packets: "<< total_packets << ", Max packets used: " << max_packets << std::endl;

    nc::NdArray<std::complex<double>> h_list_single_channel;
    
    if(__FLAG_interpolate_phase)
        h_list_single_channel = h_list;
    else
        h_list_single_channel = h_list(h_list.rSlice(),nc::Slice(__snum_start, __snum_end));

    auto num_poses = nc::shape(pose_list).rows;
    if(h_list_single_channel.shape().cols == 0)
    {
        THROW_CSI_INVALID_ARGUMENT_ERROR("No subcarrier selected for CSI data.\n");
    }
    
    if(__FLAG_debug) std::cout << "log [compute_AOA] : get lambda, phi and theta values" << std::endl;
    
    get_matrix_block(eigen_rep_lambda, __precomp__eigen_rep_lambda, __nphi*__ntheta, num_poses);
    get_matrix_block(eigen_rep_phi, __precomp__eigen_rep_phi, __nphi*__ntheta, num_poses);
    get_matrix_block(eigen_rep_theta, __precomp__eigen_rep_theta, __nphi*__ntheta, num_poses);

    if(__FLAG_debug)  std::cout << "log [compute_AOA] : get yaw, pitch and rho values" << std::endl;
    auto pose_x = pose_list(pose_list.rSlice(),0);
    auto pose_y = pose_list(pose_list.rSlice(),1);
    auto pose_z = pose_list(pose_list.rSlice(),2);

    auto yaw_list = nc::arctan2(pose_y,pose_x);
    yaw_list  = nc::angle(-nc::exp(nc::multiply(yaw_list,std::complex<double>(0,1))));
    EigenDoubleMatrix eigen_yaw_list_tmp = EigenDoubleMatrixMap(yaw_list.data(), 
                                                yaw_list.numRows(), 
                                                yaw_list.numCols());
    EigenDoubleMatrix eigen_yaw_list = eigen_yaw_list_tmp.transpose();
    
    auto pitch_list = nc::arctan2(pose_z, nc::sqrt(nc::square(pose_x) + nc::square(pose_y)));
    pitch_list = nc::angle(-nc::exp(nc::multiply(pitch_list,std::complex<double>(0,1))));
    EigenDoubleMatrix eigen_pitch_list_tmp = EigenDoubleMatrixMap(pitch_list.data(), 
                                            pitch_list.numRows(), 
                                            pitch_list.numCols());
    EigenDoubleMatrix eigen_pitch_list = eigen_pitch_list_tmp.transpose();
    
    
    auto rho_list = nc::sqrt(nc::square(pose_x)+nc::square(pose_y)+nc::square(pose_z));
    EigenDoubleMatrix eigen_rho_list_tmp = EigenDoubleMatrixMap(rho_list.data(), 
                                            rho_list.numRows(), 
                                            rho_list.numCols());
    EigenDoubleMatrix eigen_rho_list = eigen_rho_list_tmp.transpose();
    
    
    assert(pitch_list.shape()==yaw_list.shape());
    get_repmat(eigen_rep_yaw, eigen_yaw_list, __nphi*__ntheta, 1);
    get_repmat(eigen_rep_pitch, eigen_pitch_list, __nphi*__ntheta, 1);
    get_repmat(eigen_rep_rho, eigen_rho_list, __nphi*__ntheta, 1);


    if(__FLAG_debug) std::cout << "log [compute_AOA] : Element-wise sin and cos for theta,pitch" << std::endl;
    
    diff_phi_yaw = eigen_rep_phi-eigen_rep_yaw;
    get_eigen_rep_angle_trig(e_cos_rep_phi_rep_yaw,diff_phi_yaw,"cos");
    get_eigen_rep_angle_trig(e_sin_rep_theta,eigen_rep_theta,"sin");
    get_eigen_rep_angle_trig(e_cos_rep_pitch,eigen_rep_pitch,"cos");
    get_eigen_rep_angle_trig(e_sin_rep_pitch,eigen_rep_pitch,"sin");
    get_eigen_rep_angle_trig(e_cos_rep_theta,eigen_rep_theta,"cos");
    
    if(__FLAG_debug) std::cout << "log [compute_AOA] : calculating eterm_3DAdjustment" << std::endl;
    get_eterm_3DAdjustment(eigen_eterm_3DAdjustment,eigen_rep_lambda);

    if(__FLAG_debug) std::cout << "log [compute_AOA] : calculating steering vector" << std::endl;
    EigenDoubleMatrix temp1, temp2, temp3, eigen_bterm_rep2, temp_prod;
    get_cwiseProduct(temp1, e_sin_rep_theta, e_cos_rep_pitch);
    get_cwiseProduct(temp2, e_sin_rep_pitch, e_cos_rep_theta);
    get_cwiseProduct(temp_prod, temp1, e_cos_rep_phi_rep_yaw);
    temp3 = temp_prod + temp2;
    get_cwiseProduct(eigen_bterm_rep2, eigen_rep_rho,temp3);


    if(__FLAG_debug) std::cout << "log [compute_AOA] : calculating e_term product" << std::endl;
    get_cwiseProduct_cd(e_term_prod,eigen_bterm_rep2,eigen_eterm_3DAdjustment);
        
    if(__FLAG_debug) std::cout << "log [compute_AOA] : calculating eterm elementwise exp" << std::endl;
    e_term_exp = e_term_prod.array().exp();

    std::complex<double>* cddataPtr = new std::complex<double>[e_term_exp.rows() * e_term_exp.cols()];
    EigencdMatrixMap(cddataPtr, e_term_exp.rows(), e_term_exp.cols()) = e_term_exp;
    auto e_term = nc::NdArray<std::complex<double>>(cddataPtr, e_term_exp.rows(), e_term_exp.cols(), __takeOwnership); 

    if(__FLAG_debug) std::cout << "log [compute_AOA] : getting profile using matmul" << std::endl;

    auto result_mat = nc::matmul(e_term, h_list_single_channel);
    auto final_result_mat = nc::prod(result_mat,nc::Axis::COL);
    auto betaProfileProd = nc::power(nc::abs(final_result_mat),2);
    auto beta_profile = nc::reshape(betaProfileProd,__ntheta,__nphi);
    
    if(__FLAG_normalize_profile)
    {
        auto sum_val = nc::sum(nc::sum(beta_profile)); 
        beta_profile = beta_profile / sum_val(0,0);
    }

    if(__FLAG_debug) std::cout << "log [compute_AOA] : Getting transpose and returning profile" << std::endl;
    beta_profile = nc::transpose((beta_profile));
    
    return beta_profile;
}
//=============================================================================================================================
/**
 * Description: 
 * Input:
 * Output:
 * */
void WSR_Module::get_block_exp(EigencdMatrix& output, 
                                EigencdMatrix& input,
                                int start, int end, int rows, int cols)
{
    output = input.block(start,end,rows,cols).array().exp();
}
//=============================================================================================================================
/**
 * Description: 
 * Input:
 * Output:
 * */
void WSR_Module::get_eterm_3DAdjustment(EigencdMatrix& output, 
                                        EigenDoubleMatrix& input)
{
    output = -4.0*std::complex<double>(0,1)*M_PI/input.array();
} 
//=============================================================================================================================
/**
 * Description: 
 * Input:
 * Output:
 * */
void WSR_Module::get_matrix_block(EigenDoubleMatrix& output, 
                                  EigenDoubleMatrix& input,
                                  int rows, int cols)
{
    output = input.block(0,0,rows,cols);
} 
//=============================================================================================================================
/**
 * Description: 
 * Input:
 * Output:
 * */
void WSR_Module::get_cwiseProduct(EigenDoubleMatrix& output, 
                                  EigenDoubleMatrix& input1,
                                  EigenDoubleMatrix& input2)
{
    output = input1.cwiseProduct(input2);
}
//=============================================================================================================================
/**
 * Description: 
 * Input:
 * Output:
 * */
void WSR_Module::get_cwiseProduct_cd(EigencdMatrix& output, 
                                  EigenDoubleMatrix& input1,
                                  EigencdMatrix& input2)
{
    output = input1.cwiseProduct(input2);
}                            
//=============================================================================================================================
/**
 * Description: 
 * Input:
 * Output:
 * */
void WSR_Module::get_repmat(EigenDoubleMatrix& output, 
                            EigenDoubleMatrix& input,
                            int rows, int cols)
{
    output = input.replicate(rows,cols);
}
//=============================================================================================================================
/**
 * Description: 
 * Input:
 * Output:
 * */
void WSR_Module::get_eigen_rep_angle_trig(EigenDoubleMatrix& output, 
                                        EigenDoubleMatrix& input,
                                        std::string trig_operation)
{
    if(trig_operation == "sin")
        output = input.array().sin();
    else
        output = input.array().cos();
}
//=============================================================================================================================
/**
 * Description: 
 * Input:
 * Output:
std::vector<double> WSR_Module::find_topN_phi(nc::NdArray<double> profile)
{
    std::vector<double> ret;
    nc::NdArray<double> phi_max = nc::amax(profile, nc::Axis::COL);
    nc::NdArray<nc::uint32> sortedIdxs = argsort(phi_max);

    int arr_idx = phi_max.shape().cols - 1;
    for(int i=0; i<__topN_phi_count;i++)
    {
        if (__FLAG_debug) std::cout << "log [calculate_AOA_profile] Top azimuth angle " << i 
                                    << " : " << phi_list(0,sortedIdxs(0,arr_idx-i))*180/M_PI << std::endl;
        ret.push_back(phi_list(0,sortedIdxs(0,arr_idx-i))*180/M_PI);
    }

    return ret;
}
* */
//=============================================================================================================================
/**
 * Description: Deprecated
 * Input:
 * Output:
std::vector<double> WSR_Module::find_topN_theta(nc::NdArray<double> profile)
{
    std::vector<double> ret;
    nc::NdArray<double> theta_max = nc::amax(profile, nc::Axis::ROW);
    nc::NdArray<nc::uint32> sortedIdxs = argsort(theta_max);

    int arr_idx = theta_max.shape().cols - 1;
    for(int i=0; i<__topN_theta_count;i++)
    {
        if (__FLAG_debug) std::cout << "log [calculate_AOA_profile] Top elevation angle " << i << " : "
                                     << 180 - theta_list(0,sortedIdxs(0,arr_idx-i))*180/M_PI << std::endl;
        ret.push_back(180 - theta_list(0,sortedIdxs(0,arr_idx-i))*180/M_PI);
    }

    return ret;
}
 * */
//=============================================================================================================================
/**
 * Description:
 * Input:
 * Output:
 * */
std::pair<std::vector<double>,std::vector<double>> WSR_Module::find_topN()
{

    std::vector<double> ret_phi, ret_theta;
    nc::NdArray <double> max_peak = nc::amax(__aoa_profile);

    nc::NdArray<double> phi_max = nc::amax(__aoa_profile, nc::Axis::COL);
//    nc::NdArray<nc::uint32> sortedIdxs_phi = argsort(phi_max); //ascending order
//    int arr_idx_phi = phi_max.shape().cols - 1;
//
//    nc::NdArray<double> theta_max = nc::amax(__aoa_profile, nc::Axis::ROW);
//    nc::NdArray<nc::uint32> sortedIdxs_theta = argsort(theta_max);
//    int arr_idx_theta = theta_max.shape().cols - 1;
//
//    int phi_idx=sortedIdxs_phi(0,arr_idx_phi);
//    int theta_idx = sortedIdxs_theta(0,arr_idx_theta);
//    int itr=1, i_phi=1, i_theta=1;
    int peak_ind = 1;
    bool check_peak = false;

    //Track indices of peaks which are stored or have been skipped
//    auto phi_indexes_stored = nc::zeros<double>(1, 360);
//    auto theta_indexes_stored = nc::zeros<double>(1, 180);
    auto all_idx_flat = nc::flip(nc::argsort((__aoa_profile.flatten())));
    //n*2 array save coordinates for sorted value in the whole profile
    nc::NdArray<int> sorted_inds(all_idx_flat.size(), 2);
    for(int i = 0; i < all_idx_flat.size(); i++){
        sorted_inds.put(i, sorted_inds.cSlice(), utils.unravel_index(all_idx_flat(0, i), __nphi,__ntheta));
    }
    int phi_idx = sorted_inds(0, 0);
    int theta_idx = sorted_inds(0, 1);

    //Get the peak AOA
    if (__FLAG_debug) std::cout << "log [calculate_AOA_profile] Top azimuth angle " << peak_ind
                                << " : " << phi_list(0,phi_idx)*180/M_PI << std::endl;
    ret_phi.push_back(phi_list(0,phi_idx)*180/M_PI);

    if (__FLAG_debug) std::cout << "log [calculate_AOA_profile] Top elevation angle " << peak_ind << " : "
                                << 180 - theta_list(0,theta_idx)*180/M_PI << std::endl;
    ret_theta.push_back(180 - theta_list(0,theta_idx)*180/M_PI);
    double confidence = get_confidence(phi_idx,theta_idx);
//    double confidence = 0;
    __aoa_confidence.push_back(confidence);

    if (__FLAG_debug) std::cout << "log [calculate_AOA_profile] confidence " << peak_ind << " : " << confidence << std::endl;
//    phi_indexes_stored(0,phi_idx) = 1;
//    theta_indexes_stored(0,theta_idx) = 1;

    // 2d matrix to label the searched index
    auto ind_profile = nc::zeros<bool>(__nphi,__ntheta);

    // radius to define local maxima
    const int radius = __peak_radius;
    // Assign true to all neighbor region
    for(int i = -radius; i < radius; i++) {
        int tmp_row=0, tmp_col=0;
        if(phi_idx+i<0)
            tmp_row = __nphi +i;
        else if(phi_idx + i>=__nphi)
            tmp_row = (phi_idx +i)% __nphi;
        else
            tmp_row = i + phi_idx;
        for(int j = -radius; j < radius; j++){
            if(theta_idx+j<0)
                tmp_row = __ntheta +j;
            else if(theta_idx + j>=__ntheta)
                tmp_row = (__ntheta +j)% __ntheta;
            else
                tmp_col = theta_idx + j;
            ind_profile(tmp_row, tmp_col) = true;
        }
    }



    //find other peaks
//    while((itr <__topN_phi_count) && (i_phi < 360))
    //Iterate all entries in the 2d matrix

    for(int itr = 1; itr <= all_idx_flat.size()&&peak_ind < _topN_count;itr++) {

//        phi_idx = sortedIdxs_phi(0,arr_idx_phi-i_phi);
//        theta_idx = sortedIdxs_theta(0,arr_idx_theta-i_theta); //NJ: Potential bug here since size of the two arrays is different
        phi_idx = sorted_inds(itr, 0);
        theta_idx = sorted_inds(itr, 1);
        float relative_peak_magnitude = 100 * (std::pow(max_peak(0, 0), 2) -
                                               std::pow(__aoa_profile(phi_idx, theta_idx), 2)) /
                                        std::pow(max_peak(0, 0), 2);

        //check if this is 1-unit near to other peaks already obtained
//        check_peak = (phi_idx+1  < 360 && phi_indexes_stored(0,phi_idx+1)   == 0) &&
//                     (phi_idx-1   > 0   && phi_indexes_stored(0,phi_idx-1)   == 0) &&
//                     (theta_idx+1 < 180 && theta_indexes_stored(0,theta_idx+1) == 0) &&
//                     (theta_idx-1 > 0   && theta_indexes_stored(0,theta_idx-1) == 0) &&
//                     (relative_peak_magnitude >= 40);
        if(relative_peak_magnitude >= 40)
            check_peak = true;
        else
            check_peak = false;
        for (int i = -radius; i < radius && check_peak; i++) {
            int tmp_row=0, tmp_col=0;
            if (phi_idx + i < 0)
                tmp_row = __nphi + i;
            else if (phi_idx + i >= __nphi)
                tmp_row = (phi_idx + i) % __nphi;
            else
                tmp_row = phi_idx + i;
            for (int j = -radius; j < radius && check_peak; j++) {
                if (theta_idx + j < 0)
                    tmp_row = __ntheta + j;
                else if (theta_idx + j >= __ntheta)
                    tmp_row = (__ntheta + j) % __ntheta;
                else
                    tmp_col = theta_idx + j;
                //if the nearby pos is checked before or value is greater than current peak value
                if (ind_profile(tmp_row, tmp_col) ||
                    __aoa_profile(tmp_col, tmp_row) > __aoa_profile(phi_idx, theta_idx) ) {
                    check_peak = false;
                    break;
                }
            }
        }
//
        if (check_peak) {
            peak_ind++;
            if (__FLAG_debug)
                std::cout << "log [calculate_AOA_profile] Top azimuth angle " << peak_ind
                          << " : " << phi_list(0, phi_idx) * 180 / M_PI << std::endl;
            ret_phi.push_back(phi_list(0, phi_idx) * 180 / M_PI);
            if (__FLAG_debug)
                std::cout << "log [calculate_AOA_profile] Top elevation angle " << peak_ind << " : "
                          << 180 - theta_list(0, theta_idx) * 180 / M_PI << std::endl;
            ret_theta.push_back(180 - theta_list(0, theta_idx) * 180 / M_PI);
            confidence = get_confidence(phi_idx, theta_idx);
            if (__FLAG_debug)
                std::cout << "log [calculate_AOA_profile] confidence" << peak_ind << " : " << confidence << std::endl;
            __aoa_confidence.push_back(confidence);
//            phi_indexes_stored(0,phi_idx) = 1;
//            theta_indexes_stored(0,theta_idx) = 1;
            for (int i = -radius; i < radius; i++) {
                int tmp_row=0, tmp_col=0;
                if (phi_idx + i < 0)
                    tmp_row = __nphi + i;
                else if (phi_idx + i >= __nphi)
                    tmp_row = (phi_idx + i) % __nphi;
                else
                    tmp_row = phi_idx + i;
                for (int j = -radius; j < radius; j++) {
                    if (theta_idx + j < 0)
                        tmp_row = __ntheta + j;
                    else if (theta_idx + j >= __ntheta)
                        tmp_row = (__ntheta + j) % __ntheta;
                    else
                        tmp_col = theta_idx + j;
                    //if the nearby pos is checked before or value is greater than current peak value
                    ind_profile(tmp_row, tmp_col) = true;
                }
            }
        }
    }

//        i_phi++;
//        i_theta++;

    return std::make_pair(ret_phi, ret_theta);
}

double WSR_Module::get_confidence(double phi_ind, double theta_ind) {
    double f = __aoa_profile(phi_ind,theta_ind);
    double sumf = __aoa_profile.sum()(0,0);
    double sigma_f = 0, sigma_n = 0;
    for(size_t ind = 0; ind<=__nphi; ind++){
        for(size_t ind_c = 0; ind_c < __ntheta;ind_c++){
            sigma_f += abs((ind-phi_ind))*abs((ind_c-theta_ind))*__aoa_profile(phi_ind,theta_ind)/sumf;
            sigma_n += abs((ind-phi_ind))*abs((ind_c-theta_ind))*sumf/(__ntheta*__nphi);
        }
    }
    return sigma_f/sigma_n;


}

//=============================================================================================================================
/**
 *
 *
 * */
std::vector<double> WSR_Module::get_aoa_error(const std::pair<std::vector<double>,std::vector<double>>& topN_AOA,
                                              std::pair<double,double> groundtruth_angles,
                                              const string& traj_type)
{
    std::vector<std::pair<double,double>> true_aoa_angles;
    std::vector<double> topN_phi = topN_AOA.first;
    std::vector<double> topN_theta = topN_AOA.second;
    std::vector<double> aoa_error_metrics;

    double true_phi= groundtruth_angles.first;
    double true_theta=groundtruth_angles.second;
    double min_aoa_error=1000, err=0, phi_error=0, theta_error=0;
    double min_phi_error =0, min_theta_error=0;
    double closest_phi=0, closest_theta=0, closest_confidence=0;

    for(int i=0; i<topN_phi.size(); i++)
    {
        //Squared error (as per WSR IJRR paper)
        // phi_error = topN_phi[i]-true_phi;
        // theta_error = topN_theta[i]-true_theta;
        phi_error = utils.anglediff(true_phi, topN_phi[i]);
        theta_error = utils.anglediff(true_theta, topN_theta[i]);
        if(traj_type == "3D")
        {
            err = std::sqrt( std::pow(phi_error,2) + std::pow(theta_error,2));
        }
        else
        {
            err = phi_error;
        }


        if(abs(min_aoa_error) > abs(err))
        {
            min_aoa_error = err;
            min_phi_error = phi_error;
            min_theta_error = theta_error;
            closest_phi = topN_phi[i];
            closest_theta = topN_theta[i];
            closest_confidence = __aoa_confidence[i];
        }
    }

    //Store the error values to the closest AOA peak
    aoa_error_metrics.push_back(closest_phi);
    aoa_error_metrics.push_back(closest_theta);
    aoa_error_metrics.push_back(closest_confidence);
    aoa_error_metrics.push_back(min_aoa_error);
    aoa_error_metrics.push_back(min_phi_error);
    aoa_error_metrics.push_back(min_theta_error);

    return aoa_error_metrics;
}
//=============================================================================================================================
/**
 *
 *
 * */
std::vector<double> WSR_Module::top_aoa_error(double phi, double theta,
                                              std::pair<double,double> groundtruth_angles,
                                              const string& traj_type)
{
    std::vector<double> ret;
    double phi_error = utils.anglediff(groundtruth_angles.first,phi);
    double theta_error = utils.anglediff(groundtruth_angles.second,theta);
    double err = 0;
    if(traj_type == "3D")
    {
        err = std::sqrt( std::pow(phi_error,2) + std::pow(theta_error,2));
    }
    else
    {
        err = phi_error;
    }

    ret.push_back(phi);
    ret.push_back(theta);
    ret.push_back(__aoa_confidence[0]); //TODO change the format
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
int WSR_Module::get_tx_pkt_count(const std::string& tx_mac_id) {
    return __tx_pkt_size[tx_mac_id];
}
//=============================================================================================================================
/**
 *
 *
 * */
int WSR_Module::get_rx_pkt_count(const std::string& tx_mac_id) {
    return __rx_pkt_size[tx_mac_id];
}
//=============================================================================================================================
/**
 *
 *
 * */
double WSR_Module::get_processing_time(const std::string& tx_mac_id) {
    return __perf_aoa_profile_cal_time[tx_mac_id];
}
//=============================================================================================================================
/**
 *
 *
 * */
double WSR_Module::get_memory_used(const std::string& tx_mac_id) {
    return __memory_used[tx_mac_id];
}
//=============================================================================================================================
/**
 *
 *
 * */
double WSR_Module::get_calculated_ts_offset(const std::string& tx_mac_id) {
    return __calculated_ts_offset[tx_mac_id];
}
//=============================================================================================================================
/**
 *
 *
 * */
int WSR_Module::get_paired_pkt_count(const std::string& tx_mac_id) {
    return __paired_pkt_count[tx_mac_id];
}
//=============================================================================================================================
/**
 *
 *
 * */
nlohmann::json WSR_Module::get_stats(double true_phi,
                           double true_theta,
                           std::vector<double>& top_aoa_error,
                           std::vector<double>& closest_AOA_error,
                           const std::string& tx_mac_id,
                           const std::string& tx_name)
{
    nlohmann::json output_stats = {
            {"a_Info_TX",{
                {"TruePhi",true_phi},
                {"Truetheta",true_theta},
                {"TX_Name",tx_name},
                {"TX_MAC_ID",tx_mac_id}
            }},
            {"b_INFO_Profile_Resolution",{
                {"nphi",__nphi},
                {"ntheta",__ntheta},
            }},
            {"e_INFO_Performance",
             {
                {"Forward_channel_packets", get_tx_pkt_count(tx_mac_id)},
                {"Reverse_channel_packets", get_rx_pkt_count(tx_mac_id)},
                {"Packets_Used", get_paired_pkt_count(tx_mac_id)},
                {"time(sec)", get_processing_time(tx_mac_id)},
                {"memory(GB)", get_memory_used(tx_mac_id)},
                {"first_forward-reverse_ts_offset(sec)", get_calculated_ts_offset(tx_mac_id)}
             }},
            {"c_Info_AOA_Top",{
                {"Phi(deg)", top_aoa_error[0]},
                {"Theta(deg)", top_aoa_error[1]},
                {"Confidence", top_aoa_error[2]},
                {"Total_AOA_Error(deg)", top_aoa_error[3]},
                {"Phi_Error(deg)", top_aoa_error[4]},
                {"Theta_Error(deg)", top_aoa_error[5]}
            }},
            {"d_Info_AOA_Closest",{
                {"Phi(deg)", closest_AOA_error[0]},
                {"Theta(deg)", closest_AOA_error[1]},
                {"Confidence", closest_AOA_error[2]},
                {"Total_AOA_Error(deg)", closest_AOA_error[3]},
                {"Phi_Error(deg)", closest_AOA_error[4]},
                {"Theta_Error(deg)", closest_AOA_error[5]}
            }}
    };

    return output_stats;
}
