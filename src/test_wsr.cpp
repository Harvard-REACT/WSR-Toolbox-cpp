#include "csitoolbox/WSR_Module.h"
#include <unistd.h>
#include <sys/types.h>
#include <unordered_map>
#include <chrono>

int main(int argc, char *argv[]){
    
    WSR_Util utils;
    // struct passwd *pw = getpwuid(getuid());
    // std::string homedir = pw->pw_dir;
    // std::string folder = "";
    // std::string config = utils.__homedir+"/catkin_ws/src/csitoolbox/config/config_3D_SAR.json";
    string traj_type = argv[1];
    std::cout << "Processing trajectory type: " << traj_type << std::endl;
    std::string config = "../config/config_rx0.json";
    WSR_Module run_module(config); // TODO: How can this be initialized only once without hardcoding config fn? maybe use if else?
    
    // omp_set_num_threads(64);
    // std::cout << "threads = " << omp_get_num_threads() << endl;
    
    //Fetch files from remote RX and TX 
    // std::string csi_collect_cmd = homedir+"/catkin_ws/src/wsr_ros/scripts/collect_csi.sh" + rx_details + 
    //                             tx_details + " " + tx2_details + " " +tx3_details +" "+ tx4_details+" "+tx5_details;

    //RX_SAR_Robot: performs 3D SAR
    //TX_SAR_Robot(s): Neighboring robots for which RX_SAR_robot calculates AOA

    /*================== Process RX_SAR_Robot files ====================*/
    std::string reverse_csi = run_module.__precompute_config["input_RX_channel_csi_fn"]["value"]["csi_fn"].dump();
    std::string trajectory_file_rx;
    if(traj_type == "gt")
        trajectory_file_rx = run_module.__precompute_config["input_trajectory_csv_fn_rx"]["value"].dump();
    else if(traj_type == "t265")
        trajectory_file_rx = run_module.__precompute_config["input_trajectory_csv_fn_rx_t265"]["value"].dump();
    else if(traj_type == "odom")
        trajectory_file_rx = run_module.__precompute_config["input_trajectory_csv_fn_rx_odom"]["value"].dump();

    
    std::string output = run_module.__precompute_config["output_aoa_profile_path"]["value"].dump();
    bool __Flag_get_mean_pos = bool(run_module.__precompute_config["get_mean_pose_RX"]["value"]);

    //Remove all double-quote characters
    reverse_csi.erase(remove( reverse_csi.begin(), reverse_csi.end(), '\"' ),reverse_csi.end());
    trajectory_file_rx.erase(remove( trajectory_file_rx.begin(), trajectory_file_rx.end(), '\"' ),trajectory_file_rx.end());
    output.erase(remove( output.begin(), output.end(), '\"' ),output.end());

    std::string rx_robot_csi = utils.__homedir + reverse_csi;
    std::string traj_fn_rx = utils.__homedir + trajectory_file_rx;
    // std::string true_traj_fn_rx = utils.__homedir + true_traj_pre + *ts_it + "_.csv";
    std::vector<std::vector<double>> trajectory_rx = utils.loadTrajFromCSV(traj_fn_rx); //Robot performing SAR
    // std::vector<std::vector<double>> true_trajectory_rx = utils.loadTrajFromCSV(true_traj_fn_rx); //Robot performing SAR
    nc::NdArray<double> displacement;
    nc::NdArray<double> trajectory_timestamp;
    
    /*============= Process the TX_SAR_Robot files =======================*/
    std::unordered_map<std::string,std::string> tx_robot_csi;

    for (auto it = run_module.__precompute_config["input_TX_channel_csi_fn"]["value"].begin(); 
    it != run_module.__precompute_config["input_TX_channel_csi_fn"]["value"].end(); ++it)
    {
        const string& tx_name =  it.key();
        auto temp =  it.value();
        string tx_mac_id = temp["mac_id"];
        string csi_data_file = temp["csi_fn"]; 
        csi_data_file.erase(remove( csi_data_file.begin(), csi_data_file.end(), '\"' ),csi_data_file.end());
        tx_robot_csi[tx_mac_id] = utils.__homedir + csi_data_file;
        
        //get timestamp for using to store AOA profile
        std::string csi_name,ts,time_val,date_val; 
        stringstream tokenize_string1(tx_robot_csi[tx_mac_id]); 
        while(getline(tokenize_string1, csi_name, '/'));
        stringstream tokenize_string2(csi_name);
        int count = 0;
        while(getline(tokenize_string2, ts, '_'))
        {
        if(count == 2) date_val = ts;
        count++;
        }
        stringstream tokenize_string3(ts);
        getline(tokenize_string3, time_val, '.');
        run_module.data_sample_ts[tx_mac_id] = date_val +"_"+ time_val;
        run_module.tx_name_list[tx_mac_id] = tx_name;
    }

    //load trajectory
    std::vector<std::vector<double>> trajectory_tx;

    //Check if moving ends
    if(bool(run_module.__precompute_config["use_relative_trajectory"]["value"]))
    {
      std::string trajectory_file_tx = run_module.__precompute_config["input_trajectory_csv_fn_tx"]["value"].dump();
      trajectory_file_tx.erase(remove( trajectory_file_tx.begin(), trajectory_file_tx.end(), '\"' ),trajectory_file_tx.end());
      std::string traj_fn_tx = utils.__homedir + trajectory_file_tx;
      trajectory_tx = utils.loadTrajFromCSV(traj_fn_tx);
    }
    std::cout << "log [WSR_Module]: Preprocessing Displacement " << std::endl;
    
    std::vector<double> antenna_offset, antenna_offset_true;
    antenna_offset_true = run_module.__precompute_config["antenna_position_offset"]["mocap_offset"].get<std::vector<double>>(); 

    if (traj_type == "gt")
    antenna_offset = antenna_offset_true;
    else if (traj_type == "t265")
    antenna_offset = run_module.__precompute_config["antenna_position_offset"]["t265_offset"].get<std::vector<double>>();
    else if (traj_type == "odom")
    antenna_offset = run_module.__precompute_config["antenna_position_offset"]["odom_offset"].get<std::vector<double>>();
    

    std::cout << "log [WSR_Module]: Got offset " << std::endl;
    nc::NdArray<double> pos,true_pos;
    //Get relative trajectory if moving ends
    if(bool(run_module.__precompute_config["use_relative_trajectory"]["value"]))
    {          
        //get relative trajectory
        auto return_val = utils.getRelativeTrajectory(trajectory_rx,trajectory_tx,antenna_offset,__Flag_get_mean_pos,true);
        trajectory_timestamp = return_val.first;
        displacement = return_val.second;
    }
    else
    {
        auto return_val = utils.formatTrajectory_v2(trajectory_rx,antenna_offset,pos,__Flag_get_mean_pos,true);
        // auto true_return_val = utils.formatTrajectory_v2(true_trajectory_rx,antenna_offset_true,true_pos,__Flag_get_mean_pos,true);
        trajectory_timestamp = return_val.first;
        displacement = return_val.second;
    }

    //Get all True AOA angles
    nlohmann::json true_positions_tx = run_module.__precompute_config["true_tx_positions"];
    auto all_true_AOA = utils.get_true_aoa(trajectory_rx, true_positions_tx); //Fix this when using moving ends.

    run_module.calculate_AOA_profile(rx_robot_csi,tx_robot_csi,displacement,trajectory_timestamp);
    auto all_aoa_profile = run_module.get_all_aoa_profile();
    auto all_topN_angles = run_module.get_TX_topN_angles();
    auto all_confidences = run_module.get_all_confidence();
    string trajType = run_module.__precompute_config["trajectory_type"]["value"];
    double true_phi, true_theta;

    std::cout << "Getting AOA profile stats for TX Neighbor robots" << std::endl;
    for(auto & itr : all_aoa_profile)
    {
        std::cout << "-----------------------------" << std::endl;
        
        std::string tx_id = itr.first;
        std::string ts = run_module.data_sample_ts[tx_id];
        auto profile = itr.second;
        std::vector<double> aoa_confidence = all_confidences[tx_id];

        if(profile.shape().rows == 1) 
        {
            //Dummy profile, error in AOA calculation
            std::cout << "Phi angle = 0" <<  std::endl;
            std::cout << "Theta angle = 0" << std::endl;
        }
        else
        {
            string profile_op_fn = utils.__homedir+output+"/"+run_module.tx_name_list[tx_id]+"_aoa_profile_"+ts+".csv";
            utils.writeToFile(profile,profile_op_fn);

            true_phi = all_true_AOA[run_module.tx_name_list[tx_id]].first;
            true_theta = all_true_AOA[run_module.tx_name_list[tx_id]].second;

            auto topN_angles = all_topN_angles[tx_id];

            std::vector<std::vector<float>> aoa_error = run_module.get_aoa_error(topN_angles,
                                                                                all_true_AOA[run_module.tx_name_list[tx_id]],
                                                                                trajType);
            
            auto stats = run_module.get_stats(true_phi, true_theta, aoa_error,
                                                tx_id, run_module.tx_name_list[tx_id],
                                                pos,pos,true_positions_tx,1); // Only estimatd pos used here. True_pos is used only when compiling aggregate results.
            //Display output
            std::cout << stats.dump(4) << std::endl;

            if(run_module.__precompute_config["debug"]["value"])
            {
                std::string viz_op = "../scripts/viz_data.sh ~/" + run_module.__precompute_config["debug_dir"]["value"].dump() + " " + run_module.tx_name_list[tx_id];
                system(viz_op.c_str());
            }
        }
    }
}