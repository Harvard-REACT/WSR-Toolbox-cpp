#include "csitoolbox/WSR_Module.h"
#include <unistd.h>
#include <sys/types.h>
#include <unordered_map>
#include <chrono>
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;

int main(int argc, char *argv[])
{
    WSR_Util utils;

    std::vector<string> csi_tx;
    std::list<std::string> mylist;
    std::set<string> timesset;
    std::list<string> timelist;

    string foldername = argv[1];
    string traj_type = argv[2];
    string rx_csi_pre = "csi_rx_";
    string tx_csi_pre = "csi_";
    string traj_pre; 
    
    if (traj_type == "odom")
        traj_pre = "odom_rx_trajectory_";
    else if (traj_type == "t265")
        traj_pre = "t265_rx_trajectory_";
    else if (traj_type == "gt")
        traj_pre = "rx_trajectory_";
    
    string timestmp = "";
    string grountruth_pos_fn = foldername+"/"+"groundtruth_positions.json";
    string csi_fn = "csi";

    //Read all unique timestamps
    for (const auto & entry : fs::directory_iterator(foldername)) 
    {   
        if(entry.path().string().find(csi_fn) == std::string::npos) continue;
        auto end_idx = entry.path().string().rfind(".");
        auto start_idx = end_idx - 17;
        timestmp = entry.path().string().substr(start_idx, 17);
        bool found = (std::find(timelist.begin(), timelist.end(), timestmp) != timelist.end());
        if(!found){
            timelist.push_back(timestmp);
        }
    }
    timelist.sort();

    //Get the files with the latest timestamp
    string latest_ts = timelist.back();
     
    std::cout << "***********************************************************************************" << std::endl;
    std::cout << "Processing data for RX_SAR_robot : " << rx_csi_pre + latest_ts + ".dat" << std::endl;
    std::cout << "***********************************************************************************" << std::endl;
    // std::string config = utils.__homedir+"/catkin_ws/src/csitoolbox/config/config_3D_SAR.json";
    std::string config = "../config/config_3D_SAR.json";
    WSR_Module run_module(config);

    std::string output = run_module.__precompute_config["output_aoa_profile_path"]["value"].dump();
    output.erase(remove( output.begin(), output.end(), '\"' ),output.end());
    std::string rx_robot_csi = foldername + "/" + rx_csi_pre + latest_ts + ".dat";
    std::string traj_fn_rx = foldername + "/" + traj_pre + latest_ts + "_.csv";
    std::cout << "Trajectory fn" << traj_fn_rx << std::endl;      
    std::vector<std::vector<double>> trajectory_rx = utils.loadTrajFromCSV(traj_fn_rx); //Robot performing SAR
    nc::NdArray<double> displacement;
    nc::NdArray<double> trajectory_timestamp;
    std::unordered_map<std::string,std::string> tx_robot_csi;
    std::unordered_map<std::string,std::string> tx_profile_timestamp;
    

    //Get the CSI data files for TX_Neighbor_robot
    for (auto it = run_module.__precompute_config["input_TX_channel_csi_fn"]["value"].begin(); 
        it != run_module.__precompute_config["input_TX_channel_csi_fn"]["value"].end(); ++it)
    {
        const string& tx_name =  it.key();
        auto temp =  it.value();
        string tx_mac_id = temp["mac_id"];
        string csi_data_file = foldername + "/" + tx_csi_pre + tx_name + "_" + latest_ts + ".dat";
        tx_robot_csi[tx_mac_id] = csi_data_file;

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
        tx_profile_timestamp[tx_mac_id] = date_val +"_"+ time_val;
        run_module.tx_name_list[tx_mac_id] = tx_name;
        run_module.data_sample_ts[tx_mac_id] = latest_ts;
    }

    //load trajectory
    std::vector<std::vector<double>> trajectory_tx;
    nc::NdArray<double> mean_pos;
    std::cout << "log [WSR_Module]: Preprocessing Trajectory " << std::endl;
    std::cout << "log [WSR_Module]: Preprocessing Trajectory " << std::endl;
    std::vector<double> antenna_offset;
    if (traj_type == "odom")
        antenna_offset = run_module.__precompute_config["antenna_position_offset"]["mocap_offset"].get<std::vector<double>>();
    else if (traj_type == "t265")
        antenna_offset = run_module.__precompute_config["antenna_position_offset"]["t265_offset"].get<std::vector<double>>();
    else if (traj_type == "gt")
        antenna_offset = run_module.__precompute_config["antenna_position_offset"]["odom_offset"].get<std::vector<double>>();
    
    auto return_val = utils.formatTrajectory_v2(trajectory_rx,antenna_offset,mean_pos);
    trajectory_timestamp = return_val.first;
    displacement = return_val.second;


    //Read TX_neighbor_robot groundtruth position and Get all True AOA angles
    std::ifstream pos_file(grountruth_pos_fn);
    if(pos_file.fail())
    {
        throw std::runtime_error("Unable to open file " + grountruth_pos_fn);
    }
    nlohmann::json TX_gt_positions;
    pos_file >> TX_gt_positions;
    nlohmann::json true_positions_tx = TX_gt_positions["true_tx_positions"];
    auto all_true_AOA = utils.get_true_aoa_v2(mean_pos, true_positions_tx);

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
        std::string ts = tx_profile_timestamp[itr.first];
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
            string profile_op_fn = utils.__homedir+output+"/"+run_module.tx_name_list[tx_id]+"_"+ts+"_aoa_profile.csv";
            utils.writeToFile(profile,profile_op_fn);

            true_phi = all_true_AOA[run_module.tx_name_list[tx_id]].first;
            true_theta = all_true_AOA[run_module.tx_name_list[tx_id]].second;

            auto topN_angles = all_topN_angles[tx_id];
            std::vector<double> top_aoa_error = run_module.top_aoa_error(topN_angles.first[0],
                                                                        topN_angles.second[0],
                                                                        all_true_AOA[run_module.tx_name_list[tx_id]],
                                                                        trajType);

            std::vector<double> closest_AOA_error = run_module.get_aoa_error(topN_angles,
                                                                            all_true_AOA[run_module.tx_name_list[tx_id]],
                                                                            aoa_confidence,
                                                                            trajType);

            auto stats = run_module.get_stats(true_phi, true_theta,
                                            top_aoa_error, closest_AOA_error,
                                            tx_id, run_module.tx_name_list[tx_id],
                                            mean_pos);

            std::cout << stats.dump(4) << std::endl;
        }
    }
    
}