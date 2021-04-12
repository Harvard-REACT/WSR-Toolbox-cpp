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
    
    string foldername = "/home/weiying/Dropbox (Harvard University)/Harvard/iros2021/wsr-iros-2021/data/Dataset1-REACT_Testbed/1_TX_1_RX/Line-of-Sight/Curved_Planar_Trajectories/RX_SAR_robot_Position_Changing/RX_P1_TX_P0";
    string dataset_type = "3D_RX";
    std::vector<string> csi_tx;
    std::list<std::string> mylist;
    std::set<string> timesset;
    std::list<string> timelist;

    foldername = argv[1];
    dataset_type = argv[2];
    string traj_folder = "/"+dataset_type+"_dataset_traj_rx";
    string csi_folder = "/"+dataset_type+"_dataset_csi_rx_tx";
    string grountruth_pos_fn = foldername+"/"+"groundtruth_positions.json";
    string rx_csi_pre = "csi_rx_";
    string tx_csi_pre = "csi_";
    string traj_pre = "rx_trajectory_";
    string timestmp = "";

    //Read all unique timestamps
    for (const auto & entry : fs::directory_iterator(foldername+csi_folder)) 
    {
        auto end_idx = entry.path().string().rfind(".");
        auto start_idx = end_idx - 17;
        timestmp = entry.path().string().substr(start_idx, 17);
        bool found = (std::find(timelist.begin(), timelist.end(), timestmp) != timelist.end());
        if(!found)
            timelist.push_back(timestmp);
    }
    timelist.sort();
    
    //Read TX_neighbor_robot groundtruth position
    std::ifstream pos_file(grountruth_pos_fn);
    if(pos_file.fail())
    {
        throw std::runtime_error("Unable to open file " + grountruth_pos_fn);
    }
    nlohmann::json TX_gt_positions;
    pos_file >> TX_gt_positions;

    WSR_Util utils;

    for (auto ts_it = timelist.begin(); ts_it != timelist.end(); ++ts_it) 
    {
      std::cout << "***********************************************************************************" << std::endl;
      std::cout << "Processing data for RX_SAR_robot : " << rx_csi_pre + *ts_it + ".dat" << std::endl;
      std::cout << "***********************************************************************************" << std::endl;
      // std::string config = utils.__homedir+"/catkin_ws/src/csitoolbox/config/config_3D_SAR.json";
      std::string config = "../config/config_3D_SAR.json";
      WSR_Module run_module(config);

      run_module.data_sample_ts = *ts_it;
      std::string output = run_module.__precompute_config["output_aoa_profile_path"]["value"].dump();
      output.erase(remove( output.begin(), output.end(), '\"' ),output.end());
      std::string rx_robot_csi = foldername + csi_folder + "/" + rx_csi_pre + *ts_it + ".dat";
      std::string traj_fn_rx = foldername + traj_folder + "/" + traj_pre + *ts_it + "_.csv";      
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
        string csi_data_file = foldername + csi_folder + "/" + tx_csi_pre + tx_name + "_" + *ts_it + ".dat";
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

      std::cout << "log [WSR_Module]: Preprocessing Trajectory " << std::endl;
      //Get relative trajectory if moving ends
      if(bool(run_module.__precompute_config["use_relative_trajectory"]["value"]))
      {          
        //get relative trajectory
        auto return_val = utils.getRelativeTrajectory(trajectory_rx,trajectory_tx);
        trajectory_timestamp = return_val.first;
        displacement = return_val.second;
      }
      else
      {
        auto return_val = WSR_Util::formatTrajectory_v2(trajectory_rx,true);
        trajectory_timestamp = return_val.first;
        displacement = return_val.second;
      }
      //Trying ideal channel


      //Get all True AOA angles
      nlohmann::json true_positions_tx = TX_gt_positions["true_tx_positions"];
      auto all_true_AOA = utils.get_true_aoa(trajectory_rx, true_positions_tx);

      run_module.calculate_AOA_profile(rx_robot_csi,tx_robot_csi,displacement,trajectory_timestamp);
      auto all_aoa_profile = run_module.get_all_aoa_profile();
      auto all_topN_angles = run_module.get_TX_topN_angles();
      string trajType = run_module.__precompute_config["trajectory_type"]["value"];
      double true_phi, true_theta;

      std::cout << "Getting AOA profile stats for TX Neighbor robots" << std::endl;
      for(auto & itr : all_aoa_profile)
      {
        std::cout << "-----------------------------" << std::endl;
        
        std::string tx_id = itr.first;
        std::string ts = tx_profile_timestamp[itr.first];
        auto profile = itr.second;

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
                                                                            trajType);

          auto stats = run_module.get_stats(true_phi, true_theta,
                                            top_aoa_error, closest_AOA_error,
                                            tx_id, run_module.tx_name_list[tx_id]);

          std::cout << stats.dump(4) << std::endl;
        }
      }
    }
}
