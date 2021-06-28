#include "csitoolbox/WSR_Module.h"
#include <unistd.h>
#include <sys/types.h>
#include <unordered_map>
#include <chrono>

int main(){
    
    WSR_Util utils;
    // struct passwd *pw = getpwuid(getuid());
    // std::string homedir = pw->pw_dir;
    // std::string folder = "";
    // std::string config = utils.__homedir+"/catkin_ws/src/csitoolbox/config/config_3D_SAR.json";
    std::string config = "../config/config_3D_SAR.json";
    WSR_Module run_module(config);
    
    //RX_SAR_Robot: performs 3D SAR
    //TX_SAR_Robot(s): Neighboring robots for which RX_SAR_robot calculates AOA

    /*================== Process RX_SAR_Robot files ====================*/
    std::string reverse_csi = run_module.__precompute_config["input_RX_channel_csi_fn"]["value"]["csi_fn"].dump();
    std::string trajectory_file_rx = run_module.__precompute_config["input_trajectory_csv_fn_rx"]["value"].dump();
    std::string output = run_module.__precompute_config["output_aoa_profile_path"]["value"].dump();

    //Remove all double-quote characters
    reverse_csi.erase(remove( reverse_csi.begin(), reverse_csi.end(), '\"' ),reverse_csi.end());
    trajectory_file_rx.erase(remove( trajectory_file_rx.begin(), trajectory_file_rx.end(), '\"' ),trajectory_file_rx.end());
    output.erase(remove( output.begin(), output.end(), '\"' ),output.end());

    std::string rx_robot_csi = utils.__homedir + reverse_csi;
    std::string traj_fn_rx = utils.__homedir + trajectory_file_rx;
    std::vector<std::vector<double>> trajectory_rx = utils.loadTrajFromCSV(traj_fn_rx); //Robot performing SAR
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
    std::cout << "log [WSR_Module]: Preprocessing Trajectory " << std::endl;
    std::vector<double> antenna_offset = run_module.__precompute_config["antenna_position_offset"]["offset"].get<std::vector<double>>();

    /*Calculate AOA*/
    for(int i=0; i<1; i++) //loop added only for testing
    {
        
        std::cout << "log [WSR_Module]: Preprocessing Trajectory " << std::endl;
        nc::NdArray<double> mean_pos;
        //Get relative trajectory if moving ends
        if(bool(run_module.__precompute_config["use_relative_trajectory"]["value"]))
        {          
          //get relative trajectory
          auto return_val = utils.getRelativeTrajectory(trajectory_rx,trajectory_tx,antenna_offset);
          trajectory_timestamp = return_val.first;
          displacement = return_val.second;
        }
        else
        {
          auto return_val = utils.formatTrajectory_v2(trajectory_rx,antenna_offset,mean_pos);
          trajectory_timestamp = return_val.first;
          displacement = return_val.second;
        }

        //Get all True AOA angles
        nlohmann::json true_positions_tx = run_module.__precompute_config["true_tx_positions"];
        auto all_true_AOA = utils.get_true_aoa(trajectory_rx, true_positions_tx);

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
}