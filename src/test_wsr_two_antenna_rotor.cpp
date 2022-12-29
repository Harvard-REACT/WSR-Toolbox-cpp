#include "csitoolbox/WSR_Module.h"
#include <unistd.h>
#include <sys/types.h>
#include <unordered_map>
#include <chrono>

int main(int argc, char *argv[])
{    
    WSR_Util utils;
    string __d_type = argv[1];
    std::cout << "Processing trajectory type: " << __d_type << std::endl;
    std::string config = "../config/config_3D_SAR.json";
    WSR_Module run_module(config);

    /*================== Process RX_SAR_Robot files ====================*/
    std::string reverse_csi = run_module.__precompute_config["input_RX_channel_csi_fn"]["value"]["csi_fn"].dump();
    std::string trajectory_file_rx;
    if(__d_type == "gt")
        trajectory_file_rx = run_module.__precompute_config["input_trajectory_csv_fn_rx"]["value"].dump();
    else if(__d_type == "joint")
        trajectory_file_rx = run_module.__precompute_config["input_orientation_csv_fn_rx_joint"]["value"].dump();

    
    std::string output = run_module.__precompute_config["debug_dir"]["value"].dump();
    bool __Flag_get_mean_pos = bool(run_module.__precompute_config["get_mean_pose_RX"]["value"]);

    //Remove all double-quote characters
    reverse_csi.erase(remove( reverse_csi.begin(), reverse_csi.end(), '\"' ),reverse_csi.end());
    trajectory_file_rx.erase(remove( trajectory_file_rx.begin(), trajectory_file_rx.end(), '\"' ),trajectory_file_rx.end());
    output.erase(remove( output.begin(), output.end(), '\"' ),output.end());
    std::string rx_robot_csi = utils.__homedir + reverse_csi;
    std::string traj_fn_rx = utils.__homedir + trajectory_file_rx;

    //Extract timestamp from the CSI file so that it can be appended to the output files
    std::string csi_name,ts,time_val,date_val; 
    stringstream tokenize_string1(rx_robot_csi); 
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


    std::vector<std::vector<double>> trajectory_rx = utils.load_Displacement_From_CSV(traj_fn_rx); //Robot performing SAR
    // std::vector<std::vector<double>> true_trajectory_rx = utils.load_Displacement_From_CSV(true_traj_fn_rx); //Robot performing SAR
    nc::NdArray<double> displacement;
    nc::NdArray<double> displacement_timestamp;
    
    /*============= Process the TX_SAR_Robot files =======================*/
    std::unordered_map<std::string,std::string> tx_robot_csi;

    for (auto it = run_module.__precompute_config["other_robot_ID"]["value"].begin(); 
        it != run_module.__precompute_config["other_robot_ID"]["value"].end(); 
        ++it)
    {
        const string& tx_name =  it.key();
        auto temp =  it.value();
        string tx_mac_id = temp["mac_id"];
        run_module.data_sample_ts[tx_mac_id] = date_val +"_"+ time_val;
        run_module.tx_name_list[tx_mac_id] = tx_name;
    }


    std::cout << "log [WSR_Module]: Preprocessing Displacement " << std::endl;    
    
    auto return_val = utils.formatDisplacementTwoAntenna(trajectory_rx,__d_type);
    displacement_timestamp = return_val.first;
    displacement = return_val.second;
    

    //Get all True AOA angles
    nlohmann::json true_positions_tx = run_module.__precompute_config["true_tx_positions"];
    auto all_true_AOA = utils.get_true_aoa(trajectory_rx, true_positions_tx); //Fix this when using moving ends.

    std::cout << "Size of displacement cols:" << nc::shape(displacement).cols << std::endl;
    run_module.calculate_AOA_using_csi_conjugate(rx_robot_csi,displacement,displacement_timestamp);
    
    auto all_aoa_profile = run_module.get_all_aoa_profile();
    auto all_topN_angles = run_module.get_TX_topN_angles();
    auto all_confidences = run_module.get_all_confidence();
    string trajType = run_module.__precompute_config["trajectory_type"]["value"];
    double true_phi, true_theta;
    nc::NdArray<double> pos = nc::zeros<double>(1, 4);

    std::cout << "Getting AOA profile stats for TX Neighbor robots" << std::endl;
    std::string viz_id = "";
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
            std::cout << profile_op_fn << std::endl;
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
        }
        viz_id = viz_id + run_module.tx_name_list[tx_id] +" ";
    }
    
    //Visualize
    if(run_module.__precompute_config["debug"]["value"])
    {
        std::string viz_op = "../scripts/viz_data.sh ~/" + run_module.__precompute_config["debug_dir"]["value"].dump() + " '" + viz_id +"'";
        std::cout << viz_op << std::endl;
        system(viz_op.c_str());
    }
}