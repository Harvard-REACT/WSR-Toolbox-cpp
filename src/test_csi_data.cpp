#include "csitoolbox/WSR_Module.h"
#include <unistd.h>
#include <sys/types.h>
#include <unordered_map>
#include <chrono>

int main(){
    
    WSR_Util utils;
    std::string config = "../config/config_3D_SAR.json";
    WSR_Module run_module(config);
    
    //RX_SAR_Robot: performs 3D SAR
    //TX_SAR_Robot(s): Neighboring robots for which RX_SAR_robot calculates AOA

    /*================== Process RX_SAR_Robot files ====================*/
    std::string reverse_csi = run_module.__precompute_config["input_RX_channel_csi_fn"]["value"]["csi_fn"].dump();
    std::string output = run_module.__precompute_config["output_aoa_profile_path"]["value"].dump();

    //Remove all double-quote characters
    reverse_csi.erase(remove( reverse_csi.begin(), reverse_csi.end(), '\"' ),reverse_csi.end());
    output.erase(remove( output.begin(), output.end(), '\"' ),output.end());

    std::string rx_robot_csi = utils.__homedir + reverse_csi;
    
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

    
    /*Test CSI data files*/
    run_module.test_csi_data(rx_robot_csi,tx_robot_csi);
    auto detected_tx = run_module.get_paired_pkt_count();

    for(auto & itr : detected_tx)
    {
      std::string tx_id = itr.first;
      auto stats = run_module.get_performance_stats(tx_id, run_module.tx_name_list[tx_id]);

      std::cout << stats.dump(4) << std::endl;
      std::cout << "********************************" << std::endl;
    }    

}