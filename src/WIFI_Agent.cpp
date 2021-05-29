//
// Created by wwang239 on 11/26/19.
// Updated by Ninad
//

#include "csitoolbox/WIFI_Agent.h"

/**
 * Convert byte to string and creat array of CSI data packets.
 * The packet is added to a vector of packets which can be accessed via class object
 * return: None
*/
WIFI_Agent::WIFI_Agent(void){
    
    field_len = (uint16_t*) calloc (byte_count,sizeof(uint16_t)*byte_count);
    if (field_len == NULL) {
        fputs ("Memory error-field-len\n",stderr); 
        exit(2);
    } 

    code = (uint16_t*) calloc (byte_count,sizeof(uint16_t)*byte_count);
    if (code == NULL) {
        fputs ("Memory error-code\n",stderr); 
        exit(2);
    }
}
//=============================================================================================================================
/**
 *
 *
 * */
WIFI_Agent::~WIFI_Agent(void){
    free (field_len);
    free (code);
    free(bytes_data);
}
//=============================================================================================================================
/**
 *
 *
 * */
std::vector<DataPacket> WIFI_Agent::get_wifi_data(std::string mac_id){
    std::vector<DataPacket> temp;
    for (int i=0;i < wifi_data_packet_array.size();i++)
    {
        if(mac2str(wifi_data_packet_array[i].mac_real) == mac_id) 
        {
            temp.push_back(wifi_data_packet_array[i]);
        }
    }

    return temp;
}

/**
 * Read a binay files size.
*/
int WIFI_Agent::getSize(std::string fn){
    std::streampos begin,end;
    csi_file__.open(fn.c_str(), std::ios::in | std::ios::binary);
    if(csi_file__.is_open()){
        begin = csi_file__.tellg();
        csi_file__.seekg (0, std::ios::end);
        end = csi_file__.tellg();
        csi_file__.close();
        file_size__ = int(end-begin);
    }
    return file_size__;
}
//=============================================================================================================================
/**
 *
 *
 * */
void WIFI_Agent::updatePktCount(){
    pkt_count+=1;
}
//=============================================================================================================================
/**
 *
 *
 * */
int WIFI_Agent::getPktCount(){
    return pkt_count ;
}
//=============================================================================================================================
/**
 *
 *
 * */
void WIFI_Agent::saveDataPacket(DataPacket wifi_data_packet){
    wifi_data_packet_array.push_back(wifi_data_packet);
    std::string mac_id_str = mac2str(wifi_data_packet.mac_real);
    if (unique_mac_ids_packets.find(mac_id_str) != unique_mac_ids_packets.end())
    {
        unique_mac_ids_packets[mac_id_str] = unique_mac_ids_packets[mac_id_str] + 1;
    }
    else
    {
        unique_mac_ids_packets[mac_id_str] = 1;
    }

}
//=============================================================================================================================
/**
 *
 *
 * */
void WIFI_Agent::reset(){
    wifi_data_packet_array.clear();
    pkt_count = 0;
}
//=============================================================================================================================
/**
 *
 *
 * */
std::string WIFI_Agent::mac2str(std::string const& s) {

    std::vector <std::string> mac_val; 
    std::stringstream check1(s); 
    std::string intermediate; 
    while(getline(check1, intermediate, ':')) 
    { 
        mac_val.push_back(intermediate); 
    } 

    std::string output = "0"+dec2hex(std::stoi(mac_val[0])) + ":" +
                         dec2hex(std::stoi(mac_val[1])) + ":" +
                         dec2hex(std::stoi(mac_val[2])) + ":" +
                         dec2hex(std::stoi(mac_val[3])) + ":" +
                         dec2hex(std::stoi(mac_val[4])) + ":" +
                         dec2hex(std::stoi(mac_val[5]));
    return output;
}
//=============================================================================================================================
/**
 *
 *
 * */
std::string WIFI_Agent::dec2hex(unsigned int i)
{
    std::stringstream ss;
    ss << std::hex << std::uppercase << i;
    return ss.str();
}
//Function to check if CSI is empty or not for a given packet
