/*
 * (c) REACT LAB, Harvard University
	Author: Weiying Wang, Ninad Jadhav
 */

#ifndef CSITOOLBOX_WIFI_AGENT_H
#define CSITOOLBOX_WIFI_AGENT_H


#include "Base.h"
#include "Error.h"

class WIFI_Agent {
    private:
       std::ifstream csi_file__;
       int file_size__ = -1, pkt_count = 0;
       std::vector<DataPacket> wifi_data_packet_array; 
       std::vector<DataPacket> wifi_spoofed_data_packet_array; 
       std::vector<std::vector<double>> csi_performax;
       std::string mac2str(std::string const& s);
       std::string dec2hex(unsigned int i);

       //TODO: get antenna number from config file. It would be useful in extracting the csi for that specific antenna (among antenna
       //1,2,3). By default set to 1. 

    public:
        std::string robot_type;
        std::unordered_map<std::string, int> unique_mac_ids_packets, unique_mac_ids_packets_spoofed;
        int getSize(std::string fn); 
        size_t readSizeT(std::string fn);
        std::vector<DataPacket> get_wifi_data(std::string mac_id);
        void updatePktCount();
        int getPktCount();
        void saveDataPacket(DataPacket wifi_data_packet);
        uint16_t *field_len = nullptr, *code=nullptr;
        uint8_t *bytes_data = nullptr;
        int byte_count = 1;
        void reset();
        WIFI_Agent();
        ~WIFI_Agent();
        void simulate_spoofed_data(int spoofed_count);
        std::vector<DataPacket> get_wifi_data_spoofed(std::string mac_id);
        void simulate_spoofed_data_multiple(int spoofed_count,
                                            std::string illegit_mac_id);
        int __packet_id_counter=0, __transmission_id=1;
        void simulate_spoofed_data_second(int spoofed_count,
                                          std::string illegit_mac_id);
        
};


#endif //CSITOOLBOX_WIFI_AGENT_H