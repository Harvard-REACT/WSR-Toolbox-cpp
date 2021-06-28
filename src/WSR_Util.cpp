/**
 * (c) REACT LAB, Harvard University
	Author: Ninad Jadhav, Weiying Wang
*/

#include "csitoolbox/WSR_Util.h"

//=============================================================================================================================
/**
 * 
 * 
 * 
 **/
void WSR_Util::displayDataPacket(DataPacket wifi_data_packet, int packet_number){
    std::cout << "packet number: " << packet_number << std::endl;
    std::cout << "timestamp_low: "<< wifi_data_packet.timestamp_low << std::endl;
    std::cout << "bfee_count: "<< wifi_data_packet.bfee_count << std::endl;
    std::cout << "Nrx: "<< wifi_data_packet.Nrx << std::endl;
    std::cout << "Ntx: "<< wifi_data_packet.Ntx << std::endl;
    std::cout << "rssi_a "<< wifi_data_packet.rssi_a << std::endl;
    std::cout << "rssi_b: "<< wifi_data_packet.rssi_b << std::endl;
    std::cout << "rssi_c: "<< wifi_data_packet.rssi_c << std::endl;
    std::cout << "noise: "<< double(wifi_data_packet.noise) << std::endl;
    std::cout << "agc: "<< wifi_data_packet.agc << std::endl;
    std::cout << "perm: ["<< wifi_data_packet.perm[0] << ","  
                        << wifi_data_packet.perm[1] << "," 
                        << wifi_data_packet.perm[2] << "]" << std::endl;
    std::cout << "rate: "<< wifi_data_packet.fake_rate_n_flags << std::endl;
    std::cout << "tv_sec: "<< wifi_data_packet.tv_sec << std::endl;
    std::cout << "tv_usec: "<< wifi_data_packet.tv_usec << std::endl;
    std::cout << fixed <<"ts: "<< wifi_data_packet.ts << std::endl; // new field added
    // std::cout << "mac:  " << wifi_data_packet.mac_real[0] <<":" << wifi_data_packet.mac_real[1] << ":" 
    //     << wifi_data_packet.mac_real[2] <<":" << wifi_data_packet.mac_real[2] <<":" 
    //     << wifi_data_packet.mac_real[4] <<":" << wifi_data_packet.mac_real[5] << std::endl;
    std::cout << "mac:  " << wifi_data_packet.mac_real << std::endl;
    std::cout << "csi: " << std::endl;
    for(int a=0;a<30;++a){
        for(int b=0;b<3;++b){
            std::cout << wifi_data_packet.csi[a][b].real() << " + " << wifi_data_packet.csi[a][b].imag() << "i    ";
        }
        std::cout << std::endl;
    }
    std::cout << "================================" << std::endl;
}

/**
 *Decode the byte data from csi files
 **/
int WSR_Util::readCsiData(std::string fn, WIFI_Agent& robot, bool __FLAG_debug){
    
    std::cout << "Reading from file : " << std::endl;
    std::cout << fn << std::endl;

    FILE *csi_file;
    long file_length, byte_count = 1, total_bytes_data_sz=0, curr_loc=0;
    size_t result = -1;
    fpos_t position;
    int status=0, error_no=-12;
    
    //std::cout << "Opening the csi file : "<<fn.c_str() << std::endl;
    csi_file = fopen ( fn.c_str() , "rb" );
    if (csi_file==NULL) {
        fputs ("log [Utils]: Error opening CSI file \n",stderr); 
        fclose(csi_file);
        exit (1);
    }

    // obtain file size and then reset pointer to start of file
    //std::cout << "Obtaining the file size and resetting the pointer" << std::endl;
    status = fseek (csi_file , 0 , SEEK_END);
    if(status != 0) {
        error_no = ferror(csi_file);
        std::cout << "log [Utils]: Error " << error_no <<" seeking end of file" << std::endl;
        fclose(csi_file);
        exit(1);
    }

    
    file_length = ftell (csi_file);
    if(__FLAG_debug) std::cout << "log [Utils]: file size : " << file_length << std::endl;
    
    status = fseek (csi_file , 0 , SEEK_SET);
    if(status != 0) {
        error_no = ferror(csi_file);
        std::cout << "log [Utils]: Error " << error_no <<" seeking beginning of file" << std::endl;
        fclose(csi_file);
        exit(1);
    }
    

    while(curr_loc < file_length-3){
        
        status = fseek (csi_file , 1 , SEEK_CUR);
        if(status != 0) {
            error_no = ferror(csi_file);
            std::cout << "log [Utils]: Error " << error_no <<" seeking position in file stream" << std::endl;
            fclose(csi_file);
            exit(1);
        }

        //curr_loc = ftell (csi_file);
        //std::cout << "Location for reading field length  : " << curr_loc << std::endl;


        //get field len
        result = fread (robot.field_len,1,robot.byte_count,csi_file);
        if (result != robot.byte_count) {
            std::cout << "result: " << result << std::endl;
            fputs ("log [Utils]: Reading error-field-len\n",stderr);
            break;
        }

        //curr_loc = ftell (csi_file);
        //std::cout << "Location for reading code : " << curr_loc << std::endl;
        
        //get code
        result = fread (robot.code,1,robot.byte_count,csi_file);
        if (result != robot.byte_count) {
            fputs ("log [Utils]: Reading error-code\n",stderr); 
            break;
        }
    
        //Show field_len and code
        //std::cout << "field_len: " << robot.field_len[0] << std::endl;
        //std::cout << "code: " << robot.code[0] << std::endl;
        total_bytes_data_sz = robot.field_len[0]-1;

        // If unhandled code, skip (seek over) the record and continue
        if (robot.code[0] == 187) { // get beamforming or phy data
            
            //curr_loc = ftell (csi_file);
            //std::cout << "Location before reading buffer : " << curr_loc << std::endl;
            
            robot.bytes_data = (uint8_t*) calloc (total_bytes_data_sz,sizeof(uint8_t)*total_bytes_data_sz);
            if (robot.bytes_data == NULL) {
                fputs ("log [Utils]: Memory error for code 187",stderr); 
                exit (2);
            }
    
            result = fread (robot.bytes_data,1,total_bytes_data_sz,csi_file);
            if (result != total_bytes_data_sz) {
                fputs ("log [Utils]: Completed reading data packets\n",stderr); 
                break;
            }
            
            //curr_loc = ftell (csi_file);
            //std::cout << "Location after reading buffer : " << curr_loc << std::endl;

            read_bfee_timestamp_mac(robot.bytes_data,robot);

            robot.updatePktCount();
        }
        else{ //skip all other info
            status = fseek (csi_file , total_bytes_data_sz , SEEK_CUR);
            if(status != 0) {
                error_no = ferror(csi_file);
                std::cout << "log [Utils]: Error " << error_no <<" seeking file pointer location" << std::endl;
                fclose(csi_file);
                exit(1);
            }
        }
        // std::cout << "-----------" << std::endl;
        // if(robot.getPktCount() > 1) {
        //     break;
        // }
    }

    // terminate
    fclose (csi_file);
    int total_packets = robot.getPktCount();
    std::cout << "log [Utils]: Exiting CSI File Read" << std::endl;
    return total_packets;
}

/**
 *Decode the byte data
 **/
void WSR_Util::read_bfee_timestamp_mac(uint8_t *inBytes, WIFI_Agent& robot)
{
	struct DataPacket wifi_data_packet;
	unsigned int len = 1, calc_len = 0;
	unsigned int i, j;
	unsigned int index = 0, remainder;
	char tmp1, tmp2;

	wifi_data_packet.timestamp_low = inBytes[0] + (inBytes[1] << 8) +
		(inBytes[2] << 16) + (inBytes[3] << 24);
    wifi_data_packet.bfee_count = inBytes[4] + (inBytes[5] << 8);
	wifi_data_packet.Nrx = inBytes[8];
	wifi_data_packet.Ntx = inBytes[9];
	wifi_data_packet.rssi_a = inBytes[10];
	wifi_data_packet.rssi_b = inBytes[11];
	wifi_data_packet.rssi_c = inBytes[12];
	wifi_data_packet.noise = inBytes[13];
	wifi_data_packet.agc = inBytes[14];
    unsigned int antenna_sel = inBytes[15];
    len = inBytes[16] + (inBytes[17] << 8);
    wifi_data_packet.fake_rate_n_flags = inBytes[18] + (inBytes[19] << 8);
    wifi_data_packet.tv_sec = inBytes[20] + (inBytes[21] << 8) +
		(inBytes[22] << 16) + (inBytes[23] << 24);
    wifi_data_packet.tv_usec = inBytes[24] + (inBytes[25] << 8) +
		(inBytes[26] << 16) + (inBytes[27] << 24);
    unsigned char *mac = &inBytes[28];
    wifi_data_packet.frame_count = inBytes[34] + (inBytes[35] << 8) + (inBytes[36] << 16) + (inBytes[37] << 24);
	unsigned char *payload = &inBytes[38];

    wifi_data_packet.ts = wifi_data_packet.tv_sec + wifi_data_packet.tv_usec*pow(10,-6); //TODO: check of open bug 8
    
    /* Create MAC address */
    // wifi_data_packet.mac_real.push_back((double)mac[0]);
    // wifi_data_packet.mac_real.push_back((double)mac[1]);
    // wifi_data_packet.mac_real.push_back((double)mac[2]);
    // wifi_data_packet.mac_real.push_back((double)mac[3]);
    // wifi_data_packet.mac_real.push_back((double)mac[4]);
    // wifi_data_packet.mac_real.push_back((double)mac[5]);

    wifi_data_packet.mac_real = std::to_string((int)mac[0])+":"+
                                std::to_string((int)mac[1])+":"+
                                std::to_string((int)mac[2])+":"+
                                std::to_string((int)mac[3])+":"+
                                std::to_string((int)mac[4])+":"+
                                std::to_string((int)mac[5]);
        
	/* Check that length matches what it should */
    calc_len = (30 * (wifi_data_packet.Nrx * wifi_data_packet.Ntx * 8 * 2 + 3) + 7) / 8;

	if (len != calc_len)
		std::cout << "Wrong beamforming matrix size." << std::endl;


    /*Calcuate CSI*/
    for (i = 0; i < 30; ++i)
	{
		index += 3;
		remainder = index % 8;
		for (j = 0; j < wifi_data_packet.Nrx * wifi_data_packet.Ntx; ++j)
		{
			tmp1 = (payload[index / 8] >> remainder) |
				(payload[index/8+1] << (8-remainder)); //real part
			
            /*printf("%d\n", tmp);*/
			
            tmp2 = (payload[index / 8+1] >> remainder) |
				(payload[index/8+2] << (8-remainder)); //imaginary part
			
			index += 16;

            std::complex<double> tmp_complex (tmp1, tmp2);
            wifi_data_packet.csi[i][j] = tmp_complex;
		}
        // std::cout << std::endl;
	}

    wifi_data_packet.perm [0] = ((antenna_sel) & 0x3) + 1;
	wifi_data_packet.perm [1] = ((antenna_sel >> 2) & 0x3) + 1;
	wifi_data_packet.perm [2] = ((antenna_sel >> 4) & 0x3) + 1;

    robot.saveDataPacket(wifi_data_packet);
    
}

/**
 * Match the forward and backward channel packets based on timestamp
 * input: CSI datapacket for transmitter and receiver
 * output: Eigen vector of forward-reverse product
 **/
Eigen::MatrixXcd WSR_Util::getForwardReverseChannel(std::vector<DataPacket> transmitter, 
                                                    std::vector<DataPacket> receiver)
{
    int rec_length = int(receiver.size()), tra_length = int(transmitter.size());
    int itr_k=0, itr_l=0;
    double a;
    std::vector<Eigen::VectorXcd> forward_reverse_channel_array;
    std::vector<Eigen::VectorXcd> csi_timestamp;
    Eigen::VectorXcd forward_reverse_product(30);
    double __offset =  pow(10,-6), __threshold = 600*pow(10,-6);
    
    std::cout << "Starting to calulate the product" << std::endl;

    while(itr_k < rec_length) { //&& not(isempty(receiver{k}))){
        if(itr_l < tra_length){
            //TODO Check if csi is empty or not.
            //if(not empty csi)
            a = transmitter[itr_l].ts - receiver[itr_k].ts + __offset;
            if (abs(a) > __threshold || a<0){
                std::cout << "abs a greater than threshold or less than 0" << std::endl;
                if(transmitter[itr_l].ts > receiver[itr_k].ts) 
                    itr_k += 1;     
                else 
                    itr_l += 1;
                
                std::cout << "Mismatch , k = " << itr_k << ", l = " <<itr_l << std::endl;
            }
            else{
                //multiply forward and reverse channel
                // std::cout << "Calculating forward reverse channel product" << std::endl;
                for(int h_i = 0;h_i< 30; h_i++){
                    forward_reverse_product(h_i, 0) = transmitter[itr_l].csi[h_i][0] * receiver[itr_k].csi[h_i][0];//Use CSI from first antenna only
                }
                // std::cout << fixed << forward_reverse_product.transpose() << std::endl;
                forward_reverse_channel_array.push_back(forward_reverse_product.transpose());
                itr_k +=1;
                itr_l +=1;
            }
            //else(empty csi)
        }
        else{
            itr_k = rec_length + 1;
        }
    }

    int fw_size = int(forward_reverse_channel_array.size());
    std::cout << fw_size << std:: endl;
    Eigen::MatrixXcd csi_forward_reverse_matrix = Eigen::MatrixXcd::Zero(fw_size,30);

    for(int mat_i = 0; mat_i < fw_size ; mat_i++){
        // std::cout << "Appending....." << std::endl;
        csi_forward_reverse_matrix.row(mat_i) = forward_reverse_channel_array[mat_i];//Use CSI from frist antenna only
    }
    
    // std::cout << fixed << csi_forward_reverse_matrix.row(fw_size-1) << std::endl;
    return csi_forward_reverse_matrix;
}

//=============================================================================================================================
/**
 * Match the forward and backward channel packets based on timestamp
 * input: CSI datapacket for transmitter and receiver
 * output: pair of NdArray pair<forward-reverse product, csi_timestamp>
 **/
std::pair<nc::NdArray<std::complex<double>>,nc::NdArray<double>> WSR_Util::getForwardReverseChannel_v2(
                                                                std::vector<DataPacket> rx_robot,
                                                                std::vector<DataPacket> tx_robot,
                                                                double predefined_offset,
                                                                double threshold,
                                                                double& calculated_ts_offset,
                                                                bool interpolate_phase,
                                                                bool sub_sample){
    int tx_length = int(tx_robot.size()), rx_length = int(rx_robot.size());
    int itr_k=0, itr_l=0;
    double a;
    bool first_csi_val = true;
    std::cout.precision(15);
    int sizeee = rx_robot.size();

    nc::NdArray<std::complex<double>> forward_reverse_channel_product;
    nc::NdArray<double> csi_timestamp;
    nc::NdArray<std::complex<double>> temp1 = nc::zeros<std::complex<double>>(nc::Shape(1,30));
    nc::NdArray<double> temp2 = nc::zeros<double>(nc::Shape(1,1));
    double interpolated_phase;
    std::complex<double> interpolated_h;


    calculated_ts_offset = rx_robot[0].ts-tx_robot[0].ts;
    auto ArrayShape = forward_reverse_channel_product.shape();
    // std::cout << "Initialized size of NdArray " << ArrayShape << std::endl;
    // std::cout << "Starting to calulate the product for NdArray" << std::endl;

    while(itr_k < tx_length) { //&& not(isempty(receiver{k}))){
        if(itr_l < rx_length){
            //TODO Check if csi is empty or not.
            //if(not empty csi)
            a = rx_robot[itr_l].ts - tx_robot[itr_k].ts - predefined_offset;
            if (abs(a) > threshold){
                // std::cout << "abs a greater than threshold or less than 0" << std::endl;
                if(rx_robot[itr_l].ts > tx_robot[itr_k].ts+predefined_offset)
                    itr_k += 1;
                else
                    itr_l += 1;

                // std::cout << "Mismatch , k = " << itr_k << ", l = " <<itr_l << std::endl;
            }
            else{

                //multiply forward and reverse channel
                for(int h_i = 0;h_i< 30; h_i++)
                {
                    temp1(0,h_i) = rx_robot[itr_l].csi[h_i][0] * tx_robot[itr_k].csi[h_i][0]; //Use CSI from first antenna only
                }
                if(interpolate_phase) 
                {
                    auto central_snum = nc::NdArray<double>(1, 1) = 15.5;
                    auto xp = nc::arange<double>(1, 31);
                    auto fp = unwrap(nc::angle(temp1(0,temp1.cSlice())));
//                    std::cout << "Unwrapped phase: " << fp << std::endl;
//                    std::cout << nc::angle(temp1(0,temp1.cSlice())) << std::endl;
                    auto mag1 = nc::abs(temp1(0,15));
                    auto mag2 = nc::abs(temp1(0,16));
                    auto fitted =  nc::polynomial::Poly1d<double>::fit(xp.transpose(),fp.transpose(),1);
                    interpolated_phase = nc::unwrap(fitted(central_snum(0,0)));
//                    std::cout << interpolated_phase << std::endl;
//                    interpolated_phase = (nc::interp(central_snum, xp, fp))(0,0);
//                    std::cout << interpolated_phase << std::endl;
//                    interpolated_phase = std::fmod(interpolated_phase + nc::constants::pi , (2 * nc::constants::pi)) - nc::constants::pi;
//                    std::cout <<std::complex<double>(0,1)*interpolated_phase<<std::endl;
                    interpolated_h = (mag1+mag2)/2*nc::exp(std::complex<double>(0,1)*interpolated_phase);
//                    std::cout << interpolated_h << std::endl;

//                temp1.tofile("tempTxt", "\n");
                }
//                std::cout << interpolated_phase << std::endl;
                assert(temp2(0,0) != rx_robot[itr_l].ts);
                temp2(0,0) = rx_robot[itr_l].ts;
                itr_k +=1;
                itr_l +=1;

                if(first_csi_val)
                {
                    if(interpolate_phase)
                        forward_reverse_channel_product = nc::NdArray<std::complex<double>>{interpolated_h};
                    else
                        forward_reverse_channel_product = temp1;

                    csi_timestamp = temp2;
                    first_csi_val = false;
                }
                else
                {
                    if(sub_sample && itr_l%2!=0) continue;
                    
                    csi_timestamp = nc::append(csi_timestamp, temp2, nc::Axis::ROW);
                    assert(csi_timestamp(-2,0)  < csi_timestamp(-1,0));
                    if(interpolate_phase)
                        forward_reverse_channel_product = nc::append(forward_reverse_channel_product,nc::NdArray<std::complex<double>>{interpolated_h},nc::Axis::ROW);
                    else
                        forward_reverse_channel_product = nc::append(forward_reverse_channel_product, temp1, nc::Axis::ROW);
                                   
                }
            }
            //else(empty csi)
        }
        else{
            itr_k = tx_length + 1;
        }
    }

    // forward_reverse_channel_product = forward_reverse_channel_product({0, itr_l},forward_reverse_channel_product.cSlice());
    // std::cout << "size: "<< forward_reverse_channel_product.shape() << std::endl;
    // std::cout << "size: "<< csi_timestamp.shape() << std::endl;

//    nc::NdArray<nc::uint32> sortedIdxs = argsort(csi_timestamp);
//    nc::NdArray<double> sorted_csi_timestamp;
//    nc::NdArray<std::complex<double>> sorted_csi_data;
//    for (auto Idx : sortedIdxs)
//    {
//        sorted_csi_timestamp = nc::vstack({sorted_csi_timestamp, csi_timestamp(Idx,csi_timestamp.cSlice())});
//        sorted_csi_data = nc::vstack({sorted_csi_data, forward_reverse_channel_product(Idx,forward_reverse_channel_product.cSlice())});
//    }
    std::cout << "timestamp diff variance:" << nc::var(nc::diff(csi_timestamp)) << std::endl;

    return std::make_pair(forward_reverse_channel_product, csi_timestamp);
}
//=============================================================================================================================
/**
 * Match the forward and backward channel packets based on counter
 * input: CSI datapacket for transmitter and receiver
 * output: pair of NdArray pair<forward-reverse product, csi_timestamp>
 **/
std::pair<nc::NdArray<std::complex<double>>,nc::NdArray<double>> WSR_Util::getForwardReverseChannelCounter(
                                                                std::vector<DataPacket> rx_robot,
                                                                std::vector<DataPacket> tx_robot,
                                                                bool interpolate_phase,
                                                                bool sub_sample){
    int tx_length = int(tx_robot.size()), rx_length = int(rx_robot.size());
    int itr_k=0, itr_l=0;
    double a;
    bool first_csi_val = true;
    std::cout.precision(15);

    nc::NdArray<std::complex<double>> forward_reverse_channel_product;
    nc::NdArray<double> csi_timestamp;
    nc::NdArray<std::complex<double>> temp1 = nc::zeros<std::complex<double>>(nc::Shape(1,31));
    nc::NdArray<double> temp2 = nc::zeros<double>(nc::Shape(1,1));
    double interpolated_phase;
    std::complex<double> interpolated_h;

    while(itr_k < tx_length && itr_l < rx_length)
        {
            //std::cout << "TX frame: " << tx_robot[itr_k].frame_count << ", RX frame:" << rx_robot[itr_l].frame_count << std::endl;
            if (tx_robot[itr_k].frame_count == rx_robot[itr_l].frame_count)
            {
                //multiply forward and reverse channel
                for(int h_i = 0;h_i< 30; h_i++)
                {
                    temp1(0,h_i) = rx_robot[itr_l].csi[h_i][0] * tx_robot[itr_k].csi[h_i][0]; //Use CSI from first antenna only
                }
                if(interpolate_phase) 
                {
                    auto central_snum = nc::NdArray<double>(1, 1) = 15.5;
                    auto xp = nc::arange<double>(0, 31);
                    auto fp = unwrap(nc::angle(temp1(0,temp1.cSlice())));
                    auto mag1 = nc::abs(temp1(0,15));
                    auto mag2 = nc::abs(temp1(0,16));
                    auto fitted =  nc::polynomial::Poly1d<double>::fit(xp.transpose(),fp.transpose(),1);
                    interpolated_phase = nc::unwrap(fitted(central_snum(0,0)));
                    interpolated_h = (mag1+mag2)/2*nc::exp(std::complex<double>(0,1)*interpolated_phase);
                    
                    //Store the interpolated phase as subcarrier 31
                    temp1(0,30) = interpolated_h;
                }
                assert(temp2(0,0) != rx_robot[itr_l].ts);
                temp2(0,0) = rx_robot[itr_l].ts;
                itr_k +=1;
                itr_l +=1;

                if(first_csi_val)
                {
                    // if(interpolate_phase)
                    //     forward_reverse_channel_product = nc::NdArray<std::complex<double>>{interpolated_h};
                    // else
                    forward_reverse_channel_product = temp1;
                    csi_timestamp = temp2;
                    first_csi_val = false;
                }
                else
                {
                    if(sub_sample && itr_l%2!=0) continue;
                    
                    csi_timestamp = nc::append(csi_timestamp, temp2, nc::Axis::ROW);
                    forward_reverse_channel_product = nc::append(forward_reverse_channel_product, temp1, nc::Axis::ROW);

                    // if(interpolate_phase)
                    //     forward_reverse_channel_product = nc::append(forward_reverse_channel_product,nc::NdArray<std::complex<double>>{interpolated_h},nc::Axis::ROW);
                                     
                }
            }
            else if (tx_robot[itr_k].frame_count < rx_robot[itr_l].frame_count )
                itr_k += 1;
            else
                itr_l += 1;
            
        }

    return std::make_pair(forward_reverse_channel_product, csi_timestamp);
}
//=============================================================================================================================
/**
 * 
 * 
 * */
int WSR_Util::formatTrajectory(std::vector<std::vector<double>>& rx_trajectory, 
                            Eigen::MatrixXd& displacement, 
                            Eigen::MatrixXd& trajectory_timestamp)
{
    
    std::cout.precision(9);
    double nsec_timestamp;

    for(int i=0; i<rx_trajectory.size(); i++){
        nsec_timestamp = rx_trajectory[i][0] + rx_trajectory[i][1]*0.000000001;
        trajectory_timestamp(i,0) = nsec_timestamp; //timestamp  //TODO: fix bug #8
        displacement(i,0) = rx_trajectory[i][2]; //x
        displacement(i,1) = rx_trajectory[i][3]; //y
        displacement(i,2) = rx_trajectory[i][4]; //z
    }

    //@TODO : use argsort to sort based on timestamp
    
    //normalize based on first pose
    double first_x = displacement(0, 0), first_y = displacement(0, 1), first_z = displacement(0, 2);
    for(int i=0; i<displacement.rows(); i++)
    {
        displacement(i,0)  =  displacement(i,0) - first_x;
        displacement(i,1)  =  displacement(i,1) - first_y;;
        displacement(i,2)  =  displacement(i,2) - first_z;;
    }
    
    return 0;
}
//=============================================================================================================================
/**
 * 
 * 
 * */
std::pair<nc::NdArray<double>, nc::NdArray<double>> WSR_Util::getRelativeTrajectory(
                                                    std::vector<std::vector<double>>& trajectory_tx,
                                                    std::vector<std::vector<double>>& trajectory_rx,
                                                    std::vector<double>& antenna_offset)
{

    //TODO : return mean pos for the relative trajectory.
    nc::NdArray<double> mean_pos1,mean_pos2;
    auto ret_val = formatTrajectory_v2(trajectory_tx,antenna_offset,mean_pos1); //Assumes both robots have identical antenna offset to local displacement sensor
    nc::NdArray<double> timestamp_tx = ret_val.first;
    nc::NdArray<double> displacement_tx = ret_val.second;

    auto ret_val2 = formatTrajectory_v2(trajectory_rx,antenna_offset,mean_pos2);
    nc::NdArray<double> timestamp_rx = ret_val2.first;
    nc::NdArray<double> displacement_rx = ret_val2.second;
    
    nc::NdArray<double> displacement, timestamp;

    if(displacement_tx.shape().rows != displacement_rx.shape().rows)
    {
        std::cout << "Mismatch of poses for moving ends" << std::endl;
        auto ret = match_trajectory_timestamps(timestamp_tx,
                                               displacement_tx,
                                               timestamp_rx,
                                               displacement_rx);
        displacement = ret.first;
        timestamp = ret.second;
        
    }
    else
    {
        for(int i =0;i<displacement_tx.shape().rows;i++)
        {
            displacement_tx.put(i,0,displacement_tx(i,0)-displacement_rx(i,0));
            displacement_tx.put(i,1,displacement_tx(i,1)-displacement_rx(i,1));
            displacement_tx.put(i,2,displacement_tx(i,2)-displacement_rx(i,2));
        }
        displacement = displacement_tx;
        timestamp = timestamp_tx;
    }
    

    //Keep the timestamp of tx which performs SAR
    return std::make_pair(timestamp,displacement);

}
//=============================================================================================================================
/**
 * 
 * 
 * */
std::pair<nc::NdArray<double>, nc::NdArray<double>> WSR_Util::match_trajectory_timestamps(
                                                nc::NdArray<double> timestamp_tx,
                                                nc::NdArray<double> displacement_tx,
                                                nc::NdArray<double> timestamp_rx,
                                                nc::NdArray<double> displacement_rx)
{   
    nc::NdArray<double> timestamp, displacement;
    int rec_length = int(timestamp_rx.size()), tra_length = int(timestamp_tx.size());
    int itr_k=0, itr_l=0;
    double a;

    while(itr_k < rec_length) { //&& not(isempty(receiver{k}))){
        if(itr_l < tra_length){
            a = timestamp_tx(itr_l,0) - timestamp_rx(itr_k,0) + __offset;
            if (abs(a) > __threshold_traj || a<0){
                std::cout << "abs a greater than threshold or less than 0" << std::endl;
                if(timestamp_tx(itr_l,0) > timestamp_rx(itr_k,0)) 
                    itr_k += 1;     
                else 
                    itr_l += 1;
            }
            else{
                displacement_tx.put(itr_l,0,displacement_tx(itr_l,0)-displacement_rx(itr_k,0));
                displacement_tx.put(itr_l,1,displacement_tx(itr_l,1)-displacement_rx(itr_k,1));
                displacement_tx.put(itr_l,2,displacement_tx(itr_l,2)-displacement_rx(itr_k,2));
                displacement = nc::vstack({displacement, displacement_tx(itr_l,displacement_tx.cSlice())});
                timestamp = nc::vstack({timestamp, timestamp_tx(itr_l,timestamp_tx.cSlice())});
                itr_k +=1;
                itr_l +=1;
            }
        }
        else{
            itr_k = rec_length + 1;
        }
    }
    return std::make_pair(timestamp,displacement);
}
//=============================================================================================================================
/**
 * 
 * 
 * */
std::pair<nc::NdArray<double>, nc::NdArray<double>> WSR_Util::formatTrajectory_v2(
                            std::vector<std::vector<double>>& rx_trajectory,
                            std::vector<double>& antenna_offset,
                            nc::NdArray<double>& mean_pos)
{
    
    nc::NdArray<double> displacement(rx_trajectory.size(),3);
    nc::NdArray<double> trajectory_timestamp(rx_trajectory.size(),1);
    std::cout.precision(9);
    double nsec_timestamp;

    for(int i=0; i<rx_trajectory.size(); i++){
        nsec_timestamp = rx_trajectory[i][0] + rx_trajectory[i][1]*0.000000001;
        trajectory_timestamp(i,0) = nsec_timestamp; //timestamp  //TODO: fix bug #8
        displacement(i,0) = rx_trajectory[i][2] - antenna_offset[0]; //x
        displacement(i,1) = rx_trajectory[i][3] - antenna_offset[1]; //y
        displacement(i,2) = rx_trajectory[i][4] - antenna_offset[2]; //z
    }

    nc::NdArray<nc::uint32> sortedIdxs = argsort(trajectory_timestamp);
    nc::NdArray<double> sorted_trajectory_timestamp(trajectory_timestamp.size(),1);
    nc::NdArray<double> sorted_displacement;
    nc::uint32 counter = 0;
    

    for (auto Idx : sortedIdxs)
    {
        // std::cout << counter << std::endl;
        sorted_trajectory_timestamp.put(counter,0,trajectory_timestamp(Idx,0));
        sorted_displacement = nc::vstack({sorted_displacement, displacement(Idx,displacement.cSlice())});
        counter++;
    }
    

    //Find first and last moving index
    //@TODO : Figure out for z_displacment when 3D traj (vertical motion)
    double first_x = sorted_displacement(0, 0), first_y = sorted_displacement(0, 1), first_z = sorted_displacement(0, 2);
    double lastX = sorted_displacement(-1,0), lastY =sorted_displacement(-1,1), lastZ = sorted_displacement(-1,2);
    nc::NdArray<double> temp1 = abs(sorted_displacement(sorted_displacement.rSlice(),0)-first_x);
    nc::NdArray<double> temp2 = abs(sorted_displacement(sorted_displacement.rSlice(),1)-first_y);
    nc::NdArray<double> temp3 = abs(sorted_displacement(sorted_displacement.rSlice(),2)-first_z);
    nc::NdArray<double> temp4 = abs(sorted_displacement(sorted_displacement.rSlice(),0)-lastX);
    nc::NdArray<double> temp5 = abs(sorted_displacement(sorted_displacement.rSlice(),1)-lastY);
    nc::NdArray<double> temp6 = abs(sorted_displacement(sorted_displacement.rSlice(),2)-lastZ);
    
    auto start_x = nc::argwhere(temp1 > 0.006);
    auto start_y = nc::argwhere(temp2 > 0.006);
    auto start_z = nc::argwhere(temp3 > 0.006);
    auto end_x = nc::argwhere(temp4 > 0.006);
    auto end_y = nc::argwhere(temp5 > 0.006);
    auto end_z = nc::argwhere(temp6 > 0.006);
    
    int start_x_val = sorted_displacement.shape().rows;
    int start_y_val = sorted_displacement.shape().rows;
    int start_z_val = sorted_displacement.shape().rows;
    int end_x_val = 0;
    int end_y_val = 0;
    int end_z_val = 0;

    //To handle no change along a specific dimension
    if(start_x.shape().cols > 0) start_x_val = start_x(0,0);
    if(start_y.shape().cols > 0) start_y_val = start_y(0,0);
    if(start_z.shape().cols > 0) start_z_val = start_z(0,0);
    if(end_x.shape().cols > 0) end_x_val = end_x(0,-1);
    if(end_y.shape().cols > 0) end_y_val = end_y(0,-1);
    if(end_z.shape().cols > 0) end_z_val = end_z(0,-1);
    
    int start_index = std::min(std::min(start_x_val,start_y_val), start_z_val);
    int end_index = std::max(std::max(end_x_val,end_y_val), end_z_val); 

    //normalize based on first moving pose
    first_x = sorted_displacement(start_index, 0);
    first_y = sorted_displacement(start_index, 1);
    first_z = sorted_displacement(start_index, 2);
    mean_pos = nc::mean(sorted_displacement({start_index,end_index},sorted_displacement.cSlice()));

    for(int i=start_index; i<end_index+1; i++)
    {
        sorted_displacement.put(i,0,sorted_displacement(i, 0) - first_x);
        sorted_displacement.put(i,1,sorted_displacement(i, 1) - first_y);
        sorted_displacement.put(i,2,sorted_displacement(i, 2) - first_z);
    }

    // std::cout << sorted_trajectory_timestamp({start_index,end_index},sorted_trajectory_timestamp.cSlice()).shape() <<std::endl;
    // std::cout << "sorted traj shape " << sorted_displacement({start_index,end_index},sorted_displacement.cSlice()).shape() << std::endl;

    return std::make_pair(sorted_trajectory_timestamp({start_index,end_index},sorted_trajectory_timestamp.cSlice()), 
                         sorted_displacement({start_index,end_index},sorted_displacement.cSlice()));
}


//=============================================================================================================================
/**
 * 
 * 
 * */
std::pair<int,int> WSR_Util::returnClosestIndices(const nc::NdArray<double>& csi_timestamp,
                                                 const nc::NdArray<double>& trajectory_timestamp)
{
    //all timestamps are sorted

    int start_index = 0, end_index=csi_timestamp.shape().rows;
    // std::cout << trajectory_timestamp.shape() << std::endl;
    // std::cout << trajectory_timestamp << std::endl;
    double min_timestamp = trajectory_timestamp(0,0); 
    double max_timestamp = trajectory_timestamp(-1,0);
    
    std::cout.precision(9);
    // std::cout << fixed << min_timestamp << ", " << max_timestamp << std::endl;

    auto startIndexes = nc::argwhere(csi_timestamp > min_timestamp);
    auto endIndexes = nc::argwhere(csi_timestamp <= max_timestamp);
    
    // std::cout << startIndexes.shape() << std::endl;
    // std::cout << endIndexes.shape() << std::endl;
    
    /* for interpolation, we need to check these conditions are false
    ** csi_timestamp.min().item() < trajectory_timestamp.min().item() ====> condition 1
    ** csi_timestamp.max().item() > trajectory_timestamp.max().item() ====> condition 2
    */

    if(startIndexes.size() == 0){
        std::cout << "ERROR: CSI Collection stopped before executing trajectory" << std::endl;
        exit(1);
    }
    else 
        start_index = startIndexes(0,0);

    if(endIndexes.size() == 0)
    {
        std::cout << "ERROR: CSI Collection started after executing trajectory" << std::endl;
        // end_index = start_index + trajectory_timestamp.shape().rows + 1;
        exit(1);
    }
    else
        end_index = endIndexes.back();    
    
    // std::cout << "CSI start index = " << start_index <<", endIndex = " << end_index << std::endl;
    return std::make_pair(start_index, end_index);
}


//=============================================================================================================================
/**
 * Inputs: inXp ==> csi_timestamp, inX ==> trajectory_timestamp, inFp ==> trajectory
 * 
 * */
std::pair<nc::NdArray<double>,std::vector<size_t> > WSR_Util::interpolate(const nc::NdArray<double>& inX, 
                                        const nc::NdArray<double>& inXp, 
                                        const nc::NdArray<double>& inFp)
{

    // check the dimension fit
    if (nc::shape(inXp).rows != nc::shape(inFp).rows)
    {
        THROW_CSI_INVALID_ARGUMENT_ERROR("time list and pose list should have same length");
        std::cout << "time list and pose list should have same length" << std::endl;
    }
    
    if (nc::shape(inFp).cols != 3)
    {
        THROW_CSI_INVALID_ARGUMENT_ERROR("the shape of pose matrix should be n by 3");
        std::cout << "the shape of pose matrix should be n by 3" << std::endl;
    }
    
    if (inX(0,0) > inXp(-1,0) || inX(-1,0)<inXp(0,0))
    {
        THROW_CSI_INVALID_ARGUMENT_ERROR("the range of time to be interpolated has no overlap with pose time");
        std::cout << "the shape of pose matrix should be n by 3" << std::endl;
    }

    int num_poses = nc::shape(inFp).rows;
    int num_csi = nc::shape(inX).rows;
    size_t start_idx=0, end_idx = nc::shape(inX).rows-1;
    
    while(inX(start_idx,0) < inXp(0,0))
    {
        start_idx++;
    }
    while(inX(end_idx,0) > inXp(-1,0))
    {
        end_idx--;
    }
    
    auto cleaned_inX = inX(nc::Slice(start_idx,end_idx+1),inX.cSlice());
    // sort the input inX array
    nc::NdArray<double> sortedX = sort(cleaned_inX);
    nc::NdArray<double> returnArray( cleaned_inX.size(),3);
    nc::uint32 currXpIdx = 0;
    nc::uint32 currXidx = 0;
    
    while (currXidx < sortedX.size())
    {
        if (inXp[currXpIdx] <= sortedX[currXidx] && sortedX[currXidx] <= inXp[currXpIdx + 1])
        {
            const double percent = static_cast<double>(sortedX[currXidx] - inXp[currXpIdx]) /
                                    static_cast<double>(inXp[currXpIdx + 1] - inXp[currXpIdx]);
            returnArray.put(currXidx,0, nc::utils::interp(inFp(currXpIdx,0), inFp(currXpIdx + 1,0), percent));
            returnArray.put(currXidx,1, nc::utils::interp(inFp(currXpIdx,1), inFp(currXpIdx + 1,1), percent));
            returnArray.put(currXidx++,2, nc::utils::interp(inFp(currXpIdx,2), inFp(currXpIdx + 1,2), percent));
        }
        else
        {
            ++currXpIdx;
        }
    }

    std::vector<size_t> indices = {start_idx, end_idx};
    return std::make_pair(returnArray, indices);

}
//=============================================================================================================================
/**
 * Inputs: inXp ==> csi_timestamp, inX ==> trajectory_timestamp, inFp ==> trajectory
 * 
 * */
void WSR_Util::writeToFile(nc::NdArray<double>& nd_array, std::string fn)
{
    ofstream myfile (fn);
    if (myfile.is_open())
    {
        for(size_t i = 0; i < nd_array.shape().rows; i++)
        {
            for(size_t j = 0; j < nd_array.shape().cols-1; j++) {
                auto value = nd_array(i,j);
                // std::cout << fixed << value << std::endl;
                myfile << fixed << value << ",";
            }
            myfile << fixed << nd_array(i,nd_array.shape().cols-1);
            // break;
            myfile << "\n";
        }
        
    }
    myfile.close();
}
//=============================================================================================================================
/**
 * Inputs: inXp ==> csi_timestamp, inX ==> trajectory_timestamp, inFp ==> trajectory
 * 
 * */ 
void WSR_Util::writeCSIToFile(nc::NdArray<std::complex<double>>& nd_array, string fn)
{
    std::cout.precision(15);
    ofstream myfile1 (fn+"_real.csv");
    ofstream myfile2 (fn+"_img.csv");

    if (myfile1.is_open() && myfile2.is_open())
    {
        for(size_t i = 0; i < nd_array.shape().rows; i++)
        {
            for(size_t j = 0; j < nd_array.shape().cols; j++) {
                auto value = nd_array(i,j);
                // std::cout << fixed << value << std::endl;
                myfile1 << fixed << value.real() << ",";
                myfile2 << fixed << value.imag() << ",";
            }
            // break;
            myfile1 << "\n";
            myfile2 << "\n";
        }
        
    }
    myfile1.close();
    myfile2.is_open();
}
//=============================================================================================================================
/**
 * 
 * 
 * */ 
void WSR_Util::writeCSIToJsonFile(nc::NdArray<std::complex<double>>& nd_array, 
                                nc::NdArray<double>&timestamp, 
                                const std::string& fn,
                                bool __FLAG_interpolate_phase)
{
    string output_file = __homedir+"/"+fn, key,key_sub;
    std::cout.precision(15);
    ofstream myfile (output_file);
    nlohmann::json interpl_traj;
    interpl_traj["channel_packets"] = {};

    if (myfile.is_open())
    {
        for(size_t i = 0; i < nd_array.shape().rows; i++)
        {
            key = std::to_string(i);
            std::complex<double> value;

            if(__FLAG_interpolate_phase)
                value = nd_array(i,30); //The interpolated phase is stored as subcarrier 31
            else
                value = nd_array(i,15);//by default it shows the subcarrier 16

            interpl_traj["channel_packets"][key]["center_subcarrier_phase"] = std::arg(value);
            interpl_traj["channel_packets"][key]["timestamp"] = timestamp(i,0);
            // interpl_traj["channel_packets"][key]["numcpp_center_subcarrier"] = nc::angle(value);
        }
        myfile << interpl_traj;
    }
    myfile.close();
}
//=============================================================================================================================
/**
 * 
 * 
 * */
void WSR_Util::writeTrajToFile(std::vector<std::vector<double>>& rx_trajectory, std::string fn)
{
    std::cout.precision(15);
    ofstream myfile (fn);
    std::vector<double> temp;

    std::cout << "Trajectory size " << rx_trajectory.size() << std::endl;
    if (myfile.is_open())
    {
        for(size_t i = 0; i < rx_trajectory.size(); i++)
        {
            for(int j=0; j< rx_trajectory[i].size(); j++)
            {
                myfile << fixed << rx_trajectory[i][j] << ",";
            }
            myfile << "\n";
        }
        
    }
    myfile.close();
}
//=============================================================================================================================
/**
 * 
 * 
 * */
void WSR_Util::writeTrajToFile(nc::NdArray<double>& rx_trajectory, const std::string& fn)
{
    string output_file = __homedir+"/"+fn, key;
    std::cout.precision(15);
    ofstream myfile (output_file);
    nlohmann::json interpl_traj;
    interpl_traj["pose_list"] = {};

    std::cout << "log [Utils]: Trajectory pose count " << rx_trajectory.shape().rows << std::endl;
    if (myfile.is_open())
    {
        for(size_t i = 0; i < rx_trajectory.shape().rows; i++)
        {
            nlohmann::json temp = {
               {"x",rx_trajectory(i,0)},
               {"y",rx_trajectory(i,1)},
               {"z",rx_trajectory(i,2)}, 
            };
            key = std::to_string(i);
            interpl_traj["pose_list"][key] = temp;
        }
        myfile << interpl_traj;   
    }
    myfile.close();
}
//=============================================================================================================================
/**
 * Inputs: inXp ==> csi_timestamp, inX ==> trajectory_timestamp, inFp ==> trajectory
 * 
 * */
std::vector<std::vector<double>> WSR_Util::loadTrajFromCSV(std::string traj_fn)
{
    std::vector<std::vector<double>> rx_trajectory;

    // File pointer 
    fstream traj_file;
    traj_file.open( traj_fn.c_str() , ios::in );
    if (!traj_file.is_open()) {
        fputs ("Error opening Trajectory file \n",stderr); 
        traj_file.close();
        exit (1);
    }

    std::string line, temp, word;
    std::vector<double> traj_temp;

    while (traj_file >> temp) 
    {   
        traj_temp.clear();
        std::stringstream s(temp); 
        while (std::getline(s, word, ',')) { 
            traj_temp.push_back(std::stod(word)); 
        }
        rx_trajectory.push_back(traj_temp); 
    }


    traj_file.close();
    return rx_trajectory;
}
//=============================================================================================================================
/**
 * additional reference : https://www.tutorialspoint.com/how-to-get-memory-usage-at-runtime-using-cplusplus
 * 
 * */
double WSR_Util::mem_usage()
{
   string line;
   double memory_used;
   string tmp;
   ifstream stat_stream2("/proc/self/status");
   int mem_line_num = 16, line_count = 0;
   if (stat_stream2.is_open())
    {
        while ( getline (stat_stream2,line) )
        {
            if(line_count == mem_line_num)
            {
                std::stringstream ss(line);
                ss >> tmp >> memory_used;
                //std::cout << "log [Utils] : Peak Memory Usage " << line << '\n';
                break;
            }
            else
            {
                line_count++;
                continue;
            }
        }
        stat_stream2.close();
    }
    return memory_used;
}
//=============================================================================================================================
/**
 *
 *
 * */
std::unordered_map<std::string, std::pair<double,double>> WSR_Util::get_true_aoa(std::vector<std::vector<double>>& rx_trajectory,
                                                                                 nlohmann::json true_positions_tx)
{
    std::unordered_map<std::string, std::pair<double,double>> true_aoa_angles;
    double temp;
    for (auto tx = true_positions_tx["value"].begin(); tx != true_positions_tx["value"].end(); tx++)
    {
        double true_phi=0, true_theta=0, x_diff=0, y_diff=0, z_diff=0;
        string tx_id = tx.key();
        nlohmann::json position = tx.value();
        double gt_x = double(position["position"]["x"]);
        double gt_y = double(position["position"]["y"]);
        double gt_z = double(position["position"]["z"]);

        //First position
        //std::cout << gt_x <<", " << displacement(0,0) << "," << gt_y << "," << displacement(0,1) << std::endl;
        x_diff = gt_x - rx_trajectory[0][2];
        y_diff = gt_y - rx_trajectory[0][3];
        z_diff = gt_z - rx_trajectory[0][4];
        double phi1 = std::atan2 (y_diff,x_diff);
//        true_phi += std::atan2 (y_diff,x_diff);
        temp = std::atan2(z_diff,std::sqrt(std::pow(x_diff,2) + std::pow(y_diff,2)));
        true_theta += (M_PI/2 - temp);

        //Last position
        x_diff = gt_x - rx_trajectory[int(rx_trajectory.size()-1)][2];
        y_diff = gt_y - rx_trajectory[int(rx_trajectory.size()-1)][3];
        z_diff = gt_z - rx_trajectory[int(rx_trajectory.size()-1)][4];
        double phi2 = std::atan2 (y_diff,x_diff);
//        true_phi += std::atan2 (y_diff,x_diff);
        temp = std::atan2(z_diff,std::sqrt(std::pow(x_diff,2) + std::pow(y_diff,2)));
        true_theta += (M_PI/2 - temp);

        //Take the average
        if(phi1*phi2 > 0)
            true_phi = (phi1+phi2)/2* 180/M_PI;
        else{
            if (phi1+phi2<0)
                true_phi = (M_PI+(phi1+phi2)/2)* 180/M_PI;
            else
                true_phi = (-M_PI+(phi1+phi2)/2)* 180/M_PI;
        }


        true_theta = true_theta/2 * 180/M_PI;
        true_aoa_angles[tx_id] = std::make_pair(true_phi, true_theta);

    }

    return true_aoa_angles;
}
//=============================================================================================================================
/**
 *
 *
 * */
nc::NdArray<double> WSR_Util::unwrap(const nc::NdArray<double> &phase_list) {
    auto unwrapped_phase_list = nc::NdArray<double>(phase_list.shape());
    int k = 0;
    double alpha = M_PI;
    for (size_t i = 0; i < 29; i++) {
        unwrapped_phase_list(0,i) = phase_list(0,i) + 2*M_PI*k;
        if(abs(phase_list(0,i+1)-phase_list(0,i))>alpha){
            if (phase_list(0,i+1)<phase_list(0,i))
                k++;
            else
                k--;
//            float d = phase_list[i] - unwrapped_phase_list[i-1];
//            d = d > M_PI ? d - 2 * M_PI : (d < -M_PI ? d + 2 * M_PI : d);
        }
    }
    unwrapped_phase_list(0,29) = phase_list[29] + 2*M_PI*k;

    return unwrapped_phase_list;
}
//=============================================================================================================================
/**
 *
 *
 * */
double floor_mod(double a, double b){
    return a-floor(a/b)*b;
}
//=============================================================================================================================
/**
 *
 *
 * */
double WSR_Util::anglediff(double a, double b)
{
    double temp  = a - b;
    auto ret_val = floor_mod(temp+180,360) - 180;
//    double
    return ret_val;
}
//=============================================================================================================================
/**
 *implementation of unravel_index in numpy
 *
 * */
nc::NdArray<int> WSR_Util::unravel_index(int ind, int row, int col){
    return nc::NdArray<int>({int(ind/col),ind-int(ind/col)*col});
}
//=============================================================================================================================
/**
 *Helper function for debugging
 *
 * */
std::string WSR_Util::bool_to_string(bool value)
{
    if(value)
        return "true";
    else
        return "false"; 
}
//=============================================================================================================================
/**
 *
 *
 * */
std::string WSR_Util::format_mac(std::string const& s) {
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
void WSR_Util::writePacketDistributionToJsonFile(const nc::NdArray<double>& csi_timestamp, 
                                                const nc::NdArray<double>& trajectory_timestamp, 
                                                const nc::NdArray<double>& displacement,
                                                std::string fn)
{
    string output_file = __homedir+"/"+fn, key;
    std::cout.precision(15);
    ofstream myfile (output_file);
    nlohmann::json packet_distribution;
    packet_distribution["pose_list"] = {};

    int k = 0;

    if (myfile.is_open())
    {
        for(size_t i = 0; i < csi_timestamp.shape().rows; i++)
        {
            //Find the first trajectory timestamp >= csi_timestamp
            for(size_t k = 0; k < trajectory_timestamp.shape().rows; k++)
            {
                if(trajectory_timestamp(k,0) < csi_timestamp(i,0))
                {
                    continue;
                }
                else
                {
                    nlohmann::json temp = {
                        {"x",displacement(k,0)},
                        {"y",displacement(k,1)},
                        {"z",displacement(k,2)}, 
                    };
                    key = std::to_string(i);
                    packet_distribution["pose_list"][key] = temp;
                    break;
                }
            }
        }
        myfile << packet_distribution;   
    }
    myfile.close();
}
//=============================================================================================================================
/**
 * CFO clean using complex conjugate
 **/
std::pair<nc::NdArray<std::complex<double>>,nc::NdArray<double>> WSR_Util::getConjugateProductChannel(
                                                                std::vector<DataPacket> rx_robot,
                                                                bool interpolate_phase,
                                                                bool sub_sample){
    int rx_length = int(rx_robot.size());
    int itr_k=0, itr_l=0;
    double a;
    bool first_csi_val = true;
    std::cout.precision(15);
    int sizeee = rx_robot.size();

    nc::NdArray<std::complex<double>> forward_reverse_channel_product;
    nc::NdArray<double> csi_timestamp;
    nc::NdArray<std::complex<double>> temp1 = nc::zeros<std::complex<double>>(nc::Shape(1,30));
    nc::NdArray<double> temp2 = nc::zeros<double>(nc::Shape(1,1));
    double interpolated_phase;
    std::complex<double> interpolated_h;

    auto ArrayShape = forward_reverse_channel_product.shape();

    while(itr_l < rx_length) { //&& not(isempty(receiver{k}))){
        //multiply forward and reverse channel
        for(int h_i = 0;h_i< 30; h_i++)
        {
            temp1(0,h_i) = rx_robot[itr_l].csi[h_i][1] * nc::conj(rx_robot[itr_l].csi[h_i][0]); //using complex conjugate
        }
        if(interpolate_phase) 
        {
            auto central_snum = nc::NdArray<double>(1, 1) = 15.5;
            auto xp = nc::arange<double>(1, 31);
            auto fp = unwrap(nc::angle(temp1(0,temp1.cSlice())));
            auto mag1 = nc::abs(temp1(0,15));
            auto mag2 = nc::abs(temp1(0,16));
            auto fitted =  nc::polynomial::Poly1d<double>::fit(xp.transpose(),fp.transpose(),1);
            interpolated_phase = nc::unwrap(fitted(central_snum(0,0)));
            interpolated_h = (mag1+mag2)/2*nc::exp(std::complex<double>(0,1)*interpolated_phase);
        }
        assert(temp2(0,0) != rx_robot[itr_l].ts);
        temp2(0,0) = rx_robot[itr_l].ts;
        itr_k +=1;
        itr_l +=1;

        if(first_csi_val)
        {
            if(interpolate_phase)
                forward_reverse_channel_product = nc::NdArray<std::complex<double>>{interpolated_h};
            else
                forward_reverse_channel_product = temp1;

            csi_timestamp = temp2;
            first_csi_val = false;
        }
        else
        {
            if(sub_sample && itr_l%2!=0) continue;
            
            csi_timestamp = nc::append(csi_timestamp, temp2, nc::Axis::ROW);
            assert(csi_timestamp(-2,0)  < csi_timestamp(-1,0));
            if(interpolate_phase)
                forward_reverse_channel_product = nc::append(forward_reverse_channel_product,nc::NdArray<std::complex<double>>{interpolated_h},nc::Axis::ROW);
            else
                forward_reverse_channel_product = nc::append(forward_reverse_channel_product, temp1, nc::Axis::ROW);
                            
        }
    }
    std::cout << "timestamp diff variance:" << nc::var(nc::diff(csi_timestamp)) << std::endl;

    return std::make_pair(forward_reverse_channel_product, csi_timestamp);
}
//=============================================================================================================================
/**
 *
 *
 * */
std::string WSR_Util::dec2hex(unsigned int i)
{
    std::stringstream ss;
    ss << std::hex << std::uppercase << i;
    return ss.str();
}
//=============================================================================================================================
/**
 * 
 * 
 * */
void WSR_Util::get_phase_diff_metrics(nc::NdArray<std::complex<double>>& channel,
                            double &mean,
                            double &stdev,
                            bool& interpolate_phase,
                            bool& moving)
{

    std::vector<double> phase_diff;
    
    if(moving)
    {
        for(int i=1; i<channel.shape().rows;i++)
        {
            if(interpolate_phase)
                phase_diff.push_back(abs((std::arg(channel(i-1,30)) - std::arg(channel(i,30)))));
            else
                phase_diff.push_back(abs((std::arg(channel(i-1,15)) - std::arg(channel(i,15)))));
        }
    }
    else
    {
        for(int i=0; i<channel.shape().rows;i++)
        {
            if(interpolate_phase)
                phase_diff.push_back(abs((std::arg(channel(i,30)))));
            else
                phase_diff.push_back(abs((std::arg(channel(i,15)))));
        }
    }

    

    double sum = std::accumulate(std::begin(phase_diff), std::end(phase_diff), 0.0);
    mean =  sum / phase_diff.size();

    double var = 0.0;
    std::for_each (std::begin(phase_diff), std::end(phase_diff), [&](const double d) {
        var += (d - mean) * (d - mean);
    });

    stdev = sqrt(var / (phase_diff.size()-1));
}
//=============================================================================================================================
/**
 * 
 * 
 * */
double WSR_Util::diff_360(double a, double b) {
    double tmp = a - b;
    if (tmp > 180)
        tmp -= 360;
    else if(tmp < -180)
        tmp += 360;
    return tmp;
}
