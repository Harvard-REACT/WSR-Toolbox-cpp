#!/usr/bin/env python
from __future__ import print_function
import argparse
import json
import profile

'''
Filter the json data to get specific fields
'''

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", help="Channel data file")
    args = parser.parse_args()
        
    f = open(args.file,"r") 
    data_json = json.loads(f.read())


    exp_sample_required = "1"
    TX_id = "tx2"

    i = 1
    done = True
    count = 1
    while i < len(data_json)+1:
        print("Key", str(count))
        
        dir_exp = data_json
        
        for key,dir_val in dir_exp.items():
            if(dir_val["1"][0]["b_INFO_Receiving_robot"]["id"] == i):
                print("Sample :", str(exp_sample_required))
                tx_info = dir_val[exp_sample_required]
                for tx_val in tx_info:
                    if tx_val["a_INFO_Transmitting_robot"]["Name"] == TX_id:
                        print(tx_val["a_INFO_Transmitting_robot"])
                        print("*****")
                        print("RX robot location:", tx_val["b_INFO_Receiving_robot"]["id"])
                        print("Profile variance:", tx_val["d_INFO_AOA_profile"]["Profile_variance"])
                        print("*****")

                        print("Top N peaks info:")
                        for aoa,aoa_val in tx_val["d_INFO_AOA_profile"]["Top_N_peaks"].items():
                            print(aoa, " Estimated Azimuth: ", aoa_val["estimated_azimuth"])
                print("----------------------------------------------")
                # x = input("next")
                i = i+1

        count+=1

if __name__=="__main__":
    main()