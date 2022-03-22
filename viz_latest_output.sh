#!/bin/bash

#Fetch the files from the RX_SAR robot (enable password less ssh)
# rsync --ignore-existing up12@192.168.1.43:~/WSR_Project/WSR-Toolbox-cpp/data/online_data/* $1

#rsync --ignore-existing up-board-03@192.168.1.30:~/REACT-Projects/WSR-Toolbox-cpp/data/online_data/* $1

echo "Running the toolbox code on latest data files"
../wsr_build/remote_processing $1 $2 1

num_robot=0
icr=1
for tx in $3;
do
   num_robot=`expr $num_robot + $icr`
done

mul=5
nrf=`expr $num_robot \* $mul`

# #Loop through all the tx to visualize their data
sleep 1
for tx in $3;
do
     
    echo "====== Fetching latest files for visualization TX : $tx ======"

    csi_phase_fn=$(ls -t1 ../debug |  head -n $nrf | grep $tx | grep all_channel)
    echo "CSI Phase for $csi_phase_fn"
    python3.8 viz_channel_data.py --file ../debug/$csi_phase_fn

    traj_pkt_fn=$(ls -t1 ../debug |  head -n $nrf | grep $tx | grep packet)
    echo "Packet distribution along the trajectory for $traj_pkt_fn"
    python3.8 viz_traj.py --file ../debug/$traj_pkt_fn

    profile_fn=$(ls -t1 ../debug |  head -n $nrf | grep $tx | grep aoa)
    echo "AOA profile for $profile_fn"
    python3.8 visualize_aoa_profile.py --file ../debug/$profile_fn --nphi 180 --ntheta 90
    # python3.8 viz_aoa_matlab.py --file ../debug/$profile_fn --nphi 360 --ntheta 90 --phi_max 180 --theta_max 90
    

done

echo "*************************************"
echo "Completed all Visualizations. EXITING"
