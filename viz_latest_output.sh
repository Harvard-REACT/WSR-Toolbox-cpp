#!/bin/bash

# echo "Running the toolbox code on latest data files"
../wsr_build/remote_processing $1 $2


#Loop through all the tx to visualize their data
sleep 1
for tx in $3;
do
    
    echo "====== Fetching latest files for visualization TX : $tx ======"

    profile_fn=$(ls -t1 |  head -n 10 | grep $tx | grep aoa)
    echo "AOA profile for $profile_fn"
    python3.8 ../scripts/visualize_aoa_profile.py --file $profile_fn

    csi_phase_fn=$(ls -t1 |  head -n 10 | grep $tx | grep all_channel)
    echo "CSI Phase for $csi_phase_fn"
    python3.8 ../scripts/viz_channel_data.py --file $csi_phase_fn

    traj_pkt_fn=$(ls -t1 |  head -n 10 | grep $tx | grep packet)
    echo "Packet distribution along the trajectory for $traj_pkt_fn"
    python3.8 ../scripts/viz_traj.py --file $traj_pkt_fn

done

echo "*************************************"
echo "Completed all Visualizations. EXITING"
