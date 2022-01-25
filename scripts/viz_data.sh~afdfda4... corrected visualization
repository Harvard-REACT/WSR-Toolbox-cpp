#!/bin/bash

num_robot=0
icr=1
echo $2
for tx in $2;
do
   num_robot=`expr $num_robot + $icr`
done

mul=5
nrf=`expr $num_robot \* $mul`

# #Loop through all the tx to visualize their data
sleep 1
for tx in $2;
do

    echo "====== Fetching latest files for visualization TX : $tx ======"

    csi_phase_fn=$(ls -t1 $1 |  head -n $nrf | grep $tx | grep all_channel)
    echo "CSI Phase for $csi_phase_fn"
    python3.8 ~/catkin_ws/src/wsr_ros/scripts/viz_channel_data.py --file $1/$csi_phase_fn

    traj_pkt_fn=$(ls -t1 $1 |  head -n $nrf | grep $tx | grep packet)
    echo "Packet distribution along the trajectory for $traj_pkt_fn"
    python3.8 ~/catkin_ws/src/wsr_ros/scripts/viz_traj.py --file $1/$traj_pkt_fn

    profile_fn=$(ls -t1 $1 |  head -n $nrf | grep $tx | grep aoa)
    echo "AOA profile for $profile_fn"
    # python3.8 ~/catkin_ws/src/wsr_ros/scripts/visualize_aoa_profile.py --file $1/$profile_fn
    # python3.8 ~/catkin_ws/src/wsr_ros/scripts/viz_aoa_matlab.py --file $1$profile_fn


done

echo "*************************************"
echo "Completed all Visualizations. EXITING"