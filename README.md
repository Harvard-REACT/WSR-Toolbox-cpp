# WSR-Toolbox-cpp
Core C++ code repo for WSR toolbox with Cython wrapper.

## Supported Environments

### Ubuntu version tested
- [x] Ubuntu 16.04
- [ ] Ubuntu 18.04
- [x] Ubuntu 20.04
- [ ] TX2

### C++ version tested and supported
- ~~[ ] c++11~~
- [x] c++14
- [x] c++17
- [ ] c++20

### Python version tested
- ~~[ ] python 3.5~~
- ~~[ ] python 3.6~~
- [x] python 3.7
- [x] python 3.8

### GCC version tested
- [x] gcc 5.4.0
- [x] gcc 9.3.0

## Setup instructions (tested on UP Squared board)

1. Create a directory named WSR_Project and clone the repository in that directory.

2. Install python 3.7 (minimum supported version for using visualization scripts) and make it default version
```
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update
sudo apt install python3.7 python3.7-tk python3-pip python3.7-dev python3-gdbm cmake
sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.5 1
sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.7 2
sudo update-alternatives --config python3
```
Select python3.7 as the default version.

3. Install the python dependency packages
```
cd ~/WSR_Project
cd WSR-Toolbox-cpp
pip3 install Cython numpy pybind11 pythran
pip3 install -r requirements.txt
```

4. Download and compile the boost_1.68 locally in $HOME/Downloads.
```
cd ~/Downloads
wget http://downloads.sourceforge.net/project/boost/boost/1.68.0/boost_1_68_0.tar.gz
tar -zxvf boost_1_68_0.tar.gz
cd boost_1_68_0/
./bootstrap.sh
cpuCores=`cat /proc/cpuinfo | grep "cpu cores" | uniq | awk '{print $NF}'` 
sudo ./b2 --with=all -j $cpuCores
```
Note: Do not run install since will break the default boost installation required for ROS systems. 

5. Compile as a standalone C++ project

Create new subdirectory in the WSR-Toolbox folder to store the build 
```
cd ~/WSR_Project/WSR-Toolbox-cpp/
mkdir wsr_build && cd wsr_build
cmake ..
cpuCores=`cat /proc/cpuinfo | grep "cpu cores" | uniq | awk '{print $NF}'`
make -j $cpuCores
```

### Updating the config file 
The config file 'config_3D_SAR.json' is located in the config directory.

1. Add the MAC_ID and the location of the csi.dat file (obtained when collecting channel data) in the config file for the robot. If its a RX_SAR_robot then use the 'mac_id' filed in input_RX_channel_csi_fn :
```
  "input_RX_channel_csi_fn":{
      "desc":"Reverse channel csi File stored on the RX robot which is performing 3D SAR",
      "value":{           
          "mac_id":"00:21:6A:C5:FC:0",
          "mac_id_val":"0:33:106:197:252:0",
          "csi_fn":"/WSR-Toolbox-cpp/data/Line-of-Sight/2D_trajectory_sample/csi_data/csi_rx_2021-03-04_154746.dat"
      }
  }
```

If its a TX_Neighbor robot, then use the 'mac_id' filed in input_RX_channel_csi_fn input_TX_channel_csi_fn. e.g
```
"input_TX_channel_csi_fn":{
    "desc":"Forward channel csi File for each of the neighboring TX robots",
    "value":{
        "tx1":{
            "mac_id_val":"00:16:EA:12:34:56",
            "mac_id":"00:21:6A:C5:FC:0",
            "csi_fn":"/WSR-Toolbox-cpp/data/Line-of-Sight/2D_trajectory_sample/csi_data/csi_tx1_2021-03-04_154746.dat"
        },
    }
```
Note: Multiple TX_Neighbor robots can be added with id as tx1 (,tx2,tx3...and so on). Only add relative path of the repository.

2. Add the location of the trajectory file for TX_Neighbor robot in the field input_trajectory_csv_fn_rx.
```
"input_trajectory_csv_fn_rx":{
    "desc":"Trajectory file",
    "value":"/WSR-Toolbox-cpp/data/Line-of-Sight/2D_trajectory_sample/trajectory_data/rx_trajectory_2021-03-04_154746_.csv"
}
```

3. Make sure that the correct flags and parameters are set during initialization. (This requires that the 'debug' config parameter is set to 'true' to see the status during code execution):
```
log [Precomp]: Important FLAGS status
  Trajectory Type = "2D" (3D)
  __FLAG_packet_threshold = true
  __FLAG_debug = true
  __FLAG_threading = true
  __FLAG_interpolate_phase = true
  __FLAG_sub_sample = false
  __FLAG_normalize_profile = true
  __FLAG_use_multiple_sub_carriers = false
  __FLAG_use_magic_mac = false
```

4. Sample data for testing is available in data/Line-of-Sight folder (2D and 3D trajectories). Add the correct path of csi.data files for RX_SAR_robot (csi_rx*) and TX_neighbor_robot (csi_tx*) as well as as the corresponding trajectory file (rx_*) in the config.json.

5. The groundtruth positions can be added (if known) in the field (for the corresponding tx):
```
"true_tx_positions":
    {
        "desc":"True groundtruth positions of TX for checking accuracy",
        "value":
        {
            "tx1":{
                "position":{ 
                "x": 0,
                "y": 0,
                "z": 0
                },
                "orientation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "w": 0
                }
            }
        }
    }
```
Each sample data folder has 'groundtruth_positions.json' file.

6. When running the code in UP-Squared board, maximum packets that can be processed is ~450. Set the correct number in the field:
```
"max_packets_to_process":{
    "desc":"Maximum number of csi packets to process",
    "value":450
},
```
Other option to try is 'sub_sample_channel_data' which picks alternate packets.
```
"sub_sample_channel_data": {
    "desc": "flag for enabling subsampling of channle data to reduce packet count",
    "value": true
}
```
## Code Execution

### Test sample CSI data files
1. To test the CSI data and channel reciprocity module run the following (This requires that the correct file locations of the sample data are added in the config_3D_SAR.json):

```
cd wsr_build
./test_csi_data
```

### Calculate AOA profiles using Core C++ framework(needs trajectory information)

2. To test the AOA calculation run the following:
```
cd wsr_build
./test_wsr
```

## Visualization
All the data files pertaining to visualization are generated in the debug directory. The visualization scripts can be found in the **scripts directory**


1. To visualize the channel phase generated by the channel reciprocity module
```
python3.7 viz_channel_data.py --file <filename>


e.g.
python3.7 viz_channel_data.py --file ../debug/tx1_2021-03-04_154746_all_channel_data.json
```

2. To visualize the profile , go to scripts directory and use the visualize_aoa_profile.py
```
cd scripts
python3.7 visualize_aoa_profile.py --file <filepath>

e.g
python3.8 visualize_aoa_profile.py --file ../debug/tx1_aoa_profile_2021-03-04_154746.csv
```

3. To visualize interpolated trajectory
```
cd scripts
python3.7 viz_traj.py --file <filepath>

e.g.
python3.8 viz_traj.py --file ../debug/tx1_2021-03-04_154746_interpl_trajectory.json
```

To use matlab visualizer (recommended), run the following script instead
```
python3.8 viz_aoa_matlab.py --file ../debug/tx1_2021-03-04_154746_interpl_trajectory.json
```

Note :If the matlab viewer is used for visualizing the AOA profile, then make sure that the matlab api for python has be installed ([reference](https://www.mathworks.com/help/matlab/matlab_external/install-the-matlab-engine-for-python.html)).

3. To visualize packet distrubution
```
cd scripts
python3.7 viz_traj.py --file <filepath>

e.g.
python3.8 viz_traj.py --file ../debug/tx1_2021-03-04_154746_packet_dist.json
```


## Compiling and using Cpython modules (WIP)
The name of the path to sample data files is hardcoded for testing. Update it accordingly based on the your system.

1. For testing the csi-reader, first run the setup.py
```
$ python3 setup.py build_ext --inplace
```
Note: sometimes if the Cpython_modules/wsr_module.cpp file is not removed the changes made in C++ modules do not take effect.

### To test parsing of robot trajectory and csi data (using cpython library)
Once the library is compiled, the csi reader function can be called using python script. To test, run the following from scripts directory. e.g

```
cd scripts
python3.7 main.py --f ../data/3D_Helix_101_mocap_2020-03-04_102615/mocap_data_a.txt_2020-03-04_102615.txt --mocap_id 101 --parser_type optitrack
```

### Testing Trajectory data
Go to the scripts directory

1. Optitrack Mocap data
```
python3 main.py --f ../data/mocap_data_a.txt_2020-02-29_171359.txt --mocap_id 102 --parser_type optitrack
```

2. T265 data
```
python3 main.py --f ../data/mocap_data_a.txt_2020-02-29_171359.txt --parser_type t265
```
