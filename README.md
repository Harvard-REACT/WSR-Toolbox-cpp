# WSR-Toolbox-cpp
Calculate AOA for multiple files in batch for faster data processing. This branch is to be used only for post-processing the data files.

## Ubuntu version tested
- [x] Ubuntu 18.04
- [x] Ubuntu 20.04

## C++ version tested and supported
- [x] c++17

## GCC version tested
- [x] gcc 5.4.0
- [x] gcc 9.3.0

## Cmake Version 3.15

## Setup instructions

1. Clone the repository in your cakin workspace under csitoolbox directory and checkout the data_processing branch
```
git clone https://github.com/Harvard-REACT/WSR-Toolbox-cpp.git
git checkout -b data_processing
```

2. Install the python dependency packages
```
pip3 install -r requirements.txt
```

3. Download and compile the boost_1.68 locally in $HOME/Downloads.
```
1. cd Downloads
2. wget http://downloads.sourceforge.net/project/boost/boost/1.68.0/boost_1_68_0.tar.gz
3. tar -zxvf boost_1_68_0.tar.gz
4. cd boost_1_68_0/
5. ./bootstrap.sh
6. cpuCores=`cat /proc/cpuinfo | grep "cpu cores" | uniq | awk '{print $NF}'` 
7.  sudo ./b2 --with=all -j $cpuCores
```

4. Compile as a standalone C++ project
Create new subdirectory to store the build 
```
mkdir wsr_build && cd wsr_build
cmake ..
cpuCores=`cat /proc/cpuinfo | grep "cpu cores" | uniq | awk '{print $NF}'`
make -j $cpuCores
```

## Remote processing and visualization of output
Since visualization cannot be done directly on the robot, we have included this functionality to execute the code on a remote computer. 

1. Copy the relevant csi and trajectory data files to some directory. Make sure to copy the groundtruth_positions.json file there and update the corresponding groundtruth locations of the transmitting robots.

2. Open the debug directly. This is where all the output files are generated. Then run the following shell script
```
cd debug
../viz_latest_output.sh <path_to_directory> <traj_type> '<tx names>'

e.g
../viz_latest_output.sh /home/jadhav/REACT-Projects/WSR-Toolbox-cpp/data/online_data/ gt 'tx2 tx3'
```

The following traj_type is currently tested : gt

Note : Make sure that the tx names are passed as a space separated string as shown in the example.


## Test CSI data using C++ executable
1. To process multiple data samples in a directory
```
data_processing <dir_path> <sample data type>
```

Sample data can be
- 2D_TX
- 2D_RX
- 3D_TX
- 3D_RX

e.g
```
cd wsr_build
./data_processing /home/jadhav/REACT-Projects/WSR-Toolbox-cpp/data/Line-of-Sight/2D_1_RX_1_TX_RX_P0_TX_P1/ 2D_TX
```

2. Make sure that the correct flags and parameters are set during initialization. (This requires that the 'debug' config parameter is enabled):
```
log [Precomp]: Important FLAGS status
  Trajectory Type = "3D"
  __FLAG_packet_threshold = true
  __FLAG_debug = true
  __FLAG_threading = true
  __FLAG_interpolate_phase = true
  __FLAG_sub_sample = false
  __FLAG_normalize_profile = true
  __FLAG_use_multiple_sub_carriers = false
  __FLAG_use_magic_mac = false

```