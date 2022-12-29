# WSR-Toolbox-cpp
Core C++ code repo for WSR toolbox with Cython wrapper.

## Supported Environments

### Ubuntu version tested
- [x] Ubuntu 16.04
- [x] Ubuntu 18.04
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

## Setup instructions

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

## Code Execution

1. Make sure that the paths of the data files and details of the robots are updated correctly in the config file as mentioned in [wiki here](https://github.com/Harvard-REACT/WSR-Toolbox-cpp/wiki/Updating-the-config-file). A detailed description of all the configuration flags can be [found here](https://github.com/Harvard-REACT/WSR-Toolbox/wiki/Configuration-File-parameters)

### Test sample CSI data files
2. To test the CSI data and channel reciprocity module run the following (This requires that the correct file locations of the sample data are added in the config_3D_SAR.json):

```
cd wsr_build
./test_csi_data
```

### Calculate AOA profiles using Core C++ framework (needs robot displacement)

2. To test the AOA calculation run the following:
```
cd wsr_build
./test_wsr <displacement type>

e.g.
./test_wsr gt
```

## Compiling and using Cpython modules
1. First run the setup.py
```
$ python3 setup.py build_ext --inplace
```
The above code will auto generate a wsr_module.cpp file in the Cpython_modules directory. A libarary file will also be generated in the scripts/libs directory.

Note: sometimes if the Cpython_modules/wsr_module.cpp file is not deleted before running setup.py, the changes made in C++ modules do not take effect.

```
cd scripts
python3 main.py --d_type gt
```
The main.py triggers the code similar to test_wsr.


3. The status of some important flags and parameters are available during initialization. (This requires that the 'debug' config parameter is set to 'true' to see the status during code execution):
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


## Visualization
All the data files pertaining to visualization are generated in the **debug** directory by default. The visualization scripts can be found in the **scripts directory**. The following data are visualized:
1. WiFi signal phase : This plot can greatly simply debugging.
2. Packet distriution along robots's displacement: Shows if the WiFi packets are distributed uniformly during robot motion or whether there are substntial packet loss
3. Angle-of-Arrival profile

An example of the above plots is show in the [wiki here]()


1. To visualize the latest data generated in the debug directory use the following
```
bash ./scripts/viz_data.sh <debug directory path> <TX_neighbor robot ids> <displacement type> <backend visualizer>

```

1. TX_neighbor robot ids: tx0, tx2, tx3 etc.
2. displacement type: 2D , 3D
3. backend visualizer : matplotlib (default), matlab

For example to visualize the data for tx2, tx3 and tx4 (as per the details in the config file) using python matplotlib, run the following command 
```
e.g.
./scripts/viz_data.sh debug/ 'tx2 tx3 tx4' 2D 
```

To use matlab visualizer (recommended for offline testing), run the following script instead
```
./scripts/viz_data.sh debug/ 'tx2 tx3 tx4' 2D matlab
```

Note :If the matlab viewer is used for visualizing the AOA profile, then make sure that the matlab api for python has be installed ([reference](https://www.mathworks.com/help/matlab/matlab_external/install-the-matlab-engine-for-python.html)).


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
