# WSR-Toolbox-cpp
Core C++ code repo for WSR toolbox with Cython wrapper.

## Ubuntu version tested
- [x] Ubuntu 16.04
- [ ] Ubuntu 18.04
- [x] Ubuntu 20.04
- [ ] TX2

## C++ version tested and supported
- ~~[ ] c++11~~
- [x] c++14
- [ ] c++17
- [ ] c++20

## Python version tested
- [x] python 3.5
- [ ] python 3.6
- [ ] python 3.7
- [x] python 3.8

## GCC version tested
- [x] gcc 5.4.0
- [x] gcc 9.3.0

## Setup instructions

1. Clone the repository in your cakin workspace under csitoolbox directory

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


## Test CSI data using C++ executable
1. To test the forward-reverse channel calculation run the following:
```
cd wsr_build
./test_wsr
```

2. To update any of the parameters, use the config_3D_SAR.json in the config directory.

3. To visualize the profile , go to scripts directory and use the visualize_aoa_profile.py
```
python3 visualize_aoa_profile.py --file <filepath>

e.g
python3 visualize_aoa_profile.py --file ../data/Realtime_data/Feb_2_2021/aoa_profiles/aoa_profile_2021-02-02_200917.csv
```

4. Make sure that the correct flags and parameters are set during initialization. (This requires that the 'debug' config parameter is enabled):
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

5. To visualize different data outputs for debugging

a. Interpolated Trajectory

```
cd scripts
python3 viz_traj.py
```

b. Channel Phase

```
cd scripts
python3 viz_channel_data.py
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
python3 main.py --f ../data/3D_Helix_101_mocap_2020-03-04_102615/mocap_data_a.txt_2020-03-04_102615.txt --mocap_id 101 --parser_type optitrack
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
