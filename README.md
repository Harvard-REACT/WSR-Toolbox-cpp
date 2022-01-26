# WSR-Toolbox ROS Kinetic
Deploying the WSR Toolbox on Ubuntu 16.04 with ROS kinetic (Supports only rospy and python2.7)

## Setup instructions

1. Install ROS kinetic and create catkin workspace.

2. Clone this repository in the _workspace_/src 

### Update the python configuration steps and check if they work without the UP-board image 
3. Update kernel to 4.15.0-142 and then reboot ? 

4. sudo apt install python-pip python-numpy -y


5. Install the python dependency packages
```
pip install Cython setuptools pybind11
```

6. Download and compile the boost_1.68 locally in $HOME/Downloads.
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


## Compiling and using Cpython modules
1. First run the setup.py
```
$ python setup.py build_ext --inplace
```
The above code will auto generate a wsr_module.cpp file in the Cpython_modules directory. A libarary file will also be generated in the scripts/libs directory.

Note: sometimes if the Cpython_modules/wsr_module.cpp file is not deleted before running setup.py, the changes made in C++ modules do not take effect.

2. **Launch File**

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
