# WSR-Toolbox ROS Melodic
Deploying the WSR Toolbox on Ubuntu 18.04 with ROS kinetic (Supports only rospy)

## Setup instructions
1. Install [ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu).

Note: For for _rosdep update_ add an extra flag to update kinetic since its an EOL distro
```
rosdep update
```

2. Create [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) using _catkin build_
```
sudo apt install python-catkin-tools python-pip git vim -y
mkdir -p ~/catkin_ws/src 
cd ~/catkin_ws
catkin build
```

3. Open the ~/.bashrc file and add the following (if not done already)
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://<IP of the ROS master>:11311" >> ~/.bashrc
echo "export ROS_HOSTNAME=<IP of your system>" >> ~/.bashrc
source ~/.bashrc
```

5. Clone this repository in the _workspace_/src 
```
cd ~/catkin_ws/src
git clone https://github.com/Harvard-REACT/WSR-Toolbox-cpp.git
cd WSR-Toolbox-cpp
git checkout wsr-melodic
```

6. Install the python dependency packages
```
pip install Cython setuptools numpy pybind11 pythran scipy pandas matplotlib
```

7. Download and compile the boost_1.68 locally in $HOME/Downloads.
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


## Generating Cython library
This requires running the setup.py in WSR_toolbox_cpp. It generates a library using cython which then can be imported in python programs and used with rospy. The catkin build command will generate the required ros messages

1. First, run the setup.py
```
cd ~/catkin_ws/src/WSR-Toolbox-cpp
catkin build
python setup.py build_ext --inplace
source ~/.bashrc
```
The above code will auto generate a wsr_module.cpp file in the Cpython_modules directory. A libarary file will also be generated in the scripts/libs directory.

Note: sometimes if the Cpython_modules/wsr_module.cpp file is not deleted before running setup.py, the changes made in C++ modules do not take effect.

2. Start the rosmaster in a seprate terminal
```
roscore
```

3. Test sample example for publisher and subscriber. Update the path of config file correctly in the wsr_pub.launch or pass it via command line
```
roslaunch wsr_toolbox_cpp wsr_pub.launch config_fn:=<path to config file>

e.g.
roslaunch wsr_toolbox_cpp wsr_pub.launch config_fn:=/home/test-u18/catkin_ws/src/WSR-Toolbox-cpp/config/config_3D_SAR.json
```

In another terminal start
```
cd scripts
python main_sub.py
```

In the third terminal, publish on the boolean topic /get_aoa from the command line to trigger AOA computation using sample data
```
rostop pub --once /get_aoa std_msgs/Bool "data: false"
```

4. The status of some important flags and parameters are available during initialization. (This requires that the 'debug' config parameter is set to 'true' to see the status during code execution):
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
  __FLAG_use_relative_displacement = false
```

## Visualization of Data

