cmake_minimum_required(VERSION 3.15)
project(wsr-toolbox-cpp)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17 -pthread
                    -Wno-unused-result -Wsign-compare
                    -DNDEBUG -fwrapv -O3
                    -fstack-protector-strong
                    -Wformat -Werror=format-security
                    -Wdate-time -D_FORTIFY_SOURCE=2 -fPIC
                    )


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
set(BOOST_INCLUDE_DIRS "$ENV{HOME}/Downloads/boost_1_68_0")
set(BOOST_LIBRARIES "$ENV{HOME}/Downloads/boost_1_68_0/stage/lib")

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

find_package(OpenMP REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${BOOST_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(_wsr_1.0.1_parallel SHARED
   src/WIFI_Agent.cpp 
   src/WSR_Module.cpp 
   src/WSR_Util.cpp
 )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

add_executable(test_wsr src/test_wsr.cpp src/WIFI_Agent.cpp src/WSR_Module.cpp src/WSR_Util.cpp)
add_executable(data_processing src/data_processing.cpp src/WIFI_Agent.cpp src/WSR_Module.cpp src/WSR_Util.cpp)
add_executable(process_dataset_all src/process_dataset_all.cpp src/WIFI_Agent.cpp src/WSR_Module.cpp src/WSR_Util.cpp)
add_executable(remote_processing src/remote_processing.cpp src/WIFI_Agent.cpp src/WSR_Module.cpp src/WSR_Util.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(test_wsr
   -Wl,--no-as-needed -lgomp -lpthread -lm -ldl
   ${BOOST_LIBRARIES} OpenMP::OpenMP_CXX
 )
target_link_libraries(data_processing
 -Wl,--no-as-needed -lgomp -lpthread -lm -ldl
 ${BOOST_LIBRARIES} OpenMP::OpenMP_CXX
)
target_link_libraries(remote_processing
 -Wl,--no-as-needed -lgomp -lpthread -lm -ldl
 ${BOOST_LIBRARIES} OpenMP::OpenMP_CXX
)
target_link_libraries(process_dataset_all
 -Wl,--no-as-needed -lgomp -lpthread -lm -ldl
 ${BOOST_LIBRARIES} OpenMP::OpenMP_CXX
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_csitoolbox.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
