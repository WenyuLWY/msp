cmake_minimum_required(VERSION 3.0.2)
project(msp)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -O3 -flto")

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "Debug")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    cv_bridge
    nodelet
    dynamic_reconfigure
    tf
    pcl_conversions
    pcl_ros
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    gazebo_msgs
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(SQLite3 REQUIRED)

# list(APPEND CMAKE_PREFIX_PATH "/root/or-tools")
# find_package(ortools CONFIG REQUIRED)


generate_dynamic_reconfigure_options(
  config/param.cfg
)

catkin_package(
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${SQLite3_INCLUDE_DIRS}
)

# ortools::ortools
set(COMMON_LIBRARIES ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${SQLite3_LIBRARIES} 
    OpenMP::OpenMP_CXX)

add_library(msp_nodelets 
  # src/pcd_modeling_nodelet.cpp
  src/sensor_input_nodelet.cpp
  src/filtering_mapping_nodelet.cpp
  # src/localization_sim_nodelet.cpp
  src/scan_planning_nodelet.cpp
  # src/map_assessment_nodelet.cpp
  )
add_dependencies(msp_nodelets ${PROJECT_NAME}_gencfg) 
target_compile_definitions(msp_nodelets PRIVATE PCL_NO_PRECOMPILE)
target_link_libraries(msp_nodelets ${COMMON_LIBRARIES})

# test
# add_executable(test_filter test/test_filter.cpp)
# target_link_libraries(test_filter ${COMMON_LIBRARIES} )

# add_executable(test_sphere test/test_sphere.cpp)
# target_link_libraries(test_sphere ${COMMON_LIBRARIES} )

# add_executable(test_ooc_save test/test_ooc_save.cpp)
# target_link_libraries(test_ooc_save ${COMMON_LIBRARIES})

# add_executable(test_ooc_load test/test_ooc_load.cpp)
# target_link_libraries(test_ooc_load ${COMMON_LIBRARIES})