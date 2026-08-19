cmake_minimum_required(VERSION 3.0.2)
project(msp_sim)

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

# list(APPEND CMAKE_PREFIX_PATH "/root/or-tools")
# find_package(ortools CONFIG REQUIRED)


catkin_package(
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

# ortools::ortools
set(COMMON_LIBRARIES ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)


add_executable(localization_sim src/localization_sim.cpp)
target_link_libraries(localization_sim ${COMMON_LIBRARIES} )
