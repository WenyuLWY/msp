#pragma once

// ros and nodelet
#include <ros/ros.h>
#include <nodelet/nodelet.h>  
#include <pluginlib/class_list_macros.h>
// #include <tf2_eigen/tf2_eigen.h>

// register pointcloud type

// msgs
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

// pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/common/common.h>
#include <pcl/common/io.h>  
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <pcl/console/time.h> 
#include <pcl/features/normal_3d.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/organized.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/register_point_struct.h>

// opencv
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// others
#include <Eigen/Dense>
#include <Eigen/Core>

// cpp
#include <bits/stdc++.h>
// #include <string>
// #include <thread>
// #include <vector>
// #include <queue>
// #include <algorithm>
// #include <unordered_set>
// #include <condition_variable>
// #include <cmath>
// #include <memory>
// #include <climits>

typedef pcl::PointXYZI PointT;


template <class ROSMsgType>
void Publish(const ros::Publisher& publisher, ROSMsgType& msg, const std::string& frame_id)
{
  msg.header.frame_id = frame_id;
  msg.header.stamp = ros::Time::now();
  publisher.publish(msg);
}

// typedef pcl::outofcore::OutofcoreOctreeDisk<PointT> OctreeT;

// typedef pcl::outofcore::OutofcoreOctreeBase <pcl::outofcore::OutofcoreOctreeDiskContainer<PointT>, PointT> OctreeT;

// using OctreeDisk = OutofcoreOctreeBase<OutofcoreOctreeDiskContainer<pcl::PointXYZ>, pcl::PointXYZ>;

// namespace msp
// {

//     struct PointXYZIR
//     {
//         PCL_ADD_POINT4D;        
//         float intensity;
//         std::uint16_t ring;
//         EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//     }   EIGEN_ALIGN16;
// }

// POINT_CLOUD_REGISTER_POINT_STRUCT(
//   msp::PointXYZIR,
//   (float, x, x)
//   (float, y, y)
//   (float, z, z)
//   (float, intensity, intensity)
//   (std::uint16_t, ring, ring)
// )