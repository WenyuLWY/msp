#include "common.h"


std::string pkg_path;
std::string map1_path, map2_path;


void cloudCallback1(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    auto start = std::chrono::steady_clock::now();
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);
        pcl::io::savePCDFileBinary(map1_path,cloud);
    auto end = std::chrono::steady_clock::now();
    double cost_ms = std::chrono::duration<double, std::milli>(end - start).count();
    // ROS_INFO("map_voxel.pcd: saved %lu points, cost %.3f ms",cloud.size(), cost_ms);
}

void cloudCallback2(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    auto start = std::chrono::steady_clock::now();
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);
        pcl::io::savePCDFileBinary(map2_path,cloud);
    auto end = std::chrono::steady_clock::now();
    double cost_ms = std::chrono::duration<double, std::milli>(end - start).count();
    // ROS_INFO("map_imad.pcd: saved %lu points, cost %.3f ms",cloud.size(), cost_ms);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_saver");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param<std::string>("pkg_path", pkg_path, "/root");
    map1_path = pkg_path + "/map" + "/map_voxel.pcd";
    map2_path = pkg_path + "/map" + "/map_imad.pcd";

    // std::cout<< "\033[1;32m"<< "pkg_path: "<< pkg_path<< "\033[0m" << std::endl;
    // std::cout<< "\033[1;32m"<< "map1_path: "<< map1_path<< "\033[0m" << std::endl;
    // std::cout<< "\033[1;32m"<< "map2_path: "<< map2_path<< "\033[0m" << std::endl;



    ros::Subscriber sub1 = nh.subscribe("/explored_areas",1,cloudCallback1);
    ros::Subscriber sub2 = nh.subscribe("/FilteringMappingNodelet/pcd_map",1,cloudCallback2);

    ros::spin();

    return 0;
}