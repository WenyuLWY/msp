#include <ros/ros.h>
#include <nodelet/nodelet.h>  
#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include "common.h"

namespace msp
{
    class MapAssessmentNodelet : public nodelet::Nodelet
    {
        public:
        MapAssessmentNodelet(){}
        ~MapAssessmentNodelet() 
        {
            keep_running.store(false);
            if (thread_map_assessment.joinable())
                thread_map_assessment.join();
            NODELET_WARN("MapAssessmentNodelet destructor");
        }


        private:

        //handlers
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

        // threading
        std::thread thread_map_assessment
        std::mutex m_buf;
        std::atomic<bool> keep_running;
        std::queue<sensor_msgs::PointCloud2::ConstPtr> buf_scan_keep;
        std::queue<nav_msgs::Odometry::ConstPtr> buf_state_estimation;

        // publishers and subscribers
        ros::Publisher pub_pcd_map;
        ros::Subscriber sub_scan_keep;

        virtual void onInit()
        {
            nh = getNodeHandle();
            private_nh = getPrivateNodeHandle();

            sub_scan_keep = nh.subscribe("/PcdModelingNodelet/scan_keep", 10, &MapAssessmentNodelet::callback_scan_keep, this);
            pub_pcd_map = private_nh.advertise<sensor_msgs::PointCloud2>("pcd_map", 10);

            keep_running.store(true);
            // thread_map_assessment = std::thread(&MapAssessmentNodelet::thread_map_assessment_function, this);
        }
    };
}// namespace msp
PLUGINLIB_EXPORT_CLASS(msp::MapAssessmentNodelet, nodelet::Nodelet)