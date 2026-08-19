#include "common.h"
#include <tf/tf.h>

namespace msp
{
    class ScanPlanningNodelet : public nodelet::Nodelet
    {
        public:
        ScanPlanningNodelet(){}
        ~ScanPlanningNodelet() 
        {
            std::cerr << "\033[31m"<<"ScanPlanningNodelet finished"<<"\033[0m" << std::endl;
        }

        private:

        //handlers
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

        // publishers and subscribers
        // ros::Publisher pub_pcd_map;
        ros::Publisher pub_waypoint;
        // ros::Subscriber sub_scan_keep;
        ros::Subscriber sub_state_estimation;
        ros::Subscriber sub_pcd_density_map;
        ros::Timer execution_timer_;

        geometry_msgs::Point robot_position_;
        Eigen::Vector3d lookahead_point_;
        Eigen::Vector3d initial_position_;
        double robot_yaw_;
        bool moving_forward_;
        bool initialized_ = false;
        const std::string frame_id_ = "map";


        virtual void onInit()
        {
            nh = getNodeHandle();
            private_nh = getPrivateNodeHandle();

            // sub_scan_keep = nh.subscribe("/PcdModelingNodelet/scan_keep", 10, &ScanPlanningNodelet::callback_scan_keep, this);
            // pub_pcd_map = private_nh.advertise<sensor_msgs::PointCloud2>("pcd_map", 10);

            execution_timer_ = nh.createTimer(ros::Duration(1.0), &ScanPlanningNodelet::execute, this);
            pub_waypoint = nh.advertise<geometry_msgs::PointStamped>("/way_point", 2);
            sub_state_estimation = nh.subscribe("/state_estimation_at_scan", 5, &ScanPlanningNodelet::StateEstimationCallback, this);
            sub_pcd_density_map = nh.subscribe("/FilteringMappingNodelet/pcd_density_map", 5, &ScanPlanningNodelet::PcdDensityMapCallback, this);

            initial_position_.x() = 0.0;
            initial_position_.y() = 0.0;
            initial_position_.z() = 0.0;

            // thread_map_assessment = std::thread(&ScanPlanningNodelet::thread_map_assessment_function, this);
        }

        void SendInitialWaypoint()
        {
            // send waypoint ahead
            double lx = 12.0;
            double ly = 0.0;
            double dx = cos(robot_yaw_) * lx - sin(robot_yaw_) * ly;
            double dy = sin(robot_yaw_) * lx + cos(robot_yaw_) * ly;

            geometry_msgs::PointStamped waypoint;
            waypoint.header.frame_id = "map";
            waypoint.header.stamp = ros::Time::now();
            waypoint.point.x = robot_position_.x + dx;
            waypoint.point.y = robot_position_.y + dy;
            waypoint.point.z = robot_position_.z;
            pub_waypoint.publish(waypoint);
        }

        void execute(const ros::TimerEvent&)
        {
            if (!initialized_)
            {
                return;
            }

            PublishWaypoint();
        }

        void StateEstimationCallback(const nav_msgs::Odometry::ConstPtr& state_estimation_msg)
        {
            robot_position_ = state_estimation_msg->pose.pose.position;
            // Todo: use a boolean
            if (std::abs(initial_position_.x()) < 0.01 && std::abs(initial_position_.y()) < 0.01 &&
                std::abs(initial_position_.z()) < 0.01)
            {
                initial_position_.x() = robot_position_.x;
                initial_position_.y() = robot_position_.y;
                initial_position_.z() = robot_position_.z;
            }
            double roll, pitch, yaw;
            geometry_msgs::Quaternion geo_quat = state_estimation_msg->pose.pose.orientation;
            tf::Matrix3x3(tf::Quaternion(geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w)).getRPY(roll, pitch, yaw);

            robot_yaw_ = yaw;

            if (state_estimation_msg->twist.twist.linear.x > 0.4)
            {
                moving_forward_ = true;
            }
            else if (state_estimation_msg->twist.twist.linear.x < -0.4)
            {
                moving_forward_ = false;
            }
            initialized_ = true;

            // std::cout<<"\033[32m"<<"Received state estimation: position ("<<robot_position_.x<<", "<<robot_position_.y<<", "<<robot_position_.z<<"), yaw "<<robot_yaw_<<", moving_forward "<<moving_forward_<<"\033[0m"<<std::endl;
        }
        
        void PublishWaypoint()
        {
            geometry_msgs::PointStamped waypoint;
            // if (exploration_finished_ && near_home_ && pp_.kRushHome)
            // {
            //     waypoint.point.x = pd_.initial_position_.x();
            //     waypoint.point.y = pd_.initial_position_.y();
            //     waypoint.point.z = pd_.initial_position_.z();
            // }
        
            double dx = lookahead_point_.x() - robot_position_.x;
            double dy = lookahead_point_.y() - robot_position_.y;

            // double r = sqrt(dx * dx + dy * dy);
            // double extend_dist =
            //     lookahead_point_in_line_of_sight_ ? pp_.kExtendWayPointDistanceBig : pp_.kExtendWayPointDistanceSmall;
            // if (r < extend_dist && pp_.kExtendWayPoint)
            // {
            //     dx = dx / r * extend_dist;
            //     dy = dy / r * extend_dist;
            // }

            // waypoint.point.x = dx + robot_position_.x;
            // waypoint.point.y = dy + robot_position_.y;
            // waypoint.point.z = lookahead_point_.z();

            waypoint.point.x= robot_position_.x + 0.3;
            waypoint.point.y = robot_position_.y;
            waypoint.point.z = robot_position_.z;
            
            Publish<geometry_msgs::PointStamped>(pub_waypoint, waypoint, frame_id_);
        }

        void PcdDensityMapCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
        {
            
        }
    };
}// namespace msp
PLUGINLIB_EXPORT_CLASS(msp::ScanPlanningNodelet, nodelet::Nodelet)