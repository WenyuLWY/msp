#include <ros/ros.h>

#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>

#include <bits/stdc++.h>


namespace msp
{    
    class LocalizationSimNode
    {
        public:
        LocalizationSimNode()
            : private_nh("~")
        {
            readParameters();

            odometry.header.frame_id = "odom";
            odometry.child_frame_id = "base_link";

            subModelState = nh.subscribe(
                "/gazebo/model_states",
                10,
                &LocalizationSimNode::modelStatesCallback,
                this);

            subLaserCloud = nh.subscribe(
                laserCloudTopic,
                100,
                &LocalizationSimNode::laserCloudCallback,
                this);

            pubOdometry = nh.advertise<nav_msgs::Odometry>("/Odometry", 10);

            // robotStateClient =
            //     nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
        }

        ~LocalizationSimNode()
        {
            std::cerr << "\033[31m"<<"LocalizationSimNode finished"<<"\033[0m" << std::endl;
        }

        private:

        // handlers
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

        // ros::ServiceClient robotStateClient;
        ros::Subscriber subModelState;
        ros::Subscriber subLaserCloud;
        ros::Publisher pubOdometry;

        std::string robot_name = "husky";
        std::string laserCloudTopic;

        int model_index = -1;

        Eigen::Affine3d initTransformMat;

        bool init = false;

        nav_msgs::Odometry odometry; 
        geometry_msgs::Pose pose_;

        std::mutex pose_mutex_;
        bool has_pose_ = false;


        void readParameters()
        {
            private_nh.param<std::string>("laserCloudTopic",laserCloudTopic,"/velodyne_points");
            private_nh.param<std::string>("robotName", robot_name,"husky");
        }


        void modelStatesCallback(const gazebo_msgs::ModelStatesConstPtr& msg)
        {
            if (!init)
            {
                if (msg->name.empty() ||
                    msg->pose.size() != msg->name.size() ||
                    msg->twist.size() != msg->name.size())
                {
                    std::cerr
                        << "\033[31m"
                        << "[msp_sim/localization_sim_node]"
                        << "Invalid /gazebo/model_states message received"
                        << "\033[0m"
                        << std::endl;

                    return;
                }


                for (size_t i = 0; i < msg->name.size(); ++i)
                {
                    if (msg->name[i] == robot_name)
                    {
                        model_index = static_cast<int>(i);
                        break;
                    }
                }

                if (model_index < 0)
                {
                    std::cerr
                        << "\033[31m"
                        << "[msp_sim/localization_sim_node]"
                        << "Model "
                        << robot_name
                        << " not found in /gazebo/model_states"
                        << "\033[0m"
                        << std::endl;

                    return;
                }
                else
                {
                    tf2::fromMsg(
                        msg->pose[model_index],
                        initTransformMat);

                    initTransformMat =
                        initTransformMat.inverse();

                    init = true;

                    std::cout
                        << "\033[1;32m"
                        << "[msp_sim/localization_sim_node]"
                        << "Model "
                        << robot_name
                        << " found in /gazebo/model_states, "
                        << "initializing LocalizationSimNode"
                        << "\033[0m"
                        << std::endl;

                    // const geometry_msgs::Pose& init_pose_msg =
                    //     msg->pose[model_index];

                    // const geometry_msgs::Twist& init_twist_msg =
                    //     msg->twist[model_index];
                }
            }

            // std::lock_guard<std::mutex> lock(pose_mutex_);

            std::unique_lock<std::mutex> lock(
                pose_mutex_,
                std::try_to_lock);

            if (!lock.owns_lock())
                return;

            pose_ = msg->pose[model_index];
            has_pose_ = true;
        }


        void laserCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);

            if (!has_pose_)
                return;

            Eigen::Affine3d transformMat;

            tf2::fromMsg(
                pose_,
                transformMat);

            transformMat =
                initTransformMat * transformMat;

            odometry.header.stamp =
                msg->header.stamp;

            odometry.pose.pose =
                tf2::toMsg(transformMat);

            pubOdometry.publish(odometry);
        }
    };

} // namespace msp


int main(int argc, char** argv)
{
    ros::init(argc, argv, "localization_sim");

    msp::LocalizationSimNode localization_sim_node;

    ros::spin();

    return 0;
}