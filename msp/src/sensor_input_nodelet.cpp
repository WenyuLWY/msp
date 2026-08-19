#include "common.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// #include <pcl/common/transforms.h>

#include <pcl_ros/transforms.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

namespace msp
{    
    class SensorInputNodelet : public nodelet::Nodelet
    {
        public:
        SensorInputNodelet(){}
        ~SensorInputNodelet() override
        {
            std::cerr << "\033[31mSensorInputNodelet finished\033[0m" << std::endl;
        }

        private:


        //handlers
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

        //params
        std::string  lidar_frame_id;
        std::string  robot_base_frame_id;
        std::string  odom_frame_id;
        std::string  frame_id;
        std::string  cloud_topic;
        std::string  sub_odometry_topic;
        std::string  pub_odometry_topic;
        std::string  mode;
        std::string  source_frame_id;
        std::string  target_frame_id;
        bool pub_registered_cloud = false;
        bool transform_odometry = false;
        // std::string  registeredCloudTopic;
        // bool publish_odom_tf = true;

        //message_filters sync
        message_filters::Subscriber<nav_msgs::Odometry> subOdometry;
        message_filters::Subscriber<sensor_msgs::PointCloud2> subLaserCloud;
        typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,sensor_msgs::PointCloud2> syncPolicy;
        typedef message_filters::Synchronizer<syncPolicy> Sync;
        boost::shared_ptr<Sync> sync_;




        //publishers
        ros::Publisher pubOdometry;
        ros::Publisher pubOdometryScan;
        ros::Publisher pubLaserCloudRegistered; 
        ros::Publisher pubLaserCloudSensorscan;
        // tf::TransformBroadcaster tfBroadcaster;
        tf2_ros::TransformBroadcaster tfBroadcaster;

        tf2_ros::Buffer tfBuffer;
        std::shared_ptr<tf2_ros::TransformListener> tfListener;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticBr;
        geometry_msgs::TransformStamped  tfSensorToBase;
        Eigen::Affine3d transformCorrectMat;
        Eigen::Affine3d transformCorrectMatInv;

        // bool need_transform_ = false;
        // bool initialized_ = false;
        tf2::Transform tfToSensor;

        virtual void onInit()
        {
            nh = getNodeHandle();
            private_nh = getPrivateNodeHandle();
            readParameters();

            //listener
            tfListener = std::make_shared<tf2_ros::TransformListener>(tfBuffer);
            staticBr = std::make_shared<tf2_ros::StaticTransformBroadcaster>();

            if(mode =="2d")
            {
                target_frame_id = robot_base_frame_id;
                pub_odometry_topic = "/odom";
            }
            else if(mode =="3d")
            {
                target_frame_id = lidar_frame_id;
                pub_odometry_topic= "/state_estimation";
                auto start1 = ros::Time::now();
                try
                {
                    tfSensorToBase = tfBuffer.lookupTransform(
                        lidar_frame_id,
                        robot_base_frame_id,
                        ros::Time(0),
                        ros::Duration(20.0));
                    auto end1 = ros::Time::now();
                    double elapsed1 = (end1 - start1).toSec();
                    tfSensorToBase.header.stamp = ros::Time::now();
                    tfSensorToBase.header.frame_id = "sensor";
                    tfSensorToBase.child_frame_id = "vehicle";
                    tfSensorToBase.transform = tfSensorToBase.transform;
                    double vehicleHeight = tfSensorToBase.transform.translation.z;
                    tfSensorToBase.transform.translation.z = 0.0; 
                    
                    geometry_msgs::TransformStamped tfVehicleToBase;
                    tfVehicleToBase.header.stamp = ros::Time::now();
                    tfVehicleToBase.header.frame_id = "vehicle";
                    tfVehicleToBase.child_frame_id = robot_base_frame_id;
                    tfVehicleToBase.transform.translation.z = vehicleHeight;
                    tfVehicleToBase.transform.rotation.w = 1.0; 

                    staticBr->sendTransform({tfSensorToBase, tfVehicleToBase});
                    ROS_INFO("[msp/sensor_input_nodelet]TF acquire success! Elapsed time: %.3f s", elapsed1);
                }
                catch (tf2::TransformException &ex)
                {
                    ROS_WARN("[msp/sensor_input_nodelet]Failed to republish TF: %s", ex.what());
                    auto end1 = ros::Time::now();
                    double elapsed1 = (end1 - start1).toSec();
                    ROS_WARN("[msp/sensor_input_nodelet] %s to %s TF not found within %.3f s, skipping.", robot_base_frame_id.c_str(), lidar_frame_id.c_str(), elapsed1);
                }
            }

            
            if(target_frame_id!=source_frame_id)
                transform_odometry=true;

            if(transform_odometry)
            {
                auto start2 = ros::Time::now();
                try
                {
                    auto tfCorrect=tfBuffer.lookupTransform(
                            target_frame_id,
                            source_frame_id,
                            ros::Time(0),
                            ros::Duration(5.0));
                    auto end1 = ros::Time::now();
                    double elapsed1 = (end1 - start2).toSec();
                    transformCorrectMat = tf2::transformToEigen(tfCorrect);
                    transformCorrectMatInv = transformCorrectMat.inverse();
                    // tf2::fromMsg(tfCorrect.transform, transformCorrectMat);
                    ROS_INFO("[msp/sensor_input_nodelet]Correction TF acquire success! Elapsed time: %.3f s", elapsed1);
                }
                catch (tf2::TransformException &ex)
                {
                    ROS_WARN("[msp/sensor_input_nodelet]Failed to republish TF: %s", ex.what());
                    auto end2 = ros::Time::now();
                    double elapsed2 = (end2 - start2).toSec();
                    ROS_ERROR("[msp/sensor_input_nodelet] %s to %s TF not found within %.3f s, Stop.", target_frame_id.c_str(), source_frame_id.c_str(), elapsed2);
                    return;
                }
            }

            subOdometry.subscribe(nh, sub_odometry_topic, 50);
            subLaserCloud.subscribe(nh, cloud_topic, 50);
            sync_.reset(new Sync(syncPolicy(50), subOdometry, subLaserCloud));
            sync_->registerCallback(boost::bind(&SensorInputNodelet::laserCloudAndOdometryHandler, this, _1, _2));

            pubLaserCloudRegistered = nh.advertise<sensor_msgs::PointCloud2>("/registered_scan", 10);
            pubLaserCloudSensorscan = nh.advertise<sensor_msgs::PointCloud2>("/sensor_scan", 10);
            pubOdometry = nh.advertise<nav_msgs::Odometry>(pub_odometry_topic, 10);
            pubOdometryScan = nh.advertise<nav_msgs::Odometry>("/state_estimation_at_scan", 10);

        }



        void readParameters()
        {
            private_nh.param<std::string>("lidar_frame_id", lidar_frame_id, "velodyne");
            private_nh.param<std::string>("robot_base_frame_id", robot_base_frame_id, "base_footprint");
            private_nh.param<std::string>("odom_frame_id", odom_frame_id, "odom");
            private_nh.param<std::string>("sub_odometry_topic", sub_odometry_topic, "/odom");
            // private_nh.param<std::string>("frame_id", frame_id, "velodyne");
            private_nh.param<std::string>("mode", mode, "2d");
            private_nh.param<std::string>("source_frame_id", source_frame_id, "base_link");


            // private_nh.param<bool>("pub_registered_cloud", pub_registered_cloud, false);
            // if(pub_registered_cloud)
            // {
            //     private_nh.param<std::string>("reg_cloud_topic", cloud_topic, "/cloud_registered");
            // }
            
            
            private_nh.param<std::string>("raw_cloud_topic", cloud_topic, "/velodyne_points");
            
            // private_nh.param<std::string>("raw_cloud_topic", raw_cloud_topic, "/velodyne_points");
            // private_nh.param<std::string>("registered_cloud_topic", registered_cloud_topic, "/cloud_registered");
            // private_nh.param<bool>("publish_odom_tf", publish_odom_tf, false);
        }

        void laserCloudAndOdometryHandler(const nav_msgs::Odometry::ConstPtr& odometryData,
                                  const sensor_msgs::PointCloud2::ConstPtr& laserCloudIn)
        {

            // if(!initialized_)
            // {
            //     if (laserCloudIn->header.frame_id != "base_footprint") 
            //     {
            //         need_transform_ = true;
            //         try 
            //         {
            //             auto tfToSensorStamped = tfBuffer.lookupTransform("base_footprint",   // target: e.g. "base_link"
            //                                                     laserCloudIn->header.frame_id,  // source: e.g. "velodyne"
            //                                                     ros::Time(1));
            //             tf2::fromMsg(tfToSensorStamped.transform, tfToSensor);
            //             need_transform_ = true;
            //         } 
            //         catch (tf2::TransformException& ex) 
            //         {
            //             ROS_WARN("[msp/sensor_inputn_nodelet]: TF lookup failed from %s to base_footprint: %s",
            //                                                             laserCloudIn->header.frame_id.c_str(),  
            //                                                             ex.what());
            //             return; 
            //         }
            //     }
            //     initialized_ = true;
            // }

            // Eigen::Isometry3d transform;
            // tf2::fromMsg(odometryData->pose.pose, transform);
            // tf2::Transform transform;
            // tf2::fromMsg(odometryData->pose.pose, transform);


            // std::cout<<"odom timestamp: "<<odometryData->header.stamp<<", cloud timestamp: "<<laserCloudIn->header.stamp<<std::endl;

            Eigen::Affine3d transformMat;
            tf2::fromMsg(odometryData->pose.pose, transformMat);
            if(transform_odometry)
                transformMat = transformCorrectMat * transformMat * transformCorrectMatInv;
            // std::cout<<"Transform Matrix: "<<transformMat.matrix()<<std::endl;
            // if(need_transform_)
            // {
            //     transform = transform * tfToSensor;
            // }
            // geometry_msgs::Transform transformData = tf2::toMsg(transform);

            nav_msgs::Odometry odom; 
            odom.header = odometryData->header;
            odom.header.frame_id = odom_frame_id;
            odom.pose.pose = tf2::toMsg(transformMat);

            sensor_msgs::PointCloud2 laserCloudRegistered;  
            pcl_ros::transformPointCloud(transformMat.matrix().cast<float>(), *laserCloudIn, laserCloudRegistered);
            laserCloudRegistered.header.stamp = laserCloudIn->header.stamp;
            laserCloudRegistered.header.frame_id = odom_frame_id;

            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header.stamp = odometryData->header.stamp;
            transformStamped.header.frame_id = odom_frame_id;
            transformStamped.transform = tf2::eigenToTransform(transformMat).transform;

            if(mode =="2d")
            {
                odom.child_frame_id = robot_base_frame_id;
                transformStamped.child_frame_id = robot_base_frame_id;
            }
            else if(mode =="3d")
            {
                //at odometry time

                odom.child_frame_id = "sensor";
                
                // tf2::toMsg(transform, odom.pose.pose);
                // odom.pose.pose = tf2::toMsg(transformData);


                // at sensor scan time
                nav_msgs::Odometry odomScan;
                odomScan.header.stamp = laserCloudIn->header.stamp;
                odomScan.header.frame_id = odom_frame_id;
                odomScan.child_frame_id = "sensor_at_scan";
                odomScan.pose=odom.pose;



                // pcl_ros::transformPointCloud(transform.matrix().cast<float>(), *laserCloudIn, laserCloudRegistered);
                // laserCloudRegistered.header = odometryData->header;
                
                
                

                // if(pub_registered_cloud)
                // {
                //     laserCloudRegistered=*laserCloudIn;
                //     pcl_ros::transformPointCloud(transformMat.inverse().matrix().cast<float>(), laserCloudRegistered, laserCloudScan);
                // }

                sensor_msgs::PointCloud2 laserCloudScan;
                laserCloudScan=*laserCloudIn;
                laserCloudScan.header.stamp = laserCloudIn->header.stamp;
                laserCloudScan.header.frame_id = "sensor_at_scan";

                //publish
                pubOdometryScan.publish(odomScan);
                pubLaserCloudSensorscan.publish(laserCloudScan);
                
                // if(publish_odom_tf)
                geometry_msgs::TransformStamped transformStampedScan=transformStamped;
                transformStampedScan.header.stamp = laserCloudIn->header.stamp;
                transformStampedScan.child_frame_id = "sensor_at_scan";
                tfBroadcaster.sendTransform(transformStampedScan);
                transformStamped.child_frame_id = "sensor";
            }

            pubOdometry.publish(odom);
            pubLaserCloudRegistered.publish(laserCloudRegistered);
            tfBroadcaster.sendTransform(transformStamped);

            
        }


    };
} // namespace msp
PLUGINLIB_EXPORT_CLASS(msp::SensorInputNodelet, nodelet::Nodelet)