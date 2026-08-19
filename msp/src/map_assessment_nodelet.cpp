#include <ros/ros.h>
#include <nodelet/nodelet.h>  
#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>


#include "common.h"
#include "ilfps.h"

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
        std::thread thread_map_assessment;
        std::mutex m_buf;
        std::atomic<bool> keep_running;
        std::queue<sensor_msgs::PointCloud2::ConstPtr> buf_scan_keep;
        std::queue<nav_msgs::Odometry::ConstPtr> buf_state_estimation;

        // publishers and subscribers
        ros::Publisher pub_pcd_map;
        ros::Publisher pub_pcd_debug;
        ros::Subscriber sub_scan_keep;

        //load parameters from launch
        double lod_density;
        double loa_accuracy;

        //members
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_map;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_map_downsampled;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_scan_keep;



        std::unique_ptr<msp::ilfps> ilfps_downsampler; 


        bool map_initialized = false;
        

        virtual void onInit()
        {


            nh = getNodeHandle();
            private_nh = getPrivateNodeHandle();
            readParameters();

            sub_scan_keep = nh.subscribe("/PcdModelingNodelet/scan_keep", 10, &MapAssessmentNodelet::callback_scan_keep, this);
            pub_pcd_map = private_nh.advertise<sensor_msgs::PointCloud2>("pcd_map", 10);
            pub_pcd_debug = private_nh.advertise<sensor_msgs::PointCloud2>("pcd_debug", 10);


            pcd_map.reset(new pcl::PointCloud<pcl::PointXYZ>());
            pcd_map_downsampled.reset(new pcl::PointCloud<pcl::PointXYZ>());
            pcd_scan_keep.reset(new pcl::PointCloud<pcl::PointXYZ>());

            
            ilfps_downsampler.reset(new msp::ilfps(private_nh,lod_density,loa_accuracy));

            keep_running.store(true);
            thread_map_assessment = std::thread(&MapAssessmentNodelet::thread_map_assessment_function, this);
        }
        void readParameters()
        {
            if (!private_nh.getParam("lod_density", lod_density)) 
            {
                NODELET_ERROR("Parameter 'lod_density' not found!");
            }
            if (!private_nh.getParam("loa_accuracy", loa_accuracy)) 
            {
                NODELET_ERROR("Parameter 'loa_accuracy' not found!");
            }
        }
        void thread_map_assessment_function()
        {
            while(keep_running.load())
            {
                sensor_msgs::PointCloud2::ConstPtr msg_scan_keep;
                m_buf.lock();
                if (!buf_scan_keep.empty())
                {
                    msg_scan_keep = buf_scan_keep.front();
                    buf_scan_keep.pop();
                }
                m_buf.unlock();
    
                if(msg_scan_keep)
                {
                    pcd_scan_keep->clear();
                    pcl::fromROSMsg(*msg_scan_keep, *pcd_scan_keep);
                    if(!map_initialized)
                    {
                        *pcd_map += *pcd_scan_keep;
                        map_initialized = true;
                        continue;  
                    }

                    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_debug(new pcl::PointCloud<pcl::PointXYZ>);
                    // pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
                    // std::vector<int> pointIdxRadiusSearch_;
                    // std::vector<float> pointRadiusSquaredDistance_;
                    // const size_t N = pcd_scan_keep->size();
                    // kdtree.setInputCloud(pcd_scan_keep);
                    // for (int i = 0; i < static_cast<int>(N); ++i)
                    // {
                    //     pointIdxRadiusSearch_.clear();
                    //     pointRadiusSquaredDistance_.clear();
                    //     const auto& Pc = pcd_scan_keep->points[i];
                    //     int num_neighbors = kdtree.radiusSearch(Pc, 0.9*lod_density, pointIdxRadiusSearch_, pointRadiusSquaredDistance_);
                    
                    //     if( num_neighbors > 4)
                    //     {        
                    //         float max_distance = 0.0f;
                    //         float min_distance = std::numeric_limits<float>::max(); 
                    //         float z_value = Pc.z;  
                    //         for(int ii = 0; ii < pointIdxRadiusSearch_.size(); ++ii)
                    //         {
                    //             float current_distance = std::sqrt(pointRadiusSquaredDistance_[ii]);

                    //             if(current_distance > max_distance)
                    //             {
                    //                 max_distance = current_distance;
                    //             }

                    //             if( current_distance> 0.0 && current_distance < min_distance) 
                    //             {
                    //                 min_distance = current_distance;
                    //             }

                    //             const auto& neighbor = kdtree.getInputCloud()->points[pointIdxRadiusSearch_[ii]];

                                
                    //         }

                    //         std::cout << "Point " << ": (" << Pc.x << ", " << Pc.y << ", " << z_value << ")";
                    //         std::cout << " - Max neighbor distance: " << max_distance << std::endl;
                    //         std::cout << " - Min neighbor distance : " << min_distance << std::endl;

                    //         pcl::PointXYZ p;
                    //         p.x = Pc.x; p.y = Pc.y; p.z = Pc.z;
                    //         cloud_debug->points.push_back(p);
                    //     }
                    // }


                    pcl::console::TicToc timer_ilfps;
                    timer_ilfps.tic();
                        ilfps_downsampler->incrementalLocalFps(pcd_scan_keep,pcd_map);
                    double timer_ilfps_end = timer_ilfps.toc();
                    std::cout << "timer_ilfps: " << timer_ilfps_end << " ms" << std::endl;

                    // *pcd_map += *pcd_scan_keep;
                    //         pcd_map_downsampled->clear();
                    //         voxel_filter_pcd_map.setInputCloud(pcd_map);
                    //         voxel_filter_pcd_map.filter(*pcd_map_downsampled);
                        
                        sensor_msgs::PointCloud2 msg_pcd_map_downsampled;
                        pcl::toROSMsg(*pcd_map, msg_pcd_map_downsampled);
                        msg_pcd_map_downsampled.header.stamp = msg_scan_keep->header.stamp;
                        msg_pcd_map_downsampled.header.frame_id = "map";
                        pub_pcd_map.publish(msg_pcd_map_downsampled);


                        // sensor_msgs::PointCloud2 msg_pcd_debug;
                        // pcl::toROSMsg(*cloud_debug, msg_pcd_debug);
                        // msg_pcd_debug.header = msg_scan_keep->header;
                        // pub_pcd_debug.publish(msg_pcd_debug);

                    
                }
                
                std::chrono::milliseconds dura(1);
                std::this_thread::sleep_for(dura);
            }
        }

        // void incremental_local_fps(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,pcl::PointCloud<pcl::PointXYZ>::Ptr& output,double lod_density)
        // {
        //     //farthest point sampling

        // }

        void callback_scan_keep(const sensor_msgs::PointCloud2ConstPtr& msg_scan_keep)
        {
            m_buf.lock();
            buf_scan_keep.push(msg_scan_keep);
            m_buf.unlock();
        }
    };
}// namespace msp
PLUGINLIB_EXPORT_CLASS(msp::MapAssessmentNodelet, nodelet::Nodelet)