#include "common.h"

#include <dynamic_reconfigure/server.h>
#include <msp/paramConfig.h>

// #include "ocams.h"
// #include "ooc.h"
// #include "pcddb.h"

#include "imad.h"

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <pcl_ros/transforms.h>


namespace msp
{    
    class FilteringMappingNodelet : public nodelet::Nodelet
    {
        public:
        FilteringMappingNodelet(){}
        ~FilteringMappingNodelet() override
        {

            if (thread_filtering_mapping.joinable())
                thread_filtering_mapping.join();
            if (thread_nn_stats.joinable())
                thread_nn_stats.join();
            std::cerr << "\033[31mFilteringMappingNodelet finished\033[0m" << std::endl;
        }

        private:


        //handlers
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;
        std::shared_ptr<dynamic_reconfigure::Server<msp::paramConfig>> dr_srv_;

        // threading
        std::thread thread_filtering_mapping;
        // std::thread thread_normal_estimation;//--- IGNORE ---
        std::thread thread_nn_stats;

        std::mutex m_buf;
        std::queue<sensor_msgs::PointCloud2::ConstPtr> buf_registered_scan;
        std::queue<nav_msgs::Odometry::ConstPtr> buf_state_estimation;
        // std::queue<NormalTask> normal_task_queue;//--- IGNORE ---
        // std::mutex m_normal_task;//--- IGNORE ---
        // std::mutex m_log;//--- IGNORE ---
        // std::condition_variable cv_normal_task;//--- IGNORE ---
        // std::atomic<bool> stop_threads{false};//--- IGNORE ---

        // timers publishers and subscribers
        ros::Timer execution_timer;
        ros::Timer calNN_timer;
        ros::Publisher pub_pcd_filtered;
        ros::Publisher pub_pcd_map;
        ros::Publisher pub_pcd_localmap;
        ros::Publisher pub_pcd_normals;
        ros::Publisher pub_pcd_density_map;
        ros::Publisher pub_img_mapbbox;
        ros::Publisher pub_registered_scan_crop;
        ros::Subscriber sub_state_estimation;
        // ros::Subscriber sub_registered_scan;
        ros::Subscriber sub_lasercloud;

        //load parameters from launch
        // std::string topic_state_estimation;
        // std::string topic_registered_scan;
        std::string  raw_cloud_topic;
        // std::string  odom_frame_id;
        double lod_density;
        double density_threshold;
        std::string pkg_path;

        pcl::PointCloud<pcl::PointNormal>::Ptr pcd_global_map = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();

        //global vis map
        std::shared_ptr<sensor_msgs::PointCloud2> pcd_map;
        std::shared_ptr<sensor_msgs::PointCloud2> pcd_density_map;
       
        

        // std::unique_ptr<msp::pcdDB> pcd_db;
        std::unique_ptr<msp::imad> IMAD_mapper;



        //
        int frame_count = 0;
        int LOCAL_MAPPING_COUNT = 1;


        //keyframe
        Eigen::Vector3d last_keyframe_pos_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond last_keyframe_q_ = Eigen::Quaterniond::Identity();
        bool init_ = false;
        double trans_thr_= 0.1;
        double rot_thr_ = 5.0 * M_PI / 180.0;

        //tf
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_{tf_buffer_};
        const std::string map_frameid = "map";
        const std::string sensor_frameid = "sensor";

        virtual void onInit()
        {
            nh = getNodeHandle();
            private_nh = getPrivateNodeHandle();
            readParameters();

            dr_srv_.reset(new dynamic_reconfigure::Server<msp::paramConfig>(private_nh));  // boost::recursive_mutex dr_mutex_;
            dynamic_reconfigure::Server<msp::paramConfig>::CallbackType cb_dr;
            cb_dr = boost::bind(&FilteringMappingNodelet::callback_dynamic_reconfigure, this, _1, _2);
            dr_srv_->setCallback(cb_dr);

            execution_timer = nh.createTimer(ros::Duration(1), &FilteringMappingNodelet::execute, this);
            calNN_timer =     nh.createTimer(ros::Duration(10), &FilteringMappingNodelet::calculateNN, this);
            // sub_state_estimation = nh.subscribe(topic_state_estimation, 10, &FilteringMappingNodelet::callback_state_estimation, this);
            // sub_registered_scan = nh.subscribe(topic_registered_scan, 100, &FilteringMappingNodelet::callback_registered_scan, this);
            sub_lasercloud = nh.subscribe(raw_cloud_topic, 100, &FilteringMappingNodelet::lasercloud_handler, this);

            pub_pcd_filtered    =   private_nh.advertise<sensor_msgs::PointCloud2>("pcd_filtered", 10);
            pub_pcd_normals     =   private_nh.advertise<sensor_msgs::PointCloud2>("pcd_normals", 10);
            pub_pcd_map         =   private_nh.advertise<sensor_msgs::PointCloud2>("pcd_map", 10,true);
            pub_pcd_localmap    =   private_nh.advertise<sensor_msgs::PointCloud2>("pcd_localmap", 10);
            pub_pcd_density_map =   private_nh.advertise<sensor_msgs::PointCloud2>("pcd_density_map", 10,true);
            pub_img_mapbbox     =   private_nh.advertise<sensor_msgs::Image>("img_mapbbox", 10);
            pub_registered_scan_crop = private_nh.advertise<sensor_msgs::PointCloud2>("registered_scan_crop", 10);
            // ocams_downsampler.reset(new msp::ocams(private_nh,lod_density));
            // OCAMS_downsampler.setLodDensity(lod_density);
            // IMAD_mapper.setLodDensity(lod_density);
            // IMAD_mapper.setLodDensity(lod_density);
            

            // pcd_db = std::make_unique<msp::pcdDB>(pkg_path,lod_density);
            IMAD_mapper = std::make_unique<msp::imad>(lod_density);
            // IMAD_mapper->setPcdDbPath(pkg_path);

            // thread_filtering_mapping = std::thread(&FilteringMappingNodelet::thread_filtering_mapping_function, this);
            // thread_nn_stats = std::thread(&FilteringMappingNodelet::thread_nn_stats_function, this);
            // thread_normal_estimation = std::thread(&FilteringMappingNodelet::thread_normal_estimation_function, this);

        }
        
        void lasercloud_handler(const sensor_msgs::PointCloud2::ConstPtr& msg_lasercloud)
        {
            // geometry_msgs::TransformStamped tf_map_to_sensor;
            // try
            // {
            //     tf_map_to_sensor = tf_buffer_.lookupTransform(
            //         map_frameid,                 
            //         sensor_frameid,               
            //         msg_lasercloud->header.stamp,     
            //         ros::Duration(1)); 
            //     if(!init_)
            //     {
            //         init_ = true;
            //         std::cout<< "\033[1;32m"<< "[msp/filtering_mapping_nodelet]: TF lookup successful, initializing map frame." << "\033[0m" << std::endl;
            //     }

            // }
            // catch (const tf2::TransformException& ex)
            // {
            //     std::cerr << "\033[31m"<< "[msp/filtering_mapping_nodelet]: TF lookup failed from " << sensor_frameid << " to " << map_frameid <<
            //                 ", skipping this point cloud. Error: " << ex.what() << "\033[0m" << std::endl;
            //     return;
            // }

            // sensor_msgs::PointCloud2 msg_lasercloud_registered;
            // tf2::doTransform(*msg_lasercloud, msg_lasercloud_registered, tf_map_to_sensor);


            // IMAD_mapper->insertPointCloud(std::move(msg_lasercloud_registered));

            IMAD_mapper->insertPointCloud(*msg_lasercloud);
            if(!IMAD_mapper->isMapUpdate())
                return;
            auto msg_pcd_map = IMAD_mapper->buildDownsampledMap();
            auto msg_pcd_density_map = IMAD_mapper->buildDensityMap();
            pub_pcd_map.publish(msg_pcd_map);
            pub_pcd_density_map.publish(msg_pcd_density_map);
        }


        void execute(const ros::TimerEvent&)
        {
            

        }

        void calculateNN(const ros::TimerEvent&)
        {
            pcl::console::TicToc timer_nn;
            timer_nn.tic();
            // IMAD_mapper->compNNDist();
            auto elapsed_nn = timer_nn.toc();
            // std::cout << "\033[1;32m" << "compNNDist time: " << elapsed_nn << " ms" << "\033[0m" << std::endl;
        }
        
        void readParameters()
        {
            // private_nh.param<std::string>("topic_state_estimation", topic_state_estimation, "/state_estimation");
            // private_nh.param<std::string>("topic_registered_scan", topic_registered_scan, "/registered_scan");
            // private_nh.param<std::string>("odom_frame_id", odom_frame_id, "odom");
            private_nh.param<std::string>("raw_cloud_topic", raw_cloud_topic, "/registered_scan");
            private_nh.param<double>("lod_density", lod_density, 0.152);
            private_nh.param<std::string>("pkg_path", pkg_path, "/root/catkin_ws/src/msp_simulation/msp");
        }

        void callback_dynamic_reconfigure(msp::paramConfig &cfg, uint32_t /*level*/) 
        {
            // lod_density= cfg.lod_density_value;
            // density_threshold= cfg.density_threshold;
            // OCAMS_downsampler.setr1thr(cfg.r1_thr);
            // OCAMS_downsampler.setr2thr(cfg.r2_thr);
            // OCAMS_downsampler.setLodFactor(cfg.lod_factor_);
            // OCAMS_downsampler.setLodDensity(lod_density);
        }

        bool isKeyFrame(const Eigen::Vector3d& cur_pos,const Eigen::Quaterniond& cur_q)
        {
            if (!init_)
            {
                init_ = true;
                return true;
            }
                
            double trans = (cur_pos - last_keyframe_pos_).norm();
            double rot = last_keyframe_q_.angularDistance(cur_q);
            return (trans > trans_thr_) || (rot > rot_thr_);
        }


        void thread_filtering_mapping_function()
        {
            while(ros::ok())
            {
                sensor_msgs::PointCloud2::ConstPtr msg_registered_scan;
                double time = 0;
                m_buf.lock();
                if (!buf_registered_scan.empty())
                {
                    // sync
                    time = buf_registered_scan.front()->header.stamp.toSec();
                    msg_registered_scan = buf_registered_scan.front();
                    buf_registered_scan.pop();
                }
                m_buf.unlock();
                
                if(msg_registered_scan)
                {
                    
                    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
                    pcl::fromROSMsg(*msg_registered_scan, *cloud);

                    // IMAD_mapper->insertPointCloud(cloud);

                    // auto pcd_map_downsampled=IMAD_mapper->buildDownsampledMap();

                    // auto msg_pcd_map_downsampled = std::make_shared<sensor_msgs::PointCloud2>();
                    // pcl::toROSMsg(*pcd_map_downsampled, *msg_pcd_map_downsampled);
                    // msg_pcd_map_downsampled->header.frame_id = "sensor_init";
                    // msg_pcd_map_downsampled->header.stamp = ros::Time(time);
                    // std::atomic_store(&pcd_map, msg_pcd_map_downsampled);

                    // pcl::console::TicToc timer1;
                    // timer1.tic();
                    // computeNNStats(pcd_map_downsampled); 
                    // auto timer1_end = timer1.toc();
                    // std::cout << "[Mapping] computeNNStats time: " << timer1_end << " ms" << std::endl;

            
                    // auto msg_pcd_density_map = std::make_shared<sensor_msgs::PointCloud2>();
                    // pcl::toROSMsg(*IMAD_mapper->buildDensityMap(), *msg_pcd_density_map);
                    // msg_pcd_density_map->header.frame_id = "sensor_init";
                    // msg_pcd_density_map->header.stamp = ros::Time(time);
                    // std::atomic_store(&pcd_density_map, msg_pcd_density_map);
                }
        


                
                
                std::chrono::milliseconds dura(1);
                std::this_thread::sleep_for(dura);
            }
            
            std::cerr << "\033[31mthread_filtering_mapping finished\033[0m" << std::endl;
        }
        
        void thread_filtering_mapping_function1()
        {
            while(ros::ok())
            {
                sensor_msgs::PointCloud2::ConstPtr msg_registered_scan;
                nav_msgs::Odometry::ConstPtr msg_state_estimation;
                double time = 0;
                m_buf.lock();
                if (!buf_registered_scan.empty() && !buf_state_estimation.empty())
                {
                    // sync
                    time = buf_registered_scan.front()->header.stamp.toSec();
                    msg_registered_scan = buf_registered_scan.front();
                    msg_state_estimation = buf_state_estimation.front();
                    buf_registered_scan.pop();
                    buf_state_estimation.pop();
                }
                m_buf.unlock();
                
                if(msg_registered_scan && msg_state_estimation)
                {
                    
                    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
                    pcl::fromROSMsg(*msg_registered_scan, *cloud);


                    Eigen::Vector3d cur_pos(msg_state_estimation->pose.pose.position.x,
                                        msg_state_estimation->pose.pose.position.y,
                                        msg_state_estimation->pose.pose.position.z);
                    Eigen::Quaterniond cur_q(msg_state_estimation->pose.pose.orientation.w,
                                        msg_state_estimation->pose.pose.orientation.x,
                                        msg_state_estimation->pose.pose.orientation.y,
                                        msg_state_estimation->pose.pose.orientation.z);
                    cur_q.normalize();
                    if (!isKeyFrame(cur_pos, cur_q))
                        continue;
                    last_keyframe_pos_ = cur_pos;
                    last_keyframe_q_ = cur_q.normalized();
                    ROS_WARN("Processing new scan and state estimation");

                    // IMAD_mapper->insertPointCloud(cloud);

                    // auto pcd_map_downsampled=IMAD_mapper->buildDownsampledMap();

                    // auto msg_pcd_map_downsampled = std::make_shared<sensor_msgs::PointCloud2>();
                    // pcl::toROSMsg(*pcd_map_downsampled, *msg_pcd_map_downsampled);
                    // msg_pcd_map_downsampled->header.frame_id = "map";
                    // msg_pcd_map_downsampled->header.stamp = ros::Time(time);
                    // std::atomic_store(&pcd_map, msg_pcd_map_downsampled);

                    // pcl::console::TicToc timer1;
                    // timer1.tic();
                    // computeNNStats(pcd_map_downsampled); 
                    // auto timer1_end = timer1.toc();
                    // std::cout << "[Mapping] computeNNStats time: " << timer1_end << " ms" << std::endl;

            
                    // auto msg_pcd_density_map = std::make_shared<sensor_msgs::PointCloud2>();
                    // pcl::toROSMsg(*IMAD_mapper->buildDensityMap(), *msg_pcd_density_map);
                    // msg_pcd_density_map->header.frame_id = "map";
                    // msg_pcd_density_map->header.stamp = ros::Time(time);
                    // std::atomic_store(&pcd_density_map, msg_pcd_density_map);
                }
        


                
                
                std::chrono::milliseconds dura(1);
                std::this_thread::sleep_for(dura);
            }
            
            std::cerr << "\033[31mthread_filtering_mapping finished\033[0m" << std::endl;
        }





        // void thread_filtering_mapping_function1()
        // {
        //     while(ros::ok())
        //     {
        //         sensor_msgs::PointCloud2::ConstPtr msg_registered_scan;
        //         nav_msgs::Odometry::ConstPtr msg_state_estimation;
        //         double time = 0;
        //         m_buf.lock();
        //         if (!buf_registered_scan.empty() && !buf_state_estimation.empty())
        //         {
        //             // sync
        //             time = buf_registered_scan.front()->header.stamp.toSec();
        //             msg_registered_scan = buf_registered_scan.front();
        //             msg_state_estimation = buf_state_estimation.front();
        //             buf_registered_scan.pop();
        //             buf_state_estimation.pop();
        //         }
        //         m_buf.unlock();
                
        //         if(msg_registered_scan && msg_state_estimation)
        //         {
        //             // ROS_WARN("Processing new scan and state estimation");
                    
        //             pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        //             pcl::fromROSMsg(*msg_registered_scan, *cloud);

        //             double cx = msg_state_estimation->pose.pose.position.x;
        //             double cy = msg_state_estimation->pose.pose.position.y;
        //             double range = 5.0;
        //             Eigen::Vector4f min_pt(cx - range, cy - range, -std::numeric_limits<float>::max(), 1.0); 
        //             Eigen::Vector4f max_pt(cx + range, cy + range, std::numeric_limits<float>::max(), 1.0); 
        //             double margin = 0;
        //             auto cloud_cropped= crop<PointT>(cloud, min_pt,max_pt,false);
        //             // auto cloud_cropped=cloud;

        //             // auto cloud_cropped=cloud;

        //             frame_count++;
                    
        //             if(frame_count >= LOCAL_MAPPING_COUNT)
        //             {
        //                 frame_count = 0;
        //                 *pcd_accumulated += *cloud_cropped; 
                        
        //                 //1. query step
        //                 pcl::console::TicToc timer_query;
        //                 timer_query.tic();
                        
        //                 //database global map
        //                 pcl::PointXYZ input_min, input_max;
        //                 pcl::getMinMax3D(*pcd_accumulated, input_min, input_max);
        //                 auto pcd_local_map = pcd_db->extractFromMapByChunk(input_min, input_max);
        //                 double timer_query_end = timer_query.toc();

        //                 //2. filtering step
        //                 auto size1 = pcd_local_map->size();
        //                 *pcd_local_map += *pcd_accumulated;
        //                 auto size2 = pcd_local_map->size();
        //                 pcl::console::TicToc timer_ocams;
                        
        //                 auto pcd_local_map_filtered = boost::make_shared<pcl::PointCloud<PointT>>();
                        
        //                 OCAMS_downsampler.setInputCloud(pcd_local_map);
        //                 timer_ocams.tic();
        //                 OCAMS_downsampler.filter(*pcd_local_map_filtered);
        //                 auto pcd_local_map_filtered_vis = OCAMS_downsampler.getVisCloud();
        //                 auto pcd_local_map_filtered_vis = OCAMS_downsampler.getVisCloud();
        //                 double timer_ocams_end = timer_ocams.toc();
                        

        //                 // NormalTask task;
        //                 // task.cloud = pcd_local_map_filtered;
        //                 // task.header = msg_registered_scan->header;
        //                 // enqueueNormalTask(task);
                        
        //                 //3.normal estimation
        //                 pcl::console::TicToc timer_normal;
        //                 timer_normal.tic();
        //                 pcl::NormalEstimation<PointT, pcl::Normal> ne;
        //                 pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
        //                 ne.setInputCloud(pcd_local_map_filtered);
        //                 ne.setSearchMethod(tree);
        //                 ne.setRadiusSearch(3*lod_density); // or setKSearch(k)
        //                     // ne.setKSearch(normal_ksearch);
        //                 pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        //                 ne.compute(*normals);
        //                 pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
        //                 // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        //                 // pcl::copyPointCloud(*pcd_local_map_filtered, *cloud_xyz);
        //                 pcl::concatenateFields(*pcd_local_map_filtered, *normals, *cloud_with_normals);
        //                 double timer_normal_end = timer_normal.toc();
                        
        //                 //4. insert to map
        //                 pcl::console::TicToc timer_db;
        //                 timer_db.tic();
        //                 pcd_db->insertPCDByChunk(cloud_with_normals);

        //                 global_map_update(pcd_accumulated, cloud_with_normals, pcd_global_map);
        //                 // density_map_update(pcd_accumulated, pcd_local_map_filtered, lod_density);
        //                 // auto pcd_density_map=density_aware(pcd_local_map_filtered, lod_density);
        //                 // density_map_update(pcd_accumulated, pcd_local_map_filtered, lod_density);
        //                 // auto pcd_density_map=density_aware(pcd_local_map_filtered, lod_density);
                        
        //                 double timer_db_end = timer_db.toc();
        //                 // computeNNStats(cloud_xyz);

        //                 {
        //                     std::lock_guard<std::mutex> lock(m_log);
        //                     // std::cout << "[Mapping] pcd_local_map size: " << size1 << std::endl;
        //                     // std::cout << "[Mapping] pcd size before filtering: " << size2 << std::endl;
        //                     // std::cout << "[Mapping] querying: " << timer_query_end << " ms" << std::endl;
        //                     // std::cout << "[Mapping] filtering: " << timer_ocams_end << " ms" << std::endl;
        //                     // std::cout << "[Mapping] insert: " << timer_db_end << " ms" << std::endl;
        //                     // std::cout << "[Mapping] pcd_local_map_filtered size: " << pcd_local_map_filtered->size() << std::endl;
        //                     // std::cout << "[Mapping] pcd_local_map size: " << size1 << std::endl;
        //                     // std::cout << "[Mapping] pcd size before filtering: " << size2 << std::endl;
        //                     // std::cout << "[Mapping] querying: " << timer_query_end << " ms" << std::endl;
        //                     // std::cout << "[Mapping] filtering: " << timer_ocams_end << " ms" << std::endl;
        //                     // std::cout << "[Mapping] insert: " << timer_db_end << " ms" << std::endl;
        //                     // std::cout << "[Mapping] pcd_local_map_filtered size: " << pcd_local_map_filtered->size() << std::endl;
        //                     // std::cout << "[Normal]  normal: " << timer_normal_end << " ms" << std::endl;
        //                 }

        //                 // computeNNStats(pcd_density_map_increment);
        //                 // std::cout<<"frame:"<< std::endl;
        //                 // computeNNStats(pcd_local_map_filtered);
        //                 // computeNNStats(pcd_local_map_filtered);
        //                 // std::cout<<"frame size: "<< pcd_local_map_filtered->size()<< std::endl;
        //                 // std::cout<<"global map:"<< std::endl;
        //                 // computeNNStats(pcd_density_map);
        //                 // std::cout<<"density map size: "<< pcd_density_map->size()<< std::endl;
        //                 pcd_accumulated->clear();


                        
        //                 auto pcd_downsample=voxel<pcl::PointNormal>(cloud_with_normals, 0.5);
        //                 cloud_with_normals.swap(pcd_downsample);
        //                 pubPointcloud<pcl::PointNormal>(pub_pcd_normals, cloud_with_normals,time);//pcd with normal, curent

        //                 pubPointcloud<PointT>(pub_pcd_localmap, pcd_local_map,time);//pcd xyz, local map


        //                 pubPointcloud<PointT>(pub_pcd_filtered, pcd_local_map_filtered,time);//pcd xyz, incremental filetering 
        //                 // pubPointcloud<pcl::PointXYZRGB>(pub_pcd_filtered, pcd_local_map_filtered_vis,time);//pcd xyz, local map visualization

        //                 // pubPointcloud<pcl::PointXYZRGB>(pub_pcd_filtered, pcd_local_map_filtered_vis,time);//pcd xyz, local map visualization

        //                 pubPointcloud<pcl::PointNormal>(pub_pcd_map, pcd_global_map,time);//map with normal, voxel downsampled, global 
        //                 // pubPointcloud<PointT>(pub_pcd_density_map, pcd_density_map,time);

        //                 pubPointcloud<PointT>(pub_registered_scan_crop, cloud_cropped,time);//pcd xyz, local map

        //                 // pubPointcloud<PointT>(pub_pcd_density_map, pcd_density_map,time);

        //                 pubPointcloud<PointT>(pub_registered_scan_crop, cloud_cropped,time);//pcd xyz, local map




        //             }
        //             else
        //             {
        //                 *pcd_accumulated += *cloud_cropped; 
        //             }

                    

        //         }

                
                
        //         std::chrono::milliseconds dura(1);
        //         std::this_thread::sleep_for(dura);
        //     }
            
        //     std::cerr << "\033[31mthread_filtering_mapping finished\033[0m" << std::endl;
        // }



        template <typename PointT>
        typename pcl::PointCloud<PointT>::Ptr crop(const typename pcl::PointCloud<PointT>::Ptr cloud_in, 
                                                    Eigen::Vector4f min_pt, 
                                                    Eigen::Vector4f max_pt,
                                                    bool negative=false)
        {
            auto cloud_cropped= boost::make_shared<pcl::PointCloud<PointT>>();
           
            // double cz = msg_state_estimation->pose.pose.position.z;

            pcl::CropBox<PointT> cf;
            cf.setMin(min_pt); 
            cf.setMax(max_pt);
            cf.setInputCloud(cloud_in);
            cf.setNegative(negative);
            cf.filter(*cloud_cropped);
            return cloud_cropped;
        }
        
        template <typename PointT>
        typename pcl::PointCloud<PointT>::Ptr voxel(const typename pcl::PointCloud<PointT>::Ptr cloud_in, double leaf_size)
        {
            auto cloud_out= boost::make_shared<pcl::PointCloud<PointT>>();
            pcl::VoxelGrid<PointT> voxel_filter;
            voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
            voxel_filter.setInputCloud(cloud_in);
            voxel_filter.filter(*cloud_out);
            return cloud_out;
        }

        // void thread_normal_estimation_function()
        // {
        //     while (ros::ok() && !stop_threads.load())
        //     {
        //         NormalTask task;
        //         {
        //             std::unique_lock<std::mutex> lock(m_normal_task);
        //             cv_normal_task.wait(lock, [this]()
        //             {
        //                 return stop_threads.load() || !normal_task_queue.empty();
        //             });

        //             if (stop_threads.load())
        //                 break;

        //             task = normal_task_queue.front();
        //             normal_task_queue.pop();
        //         }
        //         if (!task.cloud || task.cloud->empty())
        //             continue;



        //         pcl::console::TicToc timer_normal;
        //         timer_normal.tic();
        //         pcl::NormalEstimation<PointT, pcl::Normal> ne;
        //         pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
        //         ne.setInputCloud(task.cloud);
        //         ne.setSearchMethod(tree);
        //         ne.setRadiusSearch(3*lod_density); // or setKSearch(k)
        //             // ne.setKSearch(normal_ksearch);
        //         pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        //         ne.compute(*normals);
        //         double timer_normal_end = timer_normal.toc();

        //         {
        //             std::lock_guard<std::mutex> lock(m_log);
        //             std::cout<< "[Normal] Normal estimation task received, cloud size: " << task.cloud->size() << std::endl;
        //             std::cout << "[Normal] normal: " << timer_normal_end << " ms" << std::endl;
        //         }

        //         pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
        //         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        //         pcl::copyPointCloud(*task.cloud, *cloud_xyz);
        //         pcl::concatenateFields(*cloud_xyz, *normals, *cloud_with_normals);
        //         pcl::PointCloud<pcl::PointNormal>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointNormal>);
        //         pcl::VoxelGrid<pcl::PointNormal> vf;
        //         vf.setInputCloud(cloud_with_normals);
        //         vf.setLeafSize(0.5, 0.5 ,0.5);
        //         vf.filter(*cloud_filtered);
        //         sensor_msgs::PointCloud2 msg_pcd_normals;
        //         pcl::toROSMsg(*cloud_filtered, msg_pcd_normals);
        //         msg_pcd_normals.header = task.header;
        //         msg_pcd_normals.header.frame_id = "map";
        //         pub_pcd_normals.publish(msg_pcd_normals);
                    
        //     }
        // }

        // void enqueueNormalTask(const NormalTask& task)
        // {
        //     std::lock_guard<std::mutex> lock(m_normal_task);

        //     if (normal_task_queue.size() > 10)
        //         return;
        //     // while (!normal_task_queue.empty())
        //     //     normal_task_queue.pop();

        //     normal_task_queue.push(task);
        //     cv_normal_task.notify_one();
        // }


        // void pcd_local_mapping(const pcl::PointCloud<PointT>::Ptr& cloud_in)
        // {

            
        // }

        // void pcd_mapping(const sensor_msgs::PointCloud2ConstPtr& msg_registered_scan,const nav_msgs::Odometry::ConstPtr& msg_state_estimation)
        // {

        // }
        

        void pubImage(ros::Publisher& publisher, const cv::Mat &img, const std::string& encoding, const double t)
        {
            std_msgs::Header header;
            header.frame_id = "camera_color_optical_frame";
            header.stamp = ros::Time(t);
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, encoding, img).toImageMsg();
            publisher.publish(img_msg);
        }

        template <typename PointT>
        void pubPointcloud(ros::Publisher& publisher, 
                                        const typename pcl::PointCloud<PointT>::Ptr& pcd, 
                                        const double t, 
                                        bool downsample=false)
        {
            auto pcd_out = boost::make_shared<pcl::PointCloud<PointT>>();
            // if(downsample)
            // {
            //     pcd_out = voxel<PointT>(pcd, 0.5);
            // }

            pcd_out = pcd;
            

            sensor_msgs::PointCloud2 msg_pcd;
            pcl::toROSMsg(*pcd_out, msg_pcd);
            msg_pcd.header.frame_id = "map";
            msg_pcd.header.stamp = ros::Time(t);
            publisher.publish(msg_pcd);
        }
        void callback_registered_scan(const sensor_msgs::PointCloud2ConstPtr& msg_registered_scan)
        {
            m_buf.lock();
            buf_registered_scan.push(msg_registered_scan);
            m_buf.unlock();
        }

        void callback_state_estimation(const nav_msgs::Odometry::ConstPtr& msg_state_estimation)
        {
            m_buf.lock();
            buf_state_estimation.push(msg_state_estimation);
            m_buf.unlock();
            // NODELET_INFO("State Estimation - Posit");
        }

    };
} // namespace msp
PLUGINLIB_EXPORT_CLASS(msp::FilteringMappingNodelet, nodelet::Nodelet)