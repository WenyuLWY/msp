
#include <dynamic_reconfigure/server.h>
#include <msp/paramConfig.h>

#include "common.h"

namespace msp
{
    class PcdModelingNodelet : public nodelet::Nodelet
    {
        public:
        PcdModelingNodelet(){}
        ~PcdModelingNodelet() 
        {
            keep_running.store(false);
            if (thread_pcd_modeling.joinable())
                thread_pcd_modeling.join();
            NODELET_WARN("PcdModelingNodelet destructor");
        }

        private:

        //handlers
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

        std::shared_ptr<dynamic_reconfigure::Server<msp::paramConfig>> dr_srv_;

        // threading
        std::thread thread_pcd_modeling;
        std::mutex m_buf;
        std::atomic<bool> keep_running;
        std::queue<sensor_msgs::PointCloud2::ConstPtr> buf_registered_scan;
        std::queue<nav_msgs::Odometry::ConstPtr> buf_state_estimation;
        
        // timers publishers and subscribers
        ros::Timer execution_timer;
        ros::Publisher pub_waypoint;
        ros::Publisher pub_scan_keep;
        ros::Publisher pub_scan_rej;
        ros::Publisher pub_pcd_normals;
        ros::Subscriber sub_explored_areas;
        ros::Subscriber sub_state_estimation;
        ros::Subscriber sub_registered_scan;
  
        //load parameters from launch
        std::string topic_explored_areas;
        std::string topic_waypoint;
        std::string topic_state_estimation;
        std::string topic_registered_scan;
        std::string lidar_type;
        bool lidar_simulation;
        double vertical_resolution,horizontal_resolution;
        int rotation_frequency;
        int samples, beams;
        int height, width, N;
        double d0,divergence;
       

        // float radius,square_distance,min_range;


        // tmp_for_debug: dynamic reconfigure parameters
        double max_incidence_angle_by_loa, max_range_by_loa;
        int normal_ksearch;
        double lod_density;
        
        
        // global parameters
        double ideal_max_range_by_lod;
        double gerc;//Geometry–constructed Effective Radius of Curvature
        double tmp_theta;

        virtual void onInit()
        {
            nh = getNodeHandle();
            private_nh = getPrivateNodeHandle();
            readParameters();

            dr_srv_.reset(new dynamic_reconfigure::Server<msp::paramConfig>(private_nh));  // boost::recursive_mutex dr_mutex_;

            dynamic_reconfigure::Server<msp::paramConfig>::CallbackType cb_dr;
            cb_dr = boost::bind(&PcdModelingNodelet::callback_dynamic_reconfigure, this, _1, _2);
            dr_srv_->setCallback(cb_dr);

            // execution_timer = nh.createTimer(ros::Duration(5.0), &PcdModelingNodelet::execute, this);

            sub_explored_areas = nh.subscribe(topic_explored_areas, 10, &PcdModelingNodelet::callback_explored_areas, this);
            sub_state_estimation = nh.subscribe(topic_state_estimation, 10, &PcdModelingNodelet::callback_state_estimation, this);
            sub_registered_scan = nh.subscribe(topic_registered_scan, 100, &PcdModelingNodelet::callback_registered_scan, this);
            
            pub_waypoint        =   nh.advertise<geometry_msgs::PointStamped>(topic_waypoint, 2);
            pub_scan_keep       =   private_nh.advertise<sensor_msgs::PointCloud2>("scan_keep", 10);
            pub_scan_rej        =   private_nh.advertise<sensor_msgs::PointCloud2>("scan_rej", 10);
            pub_pcd_normals     =   private_nh.advertise<sensor_msgs::PointCloud2>("pcd_normals", 10);

            keep_running.store(true);
            thread_pcd_modeling = std::thread(&PcdModelingNodelet::thread_pcd_modeling_function, this);

        }

        void readParameters()
        {
            if (!private_nh.getParam("lod_density", lod_density)) {
                NODELET_ERROR("Parameter 'lod_density' not found!");
            }


            if (!private_nh.getParam("topic_explored_areas", topic_explored_areas)) {
                NODELET_ERROR("Topic 'explored_areas' not found!");
            }

            if (!private_nh.getParam("topic_waypoint", topic_waypoint)) {
                NODELET_ERROR("Topic 'topic_waypoint' not found!");
            }

            if (!private_nh.getParam("topic_state_estimation", topic_state_estimation)) {
                NODELET_ERROR("Topic 'topic_state_estimation' not found!");
            }

            if (!private_nh.getParam("topic_registered_scan", topic_registered_scan)) {
                NODELET_ERROR("Topic 'topic_registered_scan' not found!");
            }

            if (!private_nh.getParam("lidar_type", lidar_type)) {
                NODELET_ERROR("Parameter 'lidar_type' not found!");
            }

            if (!private_nh.getParam("lidar_simulation", lidar_simulation)) {
                NODELET_ERROR("Parameter 'lidar_simulation' not found!");
            }

            if (!private_nh.getParam("vertical_resolution", vertical_resolution)) {
                NODELET_ERROR("Parameter 'vertical_resolution' not found!");
            }

            if (!private_nh.getParam("horizontal_resolution", horizontal_resolution)) {
                NODELET_ERROR("Parameter 'horizontal_resolution' not found!");
            }

            if (!private_nh.getParam("rotation_frequency", rotation_frequency)) {
                NODELET_ERROR("Parameter 'rotation_frequency' not found!");
            }

            if (!private_nh.getParam("samples", samples)) {
                NODELET_ERROR("Parameter 'samples' not found!");
            }

            if (!private_nh.getParam("beams", beams)) {
                NODELET_ERROR("Parameter 'beams' not found!");
            }
            
            if (!private_nh.getParam("initial_spot_size", d0)) {
                NODELET_ERROR("Parameter 'initial_spot_size' not found!");
            }

            if (!private_nh.getParam("horizontal_beam_divergence", divergence)) {
                NODELET_ERROR("Parameter 'horizontal_beam_divergence' not found!");
            }

            // square_distance = radius * radius;
            //horizontal_resolution is about 0.192deg for vip16 with 1875 samples at 10hz. 0.003rad
            //vertical_resolution is about 2deg for vip16 with 16 beams(15-(-15)/(16-1) = 2).
            if(lidar_type=="VLP-16")
            {
                if(lidar_simulation)
                {
                    double tmp_h,tmp_v;
                    tmp_h = 360.0/samples;
                    tmp_v = 2;
                    if(std::abs(tmp_h - horizontal_resolution) > 1e-6 || std::abs(tmp_v - vertical_resolution) > 1e-6)
                    {
                        ROS_ERROR("Check the parameters in the launch file carefully.");
                    }
                    else
                    {
                        horizontal_resolution=tmp_h*M_PI/180.0;
                        vertical_resolution=tmp_v*M_PI/180.0;
                    }
                    height=samples;
                    width=beams;
                }
                else
                {
                    ROS_ERROR("Please provide the parameters yourself");
                }
            }
            else
            {
                ROS_ERROR("In development.");
            }


            N=height*width;
            ideal_max_range_by_lod = lod_density/horizontal_resolution;
            gerc = ideal_max_range_by_lod;
            tmp_theta=horizontal_resolution-divergence;
            if(std::abs(tmp_theta) <=0)
            {
                ROS_ERROR("Spot size is always bigger than the distance bewteen two points, which is impossible.");
            }


            NODELET_INFO("PcdModelingNodelet initialized with parameters:");
        }

        void callback_dynamic_reconfigure(msp::paramConfig &cfg, uint32_t /*level*/) 
        {
            max_incidence_angle_by_loa    = cfg.max_incidence_angle_by_loa;
            max_range_by_loa = cfg.max_range_by_loa;
            normal_ksearch = cfg.normal_ksearch;

            // double lod_val = 0.152;
            // switch (cfg.lod_density_mode) 
            // {
            //     case 0: lod_val = 0.152; break; 
            //     case 1: lod_val = 0.025; break; 
            //     case 2: lod_val = 0.013; break; 
            //     case 3: lod_val = 0.013; break; 
            // }

            // lod_density = lod_val;
            // cfg.lod_density_value = lod_val;

            ideal_max_range_by_lod = lod_density/horizontal_resolution;
            gerc = ideal_max_range_by_lod;
            cfg.ideal_max_range_by_lod = ideal_max_range_by_lod;
            // output lod_density
        }

        void execute(const ros::TimerEvent&)
        {
            // geometry_msgs::PointStamped waypoint;
            // waypoint.header.frame_id = "map";
            // waypoint.header.stamp = ros::Time::now();
            // waypoint.point.x = 1.0;
            // waypoint.point.y = 2.0;
            // waypoint.point.z = 0.0;
            // pub_waypoint.publish(waypoint);
            // NODELET_INFO("Published waypoint");
        }

        void thread_pcd_modeling_function()
        {
            while(keep_running.load())
            {
                sensor_msgs::PointCloud2::ConstPtr msg_registered_scan;
                nav_msgs::Odometry::ConstPtr msg_state_estimation;
                m_buf.lock();
                if (!buf_registered_scan.empty() && !buf_state_estimation.empty())
                {
                    // do a sync
                    msg_registered_scan = buf_registered_scan.front();
                    msg_state_estimation = buf_state_estimation.front();
                    buf_registered_scan.pop();
                    buf_state_estimation.pop();
                }
                m_buf.unlock();
    
                if(msg_registered_scan && msg_state_estimation)
                {
                    pcd_modeling(msg_registered_scan,msg_state_estimation);
                }
                
                std::chrono::milliseconds dura(1);
                std::this_thread::sleep_for(dura);
            }
        }

        void pcd_modeling(const sensor_msgs::PointCloud2ConstPtr& msg_registered_scan,const nav_msgs::Odometry::ConstPtr& msg_state_estimation)
        {
            //OUPTput ideal_max_range_by_lod lod_density_value
            // NODELET_INFO("max_range_by_spot_size: %f,max_range_by_spot_size1: %f", max_range_by_spot_size,max_range_by_spot_size1);

            

            pcl::PointCloud<msp::PointXYZIR>::Ptr cloud(new pcl::PointCloud<msp::PointXYZIR>);
            pcl::fromROSMsg(*msg_registered_scan, *cloud);
            if (cloud->empty()) return;

            // if (cloud->isOrganized ())
            // {
            //     NODELET_WARN("Input cloud is  organized.");
            //     // return;
            // }
  
            // pcl::PointCloud<msp::PointXYZIR>::Ptr cloud(new pcl::PointCloud<msp::PointXYZIR>);
            // std::vector<int> nan_indices;
            // pcl::removeNaNFromPointCloud(*cloud_in, *cloud, nan_indices);
            // if (cloud->empty()) return;



            // pcl::search::OrganizedNeighbor<msp::PointXYZIR>::Ptr organized_tree(new pcl::search::OrganizedNeighbor<msp::PointXYZIR>);
            // organized_tree->setInputCloud(cloud);
            
               
            

            

            pcl::console::TicToc timer_res;
            timer_res.tic();
            double res = computeCloudResolution(cloud);
            // NODELET_INFO("Cloud resolution: %f", res);
            double timer_res_end = timer_res.toc();
            // std::cout << "res: " << timer_res_end << " ms" << std::endl;

            //step1: normals on current scan or pcd map;
            pcl::console::TicToc timer_normal;
            timer_normal.tic();
            
                pcl::NormalEstimation<msp::PointXYZIR, pcl::Normal> ne;
                pcl::search::KdTree<msp::PointXYZIR>::Ptr tree(new pcl::search::KdTree<msp::PointXYZIR>());
                ne.setInputCloud(cloud);
                ne.setSearchMethod(tree);
                    // ne.setRadiusSearch(3*res); // or setKSearch(k)
                    ne.setKSearch(normal_ksearch);
                pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
                ne.compute(*normals);

            double timer_normal_end = timer_normal.toc();
            // std::cout << "normal: " << timer_normal_end << " ms" << std::endl;



            //thread1: incidence_angle
            pcl::console::TicToc timer_incidence_angle;
            timer_incidence_angle.tic();


            // std::vector<int> idx_kept(N,0),idx_rejected(N,0); 
            double x0 = msg_state_estimation->pose.pose.position.x;
            double y0 = msg_state_estimation->pose.pose.position.y;
            double z0 = msg_state_estimation->pose.pose.position.z;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_kept(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rejected(new pcl::PointCloud<pcl::PointXYZRGB>);

            std::vector<std::vector<float>> incidence_angles(height, std::vector<float>(width, 0.0f));
            for (int row = 0; row < height; ++row) {
                for (int col = 0; col < width; ++col) {
                    int idx = row * width + col;
                    const auto& pt = cloud->points[idx];
                    if (!pcl::isFinite(pt)) continue;
                    const auto& normal = normals->points[idx];
                    Eigen::Vector3f laser_dir(pt.x - x0, pt.y - y0, pt.z - z0);
                    
                    Eigen::Vector3f laser_dir_h(laser_dir.x(), laser_dir.y(), 0.0f);
                    laser_dir_h.normalize();
                    Eigen::Vector3f laser_dir_v(0.0f, laser_dir.y(), laser_dir.z());
                    laser_dir_v.normalize();
                    float laser_distance = laser_dir.norm();
                    laser_dir.normalize();

                    Eigen::Vector3f normal_vec(normal.normal_x, normal.normal_y, normal.normal_z);
                    Eigen::Vector3f normal_vec_h(normal_vec.x(), normal_vec.y(), 0.0f);
                    // normal_vec_h.normalize();
                    Eigen::Vector3f normal_vec_v(0.0f, normal_vec.y(), normal_vec.z());
                    // normal_vec_v.normalize();
                    normal_vec.normalize();


                    // incidence
                    float cos_theta = laser_dir.dot(normal_vec);
                    cos_theta = std::max(-1.0f, std::min(1.0f, cos_theta));
                    float theta_deg = std::acos(std::abs(cos_theta)) * 180.0f / M_PI;
                    incidence_angles[row][col] = theta_deg;

                    float theta_deg_h=0;
                    float cos_theta_h=1; 
                    // horizontal_incidence 
                    if (normal_vec_h.norm() > 1e-6)
                    {
                        normal_vec_h.normalize();
                        cos_theta_h= laser_dir_h.dot(normal_vec_h);
                        cos_theta_h = std::abs(std::max(-1.0f, std::min(1.0f, cos_theta_h)));
                        theta_deg_h= std::acos(cos_theta_h) * 180.0f / M_PI;
                    }
                    else
                    {
                        theta_deg_h = 90.0f;
                        ROS_ERROR("Normal vector horizontal component norm error.");
                    }

                    // if (theta_deg_h > max_incidence_angle_by_loa || laser_distance > max_range_by_loa)

                    double tmp_spotsize = (d0 + laser_distance*divergence)/cos_theta_h;

                    pcl::PointXYZRGB p;
                    p.x = pt.x; p.y = pt.y; p.z = pt.z;
                    p.r = 255; p.g = 255; p.b = 255;

                    // lod_density and tmp
                    bool bool_angle = (theta_deg_h >= max_incidence_angle_by_loa);
                    bool bool_distance = (laser_distance >= max_range_by_loa);
                    bool bool_spotsize = (tmp_spotsize >= lod_density);

                    if (bool_angle || bool_distance || bool_spotsize) //rej
                    {
                        if(bool_angle && !bool_spotsize)
                        {
                            p.r = 255; p.g = 165; p.b = 0; //orange, only angle exceeds the limit
                        }
                        else if(!bool_angle && bool_spotsize)
                        {
                            p.r = 255; p.g = 255; p.b = 0; //yellow, only spotsize exceeds the lod limit
                        }
                        else if (bool_angle && bool_spotsize)
                        {
                            p.r = 255; p.g = 0; p.b = 0; //red, both exceed the limit
                        }
                        cloud_rejected->points.push_back(p);
                    }
                    else //keep
                    {
                        p.r = 0; p.g = 255; p.b = 0;
                        cloud_kept->points.push_back(p);
                    }
                }
            }
            double timer_incidence_angle_end=timer_incidence_angle.toc();
            // std::cout << "incidence_angle: " << timer_incidence_angle_end << " ms" << std::endl;


            

            // int count1 = std::count_if(idx_kept.begin(), idx_kept.end(), [](int x){ return x != 0; });

            // int count2 = std::count_if(idx_rejected.begin(), idx_rejected.end(), [](int x){ return x != 0; });
            // NODELET_INFO("Kept points: %d, Rejected points: %d", count1, count2);

            //publish



            // pcl::PointCloud<pcl::PointNormal>::Ptr cloud_rejected_with_normals(new pcl::PointCloud<pcl::PointNormal>);
            // pcl::PointCloud<pcl::Normal>::Ptr normals_rejected(new pcl::PointCloud<pcl::Normal>);
            // pcl::copyPointCloud(*normals, idx_rejected, *normals_rejected);
            // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rejected_xyz(new pcl::PointCloud<pcl::PointXYZ>);
            // pcl::copyPointCloud(*cloud_rejected, *cloud_rejected_xyz);
            // pcl::concatenateFields(*cloud_rejected_xyz, *normals_rejected, *cloud_rejected_with_normals);





            
            sensor_msgs::PointCloud2 msg_kept, msg_rejected;
            pcl::toROSMsg(*cloud_kept, msg_kept);
            msg_kept.header = msg_registered_scan->header;
            pub_scan_keep.publish(msg_kept);

            pcl::toROSMsg(*cloud_rejected, msg_rejected);
            msg_rejected.header = msg_registered_scan->header;
            pub_scan_rej.publish(msg_rejected);


            //publish normals
            pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*cloud, *cloud_xyz);
            pcl::concatenateFields(*cloud_xyz, *normals, *cloud_with_normals);
            
            pcl::PointCloud<pcl::PointNormal>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointNormal>);
            pcl::VoxelGrid<pcl::PointNormal> sor;
            sor.setInputCloud(cloud_with_normals);
            sor.setLeafSize(0.5f, 0.5f, 0.5f);
            sor.filter(*cloud_filtered);

            sensor_msgs::PointCloud2 msg_pcd_normals;
            pcl::toROSMsg(*cloud_filtered, msg_pcd_normals);
            msg_pcd_normals.header = msg_registered_scan->header;
            pub_pcd_normals.publish(msg_pcd_normals);

            
            // NODELET_INFO("Registered Scan - Frame ID: %s, Timestamp: %u", msg_registered_scan->header.frame_id.c_str(), msg_registered_scan->header.stamp.sec);
        }

        double computeCloudResolution(const pcl::PointCloud<msp::PointXYZIR>::ConstPtr &cloud)
        {
            double res = 0.0;
            int n_points = 0;
            int nres;
            std::vector<int> indices (2);
            std::vector<float> sqr_distances (2);
            pcl::search::KdTree<msp::PointXYZIR> tree;
            tree.setInputCloud (cloud);

            for (std::size_t i = 0; i < cloud->size (); ++i)
            {
                if (! std::isfinite ((*cloud)[i].x))
                {
                    continue;
                }

                nres = tree.nearestKSearch (i, 2, indices, sqr_distances);

                if (nres == 2)
                {
                    res += sqrt (sqr_distances[1]);
                    ++n_points;
                }
            }

            if (n_points != 0)
            {
                res /= n_points;
            }

            return res;
        }

        void callback_explored_areas(const sensor_msgs::PointCloud2ConstPtr& msg_explored_areas)
        {
            
            // uint32_t num_points = msg_explored_areas->width * msg_explored_areas->height;
            // NODELET_INFO("Number of points in PointCloud2: %u", num_points);
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
PLUGINLIB_EXPORT_CLASS(msp::PcdModelingNodelet, nodelet::Nodelet)