#pragma once
#include "common.h"

#include <boost/filesystem.hpp>
#include <boost/functional/hash.hpp>
#include "ikd_Tree_impl.h"

namespace msp
{   
    class ikdtree
    {
        using PointType = ikdtreeNS::ikdTree_PointType;
        using PointVector = ikdtreeNS::KD_TREE<PointType>::PointVector;

        
        public:
            ikdtree() {}
            ~ikdtree() {}

            ikdtreeNS::KD_TREE<PointType> ikd_tree_{0.3f, 0.6f, 0.2f};

            PointVector points_inserted; 
            PointVector points_updated; 
            PointVector points_deleted; 

            void update( const std::vector<Eigen::Vector3d>& points_inserted_in,
                        const std::vector<Eigen::Vector3d>& points_deleted_in,
                        const std::vector<Eigen::Vector3d>& points_updated_in)
            {
                
                points_inserted.clear();
                points_deleted.clear();
                points_updated.clear();

                for (const auto& p : points_inserted_in)
                {
                    points_inserted.emplace_back(PointType{static_cast<float>(p.x()),
                                                        static_cast<float>(p.y()),
                                                        static_cast<float>(p.z())});
                }
                for (const auto& p : points_deleted_in)
                {
                    points_deleted.emplace_back(PointType{static_cast<float>(p.x()),
                                                        static_cast<float>(p.y()),
                                                        static_cast<float>(p.z())});
                }
                for (const auto& p : points_updated_in)
                {
                    points_updated.emplace_back(PointType{static_cast<float>(p.x()),
                                                        static_cast<float>(p.y()),
                                                        static_cast<float>(p.z())});
                }

                if(!initialized_)
                {
                    ikd_tree_.Build(points_inserted);
                    initialized_ = true;
                }
                else
                {
                        
                    ikd_tree_.Delete_Points(points_deleted);
                    ikd_tree_.Add_Points(points_updated,false);
                    ikd_tree_.Add_Points(points_inserted,false);
                } 

            }

            void Nearest_Search(const Eigen::Vector3d& pt_in, int k,
                    std::vector<Eigen::Vector3d>& neighbors_out,
                    std::vector<float>& neighbor_d2)
            {
                PointType query_pt(static_cast<float>(pt_in.x()),
                                    static_cast<float>(pt_in.y()),
                                    static_cast<float>(pt_in.z()));

                PointVector neighbors_in;
                ikd_tree_.Nearest_Search(query_pt, k, neighbors_in, neighbor_d2);
                neighbors_out.reserve(neighbors_in.size());
                for (const auto& pt : neighbors_in)
                {
                    Eigen::Vector3d point(pt.x, pt.y, pt.z);
                    neighbors_out.emplace_back(point);
                }
            }

        private:
            bool initialized_ = false;
    };//end class
}// namespace msp



namespace msp
{           
    struct Coord3Key
    {
        int x= INT_MAX;
        int y= INT_MAX;
        int z= INT_MAX;

        bool operator==(const Coord3Key& other) const
        {
            return x == other.x && y == other.y && z == other.z;
        }


        bool is_valid() const
        {
            return !(x == INT_MAX && y == INT_MAX && z == INT_MAX);
        }
    };

    struct VoxelData
    {
        Eigen::Vector3d rep;
        Eigen::Vector3d voxel_min;

        // Eigen::Vector3d nn;
        // double nn_dist2 = std::numeric_limits<double>::infinity();
        // Coord3Key nn_key;
        // std::unordered_set<Coord3Key, Coord3KeyHash> nn_relationships;

        VoxelData(const Eigen::Vector3d& r, const Eigen::Vector3d& min)
            : rep(r), voxel_min(min)
        {}

    };

    struct Coord3KeyHash
    {
        std::size_t operator()(const Coord3Key& k) const
        {
            std::size_t seed = 0;
            boost::hash_combine(seed, k.x);
            boost::hash_combine(seed, k.y);
            boost::hash_combine(seed, k.z);
            return seed;
        }
    };
}// namespace msp


namespace msp
{               
    class imad
    {
        private:
        
            //nn search
            ikdtree ikd_tree_;
            double sum_nn_dist = 0.0;

            //database
            bool use_database_ = false;
            std::string pkg_path_;

            //downsample
            const double chunk_size_ = 1.0;
            int chunk_res_n;
            double leaf_size_ = 0.1;
            double inverse_leaf_size_ = 0.0;
            // int decimation = 5;
            double lod_density2_ = 0.0;
            int nbr_voxel_range_ = 1; 
            
            //global map in memory
            std::unordered_map<Coord3Key, VoxelData, Coord3KeyHash> global_voxel_hashmap_;
            std::unordered_map<Coord3Key, Eigen::Vector3d, Coord3KeyHash> pcd_density_globalmap_hashmap_;

            // double voxel_size_ = 0.1;
            // double d0;
            // double weight_alpha_;
            // double max_weight_ ;


            mutable std::mutex map_mutex_;
            std::mutex ikdtree_mutex_;
            std::mutex pcd_mutex_;
            std::queue<sensor_msgs::PointCloud2> pcd_buf;
            std::condition_variable pcd_cv_;
            std::thread thread_processing;
            bool stop_processing_ = false;

            std::atomic<bool> MapUpdate{false};
            ros::Time timestamp;

        public:

            void insertPointCloud(sensor_msgs::PointCloud2 msg_cloud_in)
            {
                {
                    std::lock_guard<std::mutex> lock(pcd_mutex_);
                    pcd_buf.push(std::move(msg_cloud_in));
                }
                pcd_cv_.notify_one();
            }

            imad(double lod_density)
            {
                // double inv_tmp=std::sqrt(decimation * decimation + 4.0 * decimation + 5.0)/lod_density;
                // double inv_tmp=std::sqrt(3.0)/lod_density;
                int mode =1;
                if(mode==1)
                {
                    nbr_voxel_range_ = 1;
                }
                else
                {
                    nbr_voxel_range_ = 2;
                }

                double inv_tmp= std::sqrt(static_cast<double>(mode))/lod_density;

                chunk_res_n = static_cast<int>(std::floor(inv_tmp)) + 1;
                // voxel_size_ = chunk_size_ / static_cast<double>(chunk_res_n);
                // inverse_leaf_size_ = static_cast<double>(chunk_res_n) / chunk_size_;
                inverse_leaf_size_ = static_cast<double>(chunk_res_n);
                leaf_size_ = 1.0 / inverse_leaf_size_;

                lod_density2_ = lod_density*lod_density;

                // voxel_size_=0.02,0.003,0.001;
                std::cout<< "\033[1;32m"<< "voxel_size: "<< leaf_size_<< "\033[0m" << std::endl;
                std::cout<< "\033[1;32m"<< "inverse_leaf_size_: "<< inverse_leaf_size_<< "\033[0m" << std::endl;
                
                thread_processing = std::thread([this]() {processing();});
            }

        private:

            void processing()
            {
                while(ros::ok())
                {
                    sensor_msgs::PointCloud2 msg_cloud_in;
                    {
                        std::unique_lock<std::mutex> lock(pcd_mutex_); 
                        pcd_cv_.wait(lock, [this] {
                                        return !pcd_buf.empty() || stop_processing_ || !ros::ok();
                                    });
                        if ((!ros::ok() ||stop_processing_ ) && pcd_buf.empty())
                            break;
                        msg_cloud_in = std::move(pcd_buf.front());
                        pcd_buf.pop();
                    }

                    incrementalDownsampingAndMapping(msg_cloud_in);               

                }
            }

            void incrementalDownsampingAndMapping(const sensor_msgs::PointCloud2& msg_cloud_in)
            {

                pcl::console::TicToc timer_all;
                timer_all.tic();
                pcl::PointCloud<PointT> cloud_in;
                pcl::fromROSMsg(msg_cloud_in, cloud_in);
                // std::unordered_map<Coord3Key, std::unordered_map<Coord3Key, VoxelData, Coord3KeyHash>, Coord3KeyHash> frame_chunks;
                std::unordered_map<Coord3Key, VoxelData, Coord3KeyHash> frame_voxel_hashmap_;

                for (const auto& pt : cloud_in.points)
                {
                    Eigen::Vector3d p(pt.x, pt.y, pt.z);
                    
                    if(use_database_)
                    {
                        // int cx = static_cast<int>(std::floor(p.x()));
                        // int cy = static_cast<int>(std::floor(p.y()));
                        // int cz = static_cast<int>(std::floor(p.z()));
                        // int vx = std::max(0, std::min(static_cast<int>(std::floor((p.x() - cx) * inverse_leaf_size_)), chunk_res_n - 1));
                        // int vy = std::max(0, std::min(static_cast<int>(std::floor((p.y() - cy) * inverse_leaf_size_)), chunk_res_n - 1));
                        // int vz = std::max(0, std::min(static_cast<int>(std::floor((p.z() - cz) * inverse_leaf_size_)), chunk_res_n - 1));
                        // Coord3Key chunk_key{cx, cy, cz};
                        // Coord3Key local_voxel_key{vx, vy, vz};
                        // auto& voxel = frame_chunks[chunk_key][local_voxel_key];
                        // voxel.points.push_back(p);
                        // voxel.sum += p;
                    }
                    else
                    {
                        Coord3Key voxel_key = point2Key(p);
                        Eigen::Vector3d voxel_min = Eigen::Vector3d(voxel_key.x, voxel_key.y, voxel_key.z) * leaf_size_;                        // Coord3Key voxel_key{cx * chunk_res_n + vx, cy * chunk_res_n + vy, cz * chunk_res_n + vz};
                        // auto [it, inserted] = global_voxel_hashmap_.try_emplace(voxel_key, p);
                        auto [it, inserted] = frame_voxel_hashmap_.try_emplace(voxel_key, VoxelData(p, voxel_min));
                        
                        if (!inserted && (p - voxel_min).squaredNorm() > (it->second.rep-voxel_min).squaredNorm())
                            it->second.rep = p;
                    }
                    
                }

               

                if(use_database_)
                {
                    // mappingByDatabase(frame_chunks);
                }
                else
                {                        

                    std::tuple<std::vector<Coord3Key>, std::vector<Eigen::Vector3d>> points_inserted;
                    std::tuple<std::vector<Coord3Key>, std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> points_updated;
                    // std::vector<Eigen::Vector3d> points_inserted;
                    // std::vector<Eigen::Vector3d> points_updated;
                    // std::vector<Eigen::Vector3d> points_deleted;

                    // ikd_tree_.clear_inputvec();

                    {
                        std::lock_guard<std::mutex> lock(map_mutex_);
                        for (const auto& kv : frame_voxel_hashmap_)
                        {
                            const Coord3Key& voxel_key = kv.first;
                            const Eigen::Vector3d& p = kv.second.rep;
                            const Eigen::Vector3d& voxel_min = kv.second.voxel_min;
                            // 插入成功,判断密度;插入失败,距离更大,更新并判断密度; 插入时把,距离更小,不更新也不判断密度
                            
                                auto [it, inserted] = global_voxel_hashmap_.try_emplace(voxel_key, VoxelData{p,voxel_min}); 
                            
                            

                            if(inserted)
                            {
                                std::get<0>(points_inserted).push_back(voxel_key);
                                std::get<1>(points_inserted).push_back(p);
                                updateDensityMap(voxel_key, p);
                            }
                            else 
                            {
                                if ((p - voxel_min).squaredNorm() > (it->second.rep - it->second.voxel_min).squaredNorm())
                                {
                                    std::get<0>(points_updated).push_back(voxel_key);
                                    std::get<1>(points_updated).push_back(it->second.rep);
                                    std::get<2>(points_updated).push_back(p);
                                    it->second.rep = p;
                                    updateDensityMap(voxel_key, p);
                                }
                            }
                        }

                        if (!std::get<0>(points_inserted).empty() ||!std::get<0>(points_updated).empty())
                        {
                            timestamp = msg_cloud_in.header.stamp;
                            MapUpdate.store(true);
                        }
                    }
                   

                    // {
                    //     std::lock_guard<std::mutex> lock(ikdtree_mutex_);
                    //     ikd_tree_.update(std::get<1>(points_inserted),std::get<1>(points_updated),std::get<2>(points_updated));
                    // }
                    
                    
                    

                    // std::cout << "\033[1;32m" << "Average nearest neighbor distance: " << (sum_nn_dist / global_voxel_hashmap_.size()) << " m" << "\033[0m" << std::endl;
                    
                    auto elapsed_total = timer_all.toc();
                    // std::cout << "\033[1;32m" << "timer_all : " << elapsed_total << " ms" << "\033[0m" << std::endl;

                }
            

            }

            Coord3Key point2Key(const Eigen::Vector3d& p) const
            {
                int cx = static_cast<int>(std::floor(p.x()));
                int cy = static_cast<int>(std::floor(p.y()));
                int cz = static_cast<int>(std::floor(p.z()));
                int vx = std::max(0, std::min(static_cast<int>(std::floor((p.x() - cx) * inverse_leaf_size_)), chunk_res_n - 1));
                int vy = std::max(0, std::min(static_cast<int>(std::floor((p.y() - cy) * inverse_leaf_size_)), chunk_res_n - 1));
                int vz = std::max(0, std::min(static_cast<int>(std::floor((p.z() - cz) * inverse_leaf_size_)), chunk_res_n - 1));
                return Coord3Key{cx * chunk_res_n + vx, cy * chunk_res_n + vy, cz * chunk_res_n + vz};
            }

            bool hasNeighbor(const Coord3Key& voxel_key, const Eigen::Vector3d& rep) const
            {
                for (int ox = nbr_voxel_range_; ox >= -nbr_voxel_range_ ; --ox)
                for (int oy = nbr_voxel_range_; oy >= -nbr_voxel_range_ ; --oy)
                for (int oz = nbr_voxel_range_; oz >= -nbr_voxel_range_ ; --oz)
                {
                    if (ox == 0 && oy == 0 && oz == 0) continue;
                    Coord3Key ngk{voxel_key.x + ox, voxel_key.y + oy, voxel_key.z + oz};
                    auto it = global_voxel_hashmap_.find(ngk);
                    if (it != global_voxel_hashmap_.end())
                    {
                        const auto& val = it->second;
                        double dist2 = (rep - val.rep).squaredNorm();
                        if(dist2 < lod_density2_)
                        {
                            return true;
                        }
                    }
                }
                return false;
            }

            bool DensityCheck(const Coord3Key& voxel_key, const Eigen::Vector3d& rep, 
                                std::vector<std::pair<Coord3Key, Eigen::Vector3d>>& nbr_list_remove, 
                                std::vector<std::pair<Coord3Key, Eigen::Vector3d>>& nbr_list_add)
            {
                bool has_neighbor = false;
                
                for (int ox = nbr_voxel_range_; ox >= -nbr_voxel_range_ ; --ox)
                for (int oy = nbr_voxel_range_; oy >= -nbr_voxel_range_ ; --oy)
                for (int oz = nbr_voxel_range_; oz >= -nbr_voxel_range_ ; --oz)
                {
                    if (ox == 0 && oy == 0 && oz == 0) continue;
                    Coord3Key ngk{voxel_key.x + ox, voxel_key.y + oy, voxel_key.z + oz};
                    auto it = global_voxel_hashmap_.find(ngk);
                    if (it != global_voxel_hashmap_.end())
                    {
                        const auto& val = it->second;
                        double dist2 = (rep - val.rep).squaredNorm();
                        if(dist2 < lod_density2_)
                        {
                            has_neighbor = true;
                            nbr_list_remove.push_back({ngk, val.rep});
                        }
                        else
                        {
                            nbr_list_add.push_back({ngk, val.rep});
                        }
                    }
                }
                
                
                return has_neighbor;
            }

            void updateDensityMap(const Coord3Key& voxel_key, const Eigen::Vector3d& rep)
            {

                bool has_neighbor = false;
                std::vector<std::pair<Coord3Key, Eigen::Vector3d>> nbr_list_remove;
                std::vector<std::pair<Coord3Key, Eigen::Vector3d>> nbr_list_add;
                has_neighbor = DensityCheck(voxel_key, rep, nbr_list_remove, nbr_list_add);

                if (has_neighbor)
                    pcd_density_globalmap_hashmap_.erase(voxel_key);
                else
                    pcd_density_globalmap_hashmap_[voxel_key] = rep;

                for (const auto& [key, point] : nbr_list_remove) 
                    pcd_density_globalmap_hashmap_.erase(key);
            
                for (const auto& [key, point] : nbr_list_add) 
                {
                    bool nbr_has_neighbor = hasNeighbor(key, point);
                    if(nbr_has_neighbor)
                        pcd_density_globalmap_hashmap_.erase(key);
                    else
                        pcd_density_globalmap_hashmap_[key] = point;

                }
            }
          
            
        public: 

            bool isMapUpdate()
            {
                return MapUpdate.exchange(false);
            }

            sensor_msgs::PointCloud2 buildDownsampledMap() const
            {
                sensor_msgs::PointCloud2 msg;
                pcl::PointCloud<pcl::PointXYZ> cloud;
                ros::Time map_timestamp;

                pcl::console::TicToc timer_nocache;
                timer_nocache.tic();
               
                {
                    map_timestamp = timestamp;
                    std::lock_guard<std::mutex> lock(map_mutex_);
                    cloud.points.reserve(global_voxel_hashmap_.size());
                    for (const auto& kv : global_voxel_hashmap_)
                    {
                        const auto& val = kv.second;
                        cloud.points.emplace_back(val.rep.x(), val.rep.y(), val.rep.z());
                    }
                }

                
                auto elapsed_nocache = timer_nocache.toc();
                // std::cout << "\033[1;32m" << "map no cache time : " << elapsed_nocache << " ms" << "\033[0m" << std::endl;

                cloud.width = cloud.points.size();
                cloud.height = 1;
                cloud.is_dense = true;
                pcl::toROSMsg(cloud, msg);
                msg.header.frame_id = "map";
                msg.header.stamp = map_timestamp;
                return msg;
            }

            sensor_msgs::PointCloud2 buildDensityMap() const
            {
                sensor_msgs::PointCloud2 msg;
                pcl::PointCloud<pcl::PointXYZ> cloud;
                ros::Time map_timestamp;

                pcl::console::TicToc timer_nocache;
                timer_nocache.tic();

                {
                    std::lock_guard<std::mutex> lock(map_mutex_);
                    map_timestamp = timestamp;
                    cloud.points.reserve(pcd_density_globalmap_hashmap_.size());
                    for (const auto& kv : pcd_density_globalmap_hashmap_)
                    {
                        cloud.points.emplace_back(kv.second.x(), kv.second.y(), kv.second.z());
                    }
                }
                auto elapsed_nocache = timer_nocache.toc();
                // std::cout << "\033[1;32m" << "density no cache time : " << elapsed_nocache << " ms" << "\033[0m" << std::endl;
               
                cloud.width = cloud.points.size();
                cloud.height = 1;
                cloud.is_dense = true;
                pcl::toROSMsg(cloud, msg);
                msg.header.frame_id = "map";
                msg.header.stamp = map_timestamp;
                return msg;
            }

            void compNNDist()
            {
                pcl::console::TicToc timer_cache;
                timer_cache.tic();
                double g_sum_nn_dist = 0.0;
                std::vector<Eigen::Vector3d> neighbors;
                std::vector<float> neighbor_d2;
                auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

                // std::unordered_map<Coord3Key, VoxelData,Coord3KeyHash> tmp_map;
                {
                    std::lock_guard<std::mutex> lock(map_mutex_);
                    // tmp_map = global_voxel_hashmap_;  
                    cloud->points.reserve(global_voxel_hashmap_.size());
                    for (const auto& kv : global_voxel_hashmap_)
                    {
                        const auto& p = kv.second.rep;
                        cloud->points.emplace_back(p.x(), p.y(), p.z());
                    }
                }
                cloud->width = cloud->points.size();
                cloud->height = 1;
                cloud->is_dense = true;
                auto elapsed_cache = timer_cache.toc();
                // std::cout << "\033[1;32m" << "nn cache time : " << elapsed_cache << " ms" << "\033[0m" << std::endl;

                pcl::console::TicToc timer_nn;
                timer_nn.tic();
                pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
                kdtree.setInputCloud(cloud);
                std::vector<int> k_indices;
                std::vector<float> k_sqr_distances;
                k_indices.reserve(2);
                k_sqr_distances.reserve(2);
                for (const auto& pt : cloud->points)
                {
                    k_indices.clear();
                    k_sqr_distances.clear();

                    if (kdtree.nearestKSearch(pt, 2, k_indices, k_sqr_distances) >= 2)
                    {
                        g_sum_nn_dist += std::sqrt(k_sqr_distances[1]);
                    }
                }
                auto elapsed_nn = timer_nn.toc();
                // std::cout << "\033[1;32m" << "nn search time : " << elapsed_nn << " ms" << "\033[0m" << std::endl;

                std::cout<<"Average NNDist: "<<g_sum_nn_dist/static_cast<double>(cloud->points.size())<<std::endl;

            }

            void compNNDistIkdtree()
            {
               
                double g_sum_nn_dist = 0.0;
                std::vector<Eigen::Vector3d> neighbors;
                std::vector<float> neighbor_d2;

                pcl::console::TicToc timer_cache;
                timer_cache.tic();
                std::unordered_map<Coord3Key, VoxelData,Coord3KeyHash> tmp_map;
                {
                    std::lock_guard<std::mutex> lock(map_mutex_);
                    tmp_map = global_voxel_hashmap_;  
                }
                auto elapsed_cache = timer_cache.toc();
                std::cout << "\033[1;32m" << "nn cache time : " << elapsed_cache << " ms" << "\033[0m" << std::endl;
                
                
                pcl::console::TicToc timer_nn;
                timer_nn.tic();
                {
                    std::lock_guard<std::mutex> lock(ikdtree_mutex_);
                  
                    for (auto& it : tmp_map)
                    {
                        VoxelData& voxel_data = it.second;
                        neighbors.clear();
                        neighbor_d2.clear();
                        ikd_tree_.Nearest_Search(voxel_data.rep, 2, neighbors, neighbor_d2);
                        auto nn_key = point2Key(neighbors[1]);
                        g_sum_nn_dist += std::sqrt(neighbor_d2[1]);
                    }
                   
                }
                auto elapsed_nn = timer_nn.toc();
                std::cout << "\033[1;32m" << "nn search time : " << elapsed_nn << " ms" << "\033[0m" << std::endl;

                
                // std::cout<<"Average NNDist: "<<g_sum_nn_dist/static_cast<double>(tmp_map.size())<<std::endl;
            }

            void setPcdDbPath(const std::string& pkg_path)
            {
                pkg_path_= pkg_path;
                use_database_=true;

                boost::filesystem::path db_path(pkg_path);
                db_path /= "map/voxels_hash"; 
                if (!boost::filesystem::exists(db_path))
                {
                    boost::filesystem::create_directories(db_path);
                }
                else
                {
                    for (auto& entry : boost::filesystem::directory_iterator(db_path))
                    {
                        boost::filesystem::remove_all(entry.path());
                    }
                }
                boost::filesystem::permissions(
                    db_path,
                    boost::filesystem::owner_all |
                    boost::filesystem::group_all |
                    boost::filesystem::others_all
                );
                db_path /= "indoor.db"; 
                //input path for pcdDB class.boost::filesystem::path 
            }

            ~imad()
            {
                {
                    std::lock_guard<std::mutex> lock(pcd_mutex_);
                    stop_processing_ = true;
                }
                pcd_cv_.notify_all();
                if (thread_processing.joinable())
                    thread_processing.join();
            }


    };//end class

}// namespace msp