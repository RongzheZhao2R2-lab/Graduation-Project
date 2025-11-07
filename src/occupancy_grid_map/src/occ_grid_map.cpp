#include "occupancy_grid_map/occ_grid_map.h"
#include "occupancy_grid_map/raycast.h"

namespace CUADC{

void Map::initMap(ros::NodeHandle &nh)
{
    // omp_set_num_threads(4);
    nh_ = nh;
    double x_size, y_size, z_size;
    nh_.param("use_pose", cam_.use_pose_, true);
    nh_.param("use_vision", cam_.use_vision_, true);
    nh_.param("use_2d_lidar", cam_.use_2d_lidar_, false);
    nh_.param("occupancy_grid_map/resolution", mp_.resolution_, -1.0);
    nh_.param("occupancy_grid_map/map_size_x", x_size, -1.0);
    nh_.param("occupancy_grid_map/map_size_y", y_size, -1.0);
    nh_.param("occupancy_grid_map/map_size_z", z_size, -1.0);
    nh_.param("occupancy_grid_map/show_local_map", mp_.show_local_map_, true);
    nh_.param("occupancy_grid_map/show_global_map", mp_.show_global_map_, false);
    nh_.param("occupancy_grid_map/obstacles_inflation", mp_.obstacles_inflation_, -1.0);
    nh_.param("occupancy_grid_map/ground_height", mp_.ground_height_, -1.0);
    nh_.param("occupancy_grid_map/cam_depth_width", cam_.cam_depth_width_, 640);
    nh_.param("occupancy_grid_map/cam_depth_height", cam_.cam_depth_height_, 480);
    nh_.param("occupancy_grid_map/k_depth_scaling_factor", cam_.k_depth_scaling_factor_, -1.0);
    nh_.param("occupancy_grid_map/fx",cam_.fx_, -1.0);
    nh_.param("occupancy_grid_map/fy",cam_.fy_, -1.0);
    nh_.param("occupancy_grid_map/cx",cam_.cx_, -1.0);
    nh_.param("occupancy_grid_map/cy",cam_.cy_, -1.0);
    nh_.param("occupancy_grid_map/depth_maxdist", cam_.depth_maxdist_, -1.0);
    nh_.param("occupancy_grid_map/depth_minidist", cam_.depth_minidist_, -1.0);
    nh_.param("occupancy_grid_map/max_ray_length", cam_.max_ray_length_, -1.0);
    nh_.param("occupancy_grid_map/min_ray_length", cam_.min_ray_length_, -1.0);
    nh_.param("occupancy_grid_map/depth_filter_margin", cam_.depth_filter_margin_, -1);
    nh_.param("occupancy_grid_map/skip_pixel", cam_.skip_pixel_, -1);
    nh_.param("occupancy_grid_map/p_occ_max", mp_.p_occ_max_, 0.7);
    nh_.param("occupancy_grid_map/p_occ_min", mp_.p_occ_min_, 0.12);
    nh_.param("occupancy_grid_map/lofree", mp_.lofree_, -1.0);
    nh_.param("occupancy_grid_map/looccu", mp_.looccu_, 1.0);
    nh_.param("occupancy_grid_map/frame_id", mp_.frame_id_, std::string("world"));
    
    // paramters init
    mp_.resolution_inv_ = 1.0 / mp_.resolution_;
    mp_.map_origin_ = Eigen::Vector3d(-x_size / 2, -y_size / 2, mp_.ground_height_);
    mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

    pos2Index_global(mp_.map_origin_, mp_.map_origin_id_);
    index2Pos_global(mp_.map_origin_id_, mp_.map_origin_);

    mp_.prob_max_logit_ = logit(mp_.p_occ_max_);
    mp_.prob_min_logit_ = logit(mp_.p_occ_min_);
    mp_.lofree_ = logit(mp_.lofree_);
    mp_.looccu_ = logit(mp_.looccu_);

    ROS_INFO_STREAM("resolution " << mp_.resolution_);
    ROS_INFO_STREAM("map size x " << x_size);
    ROS_INFO_STREAM("map size y " << y_size);
    ROS_INFO_STREAM("map size z " << z_size);
    ROS_INFO_STREAM("obstacles inflation " << mp_.obstacles_inflation_);
    ROS_INFO_STREAM("ground height " << mp_.ground_height_);
    ROS_INFO_STREAM("cam depth width " << cam_.cam_depth_width_);
    ROS_INFO_STREAM("cam depth height " << cam_.cam_depth_height_);
    ROS_INFO_STREAM("k depth scaling factor " << cam_.k_depth_scaling_factor_);
    ROS_INFO_STREAM("fx " << cam_.fx_);
    ROS_INFO_STREAM("fy " << cam_.fy_);
    ROS_INFO_STREAM("cx " << cam_.cx_);
    ROS_INFO_STREAM("cy " << cam_.cy_);
    ROS_INFO_STREAM("depth maxdist " << cam_.depth_maxdist_);
    ROS_INFO_STREAM("depth minidist " << cam_.depth_minidist_);
    ROS_INFO_STREAM("max ray length " << cam_.max_ray_length_);
    ROS_INFO_STREAM("skip pixel " << cam_.skip_pixel_);
    ROS_INFO_STREAM("frame id" << mp_.frame_id_);
    ROS_INFO_STREAM("max logit " << mp_.prob_max_logit_);
    ROS_INFO_STREAM("min logit " << mp_.prob_min_logit_);
    ROS_INFO_STREAM("lofree " << mp_.lofree_);
    ROS_INFO_STREAM("looccu " << mp_.looccu_);

    for(uint i = 0; i < 3; ++i)
        mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);
    
    mp_.new_map_min_boundary_ = mp_.map_origin_;
    mp_.new_map_max_boundary_ = mp_.map_size_ + mp_.map_origin_;
    mp_.old_map_min_boundary_ = mp_.new_map_min_boundary_;
    mp_.old_map_max_boundary_ = mp_.new_map_max_boundary_;
    mp_.raycast_num_ = 0;
    
    mp_.intersection_id_max_ = Eigen::Vector3i::Zero();
    mp_.intersection_id_min_ = Eigen::Vector3i::Zero();

    int voxel_num = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);
    InflateVoxel v(VoxelState::FREE);
    occ_map_.occupany_map_ = std::vector<double>(voxel_num, mp_.prob_min_logit_ - 0.5 * std::fabs(mp_.lofree_));
    occ_map_.occupany_map_inflate_ = std::vector<InflateVoxel>(voxel_num, v);
    occ_map_.flag_rayend_ = std::vector<char>(voxel_num, -1);
    occ_map_.flag_traverse_ = std::vector<char>(voxel_num, -1);

    if(cam_.use_vision_)
        cam_.proj_points_.resize(cam_.cam_depth_width_ * cam_.cam_depth_height_ / cam_.skip_pixel_ / cam_.skip_pixel_);
    else
        cam_.proj_points_.resize(10000);

    // ros init
    ROS_INFO("ros init");
    depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh_, "occupancy_grid_map/depth", 10, ros::TransportHints().tcpNoDelay()));
    pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, "occupancy_grid_map/pose", 10, ros::TransportHints().tcpNoDelay()));
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "occupancy_grid_map/odom", 10, ros::TransportHints().tcpNoDelay()));
    scan_sub_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, "occupancy_grid_map/scan", 10, ros::TransportHints().tcpNoDelay()));
    if(cam_.use_pose_ && cam_.use_vision_)
    {
        ROS_INFO("Pose and vision");
        sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
            SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
        sync_image_pose_->registerCallback(boost::bind(&Map::depthPoseCallback, this, _1, _2));
    }
    else if(!cam_.use_pose_ && cam_.use_vision_)
    {
        ROS_INFO("Odometry and vision");
        sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
            SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
        sync_image_odom_->registerCallback(boost::bind(&Map::depthOdomCallback, this, _1, _2));
    }
    else if (cam_.use_pose_ && !cam_.use_vision_)
    {
        ROS_INFO("Pose and lidar");
        sync_scan_pose_.reset(new message_filters::Synchronizer<SyncPolicyScanPose>(
            SyncPolicyScanPose(100), *scan_sub_, *pose_sub_));
        sync_scan_pose_->registerCallback(boost::bind(&Map::scanPoseCallback, this, _1, _2));
    }
    else
    {
        ROS_INFO("Odometry and lidar");
        sync_scan_odom_.reset(new message_filters::Synchronizer<SyncPolicyScanOdom>(
            SyncPolicyScanOdom(100), *scan_sub_, *odom_sub_));
        sync_scan_odom_->registerCallback(boost::bind(&Map::scanOdomCallback, this, _1, _2));
    }

    // ros publisher
    ROS_INFO("ros publisher");
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("occupany_grid_map/grid_map", 10);
    map_inf_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("occupany_grid_map/inflate_grid_map", 10);

#ifdef _MAPDEBUG_
    test_camera_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("occupany_grid_map/test_camera_cloud_map", 10);
    test_camera_cloud_voxel_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("occupany_grid_map/test_camera_cloud_voxel_map", 10);
    test_outside_map_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("occupany_grid_map/test_outside_bound_map", 10);
    test_boundary_pub_ = nh_.advertise<visualization_msgs::Marker>("occupany_grid_map/test_map_boundary", 10);
    test_cloud_in_map_occu_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("occupany_grid_map/test_map_inside_occcu", 10);
    test_cloud_in_map_free_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("occupany_grid_map/test_map_inside_free", 10);
    test_raycast_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("occupany_grid_map/test_raycast", 10);
    test_scan_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("occupany_grid_map/test_scan_cloud", 10);
#endif // _MAPDEBUG_
    
    // ros Timer
    ROS_INFO("ros Timer");
    occ_timer_ = nh_.createTimer(ros::Duration(0.05), &Map::updateOccupancyCallback, this);
    vis_timer_ = nh.createTimer(ros::Duration(0.05), &Map::visualCallback, this);

    mp_.occ_need_update_ = false;
    mp_.local_updated_ = false;
    cam_.last_cam_position_update_sign_ = false;
    cam_.have_last_cam_position_ = false;
    cam_.proj_points_cnt_ = 0;
    if(cam_.use_vision_)
        cam_.cam2body_ << 0.0, 0.0, 1.0, 0.035,
                        -1.0, 0.0, 0.0, 0.0,
                        0.0, -1.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 1.0;
    else
        cam_.cam2body_ = Eigen::Matrix4d::Identity();
}

void Map::projectDepthImage()
{
    // auto start_time = std::chrono::high_resolution_clock::now();

    Eigen::Vector3d update_local_boundary_max(mp_.new_map_min_boundary_);
    Eigen::Vector3d update_local_boundary_min(mp_.new_map_max_boundary_);

    cam_.proj_points_cnt_ = 0;
    int cols = cam_.depth_image_.cols;
    int rows = cam_.depth_image_.rows;

    Eigen::Matrix3d camera_r = cam_.camera_q_.toRotationMatrix();

    Eigen::Vector3d proj_pt;
    uint16_t *row_ptr;

#ifdef _MAPDEBUG_
    pcl::PointCloud<pcl::PointXYZ>().swap(cloud_camera_test_);
#endif

    pcl::PointXYZ pt;
    for(int v = cam_.depth_filter_margin_; v < rows - cam_.depth_filter_margin_; v += cam_.skip_pixel_)
    {
        row_ptr = cam_.depth_image_.ptr<uint16_t>(v) + cam_.depth_filter_margin_;

        for(int u = cam_.depth_filter_margin_; u < cols - cam_.depth_filter_margin_; u += cam_.skip_pixel_)
        {
            double d = (*row_ptr) / cam_.k_depth_scaling_factor_;
            row_ptr = row_ptr + cam_.skip_pixel_;
            // if(u == cols / 2 && v == rows / 2)
            // {
            //     ROS_INFO_STREAM("Depth is " << d);
            // }
            if (*row_ptr == 0)
            {
                d =  cam_.max_ray_length_ + 0.2;
            }
            else if(d <= cam_.depth_minidist_)  
                continue;
            else if(d > cam_.depth_maxdist_)
            {
                 d = cam_.max_ray_length_ + 0.2;
            }
            // camera坐标系
            proj_pt(0) = (u - cam_.cx_) * d / cam_.fx_;
            proj_pt(1) = (v - cam_.cy_) * d / cam_.fy_;
            proj_pt(2) = d;
            // 转换到world坐标系
            proj_pt = camera_r * proj_pt + cam_.camera_position_;

            cam_.proj_points_[cam_.proj_points_cnt_++] = proj_pt;

            update_local_boundary_max = maxVector(proj_pt, update_local_boundary_max);
            update_local_boundary_min = minVector(proj_pt, update_local_boundary_min);
#ifdef _MAPDEBUG_
            pt.x = proj_pt.x();
            pt.y = proj_pt.y();
            pt.z = proj_pt.z();
            cloud_camera_test_.push_back(pt);
#endif
        }
    }
    --cam_.proj_points_cnt_;
    update_local_boundary_max = maxVector(cam_.camera_position_, update_local_boundary_max);
    update_local_boundary_min = minVector(cam_.camera_position_, update_local_boundary_min);
    // ROS_INFO_STREAM("Proj update_local_boundary_max " << update_local_boundary_max);
    // ROS_INFO_STREAM("Proj update_local_boundary_min " << update_local_boundary_min);
    // ROS_INFO_STREAM("the size of proj_points is " << cam_.proj_points_cnt_);
    if(cam_.proj_points_cnt_)
    {
        if(MapBoundary(update_local_boundary_max, update_local_boundary_min, cam_.camera_position_, mp_.map_size_, mp_.old_map_max_boundary_, mp_.old_map_min_boundary_, 
                    mp_.new_map_max_boundary_, mp_.new_map_min_boundary_))
            moveOld2New(mp_.old_map_max_boundary_, mp_.old_map_min_boundary_, 
                        mp_.new_map_max_boundary_, mp_.new_map_min_boundary_,
                        mp_.intersection_id_max_, mp_.intersection_id_min_);
    }
    // auto end_time = std::chrono::high_resolution_clock::now();
    // auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
    // ROS_INFO_STREAM("projectDepthImage execution time: " << duration_ns << "ms");
    // ROS_INFO_STREAM("new_map_max_boundary " << '\n' << mp_.new_map_max_boundary_);
#ifdef _MAPDEBUG_
    if(test_camera_cloud_pub_.getNumSubscribers() > 0)
    {
        cloud_camera_test_.width = cloud_camera_test_.points.size();
        cloud_camera_test_.height = 1;
        cloud_camera_test_.is_dense = true;
        cloud_camera_test_.header.frame_id = "map";
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud_camera_test_, cloud_msg);
        test_camera_cloud_pub_.publish(cloud_msg);
    }
    if(test_boundary_pub_.getNumSubscribers() > 0)
    {
        visualization_msgs::Marker routeMarker, trajMarker;
        routeMarker.id = 0;
        routeMarker.type = visualization_msgs::Marker::LINE_LIST;
        routeMarker.header.stamp = ros::Time::now();
        routeMarker.header.frame_id = "map";
        routeMarker.pose.orientation.w = 1.00;
        routeMarker.action = visualization_msgs::Marker::ADD;
        routeMarker.ns = "route";
        routeMarker.color.r = 1.00;
        routeMarker.color.g = 0.00;
        routeMarker.color.b = 0.00;
        routeMarker.color.a = 1.00;
        routeMarker.scale.x = 0.01;

        trajMarker = routeMarker;
        trajMarker.header.frame_id = "map";
        trajMarker.id = 0;
        trajMarker.ns = "trajectory";
        trajMarker.color.r = 0.00;
        trajMarker.color.g = 0.50;
        trajMarker.color.b = 1.00;
        trajMarker.scale.x = 0.01;
        
        geometry_msgs::Point p_max, p_max_1, p_max_2, p_max_3;
        p_max.x = mp_.new_map_max_boundary_.x();
        p_max.y = mp_.new_map_max_boundary_.y();
        p_max.z = mp_.new_map_max_boundary_.z();

        geometry_msgs::Point p_min, p_min_1, p_min_2, p_min_3;
        p_min.x = mp_.new_map_min_boundary_.x();
        p_min.y = mp_.new_map_min_boundary_.y();
        p_min.z = mp_.new_map_min_boundary_.z();

        p_max_1.x = p_max.x;
        p_max_1.y = p_min.y;
        p_max_1.z = p_max.z;

        p_max_2.x = p_min.x;
        p_max_2.y = p_min.y;
        p_max_2.z = p_max.z;

        p_max_3.x = p_min.x;
        p_max_3.y = p_max.y;
        p_max_3.z = p_max.z;

        trajMarker.points.push_back(p_max);
        trajMarker.points.push_back(p_max_1);

        trajMarker.points.push_back(p_max_1);
        trajMarker.points.push_back(p_max_2);

        trajMarker.points.push_back(p_max_2);
        trajMarker.points.push_back(p_max_3);

        trajMarker.points.push_back(p_max_3);
        trajMarker.points.push_back(p_max);

        p_min_1.x = p_min.x;
        p_min_1.y = p_max.y;
        p_min_1.z = p_min.z;

        p_min_2.x = p_max.x;
        p_min_2.y = p_max.y;
        p_min_2.z = p_min.z;

        p_min_3.x = p_max.x;
        p_min_3.y = p_min.y; 
        p_min_3.z = p_min.z;

        trajMarker.points.push_back(p_min);
        trajMarker.points.push_back(p_min_1);

        trajMarker.points.push_back(p_min_1);
        trajMarker.points.push_back(p_min_2);

        trajMarker.points.push_back(p_min_2);
        trajMarker.points.push_back(p_min_3);

        trajMarker.points.push_back(p_min_3);
        trajMarker.points.push_back(p_min);



        trajMarker.points.push_back(p_max);
        trajMarker.points.push_back(p_min_2);

        trajMarker.points.push_back(p_max_1);
        trajMarker.points.push_back(p_min_3);

        trajMarker.points.push_back(p_max_2);
        trajMarker.points.push_back(p_min);

        trajMarker.points.push_back(p_max_3);
        trajMarker.points.push_back(p_min_1);
        test_boundary_pub_.publish(trajMarker);
    }
#endif // DEBUG
}

// 计算占用概率
void Map::occupanyProb()
{
    mp_.raycast_num_++;
    // Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
    Eigen::Vector3d pt_w, ray_pt;
    Eigen::Vector3i id, id_global;
    int voxel_idx;
    double length = 0;
    bool occu_sign = false;

    Eigen::Vector3d boundary_max(mp_.new_map_min_boundary_);
    Eigen::Vector3d boundary_min(mp_.new_map_max_boundary_);
    RayCaster raycaster;

#ifdef _MAPDEBUG_
    pcl::PointCloud<pcl::PointXYZ>().swap(voxel_cloud_camera_test_);
    pcl::PointCloud<pcl::PointXYZ>().swap(outside_cloud_test_);
    pcl::PointCloud<pcl::PointXYZ>().swap(cloud_in_map_occu_test_);
    pcl::PointCloud<pcl::PointXYZ>().swap(cloud_in_map_free_test_);
    pcl::PointCloud<pcl::PointXYZ>().swap(raycast_test_);
    pcl::PointXYZ p_c, p_f, p_r, p_v;
#endif // _MAPDEBUG_

    for(uint32_t i = 0; i < cam_.proj_points_cnt_; ++i)
    {
        pt_w = cam_.proj_points_.at(i);
        occu_sign = false;
        // 不在地图内部
        if(!isInMap(pt_w))
        {
            // 寻找该点与地图边界的交点
            pt_w = rayMapBound(pt_w, cam_.camera_position_);
            // 判断是否在扫描范围内
            length = (pt_w - cam_.camera_position_).norm();
            if(length > cam_.max_ray_length_)
            {

#ifdef _MAPDEBUG_
                p_c.x = pt_w.x();
                p_c.y = pt_w.y();
                p_c.z = pt_w.z();
                outside_cloud_test_.push_back(p_c);

#endif // _MAPDEBUG_
                pt_w = (pt_w - cam_.camera_position_) / length * cam_.max_ray_length_ + cam_.camera_position_;
            }

// #ifdef _MAPDEBUG_
//             p_c.x = pt_w.x();
//             p_c.y = pt_w.y();
//             p_c.z = pt_w.z();
//             outside_cloud_test_.push_back(p_c);
// #endif // _MAPDEBUG_

            // 设置为未被占用
            occu_sign = false;
        }
        else    // 在地图内部
        {
            length = (pt_w - cam_.camera_position_).norm();
            if(length > cam_.max_ray_length_)
            {
                pt_w = (pt_w - cam_.camera_position_) / length * cam_.max_ray_length_ + cam_.camera_position_;
                // 设置为未被占用
                occu_sign = false;
            }
            else
            {
                // 设置为被占用
                occu_sign = true;
            }
#ifdef _MAPDEBUG_
            if(occu_sign && test_cloud_in_map_occu_pub_.getNumSubscribers() > 0)
            {
                p_c.x = pt_w.x();
                p_c.y = pt_w.y();
                p_c.z = pt_w.z();
                cloud_in_map_occu_test_.push_back(p_c);
            }
            else if(!occu_sign && test_cloud_in_map_free_pub_.getNumSubscribers() > 0)
            {
                p_f.x = pt_w.x();
                p_f.y = pt_w.y();
                p_f.z = pt_w.z();
                cloud_in_map_free_test_.push_back(p_f);
            }
#endif // _MAPDEBUG_
            
        }
        // 计算点云的范围
        boundary_max = maxVector(pt_w, boundary_max);
        boundary_min = minVector(pt_w, boundary_min);

        // 判断先前是否被访问过
        voxel_idx = toAddress(pt_w);
        if(occ_map_.flag_rayend_.at(voxel_idx) == mp_.raycast_num_)
            continue;
        else
            occ_map_.flag_rayend_.at(voxel_idx) = mp_.raycast_num_;
            
        // 转换成索引
        pos2Index_global(pt_w, id_global);
        pos2Index(pt_w, id);
        PointBound(id);
        if(occu_sign)   // 占用
            setOccupany(id, CUADC::VoxelState::OCCUPANCY);
        else    // 未被占用
            setOccupany(id, CUADC::VoxelState::FREE);

        Eigen::Vector3i pt_id, cam_id;
        pos2Index_global(pt_w, pt_id);
        pos2Index_global(cam_.camera_position_, cam_id);
        getRayIndices(cam_id, pt_id, cam_.ray_);
        // Raycast(Eigen::Vector3d(1.0 * pt_id.x(), 1.0 * pt_id.y(), 1.0 * pt_id.z()), 
        //         Eigen::Vector3d(1.0 * cam_id.x(), 1.0 * cam_id.y(), 1.0 * cam_id.z()), 
        //         mp_.new_map_min_boundary_  / mp_.resolution_, mp_.new_map_max_boundary_ / mp_.resolution_, cam_.ray_point_set_);
        // raycast(Eigen::Vector3d(1.0 * cam_id.x(), 1.0 * cam_id.y(), 1.0 * cam_id.z()), 
        //         Eigen::Vector3d(1.0 * pt_id.x(), 1.0 * pt_id.y(), 1.0 * pt_id.z()), cam_.ray_point_set_);
        
#ifdef _MAPDEBUG_
        // if(occu_sign)
        // {
            p_v.x =  (1.0 * pt_id.x() + 0.5) * mp_.resolution_;
            p_v.y =  (1.0 * pt_id.y() + 0.5) * mp_.resolution_;
            p_v.z =  (1.0 * pt_id.z() + 0.5) * mp_.resolution_;
            voxel_cloud_camera_test_.push_back(p_v);
        // }
#endif // _MAPDEBUG_
        Eigen::Vector3d p;
        Eigen::Vector3i id_tmp;
        for(size_t j = 0; j < cam_.ray_.size(); ++j)
        {   
            // p = cam_.ray_point_set_.at(j);
            // Eigen::Vector3d tmp = (p + half) * mp_.resolution_;
            Eigen::Vector3d tmp;
            index2Pos_global(cam_.ray_.at(j), tmp);
            // pos2Index_global(tmp, id_tmp);
            id_tmp = cam_.ray_.at(j);
            if(id_global == id_tmp || cam_id == id_tmp)
                continue;

            voxel_idx = toAddress(tmp);
            if(occ_map_.flag_traverse_.at(voxel_idx) == mp_.raycast_num_ || occ_map_.flag_rayend_.at(voxel_idx) == mp_.raycast_num_)
                continue;
            else
                occ_map_.flag_traverse_.at(voxel_idx) = mp_.raycast_num_;

#ifdef _MAPDEBUG_
            index2Pos_global(id_tmp, tmp);
            p_r.x = tmp.x();
            p_r.y = tmp.y();
            p_r.z = tmp.z();
            raycast_test_.push_back(p_r);
#endif // _MAPDEBUG_

            setOccupany(tmp, VoxelState::FREE);
        }
    }

#ifdef _MAPDEBUG_
    outside_cloud_test_.width = outside_cloud_test_.points.size();
    outside_cloud_test_.height = 1;
    outside_cloud_test_.is_dense = true;
    outside_cloud_test_.header.frame_id = "map";
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(outside_cloud_test_, cloud_msg);
    if(test_outside_map_cloud_pub_.getNumSubscribers() > 0)
        test_outside_map_cloud_pub_.publish(cloud_msg);
    
    cloud_in_map_occu_test_.width = cloud_in_map_occu_test_.points.size();
    cloud_in_map_occu_test_.height = 1;
    cloud_in_map_occu_test_.is_dense = true;
    cloud_in_map_occu_test_.header.frame_id = "map";
    sensor_msgs::PointCloud2 occu_cloud_msg;
    pcl::toROSMsg(cloud_in_map_occu_test_, occu_cloud_msg);
    if(test_cloud_in_map_occu_pub_.getNumSubscribers() > 0)
        test_cloud_in_map_occu_pub_.publish(occu_cloud_msg);
    
    cloud_in_map_free_test_.width = cloud_in_map_free_test_.points.size();
    cloud_in_map_free_test_.height = 1;
    cloud_in_map_free_test_.is_dense = true;
    cloud_in_map_free_test_.header.frame_id = "map";
    sensor_msgs::PointCloud2 free_cloud_msg;
    pcl::toROSMsg(cloud_in_map_free_test_, free_cloud_msg);
    if(test_cloud_in_map_free_pub_.getNumSubscribers() > 0)
        test_cloud_in_map_free_pub_.publish(free_cloud_msg);

    raycast_test_.width = raycast_test_.points.size();
    raycast_test_.height = 1;
    raycast_test_.is_dense = true;
    raycast_test_.header.frame_id = "map";
    sensor_msgs::PointCloud2 raycast_cloud_msg;
    pcl::toROSMsg(raycast_test_, raycast_cloud_msg);
    if(test_raycast_pub_.getNumSubscribers() > 0)
        test_raycast_pub_.publish(raycast_cloud_msg);

    voxel_cloud_camera_test_.width = voxel_cloud_camera_test_.points.size();
    voxel_cloud_camera_test_.height = 1;
    voxel_cloud_camera_test_.is_dense = true;
    voxel_cloud_camera_test_.header.frame_id = "map";
    sensor_msgs::PointCloud2 camera_voxel_msg;
    pcl::toROSMsg(voxel_cloud_camera_test_, camera_voxel_msg);
    if(test_camera_cloud_voxel_pub_.getNumSubscribers() > 0)
        test_camera_cloud_voxel_pub_.publish(camera_voxel_msg);
#endif // _MAPDEBUG_

    boundary_max = maxVector(cam_.camera_position_, boundary_max);
    boundary_min = minVector(cam_.camera_position_, boundary_min);

    pos2Index(boundary_max, mp_.local_bound_max_);
    pos2Index(boundary_min, mp_.local_bound_min_);
    PointBound(mp_.local_bound_max_);
    PointBound(mp_.local_bound_min_);
    mp_.old_map_max_boundary_ = mp_.new_map_max_boundary_;
    mp_.old_map_min_boundary_ = mp_.new_map_min_boundary_;
    mp_.local_updated_ = true;
}

Eigen::Vector3d Map::rayMapBound(const Eigen::Vector3d &pt, const Eigen::Vector3d &cam_pos)
{
    Eigen::Vector3d ray_dir = (pt - cam_pos).normalized();
    Eigen::Vector3d max_ray = mp_.new_map_max_boundary_ - cam_pos;
    Eigen::Vector3d min_ray = mp_.new_map_min_boundary_ - cam_pos;
    double min_t = DBL_MAX;
    double t_mp;
    for(size_t i = 0; i < 3; ++i)
    {
        if(std::fabs(ray_dir[i]) > 0)
        {
            t_mp = max_ray(i) / ray_dir(i);
            if(t_mp > 0 && min_t > t_mp)
                min_t = t_mp;

            t_mp = min_ray(i) / ray_dir(i);
            if(t_mp > 0 && min_t > t_mp)
                min_t = t_mp;
        }
    }
    return cam_pos + min_t * ray_dir;
}


void Map::raycast(const Eigen::Vector3d &start, const Eigen::Vector3d &end, std::vector<Eigen::Vector3d> &ray)
{  
    std::vector<Eigen::Vector3d>().swap(ray);
    Eigen::Vector3d direct = (end - start).normalized();
    Eigen::Vector3d ray_point;
    double distance = (end - start).norm();
    int piece_num = floor(distance / (mp_.resolution_ * 9.0));
    for(int i = 0; i < piece_num; ++i)
    {
        // if((ray_point - end).norm() < 0.01)
        //     break;
        ray_point = i * mp_.resolution_ * 9.0 * direct + start;
        ray.push_back(ray_point);
    }
}

void Map::InflateLocalMap()
{
    // auto start_time = std::chrono::high_resolution_clock::now();

    Eigen::Vector3i id;

    // 清除膨胀数据 - 并行化
    // #pragma omp parallel for collapse(3) private(id)
    for(int x = 0; x <= mp_.map_voxel_num_(0); ++x)
        for(int y = 0; y <= mp_.map_voxel_num_(1); ++y)
            for(int z = 0; z <= mp_.map_voxel_num_(2); ++z)
            {
                id << x, y, z;
                PointBound(id);
                occ_map_.occupany_map_inflate_.at(toAddress(id)).state = VoxelState::FREE;
            }

    // 膨胀体素（Ray Cast）- 并行化
    // #pragma omp parallel for collapse(3) private(id)
    for(int x = mp_.local_bound_min_(0); x <= mp_.local_bound_max_(0); ++x)
        for(int y = mp_.local_bound_min_(1); y <= mp_.local_bound_max_(1); ++y)
            for(int z = mp_.local_bound_min_(2); z <= mp_.local_bound_max_(2); ++z)
            {
                id << x, y, z;
                PointBound(id);
                if(occ_map_.occupany_map_.at(toAddress(id)) >= mp_.prob_max_logit_)
                    inflatePoint(id);
            }

    // 膨胀交集区域 - 并行化
    // #pragma omp parallel for collapse(3) private(id)
    for(int x = mp_.intersection_id_min_.x(); x <= mp_.intersection_id_max_.x(); ++x)
        for(int y = mp_.intersection_id_min_.y(); y <= mp_.intersection_id_max_.y(); ++y)
            for(int z = mp_.intersection_id_min_.z(); z <= mp_.intersection_id_max_.z(); ++z)
            {
                id << x, y, z;
                PointBound(id);
                if(occ_map_.occupany_map_.at(toAddress(id)) >= mp_.prob_max_logit_)
                    inflatePoint(id);
            }

    // auto end_time = std::chrono::high_resolution_clock::now();
    // auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
    // ROS_INFO_STREAM("InflateLocalMap execution time: " << duration_ns / 1000000.0 << "ms");
}


// 更新地图
void Map::updateOccupancyCallback(const ros::TimerEvent& /*event*/)
{
    if(!mp_.occ_need_update_ && !cam_.have_last_cam_position_)
        return;
    
    // 进行投影
    if(cam_.use_vision_)
        projectDepthImage();

    // 计算占用概率
    if(cam_.proj_points_cnt_)
        occupanyProb();
    
    if(mp_.local_updated_)
        InflateLocalMap();

    mp_.occ_need_update_ = false;
    mp_.local_updated_ = false;
}

void Map::visualCallback(const ros::TimerEvent& )
{
    publishMap();

    publishInflateMap();
}

void Map::publishMap()
{
    // auto start_time = std::chrono::high_resolution_clock::now();
    if(map_pub_.getNumSubscribers() <= 0)
        return;
    pcl::PointXYZ pt;
    
    pcl::PointCloud<pcl::PointXYZ>empty_cloud;
    empty_cloud.reserve(mp_.map_voxel_num_.x() * mp_.map_voxel_num_.y() * mp_.map_voxel_num_.z());
    empty_cloud.swap(cloud_);

    Eigen::Vector3i id;
    Eigen::Vector3d pos;
    for(int x = 0; x < mp_.map_voxel_num_.x(); ++x)
        for(int y = 0; y < mp_.map_voxel_num_.y(); ++y)
            for(int z = 0; z < mp_.map_voxel_num_.z(); ++z)
            {
                id << x, y, z;
                PointBound(id);
                if(occ_map_.occupany_map_.at(toAddress(id)) >= mp_.prob_max_logit_)
                {
                    // ROS_INFO_STREAM("publishMap loop");
                    index2Pos(id, pos);
                    pt.x = pos.x();
                    pt.y = pos.y();
                    pt.z = pos.z();
                    cloud_.push_back(pt);
                }
            }

    cloud_.width = cloud_.points.size();
    cloud_.height = 1;
    cloud_.is_dense = true;
    cloud_.header.frame_id = mp_.frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud_, cloud_msg);
    map_pub_.publish(cloud_msg);

    // auto end_time = std::chrono::high_resolution_clock::now();
    // auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
    // ROS_INFO_STREAM("publishMap execution time: " << duration_ns / 1000000.0 << " ms");
}

void Map::publishInflateMap()
{
    // auto start_time = std::chrono::high_resolution_clock::now();
    if(map_inf_pub_.getNumSubscribers() <= 0)
        return;
    
    pcl::PointCloud<pcl::PointXYZ>empty_cloud_inflate;
    empty_cloud_inflate.reserve(mp_.map_voxel_num_.x() * mp_.map_voxel_num_.y() * mp_.map_voxel_num_.z());
    empty_cloud_inflate.swap(cloud_inflate_);

    pcl::PointXYZ pt;

    Eigen::Vector3i min_idx = mp_.local_bound_min_;
    Eigen::Vector3i max_idx = mp_.local_bound_max_;
    Eigen::Vector3i id;
    Eigen::Vector3d pos;

    if(mp_.show_local_map_)
    {
        for(int x = min_idx(0); x <= max_idx(0); ++x)
            for(int y = min_idx(1); y <= max_idx(1); ++y)
                for(int z = min_idx(2); z <= max_idx(2); ++z)
                {
                    
                    id << x, y, z;
                    PointBound(id);
                    if(VoxelState::FREE == occ_map_.occupany_map_inflate_.at(toAddress(id)).state)
                        continue;
                    index2Pos(id, pos);
                    pt.x = pos.x();
                    pt.y = pos.y();
                    pt.z = pos.z();
                    cloud_inflate_.push_back(pt);
                }
    }
    else
    {
        for(int x = 0; x < mp_.map_voxel_num_.x(); ++x)
            for(int y = 0; y < mp_.map_voxel_num_.y(); ++y)
                for(int z = 0; z < mp_.map_voxel_num_.z(); ++z)
                {
                    id << x, y, z;
                    PointBound(id);
                    if(VoxelState::FREE == occ_map_.occupany_map_inflate_.at(toAddress(id)).state)
                        continue;
                    index2Pos(id, pos);
                    pt.x = pos.x();
                    pt.y = pos.y();
                    pt.z = pos.z();
                    cloud_inflate_.push_back(pt);
                }
    }

    cloud_inflate_.width = cloud_inflate_.points.size();
    cloud_inflate_.height = 1;
    cloud_inflate_.is_dense = true;
    cloud_inflate_.header.frame_id = mp_.frame_id_;
    sensor_msgs::PointCloud2 cloud_msg_inflate;

    pcl::toROSMsg(cloud_inflate_, cloud_msg_inflate);
    map_inf_pub_.publish(cloud_msg_inflate);
    // // 记录结束时间
    // auto end_time = std::chrono::high_resolution_clock::now();

    // // 计算并打印时间
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    // ROS_INFO_STREAM("publishMap execution time: " << duration << "ms");
}

void Map::depthPoseCallback(const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose)
{
    // ROS_INFO("depthPoseCallback");
    depthfunc(img, pose->pose);
}
void Map::depthOdomCallback(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom)
{
    // ROS_INFO("depthOdomCallback");
    depthfunc(img, odom->pose.pose);
}

void Map::depthfunc(const sensor_msgs::ImageConstPtr& img, const geometry_msgs::Pose& pose)
{
    Eigen::Quaterniond q_wb = Eigen::Quaterniond(pose.orientation.w, 
                                                 pose.orientation.x, 
                                                 pose.orientation.y, 
                                                 pose.orientation.z);

    Eigen::Matrix3d body_r_m = q_wb.toRotationMatrix();
    Eigen::Matrix4d body2world;
    body2world.block<3, 3>(0, 0) = body_r_m;
    body2world(0, 3) = pose.position.x;
    body2world(1, 3) = pose.position.y;
    body2world(2, 3) = pose.position.z;
    body2world(3, 3) = 1.0;

    Eigen::Matrix4d cam_T = body2world * cam_.cam2body_;
    if(cam_.last_cam_position_update_sign_)
    {
        cam_.last_camera_position_ = cam_.camera_position_;
        cam_.last_camera_q_ = cam_.camera_q_;
    }
    cam_.camera_position_(0) = cam_T(0, 3);
    cam_.camera_position_(1) = cam_T(1, 3);
    cam_.camera_position_(2) = cam_T(2, 3);
    cam_.camera_q_ = Eigen::Quaterniond(cam_T.block<3, 3>(0, 0));

    /* 获取深度图 */
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
    if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
        (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, cam_.k_depth_scaling_factor_);
    }
    cv_ptr->image.copyTo(cam_.depth_image_);
    mp_.occ_need_update_ = true;
}

void Map::scanPoseCallback(const sensor_msgs::LaserScanConstPtr& point_cloud, const geometry_msgs::PoseStampedConstPtr& pose)
{
    // ROS_INFO("scanPoseCallback");
    scanfunc(point_cloud, pose->pose);
}

void Map::scanOdomCallback(const sensor_msgs::LaserScanConstPtr& point_cloud, const nav_msgs::OdometryConstPtr& odom)
{
    // ROS_INFO("scanOdomCallback");
    scanfunc(point_cloud, odom->pose.pose);
}

void Map::scanfunc(const sensor_msgs::LaserScanConstPtr& point_cloud, const geometry_msgs::Pose& pose)
{
    Eigen::Vector3d update_local_boundary_max(mp_.new_map_min_boundary_);
    Eigen::Vector3d update_local_boundary_min(mp_.new_map_max_boundary_);
    
    Eigen::Quaterniond q_wb = Eigen::Quaterniond(pose.orientation.w, 
                                                 pose.orientation.x, 
                                                 pose.orientation.y, 
                                                 pose.orientation.z);

    Eigen::Matrix3d body_r_m = q_wb.toRotationMatrix();
    Eigen::Matrix4d body2world;
    body2world.block<3, 3>(0, 0) = body_r_m;
    body2world(0, 3) = pose.position.x;
    body2world(1, 3) = pose.position.y;
    body2world(2, 3) = pose.position.z;
    body2world(3, 3) = 1.0;

    Eigen::Matrix4d cam_T = body2world * cam_.cam2body_;
    if(cam_.last_cam_position_update_sign_)
    {
        cam_.last_camera_position_ = cam_.camera_position_;
        cam_.last_camera_q_ = cam_.camera_q_;
    }
    cam_.camera_position_(0) = cam_T(0, 3);
    cam_.camera_position_(1) = cam_T(1, 3);
    cam_.camera_position_(2) = cam_T(2, 3);
    cam_.last_cam_position_update_sign_ = true;
    cam_.camera_q_ = Eigen::Quaterniond(cam_T.block<3, 3>(0, 0));
    
    cam_.proj_points_cnt_ = 0;

    float angle_min = point_cloud->angle_min;     // 起始角度
    float angle_increment = point_cloud->angle_increment; // 每次角度增量
    const std::vector<float>& ranges = point_cloud->ranges; // 激光测量距离

    Eigen::Vector3d p_tmp;
    float angle, range;
    for (size_t i = 0; i < ranges.size(); ++i)
    {
        range = ranges[i];
        bool sign = false;
        if(!std::isfinite(range) || range > point_cloud->range_max)
            range = cam_.max_ray_length_ + 0.2;
        else if(range < point_cloud->range_min)
            range = point_cloud->range_min;
        else
        {
            sign = true;
        }
        // 当前激光束的角度
        angle = angle_min + i * angle_increment;
        p_tmp(0) = range * cos(angle);
        p_tmp(1) = range * sin(angle);
        p_tmp(2) = 0.0;
        p_tmp = cam_.camera_q_ * p_tmp + cam_.camera_position_;
        if(cam_.use_2d_lidar_)
            p_tmp(2) = 0.0;
        cam_.proj_points_[cam_.proj_points_cnt_++] = p_tmp;
        if(sign)
        {
            update_local_boundary_max = maxVector(p_tmp, update_local_boundary_max);
            update_local_boundary_min = minVector(p_tmp, update_local_boundary_min);
        }
#ifdef _MAPDEBUG_
        if(test_scan_cloud_pub_.getNumSubscribers() > 0)
        {
            pcl::PointCloud<pcl::PointXYZ>().swap(scan_cloud_test_);
            pcl::PointXYZ pt;
            for(uint32_t i = 0; i < cam_.proj_points_cnt_; i += cam_.skip_pixel_)
            {
                pt.x = cam_.proj_points_.at(i).x();
                pt.y = cam_.proj_points_.at(i).y();;
                pt.z = cam_.proj_points_.at(i).z();;
                scan_cloud_test_.push_back(pt);
            }
            scan_cloud_test_.width = scan_cloud_test_.points.size();
            scan_cloud_test_.height = 1;
            scan_cloud_test_.is_dense = true;
            scan_cloud_test_.header.frame_id = mp_.frame_id_;
            sensor_msgs::PointCloud2 scan_cloud_msg;

            pcl::toROSMsg(scan_cloud_test_, scan_cloud_msg);
            test_scan_cloud_pub_.publish(scan_cloud_msg);
        }
#endif // _MAPDEBUG_
    }
    update_local_boundary_max = maxVector(cam_.camera_position_, update_local_boundary_max);
    update_local_boundary_min = minVector(cam_.camera_position_, update_local_boundary_min);
    if(cam_.proj_points_cnt_)
    {
        if(MapBoundary(update_local_boundary_max, update_local_boundary_min, cam_.camera_position_, mp_.map_size_, mp_.old_map_max_boundary_, mp_.old_map_min_boundary_, 
                    mp_.new_map_max_boundary_, mp_.new_map_min_boundary_))
            moveOld2New(mp_.old_map_max_boundary_, mp_.old_map_min_boundary_, 
                        mp_.new_map_max_boundary_, mp_.new_map_min_boundary_,
                        mp_.intersection_id_max_, mp_.intersection_id_min_);
    }
    mp_.occ_need_update_ = true;
#ifdef _MAPDEBUG_
    if(test_boundary_pub_.getNumSubscribers() > 0)
    {
        visualization_msgs::Marker routeMarker, trajMarker;
        routeMarker.id = 0;
        routeMarker.type = visualization_msgs::Marker::LINE_LIST;
        routeMarker.header.stamp = ros::Time::now();
        routeMarker.header.frame_id = "map";
        routeMarker.pose.orientation.w = 1.00;
        routeMarker.action = visualization_msgs::Marker::ADD;
        routeMarker.ns = "route";
        routeMarker.color.r = 1.00;
        routeMarker.color.g = 0.00;
        routeMarker.color.b = 0.00;
        routeMarker.color.a = 1.00;
        routeMarker.scale.x = 0.01;

        trajMarker = routeMarker;
        trajMarker.header.frame_id = "map";
        trajMarker.id = 0;
        trajMarker.ns = "trajectory";
        trajMarker.color.r = 0.00;
        trajMarker.color.g = 0.50;
        trajMarker.color.b = 1.00;
        trajMarker.scale.x = 0.01;
        
        geometry_msgs::Point p_max, p_max_1, p_max_2, p_max_3;
        p_max.x = mp_.new_map_max_boundary_.x();
        p_max.y = mp_.new_map_max_boundary_.y();
        p_max.z = mp_.new_map_max_boundary_.z();

        geometry_msgs::Point p_min, p_min_1, p_min_2, p_min_3;
        p_min.x = mp_.new_map_min_boundary_.x();
        p_min.y = mp_.new_map_min_boundary_.y();
        p_min.z = mp_.new_map_min_boundary_.z();

        p_max_1.x = p_max.x;
        p_max_1.y = p_min.y;
        p_max_1.z = p_max.z;

        p_max_2.x = p_min.x;
        p_max_2.y = p_min.y;
        p_max_2.z = p_max.z;

        p_max_3.x = p_min.x;
        p_max_3.y = p_max.y;
        p_max_3.z = p_max.z;

        trajMarker.points.push_back(p_max);
        trajMarker.points.push_back(p_max_1);

        trajMarker.points.push_back(p_max_1);
        trajMarker.points.push_back(p_max_2);

        trajMarker.points.push_back(p_max_2);
        trajMarker.points.push_back(p_max_3);

        trajMarker.points.push_back(p_max_3);
        trajMarker.points.push_back(p_max);

        p_min_1.x = p_min.x;
        p_min_1.y = p_max.y;
        p_min_1.z = p_min.z;

        p_min_2.x = p_max.x;
        p_min_2.y = p_max.y;
        p_min_2.z = p_min.z;

        p_min_3.x = p_max.x;
        p_min_3.y = p_min.y; 
        p_min_3.z = p_min.z;

        trajMarker.points.push_back(p_min);
        trajMarker.points.push_back(p_min_1);

        trajMarker.points.push_back(p_min_1);
        trajMarker.points.push_back(p_min_2);

        trajMarker.points.push_back(p_min_2);
        trajMarker.points.push_back(p_min_3);

        trajMarker.points.push_back(p_min_3);
        trajMarker.points.push_back(p_min);



        trajMarker.points.push_back(p_max);
        trajMarker.points.push_back(p_min_2);

        trajMarker.points.push_back(p_max_1);
        trajMarker.points.push_back(p_min_3);

        trajMarker.points.push_back(p_max_2);
        trajMarker.points.push_back(p_min);

        trajMarker.points.push_back(p_max_3);
        trajMarker.points.push_back(p_min_1);
        test_boundary_pub_.publish(trajMarker);
    }
#endif
}

} // namspace CUADC