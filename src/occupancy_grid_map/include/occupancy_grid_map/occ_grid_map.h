#pragma once

#include <vector>
#include <string>
#include <queue>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

// 引入OpenMP并行计算库
#include <omp.h>

#include <chrono>

#define _MAPDEBUG_

namespace CUADC{

enum VoxelState{
    FREE=0u,
    OCCUPANCY=1u
};

struct InflateVoxel
{
    VoxelState state = VoxelState::FREE;
    // Eigen::Vector3i camFrom = Eigen::Vector3i::Zero();
    InflateVoxel(VoxelState s):state(s){};
};

double logit(double x)
{
    return log(x / (1 - x));
}

class Map;
using MapPtr = std::shared_ptr<Map>;
// typedef Map* MapPtr;

struct Camera
{
    int cam_depth_width_,cam_depth_height_;
    double fx_, fy_, cx_, cy_;

    double depth_maxdist_, depth_minidist_;
    double max_ray_length_, min_ray_length_;

    int depth_filter_margin_;
    int skip_pixel_;
    
    double k_depth_scaling_factor_;
    
    bool use_pose_, use_vision_, use_2d_lidar_;
    bool last_cam_position_update_sign_;
    bool have_last_cam_position_;
    Eigen::Vector3d camera_position_, last_camera_position_;        // 相机位置
    Eigen::Quaterniond camera_q_, last_camera_q_;        // 相机姿态
    cv::Mat depth_image_;        // 深度图
    
    Eigen::Matrix4d cam2body_;

    std::vector<Eigen::Vector3d> proj_points_;
    std::vector<Eigen::Vector3d> ray_point_set_;
    std::vector<Eigen::Vector3i> ray_;
    uint32_t proj_points_cnt_;
};

struct MapParamters
{
    Eigen::Vector3d map_origin_, map_size_; // 地图原点，地图大小
    Eigen::Vector3i map_origin_id_; // 地图原点索引(global)
    Eigen::Vector3d new_map_min_boundary_, new_map_max_boundary_;   // 当前时刻地图边界
    Eigen::Vector3d old_map_min_boundary_, old_map_max_boundary_;   // 上一时刻地图边界
    Eigen::Vector3i intersection_id_max_, intersection_id_min_; // 两帧地图之间的相交索引
    Eigen::Vector3i local_bound_min_, local_bound_max_;
    Eigen::Vector3i map_voxel_num_;     // 体素数量
    double resolution_, resolution_inv_;    // 分辨率(0-1)
    double obstacles_inflation_;
    double ground_height_;

    std::string frame_id_;
    double p_occ_max_, p_occ_min_;      // 每个格子的最大概率 
    double prob_max_logit_, prob_min_logit_;  // 将每个格子的概率转换成logit

    double lofree_, looccu_;    // 每次观测后的增量
    bool occ_need_update_, local_updated_;
    bool show_local_map_, show_global_map_;

    // 每次扫描后每个栅格的标志位
    char raycast_num_;
};

struct OccupanyGridMap
{
    std::vector<double> occupany_map_;      // 原始栅格占用地图，用后验概率表示，当大于0.5时认为被占用
    std::vector<InflateVoxel> occupany_map_inflate_;     // 膨胀地图，1代表占用，0代表空闲
    std::vector<char> flag_rayend_;     // 用于判断终点在当前帧下是否被访问过
    std::vector<char> flag_traverse_;   // 用于判断射线经过的点在当前帧下是否被访问过
    std::queue<std::pair<Eigen::Vector3d, double>>  cache_voxel_; // 存放上一帧地图中的坐标与概率值
};

class Map
{
public:
    Map() {}
    ~Map() {}

    void initMap(ros::NodeHandle &nh);

    inline int toAddress(const Eigen::Vector3i &index);

    inline int toAddress(const Eigen::Vector3d &pos);

    inline void pos2Index(const Eigen::Vector3d &pos, Eigen::Vector3i &id);

    inline void pos2Index_global(const Eigen::Vector3d &pos, Eigen::Vector3i &id);

    inline void index2Pos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);

    inline void index2Pos_global(const Eigen::Vector3i& id, Eigen::Vector3d& pos);

    inline bool isInMap(const Eigen::Vector3d& pos);
    
    inline bool PointBound(Eigen::Vector3i& id);

    bool isOccupied(const Eigen::Vector3i& id){
        return occ_map_.occupany_map_inflate_.at(toAddress(id)).state;
    }

    bool isOccupied_global(const Eigen::Vector3d& pos_global){
        if(!isInMap(pos_global))    // 在地图外
            return VoxelState::FREE;
        else    // 在地图内
        {
            Eigen::Vector3i id;
            pos2Index(pos_global, id);
            PointBound(id);
            return occ_map_.occupany_map_inflate_.at(toAddress(id)).state;
        }
    }

    bool isOccupied_global(const Eigen::Vector3i& id_global){
        Eigen::Vector3d pos_global;
        index2Pos_global(id_global, pos_global);
        if(!isInMap(pos_global))    // 在地图外
            return VoxelState::FREE;
        else    // 在地图内
        {
            Eigen::Vector3i id;
            pos2Index(pos_global, id);
            PointBound(id);
            return occ_map_.occupany_map_inflate_.at(toAddress(id)).state;
        }
    }

    double getCamMaxRayLength(){return cam_.max_ray_length_;}

    void setInflateOccpancy(Eigen::Vector3i &id);

    void getRayIndices(const Eigen::Vector3i& start, const Eigen::Vector3i& end, std::vector<Eigen::Vector3i>& rayIndices);
private:

    void projectDepthImage();

    void occupanyProb();

    Eigen::Vector3d rayMapBound(const Eigen::Vector3d &pt, const Eigen::Vector3d &cam_pos);

    inline void setOccupany(const Eigen::Vector3d &pos, VoxelState state);

    inline void setOccupany(const Eigen::Vector3i &id, VoxelState state);

    inline void inflatePoint(const Eigen::Vector3i& pt);

    void raycast(const Eigen::Vector3d &start, const Eigen::Vector3d &end, std::vector<Eigen::Vector3d> &ray);

    void updateOccupancyCallback(const ros::TimerEvent& /*event*/);

    void InflateLocalMap();

    void visualCallback(const ros::TimerEvent& );

    void publishMap();

    void publishInflateMap();

    void depthPoseCallback(const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose);
    
    void depthOdomCallback(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom);

    void depthfunc(const sensor_msgs::ImageConstPtr& img, const geometry_msgs::Pose& pose);

    void scanPoseCallback(const sensor_msgs::LaserScanConstPtr& point_cloud, const geometry_msgs::PoseStampedConstPtr& pose);

    void scanOdomCallback(const sensor_msgs::LaserScanConstPtr& point_cloud, const nav_msgs::OdometryConstPtr& odom);

    void scanfunc(const sensor_msgs::LaserScanConstPtr& point_cloud, const geometry_msgs::Pose& pose);

    // 找到两个三维向量中元素最大的向量
    Eigen::Vector3d maxVector(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2);
    // 找到两个三维向量中元素最小的向量
    Eigen::Vector3d minVector(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2);
    // 计算两个地图之间的交集
    bool intersectionRect(const Eigen::Vector3d &new_left_top, const Eigen::Vector3d &new_right_bottom,
                          const Eigen::Vector3d &old_left_top, const Eigen::Vector3d &old_right_bottom,
                          std::pair<Eigen::Vector3d, Eigen::Vector3d> &res);
    // 将上一帧地图中的数据复制到新地图中
    void moveOld2New(const Eigen::Vector3d &old_map_max_boundary, const Eigen::Vector3d &old_map_min_boundary,
                     const Eigen::Vector3d &new_map_max_boundary, const Eigen::Vector3d &new_map_min_boundary,
                     Eigen::Vector3i &intersection_max_id_new, Eigen::Vector3i &intersection_min_id_new);
    // 求出地图边界
    bool MapBoundary(const Eigen::Vector3d &update_local_boundary_max, const Eigen::Vector3d &update_local_boundary_min,
                     const Eigen::Vector3d &cam_position, const Eigen::Vector3d &map_size, const Eigen::Vector3d &old_map_max_boundary, const Eigen::Vector3d &old_map_min_boundary,
                     Eigen::Vector3d &new_map_max_boundary, Eigen::Vector3d &new_map_min_boundary);
public:
    MapParamters mp_;
private:
    Camera cam_;
    OccupanyGridMap occ_map_;

    ros::NodeHandle nh_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped>
        SyncPolicyImagePose;
    typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
        SyncPolicyImageOdom;
    typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, geometry_msgs::PoseStamped>
        SyncPolicyScanPose;
    typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyScanPose>> SynchronizerScanPose;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry>
        SyncPolicyScanOdom;
    typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyScanOdom>> SynchronizerScanOdom;

    std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> scan_sub_;

    ros::Publisher map_pub_, map_inf_pub_;
    ros::Timer occ_timer_, vis_timer_;

#ifdef _MAPDEBUG_
    ros::Publisher test_camera_cloud_pub_, test_outside_map_cloud_pub_, test_boundary_pub_;
    ros::Publisher test_camera_cloud_voxel_pub_;
    ros::Publisher test_cloud_in_map_occu_pub_, test_cloud_in_map_free_pub_;
    ros::Publisher test_raycast_pub_;
    ros::Publisher test_scan_cloud_pub_;
#endif

    SynchronizerImagePose sync_image_pose_;
    SynchronizerImageOdom sync_image_odom_;
    SynchronizerScanPose sync_scan_pose_;
    SynchronizerScanOdom sync_scan_odom_;

    // point cloud
#ifdef _MAPDEBUG_
    pcl::PointCloud<pcl::PointXYZ> cloud_camera_test_;
    pcl::PointCloud<pcl::PointXYZ> voxel_cloud_camera_test_;
    pcl::PointCloud<pcl::PointXYZ> outside_cloud_test_;
    pcl::PointCloud<pcl::PointXYZ> cloud_in_map_occu_test_;
    pcl::PointCloud<pcl::PointXYZ> cloud_in_map_free_test_;
    pcl::PointCloud<pcl::PointXYZ> raycast_test_;
    pcl::PointCloud<pcl::PointXYZ> scan_cloud_test_;
#endif

    pcl::PointCloud<pcl::PointXYZ> cloud_;
    pcl::PointCloud<pcl::PointXYZ> cloud_inflate_;
};

inline int Map::toAddress(const Eigen::Vector3i &index){
    return mp_.map_voxel_num_(2) * mp_.map_voxel_num_(1) * index(0) + mp_.map_voxel_num_(2) * index(1) + index(2);
}

inline int Map::toAddress(const Eigen::Vector3d &pos){
    Eigen::Vector3i id;
    pos2Index(pos, id);
    PointBound(id);
    return toAddress(id);
}

inline void Map::pos2Index(const Eigen::Vector3d &pos, Eigen::Vector3i &id){
    pos2Index_global(pos, id);
    id = id - mp_.map_origin_id_;
}

inline void Map::pos2Index_global(const Eigen::Vector3d &pos, Eigen::Vector3i &id)
{
    for(int i = 0; i < 3; ++i) id(i) = floor(pos(i) * mp_.resolution_inv_);
}

inline void Map::index2Pos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
  Eigen::Vector3i global_id = id + mp_.map_origin_id_;
  index2Pos_global(global_id, pos);
//   for (int i = 0; i < 3; ++i) pos(i) = ((double)id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}

inline void Map::index2Pos_global(const Eigen::Vector3i& id, Eigen::Vector3d& pos)
{
    for(int i = 0; i < 3; ++i)  pos(i) = ((double)id(i) + 0.5) * mp_.resolution_;
}

inline bool Map::isInMap(const Eigen::Vector3d& pos) {
  if (pos(0) < mp_.new_map_min_boundary_(0) + 1e-4 || pos(1) < mp_.new_map_min_boundary_(1) + 1e-4 ||
      pos(2) < mp_.new_map_min_boundary_(2) + 1e-4) {
    // cout << "less than min range!" << endl;
    return false;
  }
  if (pos(0) > mp_.new_map_max_boundary_(0) - 1e-4 || pos(1) > mp_.new_map_max_boundary_(1) - 1e-4 ||
      pos(2) > mp_.new_map_max_boundary_(2) - 1e-4) {
    return false;
  }
  return true;
}

inline void Map::setOccupany(const Eigen::Vector3d &pos, VoxelState state)
{
    Eigen::Vector3i id;
    pos2Index(pos, id);
    PointBound(id);
    setOccupany(id, state);
}

inline void Map::setOccupany(const Eigen::Vector3i &id, VoxelState state)
{
    int address = toAddress(id);
    if(VoxelState::FREE == state)
    {
        occ_map_.occupany_map_.at(address) += mp_.lofree_;
        if(occ_map_.occupany_map_.at(address) <= mp_.prob_min_logit_)
        {    
            occ_map_.occupany_map_.at(address) = mp_.prob_min_logit_ - 0.5 * std::fabs(mp_.lofree_);
            // ROS_INFO("prob_min_logit_");
        }
    }
    else if(VoxelState::OCCUPANCY == state)
    {
        if(occ_map_.occupany_map_.at(address) >= mp_.prob_max_logit_){
            occ_map_.occupany_map_.at(address) += mp_.looccu_;
            if(occ_map_.occupany_map_.at(address) >= mp_.prob_max_logit_ + 3.0 * std::fabs(mp_.looccu_))
                occ_map_.occupany_map_.at(address) = mp_.prob_max_logit_ + 3.0 * std::fabs(mp_.looccu_);
        }
        else
            occ_map_.occupany_map_.at(address) += mp_.looccu_;
    }
}

inline void Map::inflatePoint(const Eigen::Vector3i& pt)
{
    Eigen::Vector3i id;
    int inf_step = ceil(mp_.obstacles_inflation_ * mp_.resolution_inv_);
    for (int x = -inf_step; x <= inf_step; ++x)
        for (int y = -inf_step; y <= inf_step; ++y)
            for (int z = -inf_step; z <= inf_step; ++z) {
                id << pt(0) + x, pt(1) + y, pt(2) + z;
                PointBound(id);
                occ_map_.occupany_map_inflate_.at(toAddress(id)).state = VoxelState::OCCUPANCY;
            }

}


inline bool Map::PointBound(Eigen::Vector3i& id)
{
    //  check x
    bool sign = true;
    if(id.x() >= mp_.map_voxel_num_.x())
    {
        id(0) = mp_.map_voxel_num_.x() - 1;
        sign = false;
    }
    else if(id.x() < 0)
    {
        id(0) = 0;
        sign = false;
    }
    //  check y
    if(id.y() >= mp_.map_voxel_num_.y())
    {
        id(1) = mp_.map_voxel_num_.y() - 1;
        sign = false;
    }
    else if(id.y() < 0)
    {
        id(1) = 0;
        sign = false;
    }
    //  check z
    if(id.z() >= mp_.map_voxel_num_.z())
    {    
        id(2) = mp_.map_voxel_num_.z() - 1;
        sign = false;
    }
    else if(id.z() < 0)
    {
        id(2) = 0;
        sign = false;
    }
    return sign;
}

void Map::setInflateOccpancy(Eigen::Vector3i &id)
{
    PointBound(id);
    occ_map_.occupany_map_inflate_.at(toAddress(id)).state = VoxelState::OCCUPANCY;
}

void Map::getRayIndices(const Eigen::Vector3i& start, const Eigen::Vector3i& end, std::vector<Eigen::Vector3i>& rayIndices) {
    rayIndices.clear(); // 清空结果容器
    Eigen::Vector3i delta = end - start;    // 计算起点到终点的差值向量
    Eigen::Vector3i step = delta.cwiseSign(); // 计算步进方向
    delta = delta.cwiseAbs();               // 取差值的绝对值

    // 计算最大变化量
    int maxDelta = std::max({delta.x(), delta.y(), delta.z()});

    // 初始化误差累积向量
    Eigen::Vector3i t(delta.x() * 2, delta.y() * 2, delta.z() * 2);

    // 初始化当前点为起点
    Eigen::Vector3i current = start;
    rayIndices.push_back(current); // 将起点加入结果

    // 遍历从起点到终点的路径
    for (int i = 0; i < maxDelta; ++i) {
        // Bresenham 步进逻辑
        if (t.x() >= maxDelta) {
            current.x() += step.x();
            t.x() -= 2 * maxDelta;
        }
        if (t.y() >= maxDelta) {
            current.y() += step.y();
            t.y() -= 2 * maxDelta;
        }
        if (t.z() >= maxDelta) {
            current.z() += step.z();
            t.z() -= 2 * maxDelta;
        }

        // 更新误差累积向量
        t.x() += delta.x() * 2;
        t.y() += delta.y() * 2;
        t.z() += delta.z() * 2;

        // 将当前点加入结果
        rayIndices.push_back(current);
    }
}

Eigen::Vector3d Map::maxVector(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2)
{
    double max_x, max_y, max_z;
    max_x = std::max(v1.x(), v2.x());    
    max_y = std::max(v1.y(), v2.y());    
    max_z = std::max(v1.z(), v2.z());
    return Eigen::Vector3d(max_x, max_y, max_z);    
}

Eigen::Vector3d Map::minVector(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2)
{
    double min_x, min_y, min_z;
    min_x = std::min(v1.x(), v2.x());    
    min_y = std::min(v1.y(), v2.y());    
    min_z = std::min(v1.z(), v2.z());
    return Eigen::Vector3d(min_x, min_y, min_z);
}

// 返回左上角与右下角的坐标
bool Map::intersectionRect(const Eigen::Vector3d &new_left_top, const Eigen::Vector3d &new_right_bottom,
                           const Eigen::Vector3d &old_left_top, const Eigen::Vector3d &old_right_bottom,
                           std::pair<Eigen::Vector3d, Eigen::Vector3d> &res)
{
    // check
    Eigen::Vector3d new_max = maxVector(new_left_top, new_right_bottom);
    Eigen::Vector3d new_min = minVector(new_left_top, new_right_bottom);
    Eigen::Vector3d old_max = maxVector(old_left_top, old_right_bottom);
    Eigen::Vector3d old_min = minVector(old_left_top, old_right_bottom);
    if(new_min.x() > old_max.x() || new_min.y() > old_max.y() || new_min.z() > old_max.z() || 
       new_max.x() < old_min.x() || new_max.y() < old_min.y() || new_max.z() < old_min.z())
        return false;

    res.first = minVector(new_max, old_max);
    res.second = maxVector(new_min, old_min);
    return true;
}

void Map::moveOld2New(const Eigen::Vector3d &old_map_max_boundary, const Eigen::Vector3d &old_map_min_boundary,
                      const Eigen::Vector3d &new_map_max_boundary, const Eigen::Vector3d &new_map_min_boundary,
                      Eigen::Vector3i &intersection_max_id_new, Eigen::Vector3i &intersection_min_id_new)
{
    // auto start_time = std::chrono::high_resolution_clock::now();
    Eigen::Vector3i intersection_max_id_old, intersection_min_id_old;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> intersection_rect;
    if(!intersectionRect(new_map_max_boundary, new_map_min_boundary, old_map_max_boundary, old_map_min_boundary, intersection_rect))
    {
        // #pragma omp parallel for collapse(3)
        for (int x = 0; x < mp_.map_voxel_num_.x(); ++x)
            for (int y = 0; y < mp_.map_voxel_num_.y(); ++y)
                for (int z = 0; z < mp_.map_voxel_num_.z(); ++z)
                {
                    Eigen::Vector3i id(x, y, z);
                    occ_map_.occupany_map_.at(toAddress(id)) = mp_.prob_min_logit_ - 0.5 * std::fabs(mp_.lofree_);
                }
        mp_.map_origin_ = new_map_min_boundary;
        pos2Index_global(mp_.map_origin_, mp_.map_origin_id_);
        index2Pos_global(mp_.map_origin_id_, mp_.map_origin_);

        // auto end_time = std::chrono::high_resolution_clock::now();
        // auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
        // ROS_INFO_STREAM("moveOld2New execution time: " << duration_ns << "ms");
        return;
    }  

    pos2Index_global(intersection_rect.first, intersection_max_id_old);
    pos2Index_global(intersection_rect.second, intersection_min_id_old);
    
    // Eigen::Vector3i map_origin_id;
    // pos2Index_global(mp_.map_origin_, map_origin_id);

    intersection_max_id_old = intersection_max_id_old - mp_.map_origin_id_;
    intersection_min_id_old = intersection_min_id_old - mp_.map_origin_id_;

    Eigen::Vector3i id_old;
    Eigen::Vector3d id_ptw;
    
    // clear the old occupancy probability and push the intersection Rect into cache 
    // #pragma omp parallel for collapse(3) private(id_old, id_ptw) schedule(dynamic)
    for (int x = 0; x < mp_.map_voxel_num_.x(); ++x)
        for (int y = 0; y < mp_.map_voxel_num_.y(); ++y)
            for (int z = 0; z < mp_.map_voxel_num_.z(); ++z)
            {
                id_old << x, y, z;
                if (id_old.x() <= intersection_max_id_old.x() && id_old.y() <= intersection_max_id_old.y() && id_old.z() <= intersection_max_id_old.z() &&
                    id_old.x() >= intersection_min_id_old.x() && id_old.y() >= intersection_min_id_old.y() && id_old.z() >= intersection_min_id_old.z())
                {
                    index2Pos(id_old, id_ptw);
                    double value = occ_map_.occupany_map_.at(toAddress(id_old));
                    
                    // #pragma omp critical
                    {
                        occ_map_.cache_voxel_.push(std::make_pair(id_ptw, value));
                    }
                }
                occ_map_.occupany_map_.at(toAddress(id_old)) = mp_.prob_min_logit_;
            }
    pos2Index_global(new_map_min_boundary, mp_.map_origin_id_);
    index2Pos_global(mp_.map_origin_id_, mp_.map_origin_);
    pos2Index_global(intersection_rect.first, intersection_max_id_new);
    pos2Index_global(intersection_rect.second, intersection_min_id_new);
    intersection_max_id_new = intersection_max_id_new - mp_.map_origin_id_;
    intersection_min_id_new = intersection_min_id_new - mp_.map_origin_id_;

    // ROS_INFO_STREAM("occ_map_.cache_voxel_ num is " << occ_map_.cache_voxel_.size());
    // ROS_INFO_STREAM("Block size is " << (intersection_max_id_old - intersection_min_id_old));
    // ROS_INFO_STREAM("New Block size is " << (intersection_max_id_new - intersection_min_id_new));
    // ROS_INFO_STREAM("intersection_max_id_old : \n" << intersection_max_id_old << '\n' <<
    //                 "intersection_min_id_old : \n" << intersection_min_id_old << '\n' <<
    //                 "intersection_max_id_new : \n" << intersection_max_id_new << '\n' <<
    //                 "intersection_min_id_new : \n" << intersection_min_id_new << '\n' <<
    //                 "old_map_max_boundary : \n" << old_map_max_boundary << '\n' <<
    //                 "old_map_min_boundary : \n" << old_map_min_boundary << '\n' <<
    //                 "new_map_max_boundary : \n" << new_map_max_boundary << '\n' <<
    //                 "new_map_min_boundary : \n" << new_map_min_boundary);
    std::pair<Eigen::Vector3d, double> new_map_intersection_tmp;
    while(!occ_map_.cache_voxel_.empty())
    {
        // ROS_ERROR("Copy");
        new_map_intersection_tmp = occ_map_.cache_voxel_.front();
        occ_map_.occupany_map_.at(toAddress(new_map_intersection_tmp.first)) = new_map_intersection_tmp.second;
        occ_map_.cache_voxel_.pop();
    }

    // auto end_time = std::chrono::high_resolution_clock::now();
    // auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
    // ROS_INFO_STREAM("moveOld2New execution time: " << duration_ns / 1000000.0 << "ms");
}

bool Map::MapBoundary(const Eigen::Vector3d &update_local_boundary_max, const Eigen::Vector3d &update_local_boundary_min,
                     const Eigen::Vector3d &cam_position, const Eigen::Vector3d &map_size, const Eigen::Vector3d &old_map_max_boundary, const Eigen::Vector3d &old_map_min_boundary,
                     Eigen::Vector3d &new_map_max_boundary, Eigen::Vector3d &new_map_min_boundary)
{
    Eigen::Vector3d max_bound = old_map_max_boundary;
    Eigen::Vector3d min_bound = old_map_min_boundary;
    
    double max_z = cam_position.z() + map_size.z() * 0.5;
    // if(update_local_boundary_max.z() > old_map_max_boundary.z())
    //     max_z = update_local_boundary_max.z();
    // else if(update_local_boundary_min.z() < old_map_min_boundary.z())
    //     max_z = update_local_boundary_min.z() + map_size.z();
    // else
    //     max_z = old_map_max_boundary.z();
    
    if(cam_.use_vision_)
    {
        if(update_local_boundary_max.x() > old_map_max_boundary.x() && update_local_boundary_max.y() > old_map_max_boundary.y())
        {
            // ROS_INFO("1");
            max_bound = update_local_boundary_max;
        }
        else if(update_local_boundary_min.x() < old_map_min_boundary.x() && update_local_boundary_max.y() > old_map_max_boundary.y())
        {
            // ROS_INFO("2");
            max_bound(0) = update_local_boundary_min.x() + map_size.x();
            max_bound(1) = update_local_boundary_max.y();
        }
        else if(update_local_boundary_min.x() < old_map_min_boundary.x() && update_local_boundary_min.y() < old_map_min_boundary.y())
        {
            // ROS_INFO("3");
            max_bound = update_local_boundary_min + map_size;
        }
        else if(update_local_boundary_max.x() > old_map_max_boundary.x() && update_local_boundary_min.y() < old_map_min_boundary.y())
        {
            // ROS_INFO("4");
            max_bound(0) = update_local_boundary_max.x();
            max_bound(1) = update_local_boundary_min.y() + map_size.y();
        }
        else if(update_local_boundary_max.x() > old_map_max_boundary.x())
        {
            // ROS_INFO("5");
            max_bound(0) = update_local_boundary_max.x();
            max_bound(1) = old_map_max_boundary.y();
        }
        else if(update_local_boundary_max.y() > old_map_max_boundary.y())
        {
            // ROS_INFO("6");
            max_bound(0) = old_map_max_boundary.x();
            max_bound(1) = update_local_boundary_max.y();
        }
        else if(update_local_boundary_min.x() < old_map_min_boundary.x())
        {
            // ROS_INFO("7");
            max_bound(0) = update_local_boundary_min.x() + map_size.x();
            max_bound(1) = old_map_min_boundary.y() + map_size.y();
        }
        else if(update_local_boundary_min.y() < old_map_min_boundary.y())
        {
            // ROS_INFO("8");
            max_bound(0) = old_map_min_boundary.x() + map_size.x();
            max_bound(1) = update_local_boundary_min.y() + map_size.y();
        }
    }
    else
        max_bound = cam_position + map_size / 2;

    max_bound(2) = max_z;
    min_bound = max_bound - map_size;
    // ROS_INFO_STREAM("new_map_max_boundary from \n" << new_map_max_boundary << " to \n" << max_bound);
    Eigen::Vector3i max_bound_idx, min_bound_idx, new_map_max_boundary_idx, new_map_min_boundary_idx;
    pos2Index_global(max_bound, max_bound_idx);
    pos2Index_global(min_bound, min_bound_idx);
    pos2Index_global(new_map_max_boundary, new_map_max_boundary_idx);
    pos2Index_global(new_map_min_boundary, new_map_min_boundary_idx);
    if(max_bound_idx == new_map_max_boundary_idx || min_bound_idx == new_map_min_boundary_idx)
    {
        // ROS_INFO_STREAM("max_bound_idx " << max_bound_idx);
        // ROS_INFO_STREAM("new_map_max_boundary_idx " << new_map_max_boundary_idx);
        // ROS_INFO_STREAM("min_bound_idx " << min_bound_idx);
        // ROS_INFO_STREAM("new_map_min_boundary_idx " << new_map_min_boundary_idx);
        // ROS_INFO_STREAM("update_local_boundary_max : \n" << update_local_boundary_max << '\n' <<
        //                 "old_map_max_boundary : \n" << old_map_max_boundary << '\n' << 
        //                 "update_local_boundary_max : \n" << update_local_boundary_max << '\n' <<
        //                 "old_map_min_boundary : \n" << old_map_min_boundary);
        
        // ROS_ERROR("false");
        return false;
    }

    new_map_max_boundary = max_bound;
    new_map_min_boundary = min_bound;

    return true;
}

}