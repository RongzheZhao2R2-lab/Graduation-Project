#pragma once

#include <ros/ros.h>

#include <Eigen/Eigen>
#include "sample.hpp"

#include "map_generate/map_generate.hpp"

#include "occupancy_grid_map/occ_grid_map.h"

#include "kdtree.hpp"

// #define RRT_DEBUG

class RRT;
using RRT_Ptr = std::shared_ptr<RRT>;

class RRT
{
public:
    RRT(){};
    ~RRT(){};
    // max_time 单位是ms
    void initSimRRT(const map_generatePtr &map_gen_ptr, const double max_time);
    void initRRT(const CUADC::MapPtr &map_ptr, const double max_time);
    bool RRTSearch(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &target_pt, std::vector<Eigen::Vector3d> &path, std::vector<Eigen::Vector3d> &sample);
    void sampleWholeTree(std::vector<Eigen::Vector3d> &sample_node_set, std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &edges_set);
    double getPathlength() const {return goal_node_->cost_from_start;};
private:
    Eigen::Vector3d generateRandom();
    Eigen::Vector3d generateRandomEllipse();
    Eigen::Vector3d Steer(const Eigen::Vector3d &x_rand, const Eigen::Vector3d &x_near, double step_size);
    void ChangeParent(SampleNodePtr &node, SampleNodePtr &parent, const double &cost_from_parent);
    bool collisionCheck(const Eigen::Vector3d &x_near, const Eigen::Vector3d &x_new);
    bool lineCollisionCheck(const Eigen::Vector3d &start, const Eigen::Vector3d &end);
    bool pointCollisionCheck(const Eigen::Vector3d &x);
    bool rayCast(const Eigen::Vector3i& start, const Eigen::Vector3i& end, const std::function<bool(const Eigen::Vector3i&)>& checkOccupied);
private:
    RandomSampler sampler_;
    SampleNodePtr root_;
    SampleNodePtr goal_node_;
    SingleKdTree kdtree_;
    bool sim_flag_;
    double max_search_time_;
    map_generatePtr map_generate_ptr_;
    CUADC::MapPtr map_ptr_;

#ifdef RRT_DEBUG
public:
    std::vector<Eigen::Vector3d> ray_cast_path;
    std::vector<Eigen::Vector3d> segmentPoint_;
#endif

};

void RRT::initSimRRT(const map_generatePtr &map_gen_ptr, const double max_time)
{
    map_generate_ptr_ = map_gen_ptr;
    max_search_time_ = max_time;
    sim_flag_ = true;
}
void RRT::initRRT(const CUADC::MapPtr &map_ptr, const double max_time)
{
    map_ptr_ = map_ptr;
    max_search_time_ = max_time;
    sim_flag_ = false;
}

bool RRT::RRTSearch(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &target_pt, std::vector<Eigen::Vector3d> &path, std::vector<Eigen::Vector3d> &sample)
{

#ifdef RRT_DEBUG
    std::vector<Eigen::Vector3d>().swap(ray_cast_path);
#endif // RRT_DEBUG 

    ros::Time start_time = ros::Time::now();

    std::vector<Eigen::Vector3d>().swap(path);
    std::vector<Eigen::Vector3d>().swap(sample);

    kdtree_.clear();

    root_ = std::make_shared<SampleNode>();
    root_->position = start_pt;
    root_->cost_from_start = 0.0;
    kdtree_.insert(root_);

    goal_node_ = std::make_shared<SampleNode>();
    goal_node_->position = target_pt;
    goal_node_->cost_from_start = DBL_MAX;

    SampleNodePtr x_near;
    SampleNodePtr x_new;

    Eigen::Vector3d sample_point;
    bool ret = false;
    while((ros::Time::now() - start_time).toNSec() / 1e6 < max_search_time_)
    {
        // 进行一次采样
        if(!ret)
            sample_point = generateRandom();
        else
            sample_point = generateRandomEllipse();

        // 进行最近邻查询
        x_near = kdtree_.nearestNeighbor(sample_point);
        if(x_near == nullptr)
        {
            ROS_ERROR("nearest query error");
            continue;
        }

        // 树进行生长
        x_new = std::make_shared<SampleNode>();
        x_new->position = Steer(sample_point, x_near->position, 1.0);
        
        // 碰撞检测
        if(collisionCheck(x_near->position, x_new->position))
            continue;

        std::vector<SampleNodePtr> near_set = kdtree_.radiusSearch(x_new->position, 6.0);

        SampleNodePtr min_node(x_near);
        bool modif = false;
        for(auto &neigh_node : near_set)
        {
            // 进行连接是否发生碰撞
            if(lineCollisionCheck(x_new->position, neigh_node->position))
                continue;
            else
                modif = true;
            if(min_node->cost_from_start + (x_new->position - min_node->position).norm() >
               neigh_node->cost_from_start + (x_new->position - neigh_node->position).norm())
                min_node = neigh_node;
        }
        if(!modif)
            continue;

        x_new->parent = min_node;
        x_new->cost_from_parent = (x_new->position - min_node->position).norm();
        x_new->cost_from_start = x_new->cost_from_parent + x_new->parent->cost_from_start;
        min_node->children.push_back(x_new);

        kdtree_.insert(x_new);

        sample.push_back(sample_point);

        double dist_to_goal = (target_pt - x_new->position).norm();
        if((x_new->position - target_pt).norm() < 6.0)
        {
            bool is_connect2goal = lineCollisionCheck(x_new->position, target_pt);
            bool is_better_path = goal_node_->cost_from_start > x_new->cost_from_start + dist_to_goal;
            if(!is_connect2goal && is_better_path)
            {
                ChangeParent(goal_node_, x_new, dist_to_goal);
                ret = true;
            }
        }

        // rewire
        double dist_curr_to_new;
        for(auto &curr_node : near_set)
        {
            double best_cost_before_rewire = goal_node_->cost_from_start;
            dist_curr_to_new = (x_new->position - curr_node->position).norm();
            if(curr_node->cost_from_start > x_new->cost_from_start + dist_curr_to_new && 
               !lineCollisionCheck(x_new->position, curr_node->position))
            {
                ChangeParent(curr_node, x_new, dist_curr_to_new);
            }
        }
    }
    // 回溯路径
    if(ret)
    {

#ifdef RRT_DEBUG
        std::vector<Eigen::Vector3d>().swap(ray_cast_path);
        std::vector<Eigen::Vector3d>().swap(segmentPoint_);
#endif // RRT_DEBUG 

        bool occ = false;
        SampleNodePtr node_ptr = goal_node_;
        while(node_ptr->parent != nullptr)
        {
            path.push_back(node_ptr->position);

#ifdef RRT_DEBUG
            occ = lineCollisionCheck(node_ptr->position, node_ptr->parent->position);
            ROS_INFO_STREAM("occ " << occ);
            if(occ)
            {
                segmentPoint_.push_back(node_ptr->position);
                segmentPoint_.push_back(node_ptr->parent->position);
            }
#endif // RRT_DEBUG

            node_ptr = node_ptr->parent;
        }
        path.push_back(root_->position);
    }
#ifdef RRT_DEBUG
    else
    {
        std::vector<Eigen::Vector3d>().swap(ray_cast_path);
        std::vector<Eigen::Vector3d>().swap(segmentPoint_);
    }
#endif // RRT_DEBUG
    return ret;
};

void RRT::sampleWholeTree(std::vector<Eigen::Vector3d> &sample_node_set, std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &edges_set)
{
    std::vector<Eigen::Vector3d>().swap(sample_node_set);
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>().swap(edges_set);
    SampleNodePtr node;
    std::queue<SampleNodePtr> Q;
    Q.push(root_);
    while(!Q.empty())
    {
        node = Q.front();
        Q.pop();
        for(const auto &leafptr : node->children)
        {
            sample_node_set.push_back(leafptr->position);
            edges_set.push_back(std::make_pair(node->position, leafptr->position));
            Q.push(leafptr);
        }
    }
}

Eigen::Vector3d RRT::generateRandom()
{
    const double goal_bias = 0.3; // 30%概率直接采样目标点
    if (sampler_.getRandom() < goal_bias) 
        return goal_node_->position;
    Eigen::Vector3d tmp, res;
    if(sim_flag_)
    {
        // double min_x = std::min(root_->position.x(), goal_node_->position.x());
        // double min_y = std::min(root_->position.y(), goal_node_->position.y());
        // double min_z = std::min(root_->position.z(), goal_node_->position.z());
        // double max_x = std::max(root_->position.x(), goal_node_->position.x());
        // double max_y = std::max(root_->position.y(), goal_node_->position.y());
        // double max_z = std::max(root_->position.z(), goal_node_->position.z());

        // tmp << (max_x - min_x) * sampler_.getRandom(),
        //        (max_y - min_y) * sampler_.getRandom(),
        //        (max_z - min_z) * sampler_.getRandom();
        // res = tmp + Eigen::Vector3d(min_x, min_y, min_z);
        tmp << map_generate_ptr_->map_size_.x() * sampler_.getRandom(),
               map_generate_ptr_->map_size_.y() * sampler_.getRandom(),
               map_generate_ptr_->map_size_.z() * sampler_.getRandom();
        res = tmp + map_generate_ptr_->map_min_boundary_;
    }
    else
    {
        tmp << map_ptr_->mp_.map_size_.x() * sampler_.getRandom(),
               map_ptr_->mp_.map_size_.y() * sampler_.getRandom(),
               map_ptr_->mp_.map_size_.z() * sampler_.getRandom();
        res = tmp + map_ptr_->mp_.map_origin_;
    }
    return res;
};

Eigen::Vector3d RRT::generateRandomEllipse()
{
    const double goal_bias = -0.4; // 30%概率直接采样目标点
    if (sampler_.getRandomSymmetry() < goal_bias) 
        return goal_node_->position;
    
    // 计算采样区域
    double c = 0.5 * (root_->position - goal_node_->position).norm();
    double a = 0.5 * goal_node_->cost_from_start;
    double b = sqrt(a * a - c * c);

    Eigen::Vector3d sample;
    // 先在a轴上进行采样
    sample(0) = a * sampler_.getRandomSymmetry();
    // 计算椭圆在该点上的切平面半径
    double r = sqrt(b * b * (1 - sample(0) / (a * a)));
    // 在圆上进行采样
    sample(1) = r * sampler_.getRandomSymmetry();
    sample(2) = r * sampler_.getRandomSymmetry();
    // 计算旋转矩阵
    Eigen::Vector3d x = Eigen::Vector3d::UnitX();
    Eigen::Quaterniond R = Eigen::Quaterniond::FromTwoVectors(x, (goal_node_->position - root_->position).normalized());
    sample = R * sample + 0.5 * (root_->position + goal_node_->position);
    return sample;
}

Eigen::Vector3d RRT::Steer(const Eigen::Vector3d &x_rand, const Eigen::Vector3d &x_near, 
                           double step_size)
{
    Eigen::Vector3d diff_vec = x_rand - x_near;
    if(diff_vec.norm() <= step_size)
        return x_rand;
    else
        return x_near + step_size * diff_vec.normalized();
};

void RRT::ChangeParent(SampleNodePtr &node, SampleNodePtr &parent, const double &cost_from_parent)
{
    if(node->parent)
        node->parent->children.remove(node);
    node->parent = parent;
    node->cost_from_parent = cost_from_parent;
    node->cost_from_start = parent->cost_from_start + cost_from_parent;
    parent->children.push_back(node);

    // 更新后代的cost
    SampleNodePtr descendant(node);
    std::queue<SampleNodePtr> Q;
    Q.push(descendant);
    while (!Q.empty())
    {
        descendant = Q.front();
        Q.pop();
        for(const auto leafptr : descendant->children)
        {
            leafptr->cost_from_start = leafptr->cost_from_parent + descendant->cost_from_start;
            Q.push(leafptr);
        }
    }
    
};

bool RRT::collisionCheck(const Eigen::Vector3d &x_near, const Eigen::Vector3d &x_new)
{       
    // 新生长出的点是否在障碍物中
    bool occ = pointCollisionCheck(x_new);
    if(occ)
        return occ;

    occ = lineCollisionCheck(x_near, x_new);

    return occ;
};

bool RRT::lineCollisionCheck(const Eigen::Vector3d &start, const Eigen::Vector3d &end)
{
    Eigen::Vector3i start_id, end_id;
    if(sim_flag_)
    {
        map_generate_ptr_->pos2Index_global(start, start_id);
        map_generate_ptr_->pos2Index_global(end, end_id);
    }
    else
    {
        map_ptr_->pos2Index_global(start, start_id);
        map_ptr_->pos2Index_global(end, end_id);
    }
    return rayCast(start_id, end_id, [this](const Eigen::Vector3i& idx){
            if(sim_flag_)
                return map_generate_ptr_->isOccupied_global(idx) == map_generate::VoxelState::OCCUPANCY;
            else
                return map_ptr_->isOccupied_global(idx) == CUADC::VoxelState::OCCUPANCY;
    });
}

bool RRT::pointCollisionCheck(const Eigen::Vector3d &x)
{
    if(sim_flag_)
        return map_generate_ptr_->isOccupied_global(x) == map_generate::VoxelState::OCCUPANCY;
    else
        return map_ptr_->isOccupied_global(x) == CUADC::VoxelState::OCCUPANCY;
};

// 三维 Bresenham 算法实现（射线投射）
bool RRT::rayCast(const Eigen::Vector3i& start, const Eigen::Vector3i& end, const std::function<bool(const Eigen::Vector3i&)>& checkOccupied) {
    Eigen::Vector3i delta = end - start;
    Eigen::Vector3i step = delta.cwiseSign();
    delta = delta.cwiseAbs();

    int maxDelta = std::max({delta.x(), delta.y(), delta.z()});
    Eigen::Vector3i t(delta.x() * 2, delta.y() * 2, delta.z() * 2);

    Eigen::Vector3i current = start;
    for (int i = 0; i <= maxDelta; ++i) {

#ifdef RRT_DEBUG
        Eigen::Vector3d tmp;
        if(sim_flag_)
            map_generate_ptr_->index2Pos_global(current, tmp);
        else
            map_ptr_->index2Pos_global(current, tmp);
        ray_cast_path.push_back(tmp);
#endif  // RRT_DEBUG

        if (checkOccupied(current)) {
            return true; // 发现碰撞
        }

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

        t.x() += delta.x() * 2;
        t.y() += delta.y() * 2;
        t.z() += delta.z() * 2;
    }
    return false; // 无碰撞
};