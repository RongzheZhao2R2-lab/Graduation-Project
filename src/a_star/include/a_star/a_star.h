#pragma once

#include <ros/ros.h>
#include <queue>
#include "map_hash.h"

#include "map_generate/map_generate.hpp"

#include "occupancy_grid_map/occ_grid_map.h"

// #define _LOG_DEBUG_

struct GridNode;
using GridNodePtr = std::shared_ptr<GridNode>;
class A_Star;
using A_StarPtr = std::shared_ptr<A_Star>;

enum GridNodeState
{
    NONE = 0,
    OPENSET = 1,
    CLOSEDSET = 2
};

enum ASTAR_RET
{
	SUCCESS,
	INIT_ERR,
	SEARCH_ERR
};

struct GridNode
{
    /* data */
    Eigen::Vector3i index_;
    GridNodeState grid_state_ = GridNodeState::NONE;
    double gScore_{DBL_MAX}, fScore_{DBL_MAX};
    GridNodePtr cameFrom_{nullptr};
    GridNode(Eigen::Vector3i index) : index_(index) {};
};


class NodeComparator
{
public:
	bool operator()(GridNodePtr node1, GridNodePtr node2)
	{
		return node1->fScore_ > node2->fScore_;
	}
};

class A_Star
{
public:

    enum SearchType
    {
        _2_DIEMS = 2,
        _3_DIEMS = 3 
    };

    enum AstarHeu
    {
        DIALOG=0u,
        EUCLIDEAN=1u,
        MANHATTAN=2u,
        DIALOG_TIEBREAKER=3u,
        EUCLIDEAN_TIEBREAKER=4u,
        MANHATTAN_TIEBREAKER=5u
    };

protected:

    bool sim_flag_;
    map_generatePtr map_generate_ptr_;
    CUADC::MapPtr map_ptr_;

    std::unordered_map<MapIndex, GridNodePtr> visited_map_;    // 访问过的所有节点
    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> open_set_;   // 小根堆

    SearchType dims_;  // 维度，支持二维与三维
    int voxel_height_; // 若为二维搜索，设置默认高度，该高度与分辨率有关
    double max_time_;   // 搜索最大时间

    Eigen::Vector3i goalIdx_;   // 目标点
    Eigen::Vector3i startIdx_;  // 起始点
    
    GridNodePtr terminatePtr_;

    double getHeu(AstarHeu function, const Eigen::Vector3i &node1, const Eigen::Vector3i &node2);

    // 对角距离启发函数
    double getHeuDia(const Eigen::Vector3i &node1, const Eigen::Vector3i &node2);
    // 欧式距离启发函数
    double getHeuEuclidean(const Eigen::Vector3i &node1, const Eigen::Vector3i &node2);
    // 曼哈顿距离启发函数
    double getHeuManhattan(const Eigen::Vector3i &node1, const Eigen::Vector3i &node2);
    
    // 含有Tie Breaker的启发函数
    // 对角距离启发函数
    double getHeuDiaTieBreaker(const Eigen::Vector3i &node1, const Eigen::Vector3i &node2);
    // 欧式距离启发函数
    double getHeuEuclideanTieBreaker(const Eigen::Vector3i &node1, const Eigen::Vector3i &node2);
    // 曼哈顿距离启发函数
    double getHeuManhattanTieBreaker(const Eigen::Vector3i &node1, const Eigen::Vector3i &node2);

    inline void AstarExpand(const Eigen::Vector3i& current, std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> & neighborSets, std::vector<double> & edgeCostSets);

    inline double perpendicularDistance(const Eigen::Vector3d &p, const Eigen::Vector3d &start, const Eigen::Vector3d &end);

public:
    A_Star(){};
    ~A_Star(){};
    // max_time 单位是ms
    void initSimAstar(const map_generatePtr &map_ptr, const double max_time, SearchType dimensions) 
    { 
        if(!map_ptr){
            std::cerr << "Error: map_ptr is nullptr!" << std::endl;
            ROS_BREAK();
        }
        map_generate_ptr_ = map_ptr; 
        max_time_ = max_time; 
        dims_ = dimensions; 
        sim_flag_ = true;
    }
    void initAstar(const CUADC::MapPtr &map_ptr, const double max_time, SearchType dimensions) 
    { 
        if(!map_ptr){
            std::cerr << "Error: map_ptr is nullptr!" << std::endl;
            ROS_BREAK();
        }
        map_ptr_ = map_ptr; 
        max_time_ = max_time; 
        dims_ = dimensions; sim_flag_ = false;
    }

    void SetSimHeight(double height) {
        if(map_generate_ptr_)
            voxel_height_ = floor(height * map_generate_ptr_->resolution_inv_);
        else
        {
            ROS_ERROR("Please use the map generate pointer.");
            ROS_BREAK();
        }
    }

    void SetHeight(const double &height) {
        if(map_ptr_)
            voxel_height_ = floor(height * map_ptr_->mp_.resolution_inv_);
        else
        {
            ROS_ERROR("Please use the map pointer.");
            ROS_BREAK();
        }
    }

    double getHeight() const 
    {
        if(map_ptr_) 
            return (1.0 * voxel_height_ + 0.5) * (map_ptr_->mp_.resolution_);
        else if(map_generate_ptr_)
            return (1.0 * voxel_height_ + 0.5) * (map_generate_ptr_->resolution_);
    }


    // Astar
    ASTAR_RET AstarGraphSearch(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt, AstarHeu function, double &cost_time);

    void getPath(std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>&path);

    std::vector<Eigen::Vector3d> getPath();

    void rdpPath(std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> &path, double epsilon, std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> &simplified);

    // 效率比rdp算法高
    void simplifyPath(std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> &path, std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> &simplify);

};

