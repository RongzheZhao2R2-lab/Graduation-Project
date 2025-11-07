#include "a_star/a_star.h"


double A_Star::getHeu(AstarHeu function, const Eigen::Vector3i &node1, const Eigen::Vector3i &node2)
{
    switch (function)
    {
        case AstarHeu::DIALOG:
            return getHeuDia(node1, node2);
        case AstarHeu::DIALOG_TIEBREAKER:
            return getHeuDiaTieBreaker(node1, node2);
        case AstarHeu::EUCLIDEAN:
            return getHeuEuclidean(node1, node2);
        case AstarHeu::EUCLIDEAN_TIEBREAKER:
            return getHeuEuclideanTieBreaker(node1, node2);
        case AstarHeu::MANHATTAN:
            return getHeuManhattan(node1, node2);
        case AstarHeu::MANHATTAN_TIEBREAKER:
            return getHeuManhattanTieBreaker(node1, node2);
    }
}

double A_Star::getHeuDia(const Eigen::Vector3i &node1, const Eigen::Vector3i &node2)
{
    double dx = abs(1.0 * (node1(0) - node2(0)));
    double dy = abs(1.0 * (node1(1) - node2(1)));
    double dz = abs(1.0 * (node1(2) - node2(2)));
    double h = 0.0;
    int diag = std::min(std::min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

double A_Star::getHeuEuclidean(const Eigen::Vector3i &node1, const Eigen::Vector3i &node2)
{
    // 欧式距离
    return sqrt(
        1.0 * pow((node1(0) - node2(0)), 2) +
        1.0 * pow((node1(1) - node2(1)), 2) +
        1.0 * pow((node1(2) - node2(2)), 2));
}

double A_Star::getHeuManhattan(const Eigen::Vector3i &node1, const Eigen::Vector3i &node2)
{
    // 曼哈顿距离
    return abs(node1(0) - node2(0)) +
           abs(node1(1) - node2(1)) +
           abs(node1(2) - node2(2));
}

double A_Star::getHeuDiaTieBreaker(const Eigen::Vector3i &node1, const Eigen::Vector3i &node2)
{
    double dx = abs(1.0 * (node1(0) - node2(0)));
    double dy = abs(1.0 * (node1(1) - node2(1)));
    double dz = abs(1.0 * (node1(2) - node2(2)));
    // return dx + dy + dz + (sqrt(3) - 3) * std::min(std::min(dx, dy), dz);
    double h = 0.0;
    int diag = std::min(std::min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return (1 + 1 / 30.0) * h;
}

double A_Star::getHeuEuclideanTieBreaker(const Eigen::Vector3i &node1, const Eigen::Vector3i &node2)
{
    return (1.0 + 1 / 30.0) * sqrt(
        1.0 * pow((node1(0) - node2(0)), 2) +
        1.0 * pow((node1(1) - node2(1)), 2) +
        1.0 * pow((node1(2) - node2(2)), 2));
}


// 曼哈顿距离启发函数
double A_Star::getHeuManhattanTieBreaker(const Eigen::Vector3i &node1, const Eigen::Vector3i &node2)
{
    return (1.0 + 1 / 30.0) * abs(node1(0) - node2(0)) +
                            abs(node1(1) - node2(1)) +
                            abs(node1(2) - node2(2));
}

inline void A_Star::AstarExpand(const Eigen::Vector3i& current, std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> & neighborSets, std::vector<double> & edgeCostSets)
{
    // 清空邻近节点容器
    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> neighbor_empy;
    if(SearchType::_2_DIEMS == dims_)
    {
        neighbor_empy.reserve(8);
        neighbor_empy.swap(neighborSets);
        // 清空边容器
        std::vector<double> edge_empty;
        edge_empty.reserve(8);
        edge_empty.swap(edgeCostSets);
    }
    else
    {
        neighbor_empy.reserve(26);
        neighbor_empy.swap(neighborSets);
        // 清空边容器
        std::vector<double> edge_empty;
        edge_empty.reserve(26);
        edge_empty.swap(edgeCostSets);
    }

    Eigen::Vector3i offset;
    Eigen::Vector3i tmp;
    bool occ;
    for(int x = -1; x <= 1; ++x)
        for(int y = -1; y <= 1; ++y)
            if(SearchType::_3_DIEMS == this->dims_)
            {
                for(int z = -1; z <= 1; ++z)
                {
                    offset << x, y, z;
                    tmp = current + offset;
                    if(sim_flag_)
                        occ = map_generate_ptr_->isOccupied_global(tmp);
                    else
                        occ = map_ptr_->isOccupied_global(tmp);

                    if(0 == occ)
                    {
                        // 记录该相邻点
                        neighborSets.push_back(tmp);
                        // 记录该点的边长
                        edgeCostSets.push_back(
                            sqrt(
                                pow(double(x), 2) + pow(double(y), 2) + pow(double(z), 2)
                            )
                        );
                    }
                }
            }
            else    // 2维
            {
                offset << x, y, 0;
                tmp = current + offset;
                if(sim_flag_)
                    occ = map_generate_ptr_->isOccupied_global(tmp);
                else
                    occ = map_ptr_->isOccupied_global(tmp);

                if(0 == occ)
                {
                    // 记录该相邻点
                    neighborSets.push_back(tmp);
                    // 记录该点的边长
                    edgeCostSets.push_back(
                        sqrt(
                            pow(double(x), 2) + pow(double(y), 2)
                        )
                    );
                }
            }
}


ASTAR_RET A_Star::AstarGraphSearch(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt, AstarHeu function, double &cost_time)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    
    if(sim_flag_)
    {
        map_generate_ptr_->pos2Index_global(start_pt, this->startIdx_);
        map_generate_ptr_->pos2Index_global(end_pt, this->goalIdx_);
    }else{
        map_ptr_->pos2Index_global(start_pt, this->startIdx_);
        map_ptr_->pos2Index_global(end_pt, this->goalIdx_);
    }

    if(SearchType::_2_DIEMS == dims_)
    {
        startIdx_(2) = this->voxel_height_;
        goalIdx_(2) = this->voxel_height_;
    }

#ifdef _LOG_DEBUG_
    std::cout << "The start id is (" << startIdx_.x() << ", " << startIdx_.y() << ", " << startIdx_.z() << ").\n";
    std::cout << "The goal id is (" << goalIdx_.x() << ", " << goalIdx_.y() << ", " << goalIdx_.z() << ")." << std::endl; 
#endif

    bool occ_goal, occ_start;
    if(sim_flag_)
    {
        occ_goal = map_generate_ptr_->isOccupied_global(goalIdx_);
        occ_start = map_generate_ptr_->isOccupied_global(startIdx_);
    }
    else{
        occ_goal = map_ptr_->isOccupied_global(goalIdx_);
        occ_start = map_ptr_->isOccupied_global(startIdx_);
    }

    if(occ_goal)
    {
#ifdef _LOG_DEBUG_
        ROS_WARN("The goal point is occupancied!");
#endif
        return ASTAR_RET::INIT_ERR;
    }
    if(occ_start)
    {
#ifdef _LOG_DEBUG_
        ROS_WARN("The start point is occupancied!");
#endif
        return ASTAR_RET::INIT_ERR;
    }


    // 释放内存
    // 清空上一次的地图访问记录
    terminatePtr_.reset();
    std::unordered_map<MapIndex, GridNodePtr> empty_map;
    empty_map.swap(visited_map_);
    if(sim_flag_)
        visited_map_.rehash(ceil(0.5 * map_generate_ptr_->map_size_.x() * map_generate_ptr_->map_size_.y() * map_generate_ptr_->map_size_.z()));
    else
        visited_map_.rehash(ceil(0.5 * map_ptr_->mp_.map_size_.x() * map_ptr_->mp_.map_size_.y() * map_ptr_->mp_.map_size_.z()));


    // 清空openset
    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> ().swap(open_set_);

    GridNodePtr startPtr = std::make_shared<GridNode>(startIdx_);
    startPtr->gScore_ = 0;
    startPtr->fScore_ = getHeu(function, startIdx_, goalIdx_);
    startPtr->grid_state_ = GridNodeState::OPENSET;

    if(goalIdx_ == startIdx_)
    {
        terminatePtr_ = startPtr;
        return ASTAR_RET::SUCCESS;
    }


    open_set_.push(startPtr);

    GridNodePtr current_ptr, neighbor_ptr;
    
    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> neighborSets;
    std::vector<double> edgeCostSets;

    MapIndex tmp;
    while(!open_set_.empty())
    {
        current_ptr = open_set_.top();
        open_set_.pop();
        // 放入closedSet
        current_ptr->grid_state_ = GridNodeState::CLOSEDSET;
        // 存入已访问过的地图visited_map_
        visited_map_[{current_ptr->index_}] = current_ptr;

        // 判断是否到达终点
        if(goalIdx_ == current_ptr->index_)
        {
            terminatePtr_ = current_ptr;
            break;
        }
        // 拓展相邻节点
        AstarExpand(current_ptr->index_, neighborSets, edgeCostSets);

        int count = 0;
        for(auto& node_idx : neighborSets)
        {
            tmp.index_ = node_idx;
            auto node = visited_map_.find(tmp);
            if(node == visited_map_.end()) // 未访问过，则加入到visited_map_和open_set_
            {
                // neighbor_ptr = new GridNode(node_idx);
                neighbor_ptr = std::make_shared<GridNode>(node_idx);
                neighbor_ptr->gScore_ = current_ptr->gScore_ + edgeCostSets[count];
                neighbor_ptr->fScore_ = getHeu(function, node_idx, goalIdx_) + neighbor_ptr->gScore_;
                neighbor_ptr->cameFrom_ = current_ptr;
                neighbor_ptr->grid_state_ = GridNodeState::OPENSET;
                open_set_.push(neighbor_ptr);
                visited_map_[{neighbor_ptr->index_}] = neighbor_ptr;
            }
            else if(GridNodeState::OPENSET == node->second->grid_state_)   // 曾访问过，并在open_set_中则更新
            {
                neighbor_ptr = node->second;
                double gScore = current_ptr->gScore_ + edgeCostSets[count];
                if(gScore < neighbor_ptr->gScore_)  // 若临接点的gscore更大则更新
                {
                    neighbor_ptr->gScore_ = gScore;
                    neighbor_ptr->fScore_ = getHeu(function, neighbor_ptr->index_, goalIdx_);
                    neighbor_ptr->cameFrom_ = current_ptr;
                }
            }
            count++;
        }
        auto end_time = std::chrono::high_resolution_clock::now();
        if(std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count() / 1000000.0 > max_time_)
        {
            cost_time = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count() / 1000000.0;
            std::cout << "Cost time : " << cost_time << "ms" << std::endl;
            ROS_WARN("Time out...Failed in A star path searching!");
            return ASTAR_RET::SEARCH_ERR;
        }
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
    cost_time = duration_ns / 1000000.0;
    return ASTAR_RET::SUCCESS;
}

// 路径是从goal到start
void A_Star::getPath(std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> &path)
{
    // 清空路径
    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>().swap(path);
    std::vector<GridNodePtr> gridPath;
    GridNodePtr end = terminatePtr_;
    while(end->cameFrom_ != nullptr)
    {
        path.push_back(end->index_);
        // 回到父节点
        end = end->cameFrom_;
    }
    path.push_back(end->index_);
    reverse(path.begin(), path.end());
}

// 路径是从goal到start
std::vector<Eigen::Vector3d> A_Star::getPath()
{
    std::vector<Eigen::Vector3d> path;
    std::vector<GridNodePtr> gridPath;
    GridNodePtr end = terminatePtr_;
    Eigen::Vector3d tmp;
    while(end->cameFrom_ != nullptr)
    {
        if(sim_flag_)
            map_generate_ptr_->index2Pos_global(end->index_, tmp);
        else
            map_ptr_->index2Pos_global(end->index_, tmp);
        path.push_back(tmp);
        // 回到父节点
        end = end->cameFrom_;
    }
    if(sim_flag_)
        map_generate_ptr_->index2Pos_global(end->index_, tmp);
    else
        map_ptr_->index2Pos_global(end->index_, tmp);
    path.push_back(tmp);
    
    return path;
}

void A_Star::rdpPath(std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> &path, double epsilon, std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> &simplified)
{
    double dmax = 0.0;
    int index = 0;
    int end = path.size() - 1;
    for(int i = 1; i < end; ++i)
    {
        Eigen::Vector3d p_i, p_start, p_end;
        if(sim_flag_)
        {
            map_generate_ptr_->index2Pos_global(path.at(i), p_i);
            map_generate_ptr_->index2Pos_global(path.at(0), p_start);
            map_generate_ptr_->index2Pos_global(path.at(end), p_end);
        }
        else
        {
            map_ptr_->index2Pos_global(path.at(i), p_i);
            map_ptr_->index2Pos_global(path.at(0), p_start);
            map_ptr_->index2Pos_global(path.at(end), p_end);
        }
        double d = perpendicularDistance(p_i, p_start, p_end);
        if(d > dmax)
        {
            index = i;
            dmax = d;
        }
    }

    if(dmax > epsilon){
        std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> recResults1;
        std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> recResults2;
        
        std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> firstLine(path.begin(), path.begin() + index + 1);
        std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> lastLine(path.begin() + index, path.end());

        rdpPath(firstLine, epsilon, recResults1);
        rdpPath(lastLine, epsilon, recResults2);

        simplified.assign(recResults1.begin(), recResults1.end() - 1);
        simplified.insert(simplified.end(), recResults2.begin(), recResults2.end());
    }else{
        simplified.clear();
        simplified.push_back(path.at(0));
        simplified.push_back(path.at(end));
    }
}

inline double A_Star::perpendicularDistance(const Eigen::Vector3d &p, const Eigen::Vector3d &start, const Eigen::Vector3d &end)
{
    Eigen::Vector3d se = (end - start).normalized();
    Eigen::Vector3d sp = p - start;
    return sp.cross(se).norm(); // 叉乘
}


// 三维 Bresenham 算法实现（射线投射）
bool rayCast(const Eigen::Vector3i& start, const Eigen::Vector3i& end, const std::function<bool(const Eigen::Vector3i&)>& checkOccupied) {
    Eigen::Vector3i delta = end - start;
    Eigen::Vector3i step = delta.cwiseSign();
    delta = delta.cwiseAbs();

    int maxDelta = std::max({delta.x(), delta.y(), delta.z()});
    Eigen::Vector3i t(delta.x() * 2, delta.y() * 2, delta.z() * 2);

    Eigen::Vector3i current = start;
    for (int i = 0; i <= maxDelta; ++i) {
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
}

void A_Star::simplifyPath(std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>& path,
                          std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>& simplify) {
    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>().swap(simplify);
    if (path.empty()) return;

    simplify.push_back(path.front());
    Eigen::Vector3i last_safe = path.front();

    for (size_t i = 1; i < path.size(); ++i) {
        const Eigen::Vector3i& current = path[i];
        // 使用射线投射检查直线路径是否安全
        bool isCollision = rayCast(last_safe, current, [this](const Eigen::Vector3i& idx) {
            if(sim_flag_)
                return map_generate_ptr_->isOccupied_global(idx) == map_generate::VoxelState::OCCUPANCY;
            else
                return map_ptr_->isOccupied_global(idx) == CUADC::VoxelState::OCCUPANCY;
        });

        if (isCollision) {
            // 发生碰撞，将前一个安全点加入简化路径
            last_safe = path[i - 1];
            simplify.push_back(last_safe);
        }
    }

    // 确保终点加入简化路径
    if (simplify.back() != path.back()) {
        simplify.push_back(path.back());
    }
};