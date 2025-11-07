#pragma once
#include <vector>
#include <memory>
#include <algorithm>
#include <limits>
#include <Eigen/Dense>
#include <list>
#include <queue>
#include <thread>
#include <mutex>
#include <stack>
#include <future>
#include <atomic>

using Eigen::Vector3d;

struct SampleNode;
using SampleNodePtr = std::shared_ptr<SampleNode>;

struct SampleNode {
    Vector3d position;
    SampleNodePtr parent;
    std::list<SampleNodePtr> children;
    double cost_from_start = 0.0;
    double cost_from_parent = 0.0;
    std::mutex node_mutex; // 节点级锁
    
    SampleNode() : parent(nullptr) {}
    SampleNode(const SampleNodePtr& s_ptr) : 
        position(s_ptr->position), parent(s_ptr->parent),
        cost_from_start(s_ptr->cost_from_start),
        cost_from_parent(s_ptr->cost_from_parent) {}
};

struct KdNode {
    SampleNodePtr node;
    int axis;
    std::unique_ptr<KdNode> left;
    std::unique_ptr<KdNode> right;
    
    KdNode(SampleNodePtr sn, int a) : node(sn), axis(a) {}
};

class ParallelKdTree {
public:
    ParallelKdTree() = default;

    // 并行构建接口
    void build(std::vector<SampleNodePtr>& nodes) {
        root_ = parallelBuild(nodes.begin(), nodes.end(), 0);
    }

    // 线程安全插入
    void insert(SampleNodePtr new_node) {
        std::lock_guard<std::mutex> lock(tree_mutex_);
        insertImpl(root_, new_node, 0);
    }

    // 并行最近邻搜索
    SampleNodePtr nearestNeighbor(const Vector3d& query) const {
        std::atomic<double> best_dist(std::numeric_limits<double>::max());
        SampleNodePtr best_node;
        std::stack<const KdNode*> node_stack;
        node_stack.push(root_.get());
        
        std::vector<std::thread> workers;
        const int thread_num = std::thread::hardware_concurrency();
        std::mutex stack_mutex;
        
        auto worker = [&] {
            while(true) {
                const KdNode* curr = nullptr;
                {
                    std::lock_guard<std::mutex> lock(stack_mutex);
                    if(node_stack.empty()) break;
                    curr = node_stack.top();
                    node_stack.pop();
                }
                
                if(!curr) continue;
                
                const Vector3d& curr_pos = curr->node->position;
                const double sq_dist = (curr_pos - query).squaredNorm();
                
                // 原子更新最优解
                double current_best = best_dist.load(std::memory_order_relaxed);
                while(sq_dist < current_best) {
                    if(best_dist.compare_exchange_weak(current_best, sq_dist,
                        std::memory_order_relaxed,
                        std::memory_order_relaxed)) 
                    {
                        std::lock_guard<std::mutex> lock(stack_mutex);
                        best_node = curr->node;
                    }
                }
                
                const int axis = curr->axis;
                const double axis_diff = query[axis] - curr_pos[axis];
                
                std::lock_guard<std::mutex> lock(stack_mutex);
                if(axis_diff < 0) {
                    if(curr->left) node_stack.push(curr->left.get());
                    if(curr->right && axis_diff*axis_diff < best_dist) 
                        node_stack.push(curr->right.get());
                } else {
                    if(curr->right) node_stack.push(curr->right.get());
                    if(curr->left && axis_diff*axis_diff < best_dist)
                        node_stack.push(curr->left.get());
                }
            }
        };
        
        for(int i=0; i<thread_num; ++i)
            workers.emplace_back(worker);
        for(auto& t : workers) t.join();
        
        return best_node;
    }

    // 并行半径搜索接口
    std::vector<SampleNodePtr> radiusSearch(const Vector3d& center, double radius) const 
    {
        std::vector<SampleNodePtr> result;
        std::mutex result_mutex;
        std::queue<const KdNode*> task_queue;
        std::atomic<bool> done{false};
        
        // 初始化任务队列
        task_queue.push(root_.get());
        
        // 线程池执行任务
        auto worker = [&] {
            while (!done.load(std::memory_order_relaxed) || !task_queue.empty()) {
                const KdNode* curr = nullptr;
                {
                    std::lock_guard<std::mutex> lock(result_mutex);
                    if (task_queue.empty()) continue;
                    curr = task_queue.front();
                    task_queue.pop();
                }
                
                if (!curr) continue;
                
                // 距离计算与结果收集
                const Vector3d diff = curr->node->position - center;
                if (diff.squaredNorm() <= radius * radius) {
                    std::lock_guard<std::mutex> lock(result_mutex);
                    result.push_back(curr->node);
                }
                
                // 子树任务生成
                const int axis = curr->axis;
                const double axis_diff = center[axis] - curr->node->position[axis];
                const double axis_sq = axis_diff * axis_diff;
                
                {
                    std::lock_guard<std::mutex> lock(result_mutex);
                    if (axis_diff <= 0 || axis_sq <= radius * radius) 
                        task_queue.push(curr->left.get());
                    if (axis_diff >= 0 || axis_sq <= radius * radius)
                        task_queue.push(curr->right.get());
                }
            }
        };
        
        // 启动工作线程（网页1线程池策略）
        const int thread_num = std::thread::hardware_concurrency();
        std::vector<std::thread> workers;
        for (int i = 0; i < thread_num; ++i)
            workers.emplace_back(worker);
        
        // 等待任务完成
        done.store(true);
        for (auto& t : workers) t.join();
        
        return result;
    }

    void clear() noexcept 
    {
        std::lock_guard<std::mutex> lock(tree_mutex_);
        root_.reset();
    }

    bool empty() const noexcept 
    {
        static_assert(noexcept(root_ == nullptr), 
            "Pointer comparison must be noexcept");
        return root_ == nullptr; 
    }

private:
    std::unique_ptr<KdNode> root_;
    mutable std::mutex tree_mutex_;
    
    // 并行构建核心
    std::unique_ptr<KdNode> parallelBuild(
        std::vector<SampleNodePtr>::iterator begin,
        std::vector<SampleNodePtr>::iterator end,
        int depth) 
    {
        const size_t size = end - begin;
        if(size == 0) return nullptr;
        
        const int axis = depth % 3;
        constexpr size_t PARALLEL_THRESHOLD = 500;
        
        // 并行排序优化
        auto cmp = [axis](const SampleNodePtr& a, const SampleNodePtr& b) {
            return a->position[axis] < b->position[axis];
        };
        
        if(size > 10000) {
            std::vector<std::thread> threads;
            const size_t seg = size/4;
            for(int i=0; i<4; ++i) {
                threads.emplace_back([=]{
                    std::sort(begin+i*seg, begin+(i+1)*seg, cmp);
                });
            }
            for(auto& t : threads) t.join();
            std::inplace_merge(begin, begin+seg, begin+2*seg, cmp);
            std::inplace_merge(begin+2*seg, begin+3*seg, end, cmp);
            std::inplace_merge(begin, begin+2*seg, end, cmp);
        } else {
            std::sort(begin, end, cmp);
        }
        
        auto median = begin + size/2;
        auto node = std::make_unique<KdNode>(*median, axis);
        
        // 并行子树构建
        if(size > PARALLEL_THRESHOLD) {
            auto left_future = std::async(std::launch::async,
                [=]{ return parallelBuild(begin, median, depth+1); });
            node->right = parallelBuild(median+1, end, depth+1);
            node->left = left_future.get();
        } else {
            node->left = parallelBuild(begin, median, depth+1);
            node->right = parallelBuild(median+1, end, depth+1);
        }
        
        return node;
    }
    
    // 插入与平衡逻辑（线程安全）
    void insertImpl(std::unique_ptr<KdNode>& tree_node, 
                   SampleNodePtr new_node, 
                   int depth) {
        constexpr int BALANCE_FACTOR = 3;
        
        if(!tree_node) {
            tree_node = std::make_unique<KdNode>(new_node, depth%3);
            return;
        }
        
        const int axis = tree_node->axis;
        const bool go_left = new_node->position[axis] < tree_node->node->position[axis];
        
        if(go_left) {
            insertImpl(tree_node->left, new_node, depth+1);
        } else {
            insertImpl(tree_node->right, new_node, depth+1);
        }
        
        // 平衡检查
        const int left_size = countNodes(tree_node->left);
        const int right_size = countNodes(tree_node->right);
        if(abs(left_size - right_size) > BALANCE_FACTOR) {
            auto nodes = flattenSubtree(tree_node);
            tree_node = parallelBuild(nodes.begin(), nodes.end(), depth);
        }
    }
    
    // 辅助函数
    int countNodes(const std::unique_ptr<KdNode>& node) const {
        if(!node) return 0;
        return 1 + countNodes(node->left) + countNodes(node->right);
    }
    
    std::vector<SampleNodePtr> flattenSubtree(
        const std::unique_ptr<KdNode>& node) const {
        std::vector<SampleNodePtr> res;
        std::stack<const KdNode*> stack;
        stack.push(node.get());
        
        while(!stack.empty()) {
            auto curr = stack.top();
            stack.pop();
            if(!curr) continue;
            
            res.push_back(curr->node);
            stack.push(curr->left.get());
            stack.push(curr->right.get());
        }
        return res;
    }
};


class SingleKdTree {
public:
    SingleKdTree() = default;

    void build(std::vector<SampleNodePtr>& nodes) {
        root_ = buildTree(nodes.begin(), nodes.end(), 0);
    }

    void insert(SampleNodePtr node) {
        insertImpl(root_, std::move(node), 0);
    }

    SampleNodePtr nearestNeighbor(const Vector3d& query) const {
        SampleNodePtr best_node;
        double best_dist = std::numeric_limits<double>::max();
        nearestSearch(root_.get(), query, best_node, best_dist);
        return best_node;
    }

    std::vector<SampleNodePtr> radiusSearch(const Vector3d& center, double radius) const {
        std::vector<SampleNodePtr> result;
        radiusSearchImpl(root_.get(), center, radius*radius, result);
        return result;
    }

    void clear() noexcept {
        root_.reset();
    }

    bool empty() const noexcept {
        return root_ == nullptr;
    }

private:
    std::unique_ptr<KdNode> root_;

    using NodeIter = typename std::vector<SampleNodePtr>::iterator;

    std::unique_ptr<KdNode> buildTree(NodeIter begin, NodeIter end, int depth) {
        if (begin == end) return nullptr;

        const int axis = depth % 3;
        auto cmp = [axis](const SampleNodePtr& a, 
                         const SampleNodePtr& b) {
            return a->position[axis] < b->position[axis];
        };

        size_t len = end - begin;
        NodeIter median = begin + len / 2;
        std::nth_element(begin, median, end, cmp);

        auto node = std::make_unique<KdNode>(*median, axis);
        node->left = buildTree(begin, median, depth + 1);
        node->right = buildTree(median + 1, end, depth + 1);

        return node;
    }

    std::unique_ptr<KdNode> rebuildTree(NodeIter begin, NodeIter end, int depth) 
    {
        if (begin == end) return nullptr;

        const int axis = depth % 3; // 三维空间维度轮换[3](@ref)
        
        // 按当前分割轴排序找中位数[2](@ref)
        std::sort(begin, end, [axis](const SampleNodePtr& a, const SampleNodePtr& b) {
            return a->position[axis] < b->position[axis];
        });
        
        NodeIter median = begin + std::distance(begin, end)/2;
        
        // 创建新节点（保留原有节点指针）[1](@ref)
        auto node = std::make_unique<KdNode>(*median, axis);
        
        // 递归构建左右子树（左闭右开区间）
        node->left = rebuildTree(begin, median, depth + 1);
        node->right = rebuildTree(median + 1, end, depth + 1);
        
        return node;
    }

    void insertImpl(std::unique_ptr<KdNode>& tree_node, 
                        SampleNodePtr new_node, 
                        int depth) {
        // 平衡因子阈值
        constexpr int BALANCE_FACTOR = 3;

        if (!tree_node) {
            tree_node = std::make_unique<KdNode>(new_node, depth % 3);
            return;
        }

        const int axis = tree_node->axis;
        const bool go_left = new_node->position[axis] < tree_node->node->position[axis];

        // 递归插入子树
        if (go_left) {
            insertImpl(tree_node->left, new_node, depth + 1);
        } else {
            insertImpl(tree_node->right, new_node, depth + 1);
        }

        // 增量平衡检查 动态平衡策略
        const int left_size = countNodes(tree_node->left);
        const int right_size = countNodes(tree_node->right);
        if (abs(left_size - right_size) > BALANCE_FACTOR) {
            auto nodes = flattenSubtree(tree_node);
            tree_node = rebuildTree(nodes.begin(), nodes.end(), depth);
        }
    }

    // 辅助函数：统计子树节点数
    int countNodes(const std::unique_ptr<KdNode>& node) const {
        if (!node) return 0;
        return 1 + countNodes(node->left) + countNodes(node->right);
    }

    // 辅助函数：展开子树为线性列表
    std::vector<SampleNodePtr> flattenSubtree(
        const std::unique_ptr<KdNode>& node) const {
        std::vector<SampleNodePtr> nodes;
        std::stack<const KdNode*> stack;
        stack.push(node.get());
        
        while (!stack.empty()) {
            const KdNode* curr = stack.top();
            stack.pop();
            if (!curr) continue;
            
            nodes.push_back(curr->node);
            stack.push(curr->left.get());
            stack.push(curr->right.get());
        }
        return nodes;
    }

    void nearestSearch(const KdNode* node, const Vector3d& query,
                         SampleNodePtr& best_node, 
                         double& best_dist) const 
    {
        // 使用迭代栈替代递归防止爆栈
        std::stack<const KdNode*> node_stack;
        node_stack.push(node);

        while (!node_stack.empty()) {
            const KdNode* curr = node_stack.top();
            node_stack.pop();

            if (!curr) continue;

            // 计算当前节点距离(平方距离避免开方运算)
            const Vector3d& curr_pos = curr->node->position;
            const double sq_dist = (curr_pos - query).squaredNorm();
            
            // 更新最优节点
            if (sq_dist < best_dist) {
                best_dist = sq_dist;
                best_node = curr->node;
            }

            const int axis = curr->axis;
            const double axis_diff = query[axis] - curr_pos[axis];
            const bool go_left = axis_diff < 0;

            // 优先搜索更近的分支
            if (go_left) {
                node_stack.push(curr->right.get()); // 可能更近的分支
                node_stack.push(curr->left.get());
            } else {
                node_stack.push(curr->left.get());  // 可能更近的分支
                node_stack.push(curr->right.get());
            }

            // 提前终止条件(文献[6]空间剪枝策略)
            if (axis_diff * axis_diff >= best_dist)
                continue;
        }
    }

    void radiusSearchImpl(const KdNode* node, const Vector3d& center,
                         double sq_radius, 
                         std::vector<SampleNodePtr>& result) const {
        if (!node) return;

        const Vector3d diff = node->node->position - center;
        if (diff.squaredNorm() <= sq_radius) {
            result.push_back(node->node);
        }

        const int axis = node->axis;
        const double axis_diff = center[axis] - node->node->position[axis];

        if (axis_diff <= 0 || axis_diff * axis_diff <= sq_radius) {
            radiusSearchImpl(node->left.get(), center, sq_radius, result);
        }
        if (axis_diff >= 0 || axis_diff * axis_diff <= sq_radius) {
            radiusSearchImpl(node->right.get(), center, sq_radius, result);
        }
    }
};