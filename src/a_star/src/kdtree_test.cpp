#include "rrt/kdtree.hpp"
#include <iostream>
#include <chrono>

int main() {
    // 创建空树
    ParallelKdTree parallel_tree;
    SingleKdTree single_tree;
    
    // 动态插入点
    SampleNodePtr node1 = std::make_shared<SampleNode>();
    node1->position << -1, -1, -1;
    SampleNodePtr node2 = std::make_shared<SampleNode>();
    node2->position << 5, 4, 2;
    SampleNodePtr node3 = std::make_shared<SampleNode>();
    node3->position << -9, -6, -3;
    auto start_time = std::chrono::high_resolution_clock::now();
    parallel_tree.insert(node1);
    parallel_tree.insert(node2);
    parallel_tree.insert(node3);
    auto end_time = std::chrono::high_resolution_clock::now();
    std::cout << "Parallel KdTree insert tree time : " << (end_time - start_time).count() / 1e9 << "s\n";

    start_time = std::chrono::high_resolution_clock::now();
    single_tree.insert(node1);
    single_tree.insert(node2);
    single_tree.insert(node3);
    end_time = std::chrono::high_resolution_clock::now();
    std::cout << "Single KdTree insert tree time : " << (end_time - start_time).count() / 1e9 << "s\n";


    // 混合使用构建和插入
    std::vector<SampleNodePtr> nodes;
    for (int i = 0; i < 1e7; ++i) {
        SampleNodePtr node = std::make_shared<SampleNode>();
        node->position = Eigen::Vector3d::Random();
        nodes.push_back(node);
    }
    start_time = std::chrono::high_resolution_clock::now();
    parallel_tree.build(nodes);
    end_time = std::chrono::high_resolution_clock::now();
    std::cout << "Parallel KdTree build tree time : " << (end_time - start_time).count() / 1e9 << "s\n";

    start_time = std::chrono::high_resolution_clock::now();
    single_tree.build(nodes);
    end_time = std::chrono::high_resolution_clock::now();
    std::cout << "Single KdTree build tree time : " << (end_time - start_time).count() / 1e9 << "s\n";
    // tree.clear();  // 会重置之前的树

    // 继续插入新点
    SampleNodePtr node4 = std::make_shared<SampleNode>();
    node4->position << -1, -1, -1;
    SampleNodePtr node5 = std::make_shared<SampleNode>();
    node5->position << -8, -8, -7;
    
    start_time = std::chrono::high_resolution_clock::now();
    parallel_tree.insert(node4);
    parallel_tree.insert(node5);
    end_time = std::chrono::high_resolution_clock::now();
    std::cout << "Parallel KdTree insert time : " << (end_time - start_time).count() / 1e9 << "s\n";

    start_time = std::chrono::high_resolution_clock::now();
    single_tree.insert(node4);
    single_tree.insert(node5);
    end_time = std::chrono::high_resolution_clock::now();
    std::cout << "Single KdTree insert time : " << (end_time - start_time).count() / 1e9 << "s\n";

    // 查询测试
    Vector3d query(6, 6, 3);
    start_time = std::chrono::high_resolution_clock::now();
    SampleNodePtr nearest = parallel_tree.nearestNeighbor(query);
    end_time = std::chrono::high_resolution_clock::now();
    std::cout << "Parallel KdTree search time : " << (end_time - start_time).count() / 1e9 << "s\n";
    
    start_time = std::chrono::high_resolution_clock::now();
    nearest = parallel_tree.nearestNeighbor(query);
    end_time = std::chrono::high_resolution_clock::now();
    std::cout << "Single KdTree search time : " << (end_time - start_time).count() / 1e9 << "s\n";

    start_time = std::chrono::high_resolution_clock::now();
    std::vector<SampleNodePtr> parallel_radius_set = parallel_tree.radiusSearch(nearest->position, 20.0);
    end_time = std::chrono::high_resolution_clock::now();
    std::cout << "Parallel Kdtree radius set size is : " << parallel_radius_set.size() << '\n';
    std::cout << "Parallel KdTree radius search time : " << (end_time - start_time).count() / 1e9 << "s\n";

    start_time = std::chrono::high_resolution_clock::now();
    std::vector<SampleNodePtr> single_radius_set = single_tree.radiusSearch(nearest->position, 20.0);
    end_time = std::chrono::high_resolution_clock::now();
    std::cout << "Single Kdtree radius set size is : " << single_radius_set.size() << '\n';
    std::cout << "Single KdTree radius search time : " << (end_time - start_time).count() / 1e9 << "s\n";



    std::cout << "Nearest neighbor: " << nearest->position.transpose() << std::endl;
    return 0;
}