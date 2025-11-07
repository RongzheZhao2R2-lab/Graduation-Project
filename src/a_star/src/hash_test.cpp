#include <iostream>
#include "a_star/map_hash.h"

int main() {
    MapIndex idx1, idx2;
    idx1.index_ = Eigen::Vector3i(-100, 200, -300);
    idx2.index_ = Eigen::Vector3i(-100, 200, -300);

    std::unordered_map<MapIndex, std::string> map;
    map[idx1] = "Value1";

    // 验证哈希无冲突
    std::cout << "Hash of idx1: " << std::hash<MapIndex>{}(idx1) << std::endl;
    std::cout << "Hash of idx2: " << std::hash<MapIndex>{}(idx2) << std::endl;
    std::cout << "Map value: " << map[idx2] << std::endl; // 输出 "Value1"

    return 0;
}
