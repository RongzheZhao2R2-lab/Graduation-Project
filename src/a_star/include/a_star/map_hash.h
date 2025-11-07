#pragma once

#include <unordered_map>
#include <cassert>
#include <Eigen/Eigen>


class MapIndex
{
public:
    MapIndex() {};
    MapIndex(const Eigen::Vector3i &index) : index_(index) {};

    Eigen::Vector3i index_;

    bool operator==(const MapIndex& index) const
    {
        return (Eigen::Vector3i::Zero() == this->index_ - index.index_);
    }
};

namespace std
{
    template<>
    struct hash<MapIndex>: public __hash_base<size_t, MapIndex>
    {
        size_t operator()(const MapIndex& index) const noexcept
        {

            // 确保 size_t 是 64 位（否则触发编译错误）
            static_assert(sizeof(size_t) >= 8, "Requires 64-bit size_t");
            
            // 确保坐标分量在 [-2^19, 2^19-1] 范围内
            constexpr int32_t MIN_VAL = -524288;   // -2^19
            constexpr int32_t MAX_VAL = 524287;    // 2^19-1
            assert(index.index_.x() >= MIN_VAL && index.index_.x() <= MAX_VAL);
            assert(index.index_.y() >= MIN_VAL && index.index_.y() <= MAX_VAL);
            assert(index.index_.z() >= MIN_VAL && index.index_.z() <= MAX_VAL);

            // 将 int32_t 转换为 uint32_t（保留补码位模式）
            const uint32_t x_uint = static_cast<uint32_t>(index.index_.x());
            const uint32_t y_uint = static_cast<uint32_t>(index.index_.y());
            const uint32_t z_uint = static_cast<uint32_t>(index.index_.z());

            // 截取低 20 位（掩码操作）
            constexpr uint32_t MASK = 0xFFFFF; // 20 位掩码 (0b11111111111111111111)
            const size_t x_part = (x_uint & MASK) << 40;
            const size_t y_part = (y_uint & MASK) << 20;
            const size_t z_part = (z_uint & MASK);

            // 组合为 64 位哈希值
            return x_part | y_part | z_part;
        }
    };
} // namespace std