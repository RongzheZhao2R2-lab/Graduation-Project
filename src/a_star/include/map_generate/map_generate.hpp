#pragma once
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <random>
#include <omp.h>

class map_generate;
using map_generatePtr = std::shared_ptr<map_generate>;

class map_generate
{
public:

    enum VoxelState{
        FREE=0u,
        OCCUPANCY=1u
    };

public:
    map_generate(ros::NodeHandle &nh) : nh_(nh)
    {
        vis_timer_ = nh.createTimer(ros::Duration(1), &map_generate::visualCallback, this);
        map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("test_map", 10);
    }

    ~map_generate(){};

    void SetParam(const Eigen::Vector3d &map_size, const double resolution, const double scale)
    {
        resolution_ = resolution;
        resolution_inv_ = 1.0 / resolution_;
        map_size_ = map_size;
        map_max_boundary_ = map_size / 2;
        map_min_boundary_ = -map_size / 2;
        std::cout << "map_min_boundary_ (" << map_min_boundary_.x() << ", "<< map_min_boundary_.y() << ", " << map_min_boundary_.z() << ").\n";
        pos2Index_global(map_min_boundary_, map_origin_id_);
        std::cout << "map_origin_id_ (" << map_origin_id_.x() << ", " << map_origin_id_.y() << ", " << map_origin_id_.z() << ").\n";
        grid_map_size_ << ceil(map_size_.x() * resolution_inv_), ceil(map_size_.y() * resolution_inv_), ceil(map_size_.z() * resolution_inv_);
        std::cout << "grid_map_size : \n" << grid_map_size_.x() << ", " << grid_map_size_.y() << ", " << grid_map_size_.z() << ").\n";
        scale_ = scale;
        gridMap_ = Eigen::MatrixXi::Zero(grid_map_size_.x(), grid_map_size_.y());
    }

    void InitFromHeightMap(const std::string& filename, const double resolution, const double MaxHeight)
    {
        resolution_ = resolution;
        resolution_inv_ = 1.0 / resolution_;
        cv::Mat img;
        try
        {
            img = cv::imread(filename, cv::COLOR_BGR2GRAY);
        }
        catch(const cv::Exception& e)
        {
            ROS_INFO_STREAM("read image failed: " << e.what());
            ROS_BREAK();
        }


        cv::Mat rotated_img;
        cv::transpose(img, rotated_img);
        cv::flip(rotated_img, rotated_img, 0);

        cv::Mat img_16s;
        rotated_img.convertTo(img_16s, CV_16S);

        int rows = img.rows;
        int cols = img.cols;
        gridMap_.resize(rows, cols);
        gridMap_.setZero();
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                gridMap_(i, j) = ceil(img_16s.at<int16_t>(rows - 1 - i, cols - 1 - j));
                // std::cout << "z " << ceil(1.0 * img_16s.at<int16_t>(rows - 1 - i, cols - 1 - j) / range * MaxHeight) << std::endl;
            }
        }
        map_size_ << rows * 1.0 * resolution_, cols * 1.0 * resolution_, MaxHeight;
        std::cout << "map_size_ (" << map_size_.x() << ", " << map_size_.y() << ", " << map_size_.z() << ").\n";
        map_max_boundary_ = map_size_ / 2;
        map_min_boundary_ = -map_size_ / 2;
        std::cout << "map_min_boundary_ (" << map_min_boundary_.x() << ", "<< map_min_boundary_.y() << ", " << map_min_boundary_.z() << ").\n";
        pos2Index_global(map_min_boundary_, map_origin_id_);
        std::cout << "map_origin_id_ (" << map_origin_id_.x() << ", " << map_origin_id_.y() << ", " << map_origin_id_.z() << ").\n";
        grid_map_size_ << ceil(map_size_.x() * resolution_inv_), ceil(map_size_.y() * resolution_inv_), ceil(map_size_.z() * resolution_inv_);
        std::cout << "grid_map_size : \n" << grid_map_size_.x() << ", " << grid_map_size_.y() << ", " << grid_map_size_.z() << ").\n";
    }

    void saveHeightmap(const std::string& filename) 
    {
        int rows = gridMap_.rows();
        int cols = gridMap_.cols();

        // 保证行列数为偶数
        if (rows % 2 != 0)
            rows += 1;
        if (cols % 2 != 0)
            cols += 1;

        // 创建16位无符号Mat（CV_16S）
        cv::Mat img(rows, cols, CV_16S, cv::Scalar(0));


        for (int i = 0; i < gridMap_.rows(); ++i) {
            for (int j = 0; j < gridMap_.cols(); ++j) {
                img.at<int16_t>(rows - 1 - i, cols - 1 - j) = static_cast<int16_t>(gridMap_(i, j));
            }
        }

        try {
            // 保存16位图片（PNG格式支持16位灰度）
            cv::imwrite(filename, img);
        }
        catch (const cv::Exception& e) {
            ROS_INFO_STREAM("image save failed: " << e.what());
        }
    }

    bool isOccupied_global(const Eigen::Vector3d& pos_global)
    {
        Eigen::Vector3i id_global;
        pos2Index_global(pos_global, id_global);
        if(!isInMap(pos_global))    // 在地图外
            return VoxelState::FREE;
        else    // 在地图内
        {
            int x = id_global.x() - map_origin_id_.x();
            int y = id_global.y() - map_origin_id_.y();
            int z = id_global.z() - map_origin_id_.z();
            if(z > gridMap_(x, y))
                return VoxelState::FREE;
            else
                return VoxelState::OCCUPANCY;
        }
    }

    bool isOccupied_global(const Eigen::Vector3i& id_global)
    {
        Eigen::Vector3d pos_global;
        index2Pos_global(id_global, pos_global);
        if(!isInMap(pos_global))    // 在地图外
            return VoxelState::FREE;
        else    // 在地图内
        {
            int x = id_global.x() - map_origin_id_.x();
            int y = id_global.y() - map_origin_id_.y();
            int z = id_global.z() - map_origin_id_.z();
            if(z > gridMap_(x, y))
                return VoxelState::FREE;
            else
                return VoxelState::OCCUPANCY;
        }
    }

    void index2Pos_global(const Eigen::Vector3i& id, Eigen::Vector3d& pos)
    {
        for(int i = 0; i < 3; ++i)  pos(i) = ((double)id(i) + 0.5) * resolution_;
    }
    
    void pos2Index_global(const Eigen::Vector3d &pos, Eigen::Vector3i &id)
    {
        for(int i = 0; i < 3; ++i)  id(i) = floor(pos(i) * resolution_inv_);
    }

    bool isInMap(const Eigen::Vector3d& pos)
    {
        if (pos(0) < map_min_boundary_(0) + 1e-4 || pos(1) < map_min_boundary_(1) + 1e-4 ||
            pos(2) < map_min_boundary_(2) + 1e-4) {
            // cout << "less than min range!" << endl;
            return false;
        }
        if (pos(0) > map_max_boundary_(0) - 1e-4 || pos(1) > map_max_boundary_(1) - 1e-4 ||
            pos(2) > map_max_boundary_(2) - 1e-4) {
            return false;
        }
        return true;
    }

    // 生成 3D 高度地图
    void generateHeightMap() {
        pcl::PointCloud<pcl::PointXYZ>().swap(map_cloud_);
        // 生成随机排列的 256 维梯度
        std::vector<int> perm(512);
        std::iota(perm.begin(), perm.begin() + 256, 0);
        std::shuffle(perm.begin(), perm.begin() + 256, std::default_random_engine(std::random_device()()));
        std::copy(perm.begin(), perm.begin() + 256, perm.begin() + 256);

        for (int x = 0; x < grid_map_size_.x(); ++x) {
            for (int y = 0; y < grid_map_size_.y(); ++y) {
                double noiseValue = perlinNoise(x * scale_, y * scale_, 0, perm);
                int zHeight = static_cast<int>((noiseValue + 1.0) / 2.0 * (grid_map_size_.z() - 1));
                gridMap_(x, y) = zHeight;

            }
        }
    };

    // 生成 3D 栅格地图
    bool generateGridMap()
    {
        Eigen::Vector2i max_z_idx;
        int max_z = 1;
        bool sign = false;
        Eigen::Vector3i id;
        Eigen::Vector3d pos;
        for(int x = 0; x < grid_map_size_.x(); ++x)
        {
            for(int y = 0; y < grid_map_size_.y(); ++y)
            {
                id << x, y, gridMap_(x, y);
                id = id + map_origin_id_;
                index2Pos_global(id, pos);
                // pt_.x = ((double)(x) + 0.5)  * resolution_ - map_size_.x() * 0.5;
                // pt_.y = ((double)(y) + 0.5)  * resolution_ - map_size_.y() * 0.5;
                // pt_.z = (1.0 * gridMap_(x, y) + 0.5) * resolution_ - map_size_.z() * 0.5;
                pt_.x = pos.x();
                pt_.y = pos.y();
                pt_.z = pos.z();
                map_cloud_.push_back(pt_);
                max_z = 1;
                sign = false;
                for(int dx = -1; dx < 2; ++dx)
                    for(int dy = -1; dy < 2; ++dy)
                    {
                        if(dx == 0 && dy == 0)
                            continue;
                        else if(x + dx < 0 || y + dy < 0 || x + dx > grid_map_size_.x() - 1 || y + dy > grid_map_size_.y() - 1)
                            continue;
                        if(1 == gridMap_(x, y) - gridMap_(x + dx, y + dy))
                            continue;
                        if(gridMap_(x, y) - gridMap_(x + dx, y + dy) > max_z)
                        {
                            max_z_idx << x + dx, y + dy;
                            max_z = gridMap_(x, y) - gridMap_(x + dx, y + dy);
                            sign = true;
                        }
                    }
                if(sign)
                {
                    for(int z = gridMap_(x, y) - 1; z > gridMap_(max_z_idx.x(), max_z_idx.y()); --z)
                    {
                        pt_.z = (1.0 * z + 0.5) * resolution_ - map_size_.z() * 0.5;
                        map_cloud_.push_back(pt_);
                    }
                }
            }
        }

        map_cloud_.width = map_cloud_.points.size();
        map_cloud_.height = 1;
        map_cloud_.is_dense = true;
        map_cloud_.header.frame_id = "map";
        pcl::toROSMsg(map_cloud_, cloud_msg_);
        map_pub_.publish(cloud_msg_);
        return true;
    }

protected:

    // 计算 Perlin 噪声的插值
    double fade(double t) 
    {
        return t * t * t * (t * (t * 6 - 15) + 10);
    };

    // 线性插值
    double lerp(double t, double a, double b) 
    {
        return a + t * (b - a);
    };

    // 计算 Perlin 噪声的梯度
    double grad(int hash, double x, double y, double z) 
    {
        int h = hash & 15;
        double u = h < 8 ? x : y;
        double v = h < 4 ? y : (h == 12 || h == 14 ? x : z);
        return ((h & 1) ? -u : u) + ((h & 2) ? -v : v);
    };

    // Perlin 噪声函数
    double perlinNoise(double x, double y, double z, const std::vector<int>& perm) 
    {
        int X = (int)std::floor(x) & 255;
        int Y = (int)std::floor(y) & 255;
        int Z = (int)std::floor(z) & 255;

        x -= std::floor(x);
        y -= std::floor(y);
        z -= std::floor(z);

        double u = fade(x);
        double v = fade(y);
        double w = fade(z);

        int A = perm[X] + Y, AA = perm[A] + Z, AB = perm[A + 1] + Z;
        int B = perm[X + 1] + Y, BA = perm[B] + Z, BB = perm[B + 1] + Z;

        return lerp(w,
            lerp(v,
                lerp(u, grad(perm[AA], x, y, z), grad(perm[BA], x - 1, y, z)),
                lerp(u, grad(perm[AB], x, y - 1, z), grad(perm[BB], x - 1, y - 1, z))
            ),
            lerp(v,
                lerp(u, grad(perm[AA + 1], x, y, z - 1), grad(perm[BA + 1], x - 1, y, z - 1)),
                lerp(u, grad(perm[AB + 1], x, y - 1, z - 1), grad(perm[BB + 1], x - 1, y - 1, z - 1))
            )
        );
    };

    void visualCallback(const ros::TimerEvent& )
    {
        map_pub_.publish(cloud_msg_);
    }


private:
    ros::NodeHandle nh_;
    ros::Timer vis_timer_;    
    ros::Publisher map_pub_;
    pcl::PointCloud<pcl::PointXYZ> map_cloud_;
    sensor_msgs::PointCloud2 cloud_msg_;
    pcl::PointXYZ pt_;

public:    
    double resolution_;     // 分辨率
    double resolution_inv_;
    Eigen::Vector3d map_size_;   // 地图大小
    Eigen::Vector3d map_max_boundary_;  // 地图边界
    Eigen::Vector3d map_min_boundary_;  // 地图边界
    Eigen::Vector3i map_origin_id_; // 地图原点
    Eigen::Vector3i grid_map_size_;     // 栅格地图大小
    Eigen::MatrixXi gridMap_;   // 地图存放高度
    double scale_ = 0.01;  // 控制 Perlin 噪声的频率
    
};


