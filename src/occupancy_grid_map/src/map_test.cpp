#include <ros/ros.h>
#include "occupancy_grid_map/raycast.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Eigen>

double resolution = 0.15;
double resolution_inv_ = 1.0 / resolution;

void raycast(const Eigen::Vector3d &start, const Eigen::Vector3d &end, std::vector<Eigen::Vector3d> &ray)
{  
    Eigen::Vector3d direct = (end - start).normalized();
    Eigen::Vector3d ray_point;
    double distance = (end - start).norm();
    int piece_num = floor(distance / (resolution * 9.5));
    ROS_INFO_STREAM("piece number is " << piece_num);
    for(int i = 0; i < piece_num; ++i)
    {
        // if((ray_point - end).norm() < 0.01)
        // {
        //     ROS_INFO("break");
        //     break;
        // }
        ray_point = i * resolution * 9.5 * direct;
        ray.push_back(ray_point);
    }
    return;
}


void getRayIndices(const Eigen::Vector3i& start, const Eigen::Vector3i& end, std::vector<Eigen::Vector3i>& rayIndices) {
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "occupancy_test");
    ros::NodeHandle nh;
    ros::Publisher test_pub = nh.advertise<sensor_msgs::PointCloud2>("occupancy_grid_map/test", 10);
    RayCaster raycaster;

    Eigen::Vector3d pt_w(10, 5, 5);
    Eigen::Vector3d start(0, 0, 0);
    Eigen::Vector3d max(100.0, 100.0, 100.0);
    Eigen::Vector3d min(-100.0, -100.0, -100.0);
    std::vector<Eigen::Vector3d> ray_point_set;
    

    Raycast(pt_w / resolution, start / resolution, min / resolution, max / resolution, ray_point_set);
    // raycast(start * resolution_inv_, pt_w * resolution_inv_, ray_point_set);

    Eigen::Vector3i start_idx(0, 0, 0);
    Eigen::Vector3i pt_w_idx(floor(pt_w(0) / resolution), floor(pt_w(1) / resolution), floor(pt_w(2) / resolution));
    std::vector<Eigen::Vector3i> ray;
    getRayIndices(start_idx, pt_w_idx, ray);
    Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
    Eigen::Vector3d p;
    
    
    ROS_INFO_STREAM("pt_w * resolution_inv_ " << pt_w * resolution_inv_);
    ROS_INFO_STREAM("start * resolution_inv_ " << start * resolution_inv_);

    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud_;

    // ROS_INFO_STREAM("ray_point_set size is " << ray_point_set.size());

    for(auto &p:ray)
    {
        pt.x = (p.x() * 1.0 + 0.5) * resolution;
        pt.y = (p.y() * 1.0 + 0.5) * resolution;
        pt.z = (p.z() * 1.0 + 0.5) * resolution;
        cloud_.push_back(pt);
    }

    // for(uint32_t i = 0; i < ray_point_set.size(); ++i)
    // {
    //     p = ray_point_set.at(i);
    //     p(0) = floor(p(0));
    //     p(1) = floor(p(1));
    //     p(2) = floor(p(2));
    //     p += half;
    //     p = resolution * p;
    //     pt.x = p.x();
    //     pt.y = p.y();
    //     pt.z = p.z();
    //     cloud_.push_back(pt);
    // }

    // Eigen::Vector3d ray_pt;
    // RayCaster raycaster2;
    // raycaster2.setInput(pt_w / resolution, start / resolution);
    // while(raycaster2.step(ray_pt))
    // {
    //     Eigen::Vector3d tmp = (ray_pt + half) * resolution;
    //     pt.x = tmp.x();
    //     pt.y = tmp.y();
    //     pt.z = tmp.z();
    //     cloud_.push_back(pt);
    // }
    cloud_.width = cloud_.points.size();
    cloud_.height = 1;
    cloud_.is_dense = true;
    cloud_.header.frame_id = "map";
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud_, cloud_msg);
    ros::Rate rate(30);
    while(ros::ok()){
        test_pub.publish(cloud_msg);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

