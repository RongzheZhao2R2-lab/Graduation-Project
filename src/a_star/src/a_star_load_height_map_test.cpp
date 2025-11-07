#include "a_star/a_star.h"
#include "map_generate/map_generate.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/opencv.hpp>

map_generatePtr map_ptr;
A_StarPtr a_star_ptr;
ros::Publisher path_pub, rdp_path_pub;

std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> path;
std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> simplified;
pcl::PointXYZ p_tmp;

Eigen::Vector3d start, goal;

bool goal_sign = true;
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal_ptr)
{
    if(goal_sign)
    {
        start << goal_ptr->pose.position.x, goal_ptr->pose.position.y, goal_ptr->pose.position.z;
        Eigen::Vector3i id;
        map_ptr->pos2Index_global(start, id);
        ROS_INFO_STREAM("startIdx : \n" << id);
        goal_sign = false;
    }
    else
    {
        goal << goal_ptr->pose.position.x, goal_ptr->pose.position.y, goal_ptr->pose.position.z;
        Eigen::Vector3i id;
        map_ptr->pos2Index_global(goal, id);
        ROS_INFO_STREAM("goalIdx : \n" << id);
        goal_sign = true;
    }

    if(!goal_sign)
        return;
    ROS_INFO_STREAM("start is (" << start.x() <<", " << start.y() << ", " << start.z() << ").");
    ROS_INFO_STREAM("goal is (" << goal.x() <<", " << goal.y() << ", " << goal.z() << ").");
    ROS_INFO_STREAM("distance is : " << (start - goal).norm());
    double cost_time = 0.0;
    ASTAR_RET ret = a_star_ptr->AstarGraphSearch(start, goal, A_Star::A_Star::DIALOG_TIEBREAKER, cost_time);
    ROS_INFO_STREAM("The cost time of A* is " << cost_time << "ms");
    if(ret == ASTAR_RET::SUCCESS)
        a_star_ptr->getPath(path);
    else
        return;

    Eigen::Vector3d pos;
    pcl::PointCloud<pcl::PointXYZ> path_test;
    if(path_pub.getNumSubscribers() > 0)
    {
        for(auto &p : path)
        {
            map_ptr->index2Pos_global(p, pos);
            p_tmp.x = pos.x();
            p_tmp.y = pos.y();
            p_tmp.z = pos.z();
            path_test.push_back(p_tmp);
        }
        path_test.width = path_test.points.size();
        path_test.height = 1;
        path_test.is_dense = true;
        path_test.header.frame_id = "map";
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(path_test, cloud_msg);
        path_pub.publish(cloud_msg);
    }
    auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>().swap(simplified);
    // a_star_ptr->rdpPath(path, 0.1, simplified);
    a_star_ptr->simplifyPath(path, simplified);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
    ROS_INFO_STREAM("simplified path time : " << duration_ns / 1000000.0 << "ms");
    pcl::PointCloud<pcl::PointXYZ> rdp_path_test;
    if(rdp_path_pub.getNumSubscribers() > 0)
    {
        for(auto &p : simplified)
        {
            map_ptr->index2Pos_global(p, pos);
            p_tmp.x = pos.x();
            p_tmp.y = pos.y();
            p_tmp.z = pos.z();
            rdp_path_test.push_back(p_tmp);
        }
        rdp_path_test.width = rdp_path_test.points.size();
        rdp_path_test.height = 1;
        rdp_path_test.is_dense = true;
        rdp_path_test.header.frame_id = "map";
        sensor_msgs::PointCloud2 rdp_cloud_msg;
        pcl::toROSMsg(rdp_path_test, rdp_cloud_msg);
        rdp_path_pub.publish(rdp_cloud_msg);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "a_star_map_generate");
    ros::NodeHandle nh("~");
    map_ptr = std::make_shared<map_generate>(nh);
    
    double resolution = 0.15;
    map_ptr->InitFromHeightMap("/home/zrz/Aerial_Robotics/src/a_star/config/heightmap.png", resolution, 30);
    map_ptr->generateGridMap();

    a_star_ptr = std::make_unique<A_Star>();
    a_star_ptr->initSimAstar(map_ptr, 20000, A_Star::SearchType::_3_DIEMS);
    // a_star_ptr->SetSimHeight(0);

    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/goal", 10, goalCallback, ros::TransportHints().tcpNoDelay());
    path_pub = nh.advertise<sensor_msgs::PointCloud2>("a_star_path_test", 10);
    rdp_path_pub = nh.advertise<sensor_msgs::PointCloud2>("a_star_rdp_path_test", 10);
    ros::spin();

    return 0;
}
