#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>

#include "map_generate/map_generate.hpp"
#include "rrt/rrt.hpp"

map_generatePtr map_ptr;
RRT_Ptr rrt_ptr;
ros::Publisher wps_pub, path_pub, start_goal_pub, new_node_pub, rrt_tree_pub;

#ifdef RRT_DEBUG
ros::Publisher ray_cast_pub;
#endif

Eigen::Vector3d start, goal;

void visualize(const std::vector<Eigen::Vector3d> &path, const std::vector<Eigen::Vector3d> &sample, 
const std::vector<Eigen::Vector3d> &x_new_set, const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> edges);

bool goal_sign = true;
std::vector<Eigen::Vector3d> path, sample, node_set;
std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> edges;

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal_ptr)
{
    if(goal_sign)
    {
        start << goal_ptr->pose.position.x, goal_ptr->pose.position.y, goal_ptr->pose.position.z;
        ROS_INFO_STREAM("start : \n" << start);
        goal_sign = false;
    }
    else
    {
        goal << goal_ptr->pose.position.x, goal_ptr->pose.position.y, goal_ptr->pose.position.z;
        ROS_INFO_STREAM("goal : \n" << goal);
        ROS_INFO_STREAM("Distance : " << (goal - start).norm());
        goal_sign = true;
    }

    if(!goal_sign)
        return;
    // 搜索
    bool ret = rrt_ptr->RRTSearch(start, goal, path, sample);
    ROS_INFO_STREAM("ret " << ret);
    rrt_ptr->sampleWholeTree(node_set, edges);
#ifdef RRT_DEBUG
    pcl::PointCloud<pcl::PointXYZ> ray_cast_path_test;
    pcl::PointXYZ tmp;
    for(auto &p : rrt_ptr->ray_cast_path)
    {
        tmp.x = p.x();
        tmp.y = p.y();
        tmp.z = p.z();
        ray_cast_path_test.push_back(tmp);
    }
    ray_cast_path_test.width = ray_cast_path_test.points.size();
    ray_cast_path_test.height = 1.0;
    ray_cast_path_test.is_dense = true;
    ray_cast_path_test.header.frame_id = "map";
    sensor_msgs::PointCloud2 ray_cast_cloud_msg;
    pcl::toROSMsg(ray_cast_path_test, ray_cast_cloud_msg);
    ray_cast_pub.publish(ray_cast_cloud_msg);
#endif

    // 可视化
    visualize(path, sample, node_set, edges);
    if(!ret)
    {
        ROS_INFO_STREAM("Search Failed!");
        return;
    }   
}

void visualize(const std::vector<Eigen::Vector3d> &path, const std::vector<Eigen::Vector3d> &sample, 
const std::vector<Eigen::Vector3d> &x_new_set, const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> edges)
{
    visualization_msgs::Marker routeMarker, samplePointsMarker, pathMarker, start_goalMarker, new_nodeMarker, rrt_treeMarker;
    routeMarker.id = 0;
    routeMarker.type = visualization_msgs::Marker::LINE_LIST;
    routeMarker.header.stamp = ros::Time::now();
    routeMarker.header.frame_id = "map";
    routeMarker.pose.orientation.w = 1.00;
    routeMarker.action = visualization_msgs::Marker::ADD;
    routeMarker.ns = "route";

    samplePointsMarker = routeMarker;
    samplePointsMarker.id = -samplePointsMarker.id - 1;
    samplePointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
    samplePointsMarker.ns = "waypoints";
    samplePointsMarker.color.r = 0.00;
    samplePointsMarker.color.g = 1.00;
    samplePointsMarker.color.b = 0.00;
    samplePointsMarker.color.a = 1.00;
    samplePointsMarker.scale.x = 0.35;
    samplePointsMarker.scale.y = 0.35;
    samplePointsMarker.scale.z = 0.35;

    pathMarker = routeMarker;
    pathMarker.header.frame_id = "map";
    pathMarker.id = 0;
    pathMarker.ns = "path";
    pathMarker.color.r = 1.00;
    pathMarker.color.g = 1.00;
    pathMarker.color.b = 1.00;
    pathMarker.color.a = 1.00;
    pathMarker.scale.x = 0.20;

    start_goalMarker = routeMarker;
    start_goalMarker.id = -samplePointsMarker.id - 1;
    start_goalMarker.type = visualization_msgs::Marker::SPHERE_LIST;
    start_goalMarker.ns = "start_goal";
    start_goalMarker.color.r = 0.50;
    start_goalMarker.color.g = 0.00;
    start_goalMarker.color.b = 0.80;
    start_goalMarker.color.a = 1.00;
    start_goalMarker.scale.x = 1.0;
    start_goalMarker.scale.y = 1.0;
    start_goalMarker.scale.z = 1.0;

    new_nodeMarker = routeMarker;
    new_nodeMarker.id = -samplePointsMarker.id - 1;
    new_nodeMarker.type = visualization_msgs::Marker::SPHERE_LIST;
    new_nodeMarker.ns = "new_node";
    new_nodeMarker.color.r = 0.95;
    new_nodeMarker.color.g = 0.80;
    new_nodeMarker.color.b = 0.70;
    new_nodeMarker.color.a = 1.00;
    new_nodeMarker.scale.x = 0.15;
    new_nodeMarker.scale.y = 0.15;
    new_nodeMarker.scale.z = 0.15;

    rrt_treeMarker = routeMarker;
    rrt_treeMarker.header.frame_id = "map";
    rrt_treeMarker.id = 0;
    rrt_treeMarker.ns = "rrt_tree";
    rrt_treeMarker.color.r = 1.00;
    rrt_treeMarker.color.g = 0.00;
    rrt_treeMarker.color.b = 0.00;
    rrt_treeMarker.color.a = 0.80;
    rrt_treeMarker.scale.x = 0.05;

    geometry_msgs::Point s_p, g_p;
    s_p.x = start.x(), s_p.y = start.y(), s_p.z = start.z();
    g_p.x = goal.x(), g_p.y = goal.y(), g_p.z = goal.z();
    start_goalMarker.points.push_back(s_p);
    start_goalMarker.points.push_back(g_p);
    start_goal_pub.publish(start_goalMarker);

    if(x_new_set.size() > 0)
    {
        for(size_t i = 0; i < x_new_set.size(); ++i)
        {
            geometry_msgs::Point p;
            p.x = x_new_set[i].x();
            p.y = x_new_set[i].y();
            p.z = x_new_set[i].z();
            new_nodeMarker.points.push_back(p);
        }
        new_node_pub.publish(new_nodeMarker);
    }

    if(edges.size() > 0)
    {
        geometry_msgs::Point p1, p2;
        for(const auto &e : edges)
        {
            p1.x = e.first.x(), p2.x = e.second.x();
            p1.y = e.first.y(), p2.y = e.second.y();
            p1.z = e.first.z(), p2.z = e.second.z();
            rrt_treeMarker.points.push_back(p1);
            rrt_treeMarker.points.push_back(p2);
        }
        rrt_tree_pub.publish(rrt_treeMarker);
    }

    if(sample.size() > 0)
    {
        for(size_t i = 0; i < sample.size(); ++i)
        {
            geometry_msgs::Point point;
            point.x = sample[i].x();
            point.y = sample[i].y();
            point.z = sample[i].z();
            samplePointsMarker.points.push_back(point);
        }
        wps_pub.publish(samplePointsMarker);
    }

    if(path.size() > 0)
    {
        Eigen::Vector3d lastX = path[0];
        for(size_t i = 1; i < path.size(); ++i)
        {
            geometry_msgs::Point point; 
            point.x = lastX.x();
            point.y = lastX.y();
            point.z = lastX.z();
            pathMarker.points.push_back(point);

            point.x = path[i].x();
            point.y = path[i].y();
            point.z = path[i].z();
            pathMarker.points.push_back(point);
            lastX = path[i];
        }
        path_pub.publish(pathMarker);
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rrt_map_gen_test");
    ros::NodeHandle nh("~");
    map_ptr = std::make_shared<map_generate>(nh);

    Eigen::Vector3d map_size(60, 60, 30);
    double resolution = 0.15;
    map_ptr->SetParam(map_size, resolution, 0.07);
    map_ptr->generateHeightMap();
    map_ptr->generateGridMap();
    map_ptr->saveHeightmap("/home/zrz/Aerial_Robotics/src/a_star/config/rrt_test.png");

    rrt_ptr = std::make_shared<RRT>();
    rrt_ptr->initSimRRT(map_ptr, 2000);

    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/goal", 10, goalCallback, ros::TransportHints().tcpNoDelay());

    wps_pub = nh.advertise<visualization_msgs::Marker>("visualizer/waypoints", 10);
    path_pub = nh.advertise<visualization_msgs::Marker>("visualizer/curve", 10);
    start_goal_pub = nh.advertise<visualization_msgs::Marker>("visualizer/start_gaol", 10);
    new_node_pub = nh.advertise<visualization_msgs::Marker>("visualizer/new_node", 10);
    rrt_tree_pub = nh.advertise<visualization_msgs::Marker>("visualizer/explore_edge", 10);
#ifdef RRT_DEBUG
    ray_cast_pub = nh.advertise<sensor_msgs::PointCloud2>("ray_cast_path_test", 10);
#endif // DEBUG RRT_DEBUG

    ros::spin();

    return 0;
}

