#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include <iostream>
#include <memory>
#include <chrono>
#include <cmath>

#include <ros/ros.h>
#include <minco_trajectory/polynomial_spline.hpp>
#include "minco_trajectory/rfs_ego.hpp"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Visualizer for the planner
class Visualizer
{
private:
    // config contains the scale for some markers
    ros::NodeHandle nh;

    ros::Publisher wayPointsPub;
    ros::Publisher curvePub;
    ros::Publisher diskPub;
    ros::Publisher robot_modelPub;

public:
    Visualizer(ros::NodeHandle &nh_)
        : nh(nh_)
    {
        wayPointsPub = nh.advertise<visualization_msgs::Marker>("/visualizer/waypoints", 10);
        curvePub = nh.advertise<visualization_msgs::Marker>("/visualizer/curve", 10);
        diskPub = nh.advertise<visualization_msgs::MarkerArray>("/visualizer/disk", 1000);
        robot_modelPub = nh.advertise<visualization_msgs::MarkerArray>("/vehicle/pose", 1000);
    }

    inline void visualize(const Spline::PolynomialSpline &curve)
    {
        visualization_msgs::Marker routeMarker, wayPointsMarker, trajMarker;

        routeMarker.id = 0;
        routeMarker.type = visualization_msgs::Marker::LINE_LIST;
        routeMarker.header.stamp = ros::Time::now();
        routeMarker.header.frame_id = "map";
        routeMarker.pose.orientation.w = 1.00;
        routeMarker.action = visualization_msgs::Marker::ADD;
        routeMarker.ns = "route";
        routeMarker.color.r = 1.00;
        routeMarker.color.g = 0.00;
        routeMarker.color.b = 0.00;
        routeMarker.color.a = 1.00;
        routeMarker.scale.x = 0.1;

        wayPointsMarker = routeMarker;
        wayPointsMarker.id = -wayPointsMarker.id - 1;
        wayPointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
        wayPointsMarker.ns = "waypoints";
        wayPointsMarker.color.r = 1.00;
        wayPointsMarker.color.g = 0.00;
        wayPointsMarker.color.b = 0.00;
        wayPointsMarker.scale.x = 0.35;
        wayPointsMarker.scale.y = 0.35;
        wayPointsMarker.scale.z = 0.35;

        trajMarker = routeMarker;
        trajMarker.header.frame_id = "map";
        trajMarker.id = 0;
        trajMarker.ns = "trajectory";
        trajMarker.color.r = 0.00;
        trajMarker.color.g = 0.50;
        trajMarker.color.b = 1.00;
        trajMarker.scale.x = 0.20;
        if (curve.getPieceNum() > 0)
        {
            Eigen::MatrixXd wps = curve.getAllPoint();
            for (int i = 0; i < wps.cols(); i++)
            {
                geometry_msgs::Point point;
                point.x = wps.col(i)(0);
                point.y = wps.col(i)(1);
                if(curve.getDimensions() == 2)
                    point.z = 0.0;
                else
                    point.z = wps.col(i)(2);
                wayPointsMarker.points.push_back(point);
            }

            wayPointsPub.publish(wayPointsMarker);
        }
        if (curve.getPieceNum() > 0)
        {
            double T = 0.01;
            Eigen::VectorXd lastX = curve.getState(0.0);
            for (double t = T; t < curve.getTotalDuration(); t += T)
            {
                geometry_msgs::Point point;
                Eigen::VectorXd X = curve.getState(t);
                point.x = lastX(0);
                point.y = lastX(1);
                if(curve.getDimensions() == 2)
                    point.z = 0.0;
                else
                    point.z = lastX(2);
                trajMarker.points.push_back(point);

                point.x = X(0);
                point.y = X(1);
                if(curve.getDimensions() == 2)
                    point.z = 0.0;
                else
                    point.z = X(2);
                trajMarker.points.push_back(point);
                lastX = X;
            }
            curvePub.publish(trajMarker);
        }
    }

inline void visualize(const RFS_EGO &rfs_ego)
    {
        visualization_msgs::Marker routeMarker, wayPointsMarker, trajMarker;

        routeMarker.id = 0;
        routeMarker.type = visualization_msgs::Marker::LINE_LIST;
        routeMarker.header.stamp = ros::Time::now();
        routeMarker.header.frame_id = "map";
        routeMarker.pose.orientation.w = 1.00;
        routeMarker.action = visualization_msgs::Marker::ADD;
        routeMarker.ns = "route";
        routeMarker.color.r = 1.00;
        routeMarker.color.g = 0.00;
        routeMarker.color.b = 0.00;
        routeMarker.color.a = 1.00;
        routeMarker.scale.x = 0.1;

        wayPointsMarker = routeMarker;
        wayPointsMarker.id = -wayPointsMarker.id - 1;
        wayPointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
        wayPointsMarker.ns = "waypoints";
        wayPointsMarker.color.r = 1.00;
        wayPointsMarker.color.g = 0.00;
        wayPointsMarker.color.b = 0.00;
        wayPointsMarker.scale.x = 0.35;
        wayPointsMarker.scale.y = 0.35;
        wayPointsMarker.scale.z = 0.35;

        trajMarker = routeMarker;
        trajMarker.header.frame_id = "map";
        trajMarker.id = 0;
        trajMarker.ns = "trajectory";
        trajMarker.color.r = 0.00;
        trajMarker.color.g = 0.50;
        trajMarker.color.b = 1.00;
        trajMarker.scale.x = 0.20;
        if (rfs_ego.minco_ptr_->getPieceNum() > 0)
        {
            Eigen::MatrixXd wps = rfs_ego.minco_ptr_->getAllPoint();
            for (int i = 0; i < wps.cols(); i++)
            {
                geometry_msgs::Point point;
                point.x = wps.col(i)(0);
                point.y = wps.col(i)(1);
                if(rfs_ego.minco_ptr_->getDimensions() == 2)
                    point.z = rfs_ego.a_star_ptr_->getHeight();
                else
                    point.z = wps.col(i)(2);
                wayPointsMarker.points.push_back(point);
            }

            wayPointsPub.publish(wayPointsMarker);
        }
        if (rfs_ego.minco_ptr_->getPieceNum() > 0)
        {
            double T = 0.01;
            Eigen::VectorXd lastX = rfs_ego.minco_ptr_->getState(0.0);
            for (double t = T; t < rfs_ego.minco_ptr_->getTotalDuration(); t += T)
            {
                geometry_msgs::Point point;
                Eigen::VectorXd X = rfs_ego.minco_ptr_->getState(t);
                point.x = lastX(0);
                point.y = lastX(1);
                if(rfs_ego.minco_ptr_->getDimensions() == 2)
                    point.z = rfs_ego.a_star_ptr_->getHeight();
                else
                    point.z = lastX(2);
                trajMarker.points.push_back(point);

                point.x = X(0);
                point.y = X(1);
                if(rfs_ego.minco_ptr_->getDimensions() == 2)
                    point.z = rfs_ego.a_star_ptr_->getHeight();
                else
                    point.z = X(2);
                trajMarker.points.push_back(point);
                lastX = X;
            }
            curvePub.publish(trajMarker);
        }
    }

    inline void visualizeDisks(const Eigen::MatrixXd &disk)
    {
        visualization_msgs::Marker diskMarker;
        visualization_msgs::MarkerArray diskMarkers;
        
        if(3 == disk.rows())
            diskMarker.type = visualization_msgs::Marker::CYLINDER;
        else
            diskMarker.type = visualization_msgs::Marker::SPHERE;
        diskMarker.header.stamp = ros::Time::now();
        diskMarker.header.frame_id = "map";
        diskMarker.pose.orientation.w = 1.00;
        diskMarker.action = visualization_msgs::Marker::ADD;
        diskMarker.ns = "disk";
        diskMarker.color.r = 1.00;
        diskMarker.color.g = 0.00;
        diskMarker.color.b = 0.00;
        diskMarker.color.a = 1.00;

        for (int i = 0; i < disk.cols(); ++i)
        {
            diskMarker.id = i;
            diskMarker.pose.position.x = disk.col(i).x();
            diskMarker.pose.position.y = disk.col(i).y();
            if(disk.rows() == 3)
            {
                diskMarker.pose.position.z = 0.5;
                diskMarker.scale.x = disk.col(i).z() * 2.0;
                diskMarker.scale.y = disk.col(i).z() * 2.0;
                diskMarker.scale.z = 1.0;
            }
            else
            {
                diskMarker.pose.position.z = disk.col(i).z();
                diskMarker.scale.z = disk.col(i)(3) * 2.0;
                diskMarker.scale.x = disk.col(i)(3) * 2.0;
                diskMarker.scale.y = disk.col(i)(3) * 2.0;
                diskMarker.scale.z = disk.col(i)(3) * 2.0;
            }
            diskMarkers.markers.push_back(diskMarker);
        }

        diskPub.publish(diskMarkers);
    }

    inline void visualizeVehicle(const std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> &pose, const std::string &path)
    {
        if (pose.empty()) 
            return;

        // **先发布一个空的 MarkerArray 来清空旧模型**
        visualization_msgs::MarkerArray empty_markers;
        visualization_msgs::Marker clear_marker;
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        empty_markers.markers.push_back(clear_marker);
        robot_modelPub.publish(empty_markers);
        ros::Duration(0.1).sleep();  // **稍作延时，确保 RViz 清除完成**

        visualization_msgs::MarkerArray robot_model_set;
        visualization_msgs::Marker robot_model;
        robot_model.type = visualization_msgs::Marker::MESH_RESOURCE;
        robot_model.action = visualization_msgs::Marker::ADD;
        robot_model.mesh_resource = path;
        robot_model.mesh_use_embedded_materials = true;
        robot_model.header.frame_id = "map";
        robot_model.ns = "visulizer";
        robot_model.scale.x = 2;
        robot_model.scale.y = 2;
        robot_model.scale.z = 2;
        robot_model.header.stamp = ros::Time::now();
        for(size_t i = 0; i < pose.size(); ++i)
        {
            robot_model.id = i;

            robot_model.scale.x = 1;
            robot_model.scale.y = 1;
            robot_model.scale.z = 1;

            if(i > 0)
                if(i < pose.size() - 1)
                    if((pose[i].first - pose[i + 1].first).norm() < 0.15)
                        continue;
                else
                    if((pose[i].first - pose[i - 1].first).norm() < 0.15)
                        continue;

            if (pose[i].second.norm() == 0)
            {
                continue;
            }
            robot_model.pose.orientation.x = pose[i].second.x();
            robot_model.pose.orientation.y = pose[i].second.y();
            robot_model.pose.orientation.z = pose[i].second.z();
            robot_model.pose.orientation.w = pose[i].second.w();

            robot_model.pose.position.x = pose[i].first.x();
            robot_model.pose.position.y = pose[i].first.y();
            robot_model.pose.position.z = pose[i].first.z();

            robot_model_set.markers.push_back(robot_model);
        }

        robot_modelPub.publish(robot_model_set);
    }
};

#endif