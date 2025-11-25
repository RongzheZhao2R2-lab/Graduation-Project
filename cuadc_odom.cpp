#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <Eigen/Eigen>
#include <tf/transform_datatypes.h>
#include <DBSCAN/dbscan.h>

nav_msgs::Odometry cuadc_odom;
ros::Publisher cuadc_odom_pub;
ros::Publisher cuadc_odom_sign_pub;

bool ds_home_sign;
ros::Publisher home_position_pub;
geometry_msgs::PoseStamped home_position;
Eigen::Vector3d p_local_odom, p_cuadc, t;

DBSCAN::DBSCAN ds_hp(200, 0.03);
DBSCAN::DBSCAN ds_yaw(200, 1 * M_PI / 180);
DBSCAN::Point tmp;
DBSCAN::Point yaw_tmp;
double roll, pitch, yaw;
double home_yaw;

nav_msgs::Odometry local_odom;

void chooseHomePosition(DBSCAN::DBSCAN &ds);

double chooseYaw(DBSCAN::DBSCAN &ds);

void localOdomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    local_odom = *msg;
    std_msgs::Bool hp_sign;
    p_local_odom << local_odom.pose.pose.position.x, local_odom.pose.pose.position.y, local_odom.pose.pose.position.z;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);


    if(!ds_home_sign)
    {
        tmp.x = local_odom.pose.pose.position.x;
        tmp.y = local_odom.pose.pose.position.y;
        tmp.z = local_odom.pose.pose.position.z;
        tmp.clusterID = UNCLASSIFIED;
        ds_hp.m_points.push_back(tmp);

        yaw_tmp.x = yaw;
        yaw_tmp.y = 0.0;
        yaw_tmp.z = 0.0;
        yaw_tmp.clusterID = UNCLASSIFIED;
        ds_yaw.m_points.push_back(yaw_tmp);
        
        if(ds_hp.m_points.size() > 500)
        {
            ds_hp.run();
            chooseHomePosition(ds_hp);
            ds_yaw.run();
            home_yaw = chooseYaw(ds_yaw);
            tf::Quaternion quat_ = tf::createQuaternionFromRPY(0, 0, home_yaw);
            geometry_msgs::Quaternion quat_msg;
            tf::quaternionTFToMsg(quat_, quat_msg);
            home_position.pose.orientation = quat_msg;
            t << home_position.pose.position.x, home_position.pose.position.y, home_position.pose.position.z;
            ds_home_sign = true;
        }
        hp_sign.data = false;
        cuadc_odom_sign_pub.publish(hp_sign);
    }
    else
    {
        // 发布起飞点
        home_position.header.frame_id = "map";
        home_position.header.stamp = ros::Time::now();
        home_position_pub.publish(home_position);

        // 发布里程计数据   ENU坐标系下的里程计数据
        cuadc_odom.child_frame_id = "base_link";
        cuadc_odom.header.frame_id = "map";
        cuadc_odom.header.stamp = ros::Time::now();
        cuadc_odom.twist = local_odom.twist;
        
        cuadc_odom.pose.pose.orientation = local_odom.pose.pose.orientation;
        p_cuadc = p_local_odom + t;
        cuadc_odom.pose.pose.position.x = p_cuadc.x();
        cuadc_odom.pose.pose.position.y = p_cuadc.y();
        cuadc_odom.pose.pose.position.z = p_cuadc.z();
        hp_sign.data = true;
        cuadc_odom_sign_pub.publish(hp_sign);
        cuadc_odom_pub.publish(cuadc_odom);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cuadc_odom");

    ds_home_sign = false;

    ros::NodeHandle nh;
    ros::Subscriber local_odom_sub = nh.subscribe<nav_msgs::Odometry>("mavros/local_position/odom", 1, &localOdomCallback, ros::TransportHints().tcpNoDelay());
    home_position_pub = nh.advertise<geometry_msgs::PoseStamped>("cuadc/home_position", 1);
    cuadc_odom_pub = nh.advertise<nav_msgs::Odometry>("cuadc/odom", 1);
    cuadc_odom_sign_pub = nh.advertise<std_msgs::Bool>("cuadc/odom/sign", 1);
    ros::spin();
    return 0;
}

void chooseHomePosition(DBSCAN::DBSCAN &ds)
{
    int lastcount = 0;
    int Max = 0;
    size_t i = 0;
    std::vector<int> point3d_index;
    std::vector<int> max_point3d;
    for(int j = 1; j <= (int)ds.getClusterPoint(); ++j)
    {
        lastcount = 0;
        point3d_index.clear();
        point3d_index.push_back(lastcount);
        for(i = 0; i < ds.m_points.size(); ++i)
        {
            if(ds.m_points.at(i).clusterID == j)
            {
                ++point3d_index.at(0);
                point3d_index.push_back(i);
            }
        }
        if(point3d_index.at(0) >= Max)
        {
            Max = point3d_index.at(0);
            max_point3d.swap(point3d_index);
        }
    }
	double x = 0, y = 0, z = 0;
	for(i = 1; i < max_point3d.size(); ++i)
	{
		x += ds.m_points.at(max_point3d.at(i)).x;
        y += ds.m_points.at(max_point3d.at(i)).y;
	}
	x /= (max_point3d.size() - 1);
    y /= (max_point3d.size() - 1);
    home_position.pose.position.x = x;
    home_position.pose.position.y = y;
    home_position.pose.position.z = z;
	ROS_INFO_STREAM("Home position is : " << home_position.pose.position.x << ", " << home_position.pose.position.y << ", " << home_position.pose.position.z);
}

double chooseYaw(DBSCAN::DBSCAN &ds)
{
    int lastcount = 0;
    int Max = 0;
    size_t i = 0;
    std::vector<int> yaw_index;
    std::vector<int> max_yaw;
    for(int j = 1; j <= (int)ds.getClusterPoint(); ++j)
    {
        lastcount = 0;
        yaw_index.clear();
        yaw_index.push_back(lastcount);
        for(i = 0; i < ds.m_points.size(); ++i)
        {
            if(ds.m_points.at(i).clusterID == j)
            {
                ++yaw_index.at(0);
                yaw_index.push_back(i);
            }
        }
        if(yaw_index.at(0) >= Max)
        {
            Max = yaw_index.at(0);
            max_yaw.swap(yaw_index);
        }
    }
	double yaw_average = 0;
	for(i = 1; i < max_yaw.size(); ++i)
	{
		yaw_average += ds.m_points.at(max_yaw.at(i)).x;
	}
	yaw_average /= (max_yaw.size() - 1);
	ROS_INFO_STREAM("average yaw is : " << yaw_average);
    return yaw_average;
}