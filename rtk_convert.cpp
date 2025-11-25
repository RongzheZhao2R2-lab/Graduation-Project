#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Eigen>

ros::Publisher odom_FLU_pub;

nav_msgs::Odometry odom;
Eigen::Vector3d position_enu;

double roll, pitch, yaw;

void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    odom = *msg;
    Eigen::Vector3d vehicle_p(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
    odom.pose.pose.position.x = vehicle_p.x();
    odom.pose.pose.position.y = vehicle_p.y();
    odom.pose.pose.position.z = vehicle_p.z();

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    
    // 将四元数转换为欧拉角
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    pitch = -pitch;
    tf::Quaternion quat_ = tf::createQuaternionFromRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion quat_msg;
    tf::quaternionTFToMsg(quat_, quat_msg);

    odom.pose.pose.orientation = quat_msg;

    odom.header.frame_id = "map";
    odom.header.stamp = ros::Time::now();
    ROS_INFO_STREAM("yaw is " << yaw * 180 / M_PI);
    odom_FLU_pub.publish(odom);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cuadc_odom");
    Eigen::AngleAxisd rotation_vector(-M_PI_2, Eigen::Vector3d(0, 0, 1));
    Eigen::Quaterniond q_(rotation_vector);
    ros::NodeHandle nh;
    ros::Subscriber qfrtk_sub = nh.subscribe<nav_msgs::Odometry>("/navOdometry", 1, &odomCallback, ros::TransportHints().tcpNoDelay());
    odom_FLU_pub = nh.advertise<nav_msgs::Odometry>("flu/odometry", 1);
    ros::spin();
    return 0;
}
