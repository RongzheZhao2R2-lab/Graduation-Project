#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <Eigen/Eigen>

ros::Publisher servo_control_pub;

geometry_msgs::Point body_target;
Eigen::Vector2d target;

std_msgs::Bool sign;

void BodyTargetCallback(const geometry_msgs::PointConstPtr &msg)
{
    body_target = *msg;
    target << body_target.x, body_target.y;
    ROS_INFO_STREAM("target.norm() " << target.norm());
    if(target.norm() < 0.02)
    {
        sign.data = true;
        servo_control_pub.publish(sign);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cuadc_test");
    ros::NodeHandle nh;
    servo_control_pub = nh.advertise<std_msgs::Bool>("ServoControl", 1);
    ros::Subscriber ellipse_sub_ = nh.subscribe<geometry_msgs::Point>("body/target", 1, &BodyTargetCallback, ros::TransportHints().tcpNoDelay());

    ros::spin();
    return 0;
}
