#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <fsm_msgs/fsm.h>
#include <std_msgs/Bool.h>
#include <cuadc/paramtersConfig.hpp>
#include <cuadc/cuadc.h>

#include <Eigen/Eigen>

std::string state_str[7] = {"INIT", "TAKEOFF", "HOVER", "EXEC_WAYPOINT", "EXEC_VISION", "RETURN", "LANDING"};

fsm_msgs::fsm exec_state;
mavros_msgs::State current_state;  
mavros_msgs::PositionTarget setpoint_raw;

geometry_msgs::PoseStamped home_position_;
Eigen::Vector3d _home_position_;
std_msgs::Bool home_position_sign_;

geometry_msgs::PoseStamped vehicle_pose_;
Eigen::Quaterniond _vehicle_pose_;
Eigen::Matrix3d R_enu_flu_;
bool vehicle_pose_sign_ = false;
// 将四元数转换为欧拉角
double roll, pitch, yaw;

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void fsmState_cb(const fsm_msgs::fsmConstPtr &msg)
{
    exec_state = *msg;
    // ROS_INFO_STREAM("the FSM state is " << exec_state.fsm_state);
}

void HomePositionCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    home_position_ = *msg;
    _home_position_ << home_position_.pose.position.x, home_position_.pose.position.y, home_position_.pose.position.z;
    Eigen::Quaterniond q_enu_flu(home_position_.pose.orientation.w, home_position_.pose.orientation.x, home_position_.pose.orientation.y, home_position_.pose.orientation.z);
    R_enu_flu_ = q_enu_flu.toRotationMatrix();
}

void HomePositionSignCallback(const std_msgs::BoolConstPtr &msg)
{
    home_position_sign_ = *msg;
}

void poseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    vehicle_pose_ = *msg;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    
    // 将四元数转换为欧拉角
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    Eigen::Quaterniond q(vehicle_pose_.pose.orientation.w, vehicle_pose_.pose.orientation.x, vehicle_pose_.pose.orientation.y, vehicle_pose_.pose.orientation.z);
    _vehicle_pose_ = q;
    vehicle_pose_sign_ = true;
    // ROS_INFO_STREAM("yaw is " << yaw);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cuadc_takeoff");

    //创建句柄
    ros::NodeHandle nh_("~");

    // 获取参数
    CUADC::CUADCConfig config;
    ROS_INFO("read Paramters begin");
    config.getParamters(nh_);
    ROS_INFO("read Paramters end");

    home_position_sign_.data = false;

    ros::NodeHandle nh;

    //订阅无人机状态话题
	ros::Subscriber state_sub              = nh.subscribe<mavros_msgs::State>("mavros/state", 100, state_cb);

    ros::Subscriber fsm_state_sub          = nh.subscribe<fsm_msgs::fsm>("cuadc/fsm", 1, fsmState_cb);

    // //请求无人机解锁服务        
	ros::ServiceClient arming_client       = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
		
	// //请求无人机设置飞行模式，本代码请求进入offboard
	ros::ServiceClient set_mode_client     = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // 起飞点
    ros::Subscriber home_position_sub      = nh.subscribe<geometry_msgs::PoseStamped>("cuadc/home_position", 1, &HomePositionCallback, ros::TransportHints().tcpNoDelay());

    ros::Subscriber home_position_sign_sub = nh.subscribe<std_msgs::Bool>("cuadc/odom/sign", 1, &HomePositionSignCallback, ros::TransportHints().tcpNoDelay());

    ros::Subscriber pose_sub               = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, &poseCallback, ros::TransportHints().tcpNoDelay());

    // 起飞
    ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 100); 
    
    ros::Rate rate(20.0); 

    // 等待起飞点
    while(!home_position_sign_.data)
    {
        ROS_INFO("Wait for Home Position!");
        ros::spinOnce();
        rate.sleep();
    }

    //等待连接到PX4无人机
    while(ros::ok() && !current_state.connected)
    {
        ROS_INFO("Ready to fly");
        ros::spinOnce();
        rate.sleep();
    }

    while(!vehicle_pose_sign_)
        ROS_INFO("wait for vehicle pose");
           
    setpoint_raw.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | 
                              mavros_msgs::PositionTarget::IGNORE_VY |
                              mavros_msgs::PositionTarget::IGNORE_VZ |
                              mavros_msgs::PositionTarget::IGNORE_AFX |
                              mavros_msgs::PositionTarget::IGNORE_AFY |
                              mavros_msgs::PositionTarget::IGNORE_AFZ |
                              mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

	setpoint_raw.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    Eigen::Vector3d control_point(config.copter.hover_wayPoint.at(0), config.copter.hover_wayPoint.at(1), config.copter.hover_wayPoint.at(2));
    control_point = R_enu_flu_ * control_point  + _home_position_;
    setpoint_raw.position.x = control_point.x();
    setpoint_raw.position.y = control_point.y();
    setpoint_raw.position.z = control_point.z();
    setpoint_raw.yaw = yaw;

    for(int i = 100; ros::ok() && i > 0; --i)
    {
		mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }

     //请求offboard模式变量
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
 
    //请求解锁变量
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    
    //请求进入offboard模式并且解锁无人机，15秒后退出，防止重复请求       
    while(ros::ok())
    {
    	//请求进入OFFBOARD模式
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
           	last_request = ros::Time::now();
       	}
        else 
		{
			//请求解锁
			if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
			{
		        if( arming_client.call(arm_cmd) && arm_cmd.response.success)
		       	{
		            ROS_INFO("Vehicle armed");
		        }
		        	last_request = ros::Time::now();
			}
		}
	    if(ros::Time::now() - last_request > ros::Duration(5.0))
	    	break;
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }   

    while(ros::ok())
    {
        if(state_str[(int)CUADC::CUADC_MultiCopter_FSM::TAKEOFF] == exec_state.fsm_state)
        {
            mavros_setpoint_pos_pub.publish(setpoint_raw);
            ROS_INFO_STREAM("Control the CUADC MultiCopter FSM state is " << exec_state.fsm_state);
        }
        else if(state_str[(int)CUADC::CUADC_MultiCopter_FSM::LANDING] == exec_state.fsm_state)
        {
            ROS_INFO_STREAM("Control the CUADC MultiCopter FSM state is " << exec_state.fsm_state);
            offb_set_mode.request.custom_mode = "AUTO.LAND";
            if (current_state.mode != "AUTO.LAND" /*&& (ros::Time::now() - last_request > ros::Duration(5.0))*/)
            {
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("AUTO.LAND enabled");
                }
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    ros::spin();
    return 0;
}