#include "cuadc/cuadc.h"

namespace CUADC
{

inline void CUADC_MultiCopter::changeState(CUADC::CUADC_MultiCopter_FSM new_state)
{
    std::string state_str[7] = {"INIT", "TAKEOFF", "HOVER", "EXEC_WAYPOINT", "EXEC_VISION", "RETURN", "LANDING"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    fsm_state_.Header.stamp = ros::Time::now();
    fsm_state_.fsm_state = state_str[int(new_state)];
    ROS_INFO_STREAM("[STATE]: from " << state_str[pre_s] << " to " << state_str[int(new_state)]);
}

inline bool CUADC_MultiCopter::checkPoint(Eigen::Vector3d &vehile_poisition, Eigen::Vector3d &check_point, const double accuracy)
{
    if((vehile_poisition - check_point).norm() > accuracy)
        return false;
    return true;
}

inline double CUADC_MultiCopter::SpeedX(double vel_x)
{
	if(vel_x > 0)
		return vel_x = (vel_x > MaxSpeed_X) ? MaxSpeed_X : vel_x;
	else
		return vel_x = (vel_x < -MaxSpeed_X) ? -MaxSpeed_X : vel_x;
}

inline double CUADC_MultiCopter::SpeedY(double vel_y)
{
	if(vel_y > 0)
		return vel_y = (vel_y > MaxSpeed_Y) ? MaxSpeed_Y : vel_y;
	else
		return vel_y = (vel_y < -MaxSpeed_Y) ? -MaxSpeed_Y : vel_y;
}

inline double CUADC_MultiCopter::SpeedZ(double vel_z)
{
	if(vel_z > 0)
		return vel_z = (vel_z > MaxSpeed_Z) ? MaxSpeed_Z : vel_z;
	else
		return vel_z = (vel_z < -MaxSpeed_Z) ? -MaxSpeed_Z : vel_z;
}

Eigen::Vector3d CUADC_MultiCopter::visionBodyControl(Eigen::Vector3d &target_position)
{
    body_vision_pid_.err_x_P = target_position.x();
    body_vision_pid_.err_y_P = target_position.y();
    body_vision_pid_.err_z_P = config_.copter.vision_height - vehicle_pos_.pose.position.z;

    body_vision_pid_.increase_vel_x_P = body_vision_pid_.KP_X_P * (body_vision_pid_.err_x_P - body_vision_pid_.err_next_x_P) + 
                                        body_vision_pid_.KI_X_P * body_vision_pid_.err_x_P +
                                        body_vision_pid_.KD_X_P * (body_vision_pid_.err_x_P - 2 * body_vision_pid_.err_next_x_P + body_vision_pid_.err_last_x_P);
    body_vision_pid_.increase_vel_y_P = body_vision_pid_.KP_Y_P * (body_vision_pid_.err_y_P - body_vision_pid_.err_next_y_P) + 
                                        body_vision_pid_.KI_Y_P * body_vision_pid_.err_y_P +
                                        body_vision_pid_.KD_Y_P * (body_vision_pid_.err_y_P - 2 * body_vision_pid_.err_next_y_P + body_vision_pid_.err_last_y_P);
    body_vision_pid_.increase_vel_z_P = body_vision_pid_.KP_Z_P * (body_vision_pid_.err_z_P - body_vision_pid_.err_next_z_P) + 
                                        body_vision_pid_.KI_Z_P * body_vision_pid_.err_z_P +
                                        body_vision_pid_.KD_Z_P * (body_vision_pid_.err_z_P - 2 * body_vision_pid_.err_next_z_P + body_vision_pid_.err_last_z_P);
    
    exp_body_vel_.twist.linear.x += body_vision_pid_.increase_vel_x_P;
    exp_body_vel_.twist.linear.y += body_vision_pid_.increase_vel_y_P;
    exp_body_vel_.twist.linear.z += body_vision_pid_.increase_vel_z_P;

    body_vision_pid_.err_last_x_P = body_vision_pid_.err_next_x_P;
    body_vision_pid_.err_next_x_P = body_vision_pid_.err_x_P;

    body_vision_pid_.err_last_y_P = body_vision_pid_.err_next_y_P;
    body_vision_pid_.err_next_y_P = body_vision_pid_.err_y_P;
    
    body_vision_pid_.err_last_z_P = body_vision_pid_.err_next_z_P;
    body_vision_pid_.err_next_z_P = body_vision_pid_.err_z_P;

    exp_body_vel_.twist.linear.x = SpeedX(exp_body_vel_.twist.linear.x);
    exp_body_vel_.twist.linear.y = SpeedY(exp_body_vel_.twist.linear.y);
    exp_body_vel_.twist.linear.z = SpeedZ(exp_body_vel_.twist.linear.z);

    Eigen::Vector3d vel(exp_body_vel_.twist.linear.x, exp_body_vel_.twist.linear.y, exp_body_vel_.twist.linear.z);
    vel = _vehicle_pose_ * vel;
    return vel;
}

Eigen::Vector3d CUADC_MultiCopter::visionGlobalControl(const Eigen::Vector3d &vehicle_position, Eigen::Vector3d &target_position)
{
    global_vision_pid_.err_x_P = target_position.x() - vehicle_position.x();
    global_vision_pid_.err_y_P = target_position.y() - vehicle_position.y();
    global_vision_pid_.err_z_P = config_.copter.vision_height - vehicle_pos_.pose.position.z;

    global_vision_pid_.increase_vel_x_P = global_vision_pid_.KP_X_P * (global_vision_pid_.err_x_P - global_vision_pid_.err_next_x_P) + 
                                          global_vision_pid_.KI_X_P * global_vision_pid_.err_x_P +
                                          global_vision_pid_.KD_X_P * (global_vision_pid_.err_x_P - 2 * global_vision_pid_.err_next_x_P + global_vision_pid_.err_last_x_P);
    global_vision_pid_.increase_vel_y_P = global_vision_pid_.KP_Y_P * (global_vision_pid_.err_y_P - global_vision_pid_.err_next_y_P) + 
                                          global_vision_pid_.KI_Y_P * global_vision_pid_.err_y_P +
                                          global_vision_pid_.KD_Y_P * (global_vision_pid_.err_y_P - 2 * global_vision_pid_.err_next_y_P + global_vision_pid_.err_last_y_P);
    global_vision_pid_.increase_vel_z_P = global_vision_pid_.KP_Z_P * (global_vision_pid_.err_z_P - global_vision_pid_.err_next_z_P) + 
                                        global_vision_pid_.KI_Z_P * global_vision_pid_.err_z_P +
                                        global_vision_pid_.KD_Z_P * (global_vision_pid_.err_z_P - 2 * global_vision_pid_.err_next_z_P + global_vision_pid_.err_last_z_P);
    
    exp_global_vel_.twist.linear.x += global_vision_pid_.increase_vel_x_P;
    exp_global_vel_.twist.linear.y += global_vision_pid_.increase_vel_y_P;
    exp_global_vel_.twist.linear.z += global_vision_pid_.increase_vel_z_P;

    global_vision_pid_.err_last_x_P = global_vision_pid_.err_next_x_P;
    global_vision_pid_.err_next_x_P = global_vision_pid_.err_x_P;

    global_vision_pid_.err_last_y_P = global_vision_pid_.err_next_y_P;
    global_vision_pid_.err_next_y_P = global_vision_pid_.err_y_P;
    
    global_vision_pid_.err_last_z_P = global_vision_pid_.err_next_z_P;
    global_vision_pid_.err_next_z_P = global_vision_pid_.err_z_P;

    exp_global_vel_.twist.linear.x = SpeedX(exp_global_vel_.twist.linear.x);
    exp_global_vel_.twist.linear.y = SpeedY(exp_global_vel_.twist.linear.y);
    exp_global_vel_.twist.linear.z = SpeedZ(exp_global_vel_.twist.linear.z);

    Eigen::Vector3d vel(exp_global_vel_.twist.linear.x, exp_global_vel_.twist.linear.y, exp_global_vel_.twist.linear.z);
    
    vel = _vehicle_pose_ * vel;

    return vel;
}

void CUADC_MultiCopter::Init_paramters()
{
    ROS_INFO("Init_paramters");
    if(config_.copter.hover_wayPoint.size() % 4 != 0)
    {
        ROS_WARN("Please check your hover_wayPoint in paramters.yaml!");
        ROS_BREAK();
    }
    if(config_.copter.wayPoint.size() % 4 != 0)
    {
        ROS_WARN("Please check your wayPoint in paramters.yaml!");
        ROS_BREAK();
    }
    if(config_.copter.wayPoint.size() /4 != config_.copter.wayPointNum)
    {
        ROS_WARN("Please check your wayPointNum in paramters.yaml!");
        ROS_BREAK();
    }
    if(config_.copter.maxHeight < config_.copter.minHeight)
    {
        ROS_WARN("Please check your maxHeight in paramters.yaml!");
        ROS_BREAK();
    }
    if(config_.copter.maxHeight < 0 || config_.copter.minHeight < 0)
    {
        ROS_WARN("Please check your Height in paramters.yaml!");
        ROS_BREAK();
    }
    if(config_.copter.hover_accuracy < 0 || config_.copter.return_accuracy < 0 || config_.copter.exec_accuracy < 0)
    {
        ROS_WARN("Please check your accuracy in paramters.yaml!");
        ROS_BREAK();
    }
    if(config_.copter.vision_time < 0)
    {
        ROS_WARN("Please check your vision_time in paramters.yaml!");
        ROS_BREAK();
    }
    if(config_.copter.vision_height > config_.copter.maxHeight)
    {
        ROS_WARN("Please check your vision_height in paramters.yaml!");
        config_.copter.vision_height = config_.copter.maxHeight;
        ROS_WARN_STREAM("Set vision height height : " << config_.copter.maxHeight);
    }
    else if(config_.copter.vision_height < config_.copter.minHeight)
    {
        ROS_WARN("Please check your vision_height in paramters.yaml!");
        config_.copter.vision_height = config_.copter.minHeight;
        ROS_WARN_STREAM("Set hover wayPoint height : " << config_.copter.minHeight);
    }
    hover_wayPoint_ << config_.copter.hover_wayPoint.at(0), config_.copter.hover_wayPoint.at(1), config_.copter.hover_wayPoint.at(2), config_.copter.hover_wayPoint.at(3);
    if(hover_wayPoint_(2) > config_.copter.maxHeight)
    {
        ROS_WARN("Please check your hover_wayPoint z in paramters.yaml!");
        hover_wayPoint_(2) = config_.copter.maxHeight;
        ROS_WARN_STREAM("Set hover wayPoint height : " << config_.copter.maxHeight);
    }
    else if(hover_wayPoint_(2) < config_.copter.minHeight)
    {
        ROS_INFO_STREAM("hover_wayPoint_(2)" << hover_wayPoint_(2));
        ROS_WARN("Please check your hover_wayPoint z in paramters.yaml!");
        hover_wayPoint_(2) = config_.copter.minHeight;
        ROS_WARN_STREAM("Set hover wayPoint height : " << config_.copter.minHeight);
    }
    returnPoint_ << config_.copter.return_wayPoint.at(0), config_.copter.return_wayPoint.at(1), config_.copter.return_wayPoint.at(2), config_.copter.return_wayPoint.at(3);
    if(returnPoint_(2) > config_.copter.maxHeight)
    {
        ROS_WARN("Please check your returnPoint z in paramters.yaml!");
        returnPoint_(2) = config_.copter.maxHeight;
        ROS_WARN_STREAM("Set return wayPoint height : " << config_.copter.maxHeight);
    }
    else if(returnPoint_(2) < config_.copter.minHeight)
    {
        ROS_INFO_STREAM("returnPoint_(2)" << returnPoint_(2));
        ROS_WARN("Please check your returnPoint z in paramters.yaml!");
        returnPoint_(2) = config_.copter.minHeight;
        ROS_WARN_STREAM("Set return wayPoint height : " << config_.copter.minHeight);
    }

    // hover_wayPoint_ << config_.copter.hover_wayPoint.at(0), config_.copter.hover_wayPoint.at(1), config_.copter.hover_wayPoint.at(2), config_.copter.hover_wayPoint.at(3);
    // returnPoint_ << config_.copter.return_wayPoint.at(0), config_.copter.return_wayPoint.at(1), config_.copter.return_wayPoint.at(2), config_.copter.return_wayPoint.at(3);
    
    exp_body_vel_.twist.linear.x = 0, exp_body_vel_.twist.linear.y = 0, exp_body_vel_.twist.linear.z = 0;
    exp_global_vel_.twist.linear.x = 0, exp_global_vel_.twist.linear.y = 0, exp_global_vel_.twist.linear.z = 0;
    Eigen::Vector4d pos;
    for(uint32_t i = 0; i < config_.copter.wayPointNum; ++i)
    {
        if(config_.copter.wayPoint.at(i*4+2) > config_.copter.maxHeight)
            config_.copter.wayPoint.at(i*4+2) = config_.copter.maxHeight;
        else if(config_.copter.wayPoint.at(i*4+2) < config_.copter.minHeight)
            config_.copter.wayPoint.at(i*4+2) = config_.copter.minHeight;
        pos << config_.copter.wayPoint.at(i*4), config_.copter.wayPoint.at(i*4+1), config_.copter.wayPoint.at(i*4+2), config_.copter.wayPoint.at(i*4+3);
        wayPoint_.push_back(pos);
    }

    // 动态调参
    static dynamic_reconfigure::Server<cuadc::pidConfig> server_;
    static dynamic_reconfigure::Server<cuadc::pidConfig>::CallbackType func_;

    func_ = boost::bind(&CUADC_MultiCopter::PidTuning, this, _1);
    server_.setCallback(func_);

    R_enu_flu_ = Eigen::Matrix3d::Identity();

    home_position_sign_.data = false;
    body_ellipse_detect_sign_.data = false;
    global_ellipse_detect_sign_.data = false;
    MaxSpeed_X = config_.vision_point.MaxSpeed_X;
    MaxSpeed_Y = config_.vision_point.MaxSpeed_Y;

    control_piont_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    control_piont_.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | 
                              mavros_msgs::PositionTarget::IGNORE_VY |
                              mavros_msgs::PositionTarget::IGNORE_VZ |
                              mavros_msgs::PositionTarget::IGNORE_AFX |
                              mavros_msgs::PositionTarget::IGNORE_AFY |
                              mavros_msgs::PositionTarget::IGNORE_AFZ |
                              mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    vel_control_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    vel_control_.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | 
                              mavros_msgs::PositionTarget::IGNORE_VY |
                              mavros_msgs::PositionTarget::IGNORE_VZ |
                              mavros_msgs::PositionTarget::IGNORE_AFX |
                              mavros_msgs::PositionTarget::IGNORE_AFY |
                              mavros_msgs::PositionTarget::IGNORE_AFZ |
                              mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    fsm_state_.fsm_state = "INIT";
    exec_state_ = CUADC_MultiCopter_FSM::INIT;
    has_vehicle_pos_ = false;
    hover_time_sign_ = false;
    current_point_start_sign_ = false;
    return_point_start_sign_ = false;
    vision_time_sign_ = false;
    global_ellipse_sign_ = false;
    servo_time_sign_ = false;
    current_idx_ = 0;
}

void CUADC_MultiCopter::Init_subscriber()
{
    ROS_INFO("Init_subscriber");
    if(config_.vehicle_pose.useGPS_RTK)
        vehile_pose_sub = nh_.subscribe<nav_msgs::Odometry>(config_.vehicle_pose.subscribe_GPS_RTK_topic, 1, &CUADC_MultiCopter::GPS_RTK_vheiclePoseCallBack, this, ros::TransportHints().tcpNoDelay());
    else
        vehile_pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>(config_.vehicle_pose.subscribe_Vision, 1, &CUADC_MultiCopter::visionPoseCallBack, this, ros::TransportHints().tcpNoDelay());
    if(config_.vision_point.useBody)
        detect_point_body_sub = nh_.subscribe<geometry_msgs::Point>(config_.vision_point.subscribe_body_point, 1, &CUADC_MultiCopter::BodyPointCallBack, this, ros::TransportHints().tcpNoDelay());
    else if(config_.vision_point.useGlobal)
        detect_point_global_sub = nh_.subscribe<geometry_msgs::Point>(config_.vision_point.subscribe_Global_point, 1, &CUADC_MultiCopter::GlobalPointCallBack, this, ros::TransportHints().tcpNoDelay());
    else if(config_.vision_point.useMix)
    {
        detect_point_body_sub = nh_.subscribe<geometry_msgs::Point>(config_.vision_point.subscribe_body_point, 1, &CUADC_MultiCopter::BodyPointCallBack, this, ros::TransportHints().tcpNoDelay());
        detect_point_global_sub = nh_.subscribe<geometry_msgs::Point>(config_.vision_point.subscribe_Global_point, 1, &CUADC_MultiCopter::GlobalPointCallBack, this, ros::TransportHints().tcpNoDelay());
    }
    home_position_sub = nh_.subscribe<geometry_msgs::PoseStamped>("cuadc/home_position", 1, &CUADC_MultiCopter::HomePositionCallback, this, ros::TransportHints().tcpNoDelay());
    vehile_pose_sign_sub = nh_.subscribe<std_msgs::Bool>("cuadc/odom/sign", 1, &CUADC_MultiCopter::OdomSignCallback, this, ros::TransportHints().tcpNoDelay());
    ellipse_body_detect_sub = nh_.subscribe<std_msgs::Bool>("ellipse/body/detect", 1, &CUADC_MultiCopter::BodyEllipseDetectCallback, this, ros::TransportHints().tcpNoDelay());
    ellipse_global_detect_sub = nh_.subscribe<std_msgs::Bool>("ellipse/body/detect", 1, &CUADC_MultiCopter::GlobalEllipseDetectCallback, this, ros::TransportHints().tcpNoDelay());
}

void CUADC_MultiCopter::Init_publisher()
{
    ROS_INFO("Init_publisher");
    control_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(config_.copter.publish_control, 1);
    
    vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(config_.copter.publish_vel_control, 1);

    servo_pub_ = nh_.advertise<std_msgs::Bool>("ServoControl", 1);

    fsm_pub_ = nh_.advertise<fsm_msgs::fsm>("cuadc/fsm", 1);

    ecec_Timer_ = nh_.createTimer(ros::Duration(ros::Rate(config_.ControlRate)), &CUADC_MultiCopter::execCallback, this);

    fsm_Timer_ = nh_.createTimer(ros::Duration(0.01), &CUADC_MultiCopter::execStatePubCallback, this);
}

void CUADC_MultiCopter::PidTuning(cuadc::pidConfig &config)
{
    global_vision_pid_.KP_X_P = config.KP_X_P;
    body_vision_pid_.KP_X_P = config.KP_X_P;

    global_vision_pid_.KI_X_P = config.KI_X_P;
    body_vision_pid_.KI_X_P = config.KI_X_P;

    global_vision_pid_.KD_X_P = config.KD_X_P;
    body_vision_pid_.KD_X_P = config.KD_X_P;

    global_vision_pid_.KP_Y_P = config.KP_Y_P;
    body_vision_pid_.KP_Y_P = config.KP_Y_P;

    global_vision_pid_.KI_Y_P = config.KI_Y_P;
    body_vision_pid_.KI_Y_P = config.KI_Y_P;

    global_vision_pid_.KD_Y_P = config.KD_Y_P;
    body_vision_pid_.KD_Y_P = config.KD_Y_P;

    global_vision_pid_.KP_Z_P = config.KP_Z_P;
    body_vision_pid_.KP_Z_P = config.KP_Z_P;

    global_vision_pid_.KI_Z_P = config.KI_Z_P;
    body_vision_pid_.KI_Z_P = config.KI_Z_P;

    global_vision_pid_.KD_Z_P = config.KD_Z_P;
    body_vision_pid_.KD_Z_P = config.KD_Z_P;

    MaxSpeed_X = config.MaxSpeed_X;
    MaxSpeed_Y = config.MaxSpeed_Y;
    MaxSpeed_Z = config.MaxSpeed_Z;
}

void CUADC_MultiCopter::HomePositionCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    home_position_ = *msg;
    _home_position_ << home_position_.pose.position.x, home_position_.pose.position.y, home_position_.pose.position.z;
    Eigen::Quaterniond q_enu_flu(home_position_.pose.orientation.w, home_position_.pose.orientation.x, home_position_.pose.orientation.y, home_position_.pose.orientation.z);
    R_enu_flu_ = q_enu_flu.toRotationMatrix();

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    double r, p;
    tf::Matrix3x3(quat).getRPY(r, p, home_yaw_);
}

void CUADC_MultiCopter::OdomSignCallback(const std_msgs::BoolConstPtr &msg)
{
    home_position_sign_ = *msg;
}

void CUADC_MultiCopter::BodyEllipseDetectCallback(const std_msgs::BoolConstPtr &msg)
{
    body_ellipse_detect_sign_ = *msg;
}

void CUADC_MultiCopter::GlobalEllipseDetectCallback(const std_msgs::BoolConstPtr &msg)
{
    global_ellipse_detect_sign_ = *msg;
    // ROS_INFO_STREAM("global_ellipse_detect_sign_" << global_ellipse_detect_sign_);
}

void CUADC_MultiCopter::GPS_RTK_vheiclePoseCallBack(const nav_msgs::OdometryConstPtr &msg)
{
    // ROS_INFO("ODOMETRY");
    vehicle_pos_.pose = msg->pose.pose;
    _vehicle_poisition_ <<  vehicle_pos_.pose.position.x, vehicle_pos_.pose.position.y, vehicle_pos_.pose.position.z;
    Eigen::Quaterniond q_wb(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    _vehicle_pose_ = q_wb;
    has_vehicle_pos_ = true;

    // 提取四元数
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    
    // 将四元数转换为欧拉角
    tf::Matrix3x3(quat).getRPY(roll_, pitch_, yaw_);
}

void CUADC_MultiCopter::visionPoseCallBack(const geometry_msgs::PoseStampedConstPtr &msg)
{
    vehicle_pos_ = *msg;
    _vehicle_poisition_ <<  vehicle_pos_.pose.position.x, vehicle_pos_.pose.position.y, vehicle_pos_.pose.position.z;
    Eigen::Quaterniond q_wb(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    _vehicle_pose_ = q_wb;
    has_vehicle_pos_ = true;

    // 提取四元数
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    
    // 将四元数转换为欧拉角
    tf::Matrix3x3(quat).getRPY(roll_, pitch_, yaw_);
}

void CUADC_MultiCopter::BodyPointCallBack(const geometry_msgs::PointConstPtr &msg)
{
    body_vision_point_ = *msg;
    _body_vision_point_ << body_vision_point_.x ,body_vision_point_.y , body_vision_point_.z;
}

void CUADC_MultiCopter::GlobalPointCallBack(const geometry_msgs::PointConstPtr &msg)
{
    global_vision_point_ = *msg;
    _global_vision_point_ << global_vision_point_.x, global_vision_point_.y, global_vision_point_.z;
}

void CUADC_MultiCopter::execCallback(const ros::TimerEvent &e)
{
    switch (exec_state_)
    {
        // 检查
        case CUADC_MultiCopter_FSM::INIT:
        {
            if(!has_vehicle_pos_ || !home_position_sign_.data)
                return;
            changeState(CUADC_MultiCopter_FSM::TAKEOFF);
            break;
        }
        case CUADC_MultiCopter_FSM::TAKEOFF:
        {
            Eigen::Vector3d current_point(hover_wayPoint_.x(), hover_wayPoint_.y(), hover_wayPoint_.z());
            current_point = R_enu_flu_ * current_point + _home_position_;
            ROS_INFO_STREAM("The Hover Point in ENU is " << current_point);
            if(checkPoint(_vehicle_poisition_, current_point, config_.copter.hover_accuracy))
                changeState(CUADC_MultiCopter_FSM::HOVER);
            control_piont_.position.x = current_point.x();
            control_piont_.position.y = current_point.y();
            control_piont_.position.z = current_point.z();
            // control_piont_.yaw = yaw_;
            control_piont_.yaw = home_yaw_;
            control_pub_.publish(control_piont_);
            break;
        }
        case CUADC_MultiCopter_FSM::HOVER:
        {
            Eigen::Vector3d current_point(hover_wayPoint_.x(), hover_wayPoint_.y(), hover_wayPoint_.z());
            current_point = R_enu_flu_ * current_point + _home_position_;
            ROS_INFO_STREAM("The Hover Point in ENU is " << current_point);
            if(checkPoint(_vehicle_poisition_, current_point, config_.copter.hover_accuracy))
            {
                if(!hover_time_sign_)
                {
                    current_point_start_ = ros::Time::now();
                    hover_time_sign_ = true;
                }
                else if(ros::Time::now() - current_point_start_ >= ros::Duration(hover_wayPoint_(3)))
                {
                    changeState(CUADC_MultiCopter_FSM::EXEC_WAYPOINT);
                    hover_time_sign_ = false;
                }
            }
            control_piont_.position.x = current_point.x();
            control_piont_.position.y = current_point.y();
            control_piont_.position.z = current_point.z();
            // control_piont_.yaw = yaw_;
            control_piont_.yaw = home_yaw_;
            control_pub_.publish(control_piont_);
            break;
        }
        case CUADC_MultiCopter_FSM::EXEC_WAYPOINT:
        {
            Eigen::Vector3d current_point(wayPoint_.at(current_idx_).x(), wayPoint_.at(current_idx_).y(), wayPoint_.at(current_idx_).z());
            current_point = R_enu_flu_ * current_point + _home_position_;
            ROS_INFO_STREAM("current point is " << current_point.x() << ", " << current_point.y() << ", " << current_point.z());
            if(checkPoint(_vehicle_poisition_, current_point, config_.copter.exec_accuracy))
            {
                if(!current_point_start_sign_)
                {
                    current_point_start_ = ros::Time::now();
                    current_point_start_sign_ = true;
                }
                else if(ros::Time::now() - current_point_start_ >= ros::Duration(wayPoint_.at(current_idx_)(3)))
                {
                    // 若当前点为最后一个航点则返航
                    if(current_idx_ == config_.copter.wayPointNum)
                    {
                        changeState(CUADC_MultiCopter_FSM::RETURN);
                        current_point_start_sign_ = false;
                        return;
                    }
                    // 进入下一个航点
                    current_idx_++;
                    // 若当前点为最后一个航点则返航
                    if(current_idx_ == config_.copter.wayPointNum)
                    {
                        changeState(CUADC_MultiCopter_FSM::RETURN);
                        current_point_start_sign_ = false;
                        return;
                    }
                    current_point_start_sign_ = false;
                    // 判断该该航点是否要执行视觉
                    if(current_idx_ == config_.copter.exec_visionPoint)
                        changeState(CUADC_MultiCopter_FSM::EXEC_VISION);
                }
            }
            control_piont_.position.x = current_point.x();
            control_piont_.position.y = current_point.y();
            control_piont_.position.z = current_point.z();
            // control_piont_.yaw = yaw_;
            control_piont_.yaw = home_yaw_;
            control_pub_.publish(control_piont_);
            break;
        }
        // 执行视觉
        case CUADC_MultiCopter_FSM::EXEC_VISION:
        {
            if(config_.copter.vision_pid_debug)
                vision_time_start_ = ros::Time::now();
            if(!vision_time_sign_ && !config_.copter.vision_pid_debug)
            {
                vision_time_start_ = ros::Time::now();
                vision_time_sign_ = true;
            }
            else if(ros::Time::now() - vision_time_start_ >= ros::Duration(config_.copter.vision_time) && !config_.copter.vision_pid_debug) // 超时
            {
                ROS_INFO_STREAM("vision time is " << ros::Time::now() - vision_time_start_);
                ROS_INFO("Put the water");
                std_msgs::Bool servo_sign;
                servo_sign.data = true;
                servo_pub_.publish(servo_sign);
                changeState(EXEC_WAYPOINT);
            }
            else if(body_ellipse_detect_sign_.data)    // 执行视觉控制
            {
                if(!config_.copter.vision_pid_debug)
                    ROS_INFO_STREAM("vision time is " << ros::Time::now() - vision_time_start_);

                if(config_.vision_point.useBody)
                {
                    // ROS_INFO_STREAM("_body_vision_point_ : " << _body_vision_point_);
                    // 舵机投放
                    Eigen::Vector2d target(_body_vision_point_.x(), _body_vision_point_.y());
                    ROS_INFO_STREAM("the distance is " << target.norm());
                    if(target.norm() < 0.02 && !servo_time_sign_)
                    {
                        ROS_INFO("Put the water");
                        std_msgs::Bool servo_sign;
                        servo_sign.data = true;
                        servo_pub_.publish(servo_sign);
                        servo_time_start_ = ros::Time::now();
                        servo_time_sign_ = true;
                    }
                    else if(ros::Time::now() - servo_time_start_ >= ros::Duration(3) && servo_time_sign_ && !config_.copter.vision_pid_debug)
                    {
                        ROS_INFO_STREAM("ros::Time::now() - servo_time_start_ is " << ros::Time::now() - servo_time_start_);
                        changeState(CUADC::CUADC_MultiCopter_FSM::EXEC_WAYPOINT);
                        return;
                    }
                    Eigen::Vector3d vel = visionBodyControl(_body_vision_point_);
                    geometry_msgs::TwistStamped vel_;
                    vel_.twist.linear.x = vel.x();
                    vel_.twist.linear.y = vel.y();
                    vel_.twist.linear.z = vel.z();
                    ROS_INFO_STREAM("the vel is " << vel.x() << ", " << vel.y() << ", " << vel.z());
                    vel_pub_.publish(vel_);
                }
                else if(config_.vision_point.useGlobal && global_ellipse_detect_sign_.data)
                {
                    global_ellipse_sign_ = true;
                    ROS_INFO_STREAM("_global_vision_point_ : " << _global_vision_point_);
                    Eigen::Vector3d vel = visionGlobalControl(_vehicle_poisition_, _global_vision_point_);
                    geometry_msgs::TwistStamped vel_;
                    vel_.twist.linear.x = vel.x();
                    vel_.twist.linear.y = vel.y();
                    vel_.twist.linear.z = vel.z();
                    vel_pub_.publish(vel_);
                }
                else if(config_.vision_point.useMix)    // 混合
                {

                }
            }
            // else if(global_ellipse_sign_)
            // {
            //     ROS_INFO_STREAM("_global_vision_point_ : " << _global_vision_point_);
            //     Eigen::Vector3d vel = visionGlobalControl(_vehicle_poisition_, _global_vision_point_);
            //     geometry_msgs::TwistStamped vel_;
            //     vel_.twist.linear.x = vel.x();
            //     vel_.twist.linear.y = vel.y();
            //     vel_.twist.linear.z = vel.z();
            //     vel_pub_.publish(vel_);
            // }
            else    // 悬停
            {
                if(!config_.copter.vision_pid_debug)
                    ROS_INFO_STREAM("vision time is " << ros::Time::now() - vision_time_start_);
                // ROS_INFO_STREAM("VISION HOVER : " << control_piont_.position);
                control_piont_.position = vehicle_pos_.pose.position;
                control_piont_.position.z = config_.copter.vision_height;
                control_pub_.publish(control_piont_);
            }
            break;
        }
        // 返航
        case CUADC_MultiCopter_FSM::RETURN:
        {
            Eigen::Vector3d current_point = returnPoint_.block<3, 1>(0, 0);
            current_point = R_enu_flu_ * current_point + _home_position_;
            if(checkPoint(_vehicle_poisition_, current_point, config_.copter.return_accuracy))
            {
                if(!return_point_start_sign_)
                {
                    return_point_start_ = ros::Time::now();
                    return_point_start_sign_ = true;
                }
                if(ros::Time::now() - return_point_start_ >= ros::Duration(returnPoint_(3)))
                {
                    changeState(CUADC_MultiCopter_FSM::LANDING);
                }
            }
            control_piont_.position.x = current_point.x();
            control_piont_.position.y = current_point.y();
            control_piont_.position.z = current_point.z();
            // control_piont_.yaw = yaw_;
            control_piont_.yaw = home_yaw_;
            control_pub_.publish(control_piont_);
            break;
        }
        // 降落
        case CUADC_MultiCopter_FSM::LANDING:
        {
            break;
        }

    }
}

void CUADC_MultiCopter::execStatePubCallback(const ros::TimerEvent &e)
{
    // ROS_INFO_STREAM("fsm state is " << fsm_state_.fsm_state);
    fsm_pub_.publish(fsm_state_);
}

} // namespace CUADC
