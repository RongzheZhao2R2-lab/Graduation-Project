#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "minco_trajectory/rfs_ego.hpp"
#include "visualizer/visualizer.hpp" 

geometry_msgs::PoseStamped point[2];
int count = 0;
bool minco_sign = false;

Eigen::Vector3d start_pt, target_pt;


void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    point[count++] = *msg;
    if(count == 2)
    {
        count = count % 2;
        start_pt << point[0].pose.position.x, point[0].pose.position.y, 2.5;
        target_pt << point[1].pose.position.x, point[1].pose.position.y, 2.5;
#ifdef DEBUG
        ROS_INFO_STREAM("start_pt is " << start_pt);
        ROS_INFO_STREAM("target_pt is " << target_pt);
        ROS_INFO_STREAM("distance is " << (target_pt - start_pt).norm());
#endif
        minco_sign = true;
    }
    else 
        minco_sign = false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rfs_ego_test");
    ros::NodeHandle nh;
    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/goal", 10, targetCallBack, ros::TransportHints().tcpNoDelay());
    
    Visualizer visualizer(nh); 

    double piece_length = 0.5;
    Spline::PolynomialSplinePtr minco_ptr_ = std::make_shared<Spline::PolynomialSpline>(3, piece_length);
    double max_speed = 1.5;

    Eigen::MatrixXd wayPoint;
    Eigen::VectorXd piece_dur_vec;

    // 障碍物
    Eigen::MatrixXd obstacle(4, 5);
    obstacle.col(0) << 5.0, 5.0, 3.0, 2.0;
    obstacle.col(1) << -5.0, -5.0, 3.0, 2.0;
    obstacle.col(2) << 5.0, -5.0, 3.0, 2.0;
    obstacle.col(3) << -5.0, 5.0, 3.0, 2.0;
    obstacle.col(4) << 0.0, 0.0, 3.0, 2.5;
    
    RFS_EGOPtr rfs_ego_ptr = std::make_unique<RFS_EGO>();

    rfs_ego_ptr->SetTestParam(minco_ptr_, 
                            10.0, 1.0, 800.0, 
                            1.0, 1.0, 1.0, 10.0, 10.0, 10.0, max_speed, 2.0, 2.0,2.0);

    ros::Rate rate(20.0);

    Eigen::Matrix<double, 3, 3> startState, targetState;
    startState.setZero();
    targetState.setZero();
    Eigen::MatrixXd optimal_points;
    Eigen::VectorXd optimal_T;

    while(ros::ok())
    {
        visualizer.visualizeDisks(obstacle);
        if(!minco_sign)
        {
            rate.sleep();
            ros::spinOnce();
            continue;
        }

        // 计算轨迹段数
        int piece_num = std::ceil((target_pt - start_pt).norm() / piece_length);

        startState.col(0) = start_pt;
        targetState.col(0) = target_pt;

        // 生成初始轨迹
        minco_ptr_->SetParam(startState, targetState, 1);
        Eigen::Matrix<double, 1, 1> time(1.0);
        minco_ptr_->generate(Eigen::Vector3d::Zero(), time.row(0));
        minco_sign = false;
        
        if(piece_num != 0)
        {
            // 计算每段的时间
            double piece_time = piece_length / max_speed;
            // 生成中间点
            wayPoint.resize(3, piece_num - 1);
            wayPoint.setZero();
            piece_dur_vec.resize(piece_num);
            // 计算时间向量
            piece_dur_vec = Eigen::VectorXd::Constant(piece_num, piece_time);

            int id = 0;
            double t_delta = 1.0 / (1.0 * piece_num);
            for(double t = t_delta; t < 1.0 - 0.5 * t_delta; t += t_delta)
            {
                wayPoint.col(id) = minco_ptr_->getState(t);
                id++;
            }
            
            optimal_points.resize(minco_ptr_->getDimensions(), minco_ptr_->getPieceNum() - 1);
            optimal_T.resize(minco_ptr_->getPieceNum());

            if(std::isnan(rfs_ego_ptr->optimize_test(obstacle, startState, targetState, wayPoint, piece_dur_vec, optimal_points, optimal_T)))
            {
                ROS_ERROR("lbfgs optimize failed!");
            }
            else
            {
                visualizer.visualize(*(rfs_ego_ptr->minco_ptr_));
            }
        }
    
    }

    return 0;
}
