#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "minco_trajectory/rfs_ego.hpp"
#include "visualizer/visualizer.hpp" 

geometry_msgs::PoseStamped point[2];
int count = 0;
bool minco_sign = false;


Eigen::Vector2d start_pt, target_pt;

void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    point[count++] = *msg;
    if(count == 2)
    {
        count = count % 2;
        start_pt << point[0].pose.position.x, point[0].pose.position.y;
        target_pt << point[1].pose.position.x, point[1].pose.position.y;
        std::cout << "distance is " << (target_pt - start_pt).norm() << std::endl;
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
    ros::init(argc, argv, "ego_test");
    ros::NodeHandle nh("~");
    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/goal", 10, targetCallBack, ros::TransportHints().tcpNoDelay());
    
    Visualizer visualizer(nh);

    double piece_length = 1.0;
    Spline::PolynomialSplinePtr minco_ptr_ = std::make_shared<Spline::PolynomialSpline>(2, piece_length);
    double max_speed = 1.0;

    Eigen::MatrixXd wayPoint;
    Eigen::VectorXd piece_dur_vec;

    // 加载地图
    map_generatePtr map_ptr = std::make_shared<map_generate>(nh);
    double resolution = 0.15;
    map_ptr->InitFromHeightMap("/home/zrz/Aerial_Robotics/src/a_star/config/heightmap.png", resolution, 30);
    bool map_ret = map_ptr->generateGridMap();

    RFS_EGOPtr rfs_ego_ptr = std::make_unique<RFS_EGO>();

    int K = 8;  // 每段轨迹上有个K个采样点

    if(map_ret)
    {
        rfs_ego_ptr->SetSimParam(minco_ptr_, map_ptr, 2000,
                                1.0, 1.0, 10000, 5000, 
                                1.0, 1.0, 1.0,
                                10.0, 10.0, 10.0,
                                max_speed, 1.0, 1.0, K);
        rfs_ego_ptr->initPublisherTest(nh);
        rfs_ego_ptr->setHeight(5.0);
    }
    else
        ROS_BREAK();

    ros::Rate rate(20);

    Eigen::Matrix<double, 2, 3> startState, targetState;
    startState.setZero();
    targetState.setZero();
    Eigen::MatrixXd optimal_points;
    Eigen::VectorXd optimal_T;

    // std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> pose;
    // 初始化
    while(ros::ok())
    {
        if(!minco_sign)
        {
            rate.sleep();
            ros::spinOnce();
            // visualizer.visualizeVehicle(pose, "file:///home/jtj/Car_robotics/src/egoplanner/egoplanner.dae");
            continue;
        }
        int piece_num = std::ceil((target_pt - start_pt).norm() / piece_length);
        
        startState.col(0) = start_pt;
        targetState.col(0) = target_pt;

        // 生成初始轨迹
        minco_ptr_->SetParam(startState, targetState, 1);
        Eigen::Matrix<double, 1, 1> time(1.0);
        minco_ptr_->generate(Eigen::Vector2d::Zero(), time.row(0));
        minco_sign = false;

        if(piece_num != 0)
        {
            // 计算每段的时间
            double piece_time = piece_length / max_speed;
            // 生成中间点
            wayPoint.resize(2, piece_num - 1);
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
            double final_cost;
            ros::Time start_time = ros::Time::now();
            bool ret = rfs_ego_ptr->optimizeRFS_Ego(startState, targetState, wayPoint, piece_dur_vec, optimal_points, optimal_T, final_cost);
            ROS_INFO_STREAM("Cost time is " << (ros::Time::now() - start_time).toSec() * 1000 << "ms");
            if(ret)
            {
                ROS_INFO_STREAM("The final cost is " << final_cost);
            }
            else{
                ROS_ERROR("Optimize failed!");
            }
            visualizer.visualize(*rfs_ego_ptr);
            rfs_ego_ptr->visualizerTest();

            // 可视化姿态
            // rfs_ego_ptr->compute3D_Pose(pose);
            // visualizer.visualizeVehicle(pose, "file:///home/jtj/Car_robotics/src/egoplanner/egoplanner.dae");
        }
    }

}