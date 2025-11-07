#pragma once
#include <a_star/a_star.h>
#include <occupancy_grid_map/occ_grid_map.h>
#include "polynomial_spline.hpp"
#include "lbfgs.hpp"

#define _RFS_EGO_TEST_

// #define _RFS_EGO_LOG_

// #define _VISUALIZER_

#ifdef _VISUALIZER_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#endif

class RFS_EGO;

using RFS_EGOPtr = std::shared_ptr<RFS_EGO>;

class RFS_EGO
{
public:
    enum CHK_RET
    {
      OBS_FREE,
      ERR,
      FINISH
    };

public:
    RFS_EGO(){};
    ~RFS_EGO(){};

    /**
     * @brief Set simulation paramters for optimization.
     *
     *
     * @param minco_ptr Input the 5 order polynomial pointer.
     * @param map_generate_ptr Input the map pointer.map_generate_ptr is used for test.
     * @param max_a_star_time Input the maximum cost time(ms) of A Star search.
     * @param weight_time Input the weight of time penalty.
     * @param weight_dis Input the weight of distance penalty.
     * @param weight_c Input the weight of collision penalty.
     * @param weight_v Input the weight of velocity feasible penalty.
     * @param weight_a Input the weight of acceleration feasible penalty.
     * @param weight_j Input the weight of jerk feasible penalty.
     * @param weight_sita Input the weight of vertical parallax angle penalty.
     * @param weight_psi Input the weight of horizontal parallax angle penalty.
     * @param weight_psi_max Input the weight of max yaw angle penalty.
     * @param v_max Input the maximum of velocity trajectory.
     * @param a_max Input the maximum of acceraltion trajectory.
     * @param j_max Input the maximum of jerk trajectory.
     * @param ps_num  Input the number of sampling points of each piece of trajectory.
     * 
     * @return 
     */
    void SetSimParam(const Spline::PolynomialSplinePtr &minco_ptr, 
                     const map_generatePtr &map_generate_ptr, const double max_a_star_time,
                     const double weight_time, const double weight_dis, 
                     const double weight_c, const double weight_c_soft, 
                     const double weight_v, const double weight_a, 
                     const double weight_j, const double weight_sita, 
                     const double weight_psi, const double weight_psi_max, 
                     const double v_max, const double a_max, const double j_max,
                     const int ps_num);

/**
     * @brief Set test paramters for optimization.
     *
     *
     * @param minco_ptr Input the 5 order polynomial pointer.
     * @param weight_time Input the weight of time penalty.
     * @param weight_dis Input the weight of distance penalty.
     * @param weight_c Input the weight of collision penalty.
     * @param weight_v Input the weight of velocity feasible penalty.
     * @param weight_a Input the weight of acceleration feasible penalty.
     * @param weight_j Input the weight of jerk feasible penalty.
     * @param weight_sita Input the weight of vertical parallax angle penalty.
     * @param weight_psi Input the weight of horizontal parallax angle penalty.
     * @param weight_psi_max Input the weight of max yaw angle penalty.
     * @param v_max Input the maximum of velocity trajectory.
     * @param a_max Input the maximum of acceraltion trajectory.
     * @param j_max Input the maximum of jerk trajectory.
     * @param ps_num  Input the number of sampling points of each piece of trajectory.
     * 
     * @return 
     */
    void SetTestParam(const Spline::PolynomialSplinePtr &minco_ptr, 
                  const double weight_time, const double weight_dis, 
                  const double weight_c, const double weight_v, 
                  const double weight_a,const double weight_j,
                  const double weight_sita, const double weight_psi,
                  const double weight_psi_max, 
                  const double v_max, const double a_max, const double j_max,
                  const int ps_num);

    /**
     * @brief Set paramters for optimization.
     *
     *
     * @param minco_ptr Input the 5 order polynomial pointer.
     * @param map_ptr Input the map pointer. map_ptr is used for the real world experiment.
     * @param max_a_star_time Input the maximum cost time(ms) of A Star search.
     * @param weight_time Input the weight of time penalty.
     * @param weight_dis Input the weight of distance penalty.
     * @param weight_c Input the weight of collision penalty.
     * @param weight_v Input the weight of velocity feasible penalty.
     * @param weight_a Input the weight of acceleration feasible penalty.
     * @param weight_j Input the weight of jerk feasible penalty.
     * @param weight_sita Input the weight of vertical parallax angle penalty.
     * @param weight_psi Input the weight of horizontal parallax angle penalty.
     * @param weight_psi_max Input the weight of max yaw angle penalty.
     * @param v_max Input the maximum of velocity trajectory.
     * @param a_max Input the maximum of acceraltion trajectory.
     * @param j_max Input the maximum of jerk trajectory.
     * @param ps_num  Input the number of sampling points of each piece of trajectory.
     * 
     * @return 
     */
    void SetParam(const Spline::PolynomialSplinePtr &minco_ptr, 
                  const CUADC::MapPtr &map_ptr, const double max_a_star_time,
                  const double weight_time, const double weight_dis, 
                  const double weight_c, const double weight_c_soft, 
                  const double weight_v, const double weight_a, 
                  const double weight_j, const double weight_sita, 
                  const double weight_psi, const double weight_psi_max, 
                  const double v_max, const double a_max, const double j_max,
                  const int ps_num);



#ifdef _RFS_EGO_TEST_
    /**
     * @brief The test example for minco trajectory which optimize the 2d / 3d spline to generate a collison free trajectory. 
     *
     *
     * @param obstacle Input the obstacle.
     * Coordine format of 2d obstacle is (x, y, r), 3d is (x, y, z, r).
     * The each 2d obstacle is a cylinder. The size of Matrix is 3 * N. N is the number of obstacles.
     * The each 2d obstacle is a sphere. The size of Matrix is 4 * N.
     * @param startState Input the state of start. 
     * The matrix consist of position, velocity and acceleration. 
     * The size of matrixs is dimensions * 3.
     * @param targetState Input the state of start. 
     * The matrix consist of position, velocity and acceleration. 
     * The size of matrixs is dimensions * 3. 
     * @param way_points Input the way points.
     * @param real_time_vector Input the real time vector.
     * @param optimal_points Output the optimal points.
     * @param optimal_T Output the optimal real Time.
     * 
     * @return 
     */
    double optimize_test(const Eigen::MatrixXd &obstacle, const Eigen::MatrixXd &startState, const Eigen::MatrixXd &targetState, const Eigen::MatrixXd &way_points, const Eigen::VectorXd &real_time_vector, Eigen::MatrixXd &optimal_points, Eigen::VectorXd &optimal_T);

    static double costFunctionCallback(void * ptr, const double *x, double *grad, const int n);

    void initPublisherTest(const ros::NodeHandle &nh);

    void visualizerTest();
#endif

    bool optimizeRFS_Ego(const Eigen::MatrixXd &startState, const Eigen::MatrixXd &targetState, const Eigen::MatrixXd &way_points, const Eigen::VectorXd &real_time_vector, Eigen::MatrixXd &optimal_points, Eigen::VectorXd &optimal_T, double &final_cost);

    static double costFunctionCallbackRFSEgo(void * ptr, const double *x, double *grad, const int n);
    
    static int earlyExitCallback(void *func_data, const double *x, const double *g,
                                 const double fx, const double xnorm, const double gnorm,
                                 const double step, int n, int k, int ls);

    void gradientFeasiblity(double *gradientC, double *gradientRT, double &costF);

    bool feasibilityGradCostV(const Eigen::VectorXd &v, Eigen::VectorXd &gradv, double &costv);

    bool feasibilityGradCostA(const Eigen::VectorXd &a, Eigen::VectorXd &grada, double &costa);

    bool feasibilityGradCostJ(const Eigen::VectorXd &j, Eigen::VectorXd &gradj, double &costj);

    bool obstacleGradCostP(const int id, const Eigen::VectorXd &pos, Eigen::VectorXd &gradp, double &costp);

    // 精细检查碰撞点，并且生成{p，v}对，先调用
    RFS_EGO::CHK_RET finelyCheckAndSetConstraintPoints(std::vector<std::pair<int, int>> &segments);

    // 检查优化后的点是否发生了新的碰撞，并且生成新发生碰撞的{p，v}对
    bool roughlyCheckConstraintPoints(void);

    bool allowRebound(const Eigen::MatrixXd &samplePoint);

    bool computePointsToCheck(const int &id_cps_end, std::vector<std::vector<Eigen::Vector3d>> &pts_check);

    bool isOccupancied(const Eigen::VectorXd& pos)
    {
        Eigen::Vector3d point;
        if(2 == minco_ptr_->getDimensions())
        {
            point << pos.x(), pos.y(), a_star_ptr_->getHeight();
        }
        else
            point = pos;
        if(sim_flag_)
            return map_generate_ptr_->isOccupied_global(point);        
        else
            return map_ptr_->isOccupied_global(point);
    }

    bool hasZeroElement(const Eigen::VectorXd& v) {
        return (v.array() == 0).any();  // 任何一个元素等于 0 返回 true
    }

    void setHeight(const double &high)
    {
        if(minco_ptr_->getDimensions() == 2)
        {
            if(sim_flag_)
                a_star_ptr_->SetSimHeight(high);
            else
                a_star_ptr_->SetHeight(high);
        }
    }
#ifdef _RFS_EGO_LOG_
    bool isRotationMatrix(const Eigen::Matrix3d& R) {
        // 计算 R * R^T，应该接近单位矩阵
        Eigen::Matrix3d shouldBeIdentity = R * R.transpose();
        Eigen::Matrix3d Idient = Eigen::Matrix3d::Identity();

        // 计算行列式
        double detR = R.determinant();

        // 检查正交性（误差允许一个小阈值）
        double epsilon = 1e-6;
        if ((shouldBeIdentity - Idient).norm() > epsilon || std::abs(detR - 1.0) > epsilon) {
            return false; // 不是有效的旋转矩阵
        }
        return true; // 符合旋转矩阵性质
    }
#endif  // _RFS_EGO_LOG_

    void compute3D_Pose(std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> &pose);
    void compute2D_Pose(std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> &pose);

    Spline::PolynomialSplinePtr minco_ptr_;

    A_StarPtr a_star_ptr_;

private:
    int variable_num_; 
    int K_;  
    double weight_time_;   
    double weight_dis_;   
    double weight_c_;   
    double weight_c_soft_;   
    double weight_v_;   
    double weight_a_;   
    double weight_j_;   
    double weight_sita_;    
    double weight_psi_; 
    double weight_psi_max_; 
    double v_max_; 
    double acc_max_;    
    double jerk_max_;   

    std::vector<std::vector<Eigen::Vector3d>> base_point_;  // ego 障碍物上的投影点
    std::vector<std::vector<Eigen::Vector3d>> direction_;   // ego 障碍物的方向向量（单位向量）
    
    std::vector<bool> flag_temp_;   // 标志位  
    int iter_num_;  // 迭代次数

    // 强制返回类型
    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;
    
#ifdef _RFS_EGO_TEST_
    ros::NodeHandle nh_;
    ros::Publisher intersection_pub_;
    ros::Publisher a_star_pub_;
    ros::Publisher sample_point_pub_;
    ros::Publisher pts_check_pub_;
    ros::Publisher a_star_path_start_end_pub_;
    ros::Publisher collision_point_pub_;
    
    pcl::PointCloud<pcl::PointXYZ> collision_point_test_;

    Eigen::MatrixXd obstacle_;  // 障碍物信息
    bool test_flag_;
#endif // _RFS_EGO_TEST_

    map_generatePtr map_generate_ptr_;

    CUADC::MapPtr map_ptr_;

    bool sim_flag_;
};