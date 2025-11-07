#pragma once

#include "bandedsystem.hpp"
// #include <lapacke.h>
#include <iostream>
#include <chrono>
#include <memory>

// #define DEBUG

namespace Spline
{

class PolynomialSpline;

using PolynomialSplinePtr = std::shared_ptr<PolynomialSpline>;

// 5阶多项式轨迹
class PolynomialSpline
{
public:

    ~PolynomialSpline(){}

    /**
     * @brief Initialize polynomial trajectory.
     *
     *
     * @param dim Input the dimensions of polynomial trajectory.
     * @param ts Input the duration of each piece of polynomial trajectory.
     * @param piece_length Input the max length of each piece of polynomial trajectory.
     * 
     * @return 
     */
    PolynomialSpline(int dim, double piece_length):dim_(dim), piece_length_(piece_length)
    {
        F_0_ << 1, 0, 0, 0, 0, 0, 
                0, 1, 0, 0, 0, 0,
                0, 0, 2, 0, 0, 0;

        F_i_ << 0, 0, 0, 0, 0, 0,
                -1, 0, 0, 0, 0, 0,
                0, -1, 0, 0, 0, 0,
                0, 0, -2, 0, 0, 0,
                0, 0, 0, -6, 0, 0,
                0, 0, 0, 0, -24, 0;

        order_ = 6;
    }

    /**
     * @brief Set paramters polynomial trajectory for optimization.
     *
     *
     * @param start_pt Input the position of start point.
     * @param start_vel Input the veloctiy of start velocity.
     * @param start_acc Input the acceleration of start point.
     * @param target_pt Input the position of target point.
     * @param target_vel Input the veloctiy of target velocity.
     * @param target_acc Input the acceleration of target point.
     * @param piece_num Input the piece number of trajectory.
     * 
     * @return 
     */
    bool SetParam(const Eigen::VectorXd &start_pt, const Eigen::VectorXd &start_vel, const Eigen::VectorXd &start_acc,
                  const Eigen::VectorXd &target_pt, const Eigen::VectorXd &target_vel, const Eigen::VectorXd &target_acc,
                  const int &piece_num);

    /**
     * @brief Set paramters polynomial trajectory for optimization.
     *
     *
     * @param startState Input the state of start. 
     * The matrix consist of position, velocity and acceleration. 
     * The size of matrixs is dimensions * 3.
     * @param targetState Input the state of start. 
     * The matrix consist of position, velocity and acceleration. 
     * The size of matrixs is dimensions * 3. 
     * @param piece_num Input the piece number of trajectory.
     * 
     * @return 
     */
    bool SetParam(const Eigen::MatrixXd &startState, const Eigen::MatrixXd &targetState, const int &piece_num);

    /**
     * @brief Generate the initial value of polynomial trajectory.
     *
     *
     * @param way_points Input the way points of polynomial trajectory.
     * @param time_vecotr Input the duration of each piece of polynomial trajectory.
     * @param coeffs Output the coefficient of polynomial trajectory.
     * 
     * @return 
     */
    void generate(const Eigen::MatrixXd &way_points, const Eigen::VectorXd &time_vecotr/*, Eigen::MatrixXd &coeffs*/);

    /**
     * @brief Get the position and piece of polynomial trajectory
     *
     *
     * @param ts Input the current time.
     * @param flag Input the flag of the return value. 
     * flag = 0, return position.
     * flag = 1, return velocity.
     * flag = 2, return acceleration. 
     * flag = 3, return jerk. 
     * flag = 4, return snap.
     * @param piece Output the current piece of trajcetory.
     * 
     * @return 
     */
    Eigen::VectorXd getStateAndPiece(const double &ts, int &piece, const int &flag = 0) const;

    /**
     * @brief Get the position of polynomial trajectory
     *
     *
     * @param ts Input the current time.
     * @param flag Input the flag of the return value. 
     * flag = 0, return position.
     * flag = 1, return velocity.
     * flag = 2, return acceleration. 
     * flag = 3, return jerk. 
     * flag = 4, return snap.
     * 
     * @return 
     */
    Eigen::VectorXd getState(const double &ts, const int &flag = 0) const;

    /**
     * @brief Get the graient {\partial J}{\partial Real T} of energy function of polynomial trajectory.
     *
     *
     * @param graientK_T Ouput the graient {\partial J}{\partial Real T}. Gradient {\partial J}{\partial Real T} size : row : piece_num_ ,col : 1.
     * 
     * @return 
     */
    void gradientJ_E_RT(double* gradientJ_RT);

    /**
     * @brief Get the graient {\partial J}{\partial C} of energy function of polynomial trajectory.
     *
     *
     * @param graientK_C Ouput the graient {\partial J}{\partial C}. Gradient {\partial J}{\partial C} size : row : 6 * piece_num_ ,col : dimensions.
     * 
     * @return 
     */
    void gradientJ_E_C(double* gradientJ_C);

    /**
     * @brief Get the cost of energy function of polynomial trajectory.
     *
     *
     * @return double The cost of energy function of polynomial trajectory.
     */
    double getJerkCost() const;

    /**
     * @brief Propagate and get the graient {\partial K}{\partial P} of polynomial trajectory.
     *
     *
     * @param graientK_C Ouput the graient {\partial K}{\partial P}. Gradient {\partial K}{\partial P} size : row : dimensions  ,col : piece_num_ - 1.
     * 
     * @return 
     */
    void PropGradientKP(double *gradientK_P);

    /**
     * @brief Propagate and get the graient {\partial K}{\partial Real T} of polynomial trajectory.
     *
     *
     * @param graientK_C Ouput the graient {\partial K}{\partial Real T}. Gradient {\partial K}{\partial Real T} size : row : piece_num_ ,col : 1.
     * 
     * @return 
     */
    void PropGradientKRT(double *gradientK_RT);

    /**
     * @brief Transform real time gradient into virtual time gradient using a diffeomorphism mapping.
     *
     *
     * @param RT Input the real time vector.
     * @param VT Input the virtual time vector.
     * @param gradientK_RT Input the graient {\partial K}{\partial Real T}. Gradient {\partial K}{\partial Real T} size : row : piece_num_ ,col : 1.
     * @param weight_time Input the weight of time penalty
     * @param gradientK_VT Output the graient {\partial K}{\partial Virtual T}. Gradient {\partial K}{\partial Virtual T} size : row : piece_num_ ,col : 1.
     * @param costT Output the cost of time.
     * 
     * @return 
     */
    void DiffphismVirtualTGrad(const double *RT, const double *VT, const double *gradientK_RT, const double &weight_time, double *gradientK_VT, double &costT);

    /**
     * @brief Transform virtual time into real time using a diffeomorphism mapping.
     *
     *
     * @param VT Input the virtual time vector.
     * @param RT Output the real time vector.
     * 
     * @return 
     */
    void VirtualT2RealT(const double *VT, double *RT);
    
    /**
     * @brief Transform real time into virtual time using a diffeomorphism mapping.
     *
     *
     * @param RT Input the real time vector.
     * @param VT Output the virtual time vector.
     * 
     * @return 
     */
    void RealT2VirtualT(const double *RT, double *VT);

    /**
     * @brief Get the G Matrix. M^T * G = {\partial K}{\partial C}.
     *
     *
     * @param graientK_C Input the graient {\partial K}{\partial C}. Gradient {\partial K}{\partial C} size : row : 6 * piece_num_ ,col : dimensions.
     * 
     * @return 
     */
    void solveG(const double *gradientK_C);

    /**
     * @brief Propagate from {\partial J}{\partial C} and {\partial J}{\partial Real T} to {\partial K}{\partial p} and {\partial K}{\partial RT}.
     *
     *
     * @param gradient_J_C Input the gradient {\partial J}{\partial C}. Gradient {\partial J}{\partial C} size : row : 6 * piece_num_ ,col : dimensions.
     * @param graientK_P Output the graient {\partial K}{\partial P}. Gradient {\partial K}{\partial P} size : row : dimensions ,col : piece_num - 1.
     * @param gradientK_RT Output the graient {\partial K}{\partial Real T}. Gradient {\partial K}{\partial Real T} size : row : piece_num_ ,col : 1.
     * 
     * @return 
     */
    void grad2P_RT(const double *gradient_J_C, double *gradientK_P, double *gradientK_RT);

    /**
     * @brief Get the {\partial D}{\partial C},{\partial D}{\partial Real T} and cost of distance penalty.
     *
     *
     * @param weight_dis The weight paramter of distance penalty function.
     * @param p_num Input the number of sampling points of each piece of trajectory.
     * @param points Input the sample points of the whole trajectory.
     * @param gradientD_C Output the gradient {\partial D}{\partial C}. Gradient {\partial D}{\partial C} size : row : 6 * piece_num_ ,col : dimensions.
     * @param gradientD_T Output the gradient {\partial D}{\partial Real T}. Gradient {\partial D}{\partial Real T} size : row : piece_num_ ,col : 1.
     * @param costD Ouput the cost of distance penalty
     * 
     * @return 
     */
    void gradientDistanceC_RT(const double &weight_dis, const int &p_num, const Eigen::MatrixXd &points, double *gradientD_C, double *gradientD_RT, double &costD);
    
    /**
     * @brief Get the {\partial D}{\partial C},{\partial D}{\partial Real T} and cost of distance penalty.
     *
     *
     * @param p_num The number of each piece trajectory of sampling point.
     * @param flag Input the flag of the return value. 
     * flag = 0, return position.
     * flag = 1, return velocity.
     * flag = 2, return acceleration. 
     * flag = 3, return jerk. 
     * flag = 4, return snap.
     * 
     * @return Eigen::MatrixXd The all sample state of trajecotry include start and terminal state.
     */
    Eigen::MatrixXd getSampleStates(const int &p_num, const int &flag);

    /**
     * @brief Get the piecec number of trajectory.
     * 
     * 
     * @return int The piecec number of trajectory.
     */
    int getPieceNum() const {return piece_num_;}

    /**
     * @brief Get the dimensions of trajectory.
     * 
     * 
     * @return int The dimensions of trajectory.
     */
    int getDimensions() const {return dim_;}

    /**
     * @brief Get the duration in initialization(real time).
     * 
     * 
     * @return double The duration in initialization.
     */
    double getInitDuration() const {return t_dur_;}
    
    /**
     * @brief Get the duration of the piece of trajectory in initialization(real time).
     * 
     * 
     * @param piece The piece of trajcetory.
     * 
     * @return double The duration of the piece trajectory in initialization.
     */
    double getDuration(int piece) const {return real_time_vector_(piece);}

    /**
     * @brief Get the total duration of the trajectory(real time).
     * 
     * 
     * @return double The total duration of the trajectory.
     */
    double getTotalDuration() const {return real_time_vector_.sum();}

    /**
     * @brief Get the all waypoints of the trajectory.
     * 
     * 
     * @return const Eigen::MatrixXd& The all waypoints of the trajectory.
     */
    const Eigen::MatrixXd& getAllPoint() const {return allPoint_;}

    /**
     * @brief Get the G Matrix to propagate gradient.
     * 
     * 
     * @return const Eigen::MatrixXd& The G Matrix.
     */ 
    const Eigen::MatrixXd& getGMatrix() const {return G_;}

    /**
     * @brief Get the real time vector.
     * 
     * 
     * @return const Eigen::VectorXd& The real time vector.
     */ 
    const Eigen::VectorXd& getRealTimeVec() const {return real_time_vector_;}

    /**
     * @brief Get the virtual time vector.
     * 
     * 
     * @return const Eigen::VectorXd& The virtual time vector.
     */  
    const Eigen::VectorXd& getVirtualTimeVec() const {return virtual_time_vector_;}

    /**
     * @brief Get the order of polynomial spline.
     * 
     * 
     * @return const int The order of polynomial spline.
     */
    const int getOrder() const {return order_;}

    /**
     * @brief Get the coefficients of polynomial spline.
     * 
     * 
     * @return const Eigen::MatrixXd& The coefficients of polynomial spline.
     */
    const Eigen::MatrixXd& getCoeffs() const {return coeffs_;}
private:

    int dim_;   // 维度
    int order_;     // 阶数
    int piece_num_; // 轨迹的段数
    double piece_length_;    // 每段轨迹的最长长度
    double t_dur_;      // 均匀轨迹中每个路标点的时间间隔
    Eigen::VectorXd real_time_vector_;    // 真实时间向量
    Eigen::VectorXd virtual_time_vector_;    // 通过微分同胚映射后的虚拟时间向量（去除时间流形的约束），直接优化该变量
    Eigen::MatrixXd coeffs_;    // 轨迹样条的参数维度
    Eigen::MatrixXd b_;     // M_ * coeffs_ = b_
    BandedSystem M_;
    Eigen::MatrixX3d start_state_, terminal_state_; // 起始状态与末状态
    Eigen::MatrixXd allPoint_;      // 存入所有点
    Eigen::Matrix<double, 6, 6> F_i_;   // M_矩阵中的中间点项
    Eigen::Matrix<double, 3, 6> F_0_;   // M_矩阵中的初始点项
    Eigen::MatrixXd G_; //  M^T * G = {\partial K}{\partial C}  G Matrix size : row: 6 * piece_num_ ,col : dimensions
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

} // namespace 
