#include "minco_trajectory/polynomial_spline.hpp"

namespace Spline
{

Eigen::VectorXd PolynomialSpline::getStateAndPiece(const double &ts, int &piece, const int &flag) const
{
    const double epsilon = 1e-10;  // 浮点数精度容差
    double total_dur = 0.0;
    int N = 0;
    double t = 0.0;

    if (ts > epsilon) {
        // 遍历时间段数组，找到当前所在段
        for (; N < real_time_vector_.rows(); ++N) {
            const double next_total = total_dur + real_time_vector_(N);
            
            // 检查是否超过当前时间（考虑浮点误差）
            if (next_total > ts + epsilon) {
                break;
            }
            total_dur = next_total;
        }

        // 计算段内相对时间
        if (std::abs(total_dur - ts) < epsilon) {
            t = real_time_vector_(N-1);  // 正好处于段末尾
            N--;              // 修正段索引
        } else {
            t = ts - total_dur;
        }
    }

    piece = N;  // 输出当前段索引

    Eigen::Matrix<double, 6, 1> beta;
    switch (flag)
    {
        case 0:
            beta << 1.0, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5);
            break;
        case 1:
            beta << 0.0, 1.0, 2.0 * t, 3.0 * pow(t, 2), 4.0 * pow(t, 3), 5.0 * pow(t, 4);
            break;
        case 2:
            beta << 0.0, 0.0, 2.0, 6.0 * t, 12.0 * pow(t, 2), 20.0 * pow(t, 3);
            break;
        case 3:
            beta << 0.0, 0.0, 0.0, 6.0, 24.0 * t, 60.0 * pow(t, 2);
            break;
        case 4:
            beta << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * t;
            break;
        default:
            beta << 1.0, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5);
            break;
    }
    return coeffs_.block(N * 6, 0, 6, dim_).transpose() * beta;
}

Eigen::VectorXd PolynomialSpline::getState(const double &ts, const int &flag) const
{
    int tmp;
    return getStateAndPiece(ts, tmp, flag);
}

bool PolynomialSpline::SetParam(const Eigen::VectorXd &start_pt, const Eigen::VectorXd &start_vel, const Eigen::VectorXd &start_acc,
                  const Eigen::VectorXd &target_pt, const Eigen::VectorXd &target_vel, const Eigen::VectorXd &target_acc,
                  const int &piece_num)
{
    if(dim_ != start_pt.rows())
    {
        std::cout << "\033[0;30;41m"
                  << "Set paramters failled!"
                  << "\033[0m" << std::endl;
        return false;
    }
    start_state_.resize(dim_, 3);
    start_state_.col(0) = start_pt;
    start_state_.col(1) = start_vel;
    start_state_.col(2) = start_acc;
    
    terminal_state_.resize(dim_, 3);
    terminal_state_.col(0) = target_pt;
    terminal_state_.col(1) = target_vel;
    terminal_state_.col(2) = target_acc;
    piece_num_ = piece_num;
    
    coeffs_.resize(order_ * piece_num_, dim_);

    b_.resize(order_ * piece_num_, dim_);
    real_time_vector_.resize(piece_num_);
    virtual_time_vector_.resize(piece_num_);
    M_.create(order_ * piece_num_, order_, order_);

    return true;
}

bool PolynomialSpline::SetParam(const Eigen::MatrixXd &startState, const Eigen::MatrixXd &targetState, const int &piece_num)
{
    return SetParam(startState.col(0), startState.col(1), startState.col(2), targetState.col(0), targetState.col(1), targetState.col(2), piece_num);
}

void PolynomialSpline::generate(const Eigen::MatrixXd &way_points, const Eigen::VectorXd &time_vecotr/*, Eigen::MatrixXd &coeffs*/)
{       

#ifdef DEBUG
    auto start_time = std::chrono::high_resolution_clock::now(); 
    // std::cout << "way points :\n" << way_points << "\n time_vecotr :\n" << time_vecotr << '\n';
#endif
    coeffs_.setZero();
    allPoint_.resize(dim_, way_points.cols() + 2);
    allPoint_.block(0, 0, dim_, 1) = start_state_.col(0);
    allPoint_.block(0, way_points.cols() + 1, dim_, 1) = terminal_state_.col(0);
    allPoint_.block(0, 1, dim_, way_points.cols()) = way_points;
    real_time_vector_ = time_vecotr;
    t_dur_ = time_vecotr(0);
    
    RealT2VirtualT(time_vecotr.data(), this->virtual_time_vector_.data());

    M_.reset();
    b_.setZero();

    M_(0, 0) = 1.0;
    M_(1, 1) = 1.0;
    M_(2, 2) = 2.0;

    // Eigen::Matrix<double, 3, 6> E_pieceNum;
    // M_.block(0, 0, 3, 6) = F_0_;
    double t_s = time_vecotr(piece_num_ - 1);
    // E_pieceNum << 1.0 , t_s, pow(t_s, 2), pow(t_s, 3), pow(t_s, 4), pow(t_s, 5),
    //               0.0, 1.0, 2.0 * t_s, 3.0 * pow(t_s, 2), 4.0 * pow(t_s, 3), 5.0 * pow(t_s, 4),
    //               0.0, 0.0, 2.0, 6.0 * t_s, 12.0 * pow(t_s, 2), 20.0 * pow(t_s, 3);
    // M_.block((piece_num_ - 1) * 6 + 3, (piece_num_ - 1) * 6, 3, 6) = E_pieceNum;
    // b_.block(0, 0, 3, dim_) = start_state_.transpose();
    // b_.block(3 + (piece_num_ - 1) * 6, 0, 3, dim_) = terminal_state_.transpose();

    // Eigen::Matrix<double, 6, 6> E_i;
    // Eigen::VectorXd D_i;
    // D_i.resize(dim_);
    
    b_.block(0, 0, 3, dim_) = start_state_.transpose();
    b_.block(3 + (piece_num_ - 1) * 6, 0, 3, dim_) = terminal_state_.transpose();

    for(int i = 0; i < piece_num_ - 1; ++i)
    {
        // t_s = time_vecotr(i - 1);
        // D_i = way_points.col(i - 1);
        // E_i << 1.0, t_s, pow(t_s, 2), pow(t_s, 3), pow(t_s, 4), pow(t_s, 5),
        //        1.0, t_s, pow(t_s, 2), pow(t_s, 3), pow(t_s, 4), pow(t_s, 5),
        //        0.0, 1.0, 2.0 * t_s, 3.0 * pow(t_s, 2), 4.0 * pow(t_s, 3), 5.0 * pow(t_s, 4),
        //        0.0, 0.0, 2.0, 6.0 * t_s, 12.0 * pow(t_s, 2), 20.0 * pow(t_s, 3),
        //        0.0, 0.0, 0.0, 6.0, 24.0 * t_s, 60.0 * pow(t_s, 2),
        //        0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * t_s;
        // M_.block((i - 1) * 6 + 3, i * 6, 6, 6) = F_i_;
        // M_.block((i - 1) * 6 + 3, (i - 1) * 6, 6, 6) = E_i;

        // b_.block((i - 1) * 6 + 3, 0, 1, dim_) = D_i.transpose();


        // t_s = time_vecotr(i - 1);
        // D_i = way_points.col(i - 1);
        // E_i << 0.0, 0.0, 0.0, 6.0, 24.0 * t_s, 60.0 * pow(t_s, 2),
        //        0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * t_s,
        //        1.0, t_s, pow(t_s, 2), pow(t_s, 3), pow(t_s, 4), pow(t_s, 5),
        //        1.0, t_s, pow(t_s, 2), pow(t_s, 3), pow(t_s, 4), pow(t_s, 5),
        //        0.0, 1.0, 2.0 * t_s, 3.0 * pow(t_s, 2), 4.0 * pow(t_s, 3), 5.0 * pow(t_s, 4),
        //        0.0, 0.0, 2.0, 6.0 * t_s, 12.0 * pow(t_s, 2), 20.0 * pow(t_s, 3);
        // M_.block((i - 1) * 6 + 3, i * 6, 6, 6) = F_i_;
        // M_.block((i - 1) * 6 + 3, (i - 1) * 6, 6, 6) = E_i;

        // b_.block((i - 1) * 6 + 5, 0, 1, dim_) = D_i.transpose();

        t_s = time_vecotr(i);
        // 路标点
        // M_(6 * i + 3, 6 * i) = 1.0;
        // M_(6 * i + 3, 6 * i + 1) = t_s;
        // M_(6 * i + 3, 6 * i + 2) = pow(t_s, 2);
        // M_(6 * i + 3, 6 * i + 3) = pow(t_s, 3);
        // M_(6 * i + 3, 6 * i + 4) = pow(t_s, 4);
        // M_(6 * i + 3, 6 * i + 5) = pow(t_s, 5);
        // // 0阶连续
        // M_(6 * i + 4, 6 * i) = 1.0;
        // M_(6 * i + 4, 6 * i + 1) = t_s;
        // M_(6 * i + 4, 6 * i + 2) = pow(t_s, 2);
        // M_(6 * i + 4, 6 * i + 3) = pow(t_s, 3);
        // M_(6 * i + 4, 6 * i + 4) = pow(t_s, 4);
        // M_(6 * i + 4, 6 * i + 5) = pow(t_s, 5);
        // M_(6 * i + 4, 6 * i + 6) = -1.0;
        // // 1阶连续
        // M_(6 * i + 5, 6 * i + 1) = 1.0;
        // M_(6 * i + 5, 6 * i + 2) = 2.0 * t_s;
        // M_(6 * i + 5, 6 * i + 3) = 3.0 * pow(t_s, 2);
        // M_(6 * i + 5, 6 * i + 4) = 4.0 * pow(t_s, 3);
        // M_(6 * i + 5, 6 * i + 5) = 5.0 * pow(t_s, 4);
        // M_(6 * i + 5, 6 * i + 7) = -1.0;
        // // 2阶连续
        // M_(6 * i + 6, 6 * i + 2) = 2.0;
        // M_(6 * i + 6, 6 * i + 3) = 6.0 * t_s;
        // M_(6 * i + 6, 6 * i + 4) = 12.0 * pow(t_s, 2);
        // M_(6 * i + 6, 6 * i + 5) = 20.0 * pow(t_s, 3);
        // M_(6 * i + 6, 6 * i + 8) = -2.0;
        // // 3阶连续
        // M_(6 * i + 7, 6 * i + 3) = 6.0;
        // M_(6 * i + 7, 6 * i + 4) = 24.0 * t_s;
        // M_(6 * i + 7, 6 * i + 5) = 60.0 * pow(t_s, 2);
        // M_(6 * i + 7, 6 * i + 9) = -6.0;
        // // 4阶连续
        // M_(6 * i + 8, 6 * i + 4) = 24.0;
        // M_(6 * i + 8, 6 * i + 5) = 120.0 * t_s;
        // M_(6 * i + 8, 6 * i + 10) = -24.0;

        // b_.row(6 * i + 3) = way_points.col(i).transpose();

        // 3阶连续
        M_(6 * i + 3, 6 * i + 3) = 6.0;
        M_(6 * i + 3, 6 * i + 4) = 24.0 * t_s;
        M_(6 * i + 3, 6 * i + 5) = 60.0 * pow(t_s, 2);
        M_(6 * i + 3, 6 * i + 9) = -6.0;
        // 4阶连续
        M_(6 * i + 4, 6 * i + 4) = 24.0;
        M_(6 * i + 4, 6 * i + 5) = 120.0 * t_s;
        M_(6 * i + 4, 6 * i + 10) = -24.0;
        // 路标点
        M_(6 * i + 5, 6 * i) = 1.0;
        M_(6 * i + 5, 6 * i + 1) = t_s;
        M_(6 * i + 5, 6 * i + 2) = pow(t_s, 2);
        M_(6 * i + 5, 6 * i + 3) = pow(t_s, 3);
        M_(6 * i + 5, 6 * i + 4) = pow(t_s, 4);
        M_(6 * i + 5, 6 * i + 5) = pow(t_s, 5);
        // 0阶连续
        M_(6 * i + 6, 6 * i) = 1.0;
        M_(6 * i + 6, 6 * i + 1) = t_s;
        M_(6 * i + 6, 6 * i + 2) = pow(t_s, 2);
        M_(6 * i + 6, 6 * i + 3) = pow(t_s, 3);
        M_(6 * i + 6, 6 * i + 4) = pow(t_s, 4);
        M_(6 * i + 6, 6 * i + 5) = pow(t_s, 5);
        M_(6 * i + 6, 6 * i + 6) = -1.0;
        // 1阶连续
        M_(6 * i + 7, 6 * i + 1) = 1.0;
        M_(6 * i + 7, 6 * i + 2) = 2 * t_s;
        M_(6 * i + 7, 6 * i + 3) = 3 * pow(t_s, 2);
        M_(6 * i + 7, 6 * i + 4) = 4 * pow(t_s, 3);
        M_(6 * i + 7, 6 * i + 5) = 5 * pow(t_s, 4);
        M_(6 * i + 7, 6 * i + 7) = -1.0;
        // 2阶连续
        M_(6 * i + 8, 6 * i + 2) = 2.0;
        M_(6 * i + 8, 6 * i + 3) = 6 * t_s;
        M_(6 * i + 8, 6 * i + 4) = 12 * pow(t_s, 2);
        M_(6 * i + 8, 6 * i + 5) = 20 * pow(t_s, 3);
        M_(6 * i + 8, 6 * i + 8) = -2.0;

        b_.row(6 * i + 5) = way_points.col(i).transpose();
    }

    M_(6 * piece_num_ - 3, 6 * piece_num_ - 6) = 1.0;
    M_(6 * piece_num_ - 3, 6 * piece_num_ - 5) = real_time_vector_(piece_num_ - 1);
    M_(6 * piece_num_ - 3, 6 * piece_num_ - 4) = pow(real_time_vector_(piece_num_ - 1), 2);
    M_(6 * piece_num_ - 3, 6 * piece_num_ - 3) = pow(real_time_vector_(piece_num_ - 1), 3);
    M_(6 * piece_num_ - 3, 6 * piece_num_ - 2) = pow(real_time_vector_(piece_num_ - 1), 4);
    M_(6 * piece_num_ - 3, 6 * piece_num_ - 1) = pow(real_time_vector_(piece_num_ - 1), 5);
    M_(6 * piece_num_ - 2, 6 * piece_num_ - 5) = 1.0;
    M_(6 * piece_num_ - 2, 6 * piece_num_ - 4) = 2.0 * real_time_vector_(piece_num_ - 1);
    M_(6 * piece_num_ - 2, 6 * piece_num_ - 3) = 3.0 * pow(real_time_vector_(piece_num_ - 1), 2);
    M_(6 * piece_num_ - 2, 6 * piece_num_ - 2) = 4.0 * pow(real_time_vector_(piece_num_ - 1), 3);
    M_(6 * piece_num_ - 2, 6 * piece_num_ - 1) = 5.0 * pow(real_time_vector_(piece_num_ - 1), 4);
    M_(6 * piece_num_ - 1, 6 * piece_num_ - 4) = 2.0;
    M_(6 * piece_num_ - 1, 6 * piece_num_ - 3) = 6.0 * real_time_vector_(piece_num_ - 1);
    M_(6 * piece_num_ - 1, 6 * piece_num_ - 2) = 12.0 * pow(real_time_vector_(piece_num_ - 1), 2);
    M_(6 * piece_num_ - 1, 6 * piece_num_ - 1) = 20.0 * pow(real_time_vector_(piece_num_ - 1), 3);

    M_.factorizeLU();
    M_.solve(b_, coeffs_);

#ifdef DEBUG
    // std::cout << "cocoeffs_ :\n" << coeffs_ << '\n';
    // std::cout << "b_ :\n" << b_ << '\n';
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
    std::cout << "solve M matrix time is " << duration_ns / 1000000.0 << " ms" << std::endl;
#endif
}

void PolynomialSpline::gradientJ_E_RT(double* gradientJ_RT)
{
    for(int i = 0; i < this->piece_num_; ++i)
    {
        gradientJ_RT[i] += 36.0 * coeffs_.row(6 * i + 3).squaredNorm() + 
                          288.0 * coeffs_.row(6 * i + 4).dot(coeffs_.row(6 * i + 3)) * real_time_vector_(i) +
                          576.0 * coeffs_.row(6 * i + 4).squaredNorm() * pow(real_time_vector_(i), 2) + 
                          720.0 * coeffs_.row(6 * i + 5).dot(coeffs_.row(6 * i + 3)) * pow(real_time_vector_(i), 2) + 
                          2880.0 * coeffs_.row(6 * i + 5).dot(coeffs_.row(6 * i + 4)) * pow(real_time_vector_(i), 3) + 
                          3600.0 * coeffs_.row(6 * i + 5).squaredNorm() * pow(real_time_vector_(i), 4);
    }
}

void PolynomialSpline::gradientJ_E_C(double* gradientJ_C)
{
    Eigen::Map<Eigen::MatrixXd> grad_j_c(gradientJ_C, 6 * piece_num_, dim_);
    for(int i = 0; i < this->piece_num_; ++i)
    {
        grad_j_c.row(6 * i + 3) += 72.0 * coeffs_.row(6 * i + 3) * real_time_vector_(i) +
                                   144.0 * coeffs_.row(6 * i + 4) * pow(real_time_vector_(i), 2) +
                                   240.0 * coeffs_.row(6 * i + 5) * pow(real_time_vector_(i), 3);

        grad_j_c.row(6 * i + 4) += 144.0 * coeffs_.row(6 * i + 3) * pow(real_time_vector_(i), 2) + 
                                   384.0 * coeffs_.row(6 * i + 4) * pow(real_time_vector_(i), 3) + 
                                   720.0 * coeffs_.row(6 * i + 5) * pow(real_time_vector_(i), 4);
                                   
        grad_j_c.row(6 * i + 5) += 240.0 * coeffs_.row(6 * i + 3) * pow(real_time_vector_(i), 3) +
                                   720.0 * coeffs_.row(6 * i + 4) * pow(real_time_vector_(i), 4) +
                                   1440.0 * coeffs_.row(6 * i + 5) * pow(real_time_vector_(i), 5);
    }
}

double PolynomialSpline::getJerkCost() const
{
    double costJerk = 0;
    for(int i = 0; i < piece_num_; ++i)
    {
        costJerk += 36.0 * coeffs_.row(6 * i + 3).squaredNorm() * real_time_vector_(i) + 
                    144.0 * coeffs_.row(6 * i + 4).dot(coeffs_.row(6 * i + 3)) * pow(real_time_vector_(i), 2) + 
                    192.0 * coeffs_.row(6 * i + 4).squaredNorm() * pow(real_time_vector_(i), 3) +
                    240.0 * coeffs_.row(6 * i + 5).dot(coeffs_.row(6 * i + 3)) * pow(real_time_vector_(i), 3) +
                    720.0 * coeffs_.row(6 * i + 5).dot(coeffs_.row(6 * i + 4)) * pow(real_time_vector_(i), 4) + 
                    720.0 * coeffs_.row(6 * i + 5).squaredNorm() * pow(real_time_vector_(i), 5);
    }
    return costJerk;
}

void PolynomialSpline::PropGradientKP(double *gradientK_P)
{
    Eigen::Map<Eigen::MatrixXd> grad_k_p(gradientK_P, dim_, piece_num_ - 1);
    for(int i = 0; i < piece_num_ - 1; ++i)
    {
        grad_k_p.col(i) = G_.row(6 * i + 5).transpose();
    }
}

void PolynomialSpline::PropGradientKRT(double *gradientK_RT)
{
    Eigen::Matrix<double, 6, 6> E_i_T;
    double t_s = 0.0;
    for(int i = 0; i < piece_num_ - 1; ++i)
    {
        t_s = real_time_vector_(i);
        E_i_T << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * t_s,
                 0.0, 0.0, 0.0, 0.0, 0.0, 120.0,
                 0.0, 1.0, 2.0 * t_s, 3.0 * pow(t_s, 2), 4.0 * pow(t_s, 3), 5.0 * pow(t_s, 4),
                 0.0, 1.0, 2.0 * t_s, 3.0 * pow(t_s, 2), 4.0 * pow(t_s, 3), 5.0 * pow(t_s, 4),
                 0.0, 0.0, 2.0, 6.0 * t_s, 12.0 * pow(t_s, 2), 20.0 * pow(t_s, 3),
                 0.0, 0.0, 0, 6.0, 24.0 * t_s, 60.0 * pow(t_s, 2);
        gradientK_RT[i] -= (G_.block(6 * i + 3, 0, 6, dim_).transpose() * E_i_T * coeffs_.block(6 * i, 0, 6, dim_)).trace();
    }
    Eigen::Matrix<double, 3, 6> E_pieceNum_T;
    t_s = real_time_vector_(piece_num_ - 1);
    E_pieceNum_T << 0, 1, 2 * t_s, 3 * pow(t_s, 2), 4 * pow(t_s, 3), 5 * pow(t_s, 4),
                    0, 0, 2, 6 * t_s, 12 * pow(t_s, 2), 20 * pow(t_s, 3),
                    0, 0, 0, 6, 24 * t_s, 60 * pow(t_s, 2);
    gradientK_RT[piece_num_ - 1] -= (G_.block(G_.rows() - 3, 0, 3, dim_).transpose() * E_pieceNum_T * coeffs_.block(coeffs_.rows() - 6, 0, 6, dim_)).trace();
}

void PolynomialSpline::DiffphismVirtualTGrad(const double *RT, const double *VT, 
                                            const double *gradientK_RT, const double &weight_time, 
                                             double *gradientK_VT, double &costT)
{
    Eigen::Map<const Eigen::VectorXd> real_time(RT, piece_num_);
    double gradRT_VT = 0.0;
    for(int i = 0; i < piece_num_; ++i)
    {
        if(VT[i] > 0)
            gradRT_VT = VT[i] + 1.0;
        else
            gradRT_VT = (1.0 - VT[i]) / pow((0.5 * VT[i] - 1.0) * VT[i] + 1.0, 2);
        // 梯度传播
        gradientK_VT[i] = (gradientK_RT[i] + weight_time) * gradRT_VT;
    }
    costT = real_time.sum() * weight_time;
}

void PolynomialSpline::VirtualT2RealT(const double *VT, double *RT)
{
    for(int i = 0; i < piece_num_; ++i)
    {
        if(VT[i] > 0)
            RT[i] = (0.5 * VT[i] + 1.0) * VT[i] + 1.0;
        else
            RT[i] = 1 / ((0.5 * VT[i] - 1.0) * VT[i] + 1.0);
    }
}

void PolynomialSpline::RealT2VirtualT(const double *RT, double *VT)
{
    for(int i = 0; i < piece_num_; ++i)
    {
        if(RT[i] > 1)
            VT[i] = sqrt(2 * RT[i] - 1.0) - 1.0;
        else
            VT[i] = 1 - sqrt(-1.0 + 2.0 / RT[i]);
    }
}

void PolynomialSpline::solveG(const double *gradientK_C)
{
#ifdef DEBUG
    auto start_time = std::chrono::high_resolution_clock::now(); 
#endif
    Eigen::Map<const Eigen::MatrixXd> grad_k_c(gradientK_C, order_ * piece_num_, dim_);
    // Eigen::PartialPivLU<Eigen::MatrixXd> plu(M_.transpose());
    G_.resize(order_ * piece_num_, dim_);
    G_.setZero();
    M_.solveAdj(grad_k_c, this->G_);
    // this->G_ = M_inv_.transpose() * grad_k_c;
    // this->G_ = plu.solve(grad_k_c);
#ifdef DEBUG
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
    std::cout << "solve G matrix time is " << duration_ns / 1000000.0 << " ms" << std::endl;
#endif
    // this->G_ = M_inv_.transpose() * grad_k_c;
    // M_.solveAdj(grad_k_c, G_);
}


void PolynomialSpline::grad2P_RT(const double *gradient_J_C, double *gradientK_P, double *gradientK_RT)
{
    solveG(gradient_J_C);
    PropGradientKP(gradientK_P);
    PropGradientKRT(gradientK_RT);
}


void PolynomialSpline::gradientDistanceC_RT(const double& weight_dis, const int &p_num, const Eigen::MatrixXd &points,
                                            double *gradientD_C, double *gradientD_RT, double &costD) 
{
    // 没有中间点
    if(points.cols() == 2)
        return;

    // 假设路径点存储在成员变量 points 中，格式为 3x(N+1) 矩阵
    const Eigen::MatrixXd &ps = points; 
    const int N = ps.cols() - 1;  // 路径点数量为 N+1，分段数为 N

    // 计算相邻路径点差值
    Eigen::MatrixXd dps = ps.rightCols(N) - ps.leftCols(N);
    Eigen::VectorXd dsqrs = dps.colwise().squaredNorm().transpose();

    // 计算代价：路径点间距平方的方差
    const double dquarmean = dsqrs.squaredNorm() / double(piece_num_ * (p_num + 1));
    costD = weight_dis * dquarmean;

    Eigen::MatrixXd grad_d_p(ps.rows(), ps.cols());
    grad_d_p.setZero();

    // 计算路径点梯度
    for (int i = 0; i <= N; ++i) 
    {
        if (i > 0) 
        {
            // 前一段差值的梯度贡献：+4 * ||p_i - p_{i-1}||^2 * (p_i - p_{i-1})
            grad_d_p.col(i) += weight_dis * (4.0 * dsqrs(i-1) * dps.col(i-1)) / N;
        }
        if (i < N) 
        {
            // 后一段差值的梯度贡献：-4 * ||p_{i+1} - p_i||^2 * (p_{i+1} - p_i)
            grad_d_p.col(i) += weight_dis * (-4.0 * dsqrs(i) * dps.col(i)) / N;
        }
    }

    // 链式法则
    Eigen::Map<Eigen::MatrixXd> grad_d_C(gradientD_C, order_ * piece_num_, dim_);
    Eigen::MatrixXd gradViolaPc;
    double gradViolaPt;
    Eigen::VectorXd vel;
    double omg, alpha;
    double s1, s2, s3, s4, s5;
    int K = p_num + 1;
    Eigen::Matrix<double, 6, 1> beta0, beta1;
    int id_p = 0;
    for(int i = 0; i < piece_num_; ++i)
    {
        s1 = 0.0;
        for(int j = 0; j <= K; ++j)
        {
            s2 = s1 * s1;
            s3 = s2 * s1;
            s4 = s2 * s2;
            s5 = s4 * s1;
            beta0 << 1.0, s1, s2, s3, s4, s5;
            beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
            alpha = 1.0 / K * j;
            vel = coeffs_.block(i * order_, 0, order_, dim_).transpose() * beta1;

            omg = (j == 0 || j == K) ? 0.5 : 1.0;

            gradViolaPc = beta0 * grad_d_p.col(id_p).transpose();
            gradViolaPt = alpha * grad_d_p.col(id_p).transpose() * vel;
            grad_d_C.block(i * order_, 0, order_, dim_) += omg * gradViolaPc;
            gradientD_RT[i] += omg * gradViolaPt;
        }
    }

}


Eigen::MatrixXd PolynomialSpline::getSampleStates(const int &p_num, const int &flag)
{
    int num_whole_traj = piece_num_ * p_num + allPoint_.cols();
    Eigen::MatrixXd states(dim_, num_whole_traj);
    double t = getTotalDuration() / (1.0 * num_whole_traj - 1.0);
    for(int i = 0; i < num_whole_traj - 1; ++i)
        states.col(i) = getState(1.0 * i * t, flag);
    states.rightCols(1) = getState(getTotalDuration(), flag);
    return states;
}

}

