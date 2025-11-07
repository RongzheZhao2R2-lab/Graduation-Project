#include "minco_trajectory/rfs_ego.hpp"

void RFS_EGO::SetSimParam(const Spline::PolynomialSplinePtr &minco_ptr, 
                          const map_generatePtr &map_generate_ptr, const double max_a_star_time,
                          const double weight_time, const double weight_dis, 
                          const double weight_c, const double weight_c_soft, 
                          const double weight_v, const double weight_a, 
                          const double weight_j, const double weight_sita, 
                          const double weight_psi, const double weight_psi_max, 
                          const double v_max, const double a_max, const double j_max,
                          const int ps_num)
{
    minco_ptr_ = minco_ptr;
    map_generate_ptr_= map_generate_ptr;
    sim_flag_ = true;
    a_star_ptr_ = std::make_shared<A_Star>();
    if(2 == minco_ptr_->getDimensions())
        a_star_ptr_->initSimAstar(map_generate_ptr, max_a_star_time, A_Star::SearchType::_2_DIEMS);
    else
        a_star_ptr_->initSimAstar(map_generate_ptr, max_a_star_time, A_Star::SearchType::_3_DIEMS);
    weight_time_ = weight_time;
    weight_dis_ = weight_dis;
    weight_c_ = weight_c;
    weight_c_soft_ = weight_c_soft;
    weight_v_ = weight_v;
    weight_a_ = weight_a;
    weight_j_ = weight_j;
    weight_sita_ = weight_sita;
    weight_psi_ = weight_psi;
    weight_psi_max_ = weight_psi_max;
    K_ = ps_num;
    v_max_ = v_max;
    acc_max_ = a_max;
    jerk_max_ = j_max;
}

void RFS_EGO::SetTestParam(const Spline::PolynomialSplinePtr &minco_ptr, 
                           const double weight_time, const double weight_dis, 
                           const double weight_c, const double weight_v, 
                           const double weight_a,const double weight_j,
                           const double weight_sita, const double weight_psi,
                           const double weight_psi_max, 
                           const double v_max, const double a_max, const double j_max, 
                           const int ps_num)
{
    minco_ptr_ = minco_ptr;
    weight_time_ = weight_time;
    weight_dis_ = weight_dis;
    weight_c_ = weight_c;
    weight_v_ = weight_v;
    weight_a_ = weight_a;
    weight_j_ = weight_j;
    weight_sita_ = weight_sita;
    weight_psi_ = weight_psi;
    weight_psi_max_ = weight_psi_max;
    K_ = ps_num;
    v_max_ = v_max;
    acc_max_ = a_max;
    jerk_max_ = j_max;
}

void RFS_EGO::SetParam(const Spline::PolynomialSplinePtr &minco_ptr, 
                       const CUADC::MapPtr &map_ptr, const double max_a_star_time,
                       const double weight_time, const double weight_dis, 
                       const double weight_c, const double weight_c_soft, 
                       const double weight_v, const double weight_a, 
                       const double weight_j, const double weight_sita, 
                       const double weight_psi, const double weight_psi_max, 
                       const double v_max, const double a_max, const double j_max,
                       const int ps_num)
{
    minco_ptr_ = minco_ptr;
    map_ptr_= map_ptr;
    sim_flag_ = false;
    a_star_ptr_ = std::make_shared<A_Star>();
    if(2 == minco_ptr_->getDimensions())
        a_star_ptr_->initAstar(map_ptr_, max_a_star_time, A_Star::SearchType::_2_DIEMS);
    else
        a_star_ptr_->initAstar(map_ptr_, max_a_star_time, A_Star::SearchType::_3_DIEMS);
    weight_time_ = weight_time;
    weight_dis_ = weight_dis;
    weight_c_ = weight_c;
    weight_c_soft_ = weight_c_soft;
    weight_v_ = weight_v;
    weight_a_ = weight_a;
    weight_j_ = weight_j;
    weight_sita_ = weight_sita;
    weight_psi_ = weight_psi;
    weight_psi_max_ = weight_psi_max;
    K_ = ps_num;
    v_max_ = v_max;
    acc_max_ = a_max;
    jerk_max_ = j_max;
}


#ifdef _RFS_EGO_TEST_
double RFS_EGO::optimize_test(const Eigen::MatrixXd &obstacle, const Eigen::MatrixXd &startState, 
                              const Eigen::MatrixXd &targetState, const Eigen::MatrixXd &way_points, 
                              const Eigen::VectorXd &real_time_vector, Eigen::MatrixXd &optimal_points, 
                              Eigen::VectorXd &optimal_T)
{
    obstacle_ = obstacle;
    // jerkOpt_.reset(startState, targetState, real_time_vector.rows());
    minco_ptr_->SetParam(startState, targetState, real_time_vector.rows());

    // set initial value
    variable_num_ = minco_ptr_->getDimensions() * (minco_ptr_->getPieceNum() - 1) + minco_ptr_->getPieceNum();
    double x_init[variable_num_];
    // way points
    memcpy(x_init, way_points.data(), way_points.size() * sizeof(x_init[0]));
    // time vecotr
    Eigen::Map<Eigen::VectorXd> VT(x_init + way_points.size(), minco_ptr_->getPieceNum());

    // convert real time vector to virtual time vector
    minco_ptr_->RealT2VirtualT(real_time_vector.data(), VT.data());
    // std::cout << "virtual time is \n" << VT << std::endl;

    // 关闭EGO碰撞项
    test_flag_ = true;

    double cost;
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 64;
    lbfgs_params.max_iterations = 200;
    lbfgs_params.min_step = 1e-32;
    lbfgs_params.past = 3;
    lbfgs_params.g_epsilon = 0.0;
    lbfgs_params.delta = 1.0e-6;
    
    int ret = lbfgs::lbfgs_optimize(variable_num_, 
                                    x_init, 
                                    &cost, 
                                    RFS_EGO::costFunctionCallback, 
                                    nullptr, 
                                    nullptr, 
                                    this, 
                                    &lbfgs_params);

    Eigen::VectorXd RT(minco_ptr_->getPieceNum());
    minco_ptr_->VirtualT2RealT(VT.data(), RT.data());
    Eigen::Map<Eigen::MatrixXd> wps(x_init, way_points.rows(), way_points.cols());
    if(ret >= 0 || ret == lbfgs::LBFGSERR_MAXIMUMLINESEARCH || 
        ret == lbfgs::LBFGS_CONVERGENCE ||
        ret == lbfgs::LBFGSERR_MAXIMUMITERATION ||
        ret == lbfgs::LBFGS_ALREADY_MINIMIZED||
        ret == lbfgs::LBFGS_STOP)
    {
        minco_ptr_->generate(wps, RT);
    }
    else
    {
        cost = INFINITY;
        std::cout << "Optimization Failed: "
                    << lbfgs::lbfgs_strerror(ret)
                    << std::endl;
    }

    // std::cout << "virtual time is \n" << VT << std::endl;

    Eigen::Map<Eigen::MatrixXd> op(x_init, way_points.rows(), way_points.cols());
    optimal_points = op;
    optimal_T = RT;
    return cost;
}


double RFS_EGO::costFunctionCallback(void * ptr, const double *x, double *grad, const int n)
{
    RFS_EGO &obj = *(RFS_EGO *)ptr;
    Eigen::Map<const Eigen::MatrixXd> wps(x, obj.minco_ptr_->getDimensions(), obj.minco_ptr_->getPieceNum() - 1);
    Eigen::Map<const Eigen::VectorXd> VT(x + wps.size(), obj.minco_ptr_->getPieceNum());
    Eigen::Map<Eigen::MatrixXd> grad_J_P(grad, obj.minco_ptr_->getDimensions(), obj.minco_ptr_->getPieceNum() - 1);
    Eigen::Map<Eigen::VectorXd> grad_J_VT(grad + obj.minco_ptr_->getDimensions() * (obj.minco_ptr_->getPieceNum() - 1), obj.minco_ptr_->getPieceNum());
    // real time vector
    Eigen::VectorXd RT(obj.minco_ptr_->getPieceNum());
    // convert virtual time to real time
    obj.minco_ptr_->VirtualT2RealT(VT.data(), RT.data());
    
    // generate a polynomial spline
    obj.minco_ptr_->generate(wps, RT);
    double costJerk = 0.0;

    // calculate gradient and cost
    // calculate gradient {partial J}{partial C} and {partial J}{partial Real T}
    // energy function
    Eigen::MatrixXd gradientJ_c(obj.minco_ptr_->getOrder() * obj.minco_ptr_->getPieceNum(), obj.minco_ptr_->getDimensions());
    gradientJ_c.setZero();

    Eigen::VectorXd gradientJ_RT(obj.minco_ptr_->getPieceNum());
    gradientJ_RT.setZero();


    obj.minco_ptr_->gradientJ_E_RT(gradientJ_RT.data());
    obj.minco_ptr_->gradientJ_E_C(gradientJ_c.data());
    costJerk = obj.minco_ptr_->getJerkCost();

    // calculate gradient {partial D}{partial C} and {partial D}{partial Real T}
    double costDis = 0.0;
    Eigen::MatrixXd cps;
    cps = obj.minco_ptr_->getSampleStates(obj.K_, 0);
    obj.minco_ptr_->gradientDistanceC_RT(obj.weight_dis_, obj.K_, cps, gradientJ_c.data(), gradientJ_RT.data(), costDis);
    std::cout << "costDis : " << costDis << '\n';

    double costF = 0.0;
    obj.gradientFeasiblity(gradientJ_c.data(), gradientJ_RT.data(), costF);
    std::cout << "costF : " << costF << '\n';
    
    // proprate the gradient
    obj.minco_ptr_->grad2P_RT(gradientJ_c.data(), grad_J_P.data(), gradientJ_RT.data());

    double costCollision = 0.0, signdist, dist;
    
    Eigen::VectorXd p_i, delta;
    Eigen::VectorXd grad_J_c_p;
    
    for(int i = 0; i < wps.cols(); ++i)
    {
        p_i = wps.col(i);
        for(int j = 0; j < obj.obstacle_.cols(); ++j)
        {
            if(2 == obj.minco_ptr_->getDimensions())    // 2维
            {
                delta = p_i - obj.obstacle_.col(j).head<2>();
                dist = delta.norm();
                signdist = dist - obj.obstacle_(2, j) - 0.3;
            }
            else    // 3维
            {
                delta = p_i - obj.obstacle_.col(j).head<3>();
                dist = delta.norm();
                signdist = dist - obj.obstacle_.col(j)(3) - 0.3;
            }

            if(signdist < 0.0)
            {
                costCollision += 0.5 * pow(signdist, 2) * obj.weight_c_;
                grad_J_P.col(i) += obj.weight_c_ * signdist * delta / dist;
            }
        }

    }

    std::cout << "costCollision : " << costCollision << '\n';

    // convert real time to virtual time.
    double costTime = 0.0;
    obj.minco_ptr_->DiffphismVirtualTGrad(RT.data(), VT.data(), gradientJ_RT.data(), obj.weight_time_, grad_J_VT.data(), costTime);
    // std::cout << "The gradient {partial K}{partial Virtual T} :\n" << grad_J_VT << std::endl;
    std::cout << "Total cost is " << costJerk + costDis + costTime + costCollision + costF <<  std::endl; 
    return costJerk + costDis + costTime + costCollision + costF; 
};

void RFS_EGO::initPublisherTest(const ros::NodeHandle &nh)
{
    intersection_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("intersection_point", 10);
    a_star_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("a_star_path", 10);
    sample_point_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("sample_point", 10);
    pts_check_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pts_check", 10);
    a_star_path_start_end_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("a_star_path_start_end", 10);
    collision_point_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("collision_point", 10);
}

void RFS_EGO::visualizerTest()
{
    if(!base_point_.size())
        return;
    
    pcl::PointCloud<pcl::PointXYZ> cloud_intersection_test_;

    for(size_t i = 0; i < base_point_.size(); ++i)
    {
        for(size_t j = 0; j < base_point_[i].size(); ++j)
        {
            pcl::PointXYZ p;
            p.x = base_point_[i][j].x();
            p.y = base_point_[i][j].y();
            p.z = base_point_[i][j].z();
            cloud_intersection_test_.push_back(p);
        }
    }
    // std::cout << "cloud_intersection_test_.points.size() " << cloud_intersection_test_.points.size() << std::endl;
    cloud_intersection_test_.width = cloud_intersection_test_.points.size();
    cloud_intersection_test_.height = 1;
    cloud_intersection_test_.is_dense = true;
    cloud_intersection_test_.header.frame_id = "map";
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud_intersection_test_, cloud_msg);
    if(intersection_pub_.getNumSubscribers() > 0)
        intersection_pub_.publish(cloud_msg);
}

#endif // _RFS_EGO_TEST_

void RFS_EGO::gradientFeasiblity(double *gradientC, double *gradientRT, double &costF)
{
#ifdef _VISUALIZER_
    collision_point_test_.clear();
#endif // _VISUALIZER_

    Eigen::Map<Eigen::MatrixXd> gradientKC(gradientC, minco_ptr_->getOrder() * minco_ptr_->getPieceNum(), minco_ptr_->getDimensions());
     
    int K = K_ + 1;
    double step;
    double s1, s2, s3, s4, s5;
    double omg, alpha;
    int i_dp = 0;
    Eigen::VectorXd pos, vel, acc, jer, sna;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
    Eigen::MatrixXd gradViolaPc, gradViolaVc, gradViolaAc, gradViolaJc;
    double gradViolaPt, gradViolaVt, gradViolaAt, gradViolaJt;
    Eigen::VectorXd gradp, gradv, grada, gradj;
    double costp, costV, costA, costJ;
    for(int i = 0; i < minco_ptr_->getPieceNum(); ++i)
    {
        Eigen::MatrixXd c = minco_ptr_->getCoeffs().block(minco_ptr_->getOrder() * i, 0, minco_ptr_->getOrder(), minco_ptr_->getDimensions());
        step = minco_ptr_->getRealTimeVec()(i) / K;
        // time
        s1 = 0.0;
        for(int j = 0; j <= K; ++j)
        {
            // time 
            s2 = s1 * s1;
            s3 = s2 * s1;
            s4 = s2 * s2;
            s5 = s4 * s1;
            beta0 << 1.0, s1, s2, s3, s4, s5;
            beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
            beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
            beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
            beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1;
            alpha = 1.0 / K * j;
            pos = c.transpose() * beta0;
            vel = c.transpose() * beta1;
            acc = c.transpose() * beta2;
            jer = c.transpose() * beta3;
            sna = c.transpose() * beta4;

            omg = (j == 0 || j == K) ? 0.5 : 1.0;
            if(!test_flag_)
                if (obstacleGradCostP(i_dp, pos, gradp, costp))
                {
                    gradViolaPc = beta0 * gradp.transpose();
                    gradViolaPt = alpha * gradp.transpose() * vel;
                    gradientKC.block(minco_ptr_->getOrder() * i, 0, minco_ptr_->getOrder(), minco_ptr_->getDimensions()) 
                                    += omg * step * gradViolaPc;
                    gradientRT[i] += omg * (costp / K + step * gradViolaPt);
                    costF += omg * step * costp;
                }
            // 速度惩罚
            if (feasibilityGradCostV(vel, gradv, costV))
            {
                gradViolaVc = beta1 * gradv.transpose();
                gradViolaVt = alpha * gradv.transpose() * acc;
                gradientKC.block(minco_ptr_->getOrder() * i, 0, minco_ptr_->getOrder(), minco_ptr_->getDimensions()) 
                                += omg * step * gradViolaVc;
                gradientRT[i] += omg * (costV / K + step * gradViolaVt);
                costF += omg * step * costV;
            }
            if(feasibilityGradCostA(acc, grada, costA))
            {
                gradViolaAc = beta2 * grada.transpose();
                gradViolaAt = alpha * grada.transpose() * jer;
                gradientKC.block(minco_ptr_->getOrder() * i, 0, minco_ptr_->getOrder(), minco_ptr_->getDimensions()) 
                                += omg * step * gradViolaAc;
                gradientRT[i] += omg * (costA / K + step * gradViolaAt);
                costF += omg * step * costA;
            }
            if(feasibilityGradCostJ(jer, gradj, costJ))
            {
                gradViolaJc = beta3 * gradj.transpose();
                gradViolaJt = alpha * gradj.transpose() * sna;
                gradientKC.block(minco_ptr_->getOrder() * i, 0, minco_ptr_->getOrder(), minco_ptr_->getDimensions()) 
                                += omg * step * gradViolaJc;
                gradientRT[i] += omg * (costJ / K + step * gradViolaJt);
                costF += omg * step * costJ;
            }
            s1 += step;
            if (j != K || (j == K && i == minco_ptr_->getPieceNum()))
            {
                ++i_dp;
            }
        }
    }
#ifdef _VISUALIZER_
    collision_point_test_.width = collision_point_test_.points.size();
    collision_point_test_.height = 1;
    collision_point_test_.is_dense = true;
    collision_point_test_.header.frame_id = "map";
    sensor_msgs::PointCloud2 collosion_msg;
    pcl::toROSMsg(collision_point_test_, collosion_msg);
    if(collision_point_pub_.getNumSubscribers() > 0)
        collision_point_pub_.publish(collosion_msg);
#endif // _VISUALIZER_
}


bool RFS_EGO::feasibilityGradCostV(const Eigen::VectorXd &v, Eigen::VectorXd &gradv, double &costv)
{
    double vpen = v.squaredNorm() - v_max_ * v_max_;
    if (vpen > 0)
    {
        gradv = weight_v_ * 6 * vpen * vpen * v;
        costv = weight_v_ * vpen * vpen * vpen;
        return true;
    }
    return false;
}

bool RFS_EGO::feasibilityGradCostA(const Eigen::VectorXd &a, Eigen::VectorXd &grada, double &costa)
{
    double apen = a.squaredNorm() - acc_max_ * acc_max_;
    if (apen > 0)
    {
        grada = weight_a_ * 6 * apen * apen * a;
        costa = weight_a_ * apen * apen * apen;
        return true;
    }
    return false;
}

bool RFS_EGO::feasibilityGradCostJ(const Eigen::VectorXd &j, Eigen::VectorXd &gradj, double &costj)
{
    double Jpen = j.squaredNorm() - acc_max_ * acc_max_;
    if (Jpen > 0)
    {
        gradj = weight_j_ * 6 * Jpen * Jpen * j;
        costj = weight_j_ * Jpen * Jpen * Jpen;
        return true;
    }
    return false;
}

bool RFS_EGO::obstacleGradCostP(const int id, const Eigen::VectorXd &pos, Eigen::VectorXd &gradp, double &costp)
{
    if(id == 0)
        return false;
    
    bool ret = false;

    if(2 == minco_ptr_->getDimensions())
        gradp.resize(2);
    else
        gradp.resize(3);
    
    gradp.setZero();
    costp = 0;

    Eigen::Vector3d pose;
    for(size_t j = 0; j < direction_[id].size(); ++j)
    {
        if(2 == minco_ptr_->getDimensions())
            pose << pos.x(), pos.y(), a_star_ptr_->getHeight();
        else
            pose = pos;

        Eigen::Vector3d ray = (pose - base_point_[id][j]);
        double dist = ray.dot(direction_[id][j]);
        double dist_err = 0.15 - dist;
        double dist_err_soft = 0.5 - dist;
        Eigen::Vector3d dist_grad = direction_[id][j];
        
#ifdef _VISUALIZER_
        if(dist_err > 0 || dist_err_soft > 0)
        {        
            pcl::PointXYZ p;
            p.x = pose.x();
            p.y = pose.y();
            p.z = pose.z();
            collision_point_test_.push_back(p);
        }
#endif  // _VISUALIZER_

        if(dist_err > 0)
        {   
            // std::cout << "dist err : " << weight_c_ * pow(dist_err, 3) << std::endl;
            ret = true;
            costp += weight_c_ * pow(dist_err, 3);
            if(2 == minco_ptr_->getDimensions())
                gradp += -weight_c_ * 3.0 * dist_err * dist_err * dist_grad.head(2);
            else
                gradp += -weight_c_ * 3.0 * dist_err * dist_err * dist_grad;
        }

        if(dist_err_soft > 0)
        {
            ret = true;
            double r = 0.05;
            double rsqr = r * r;
            double term = sqrt(1.0 + dist_err_soft * dist_err_soft / rsqr);
            costp += weight_c_soft_ * rsqr * (term - 1.0);
            // std::cout << "soft dist err : " << weight_c_soft_ * rsqr * (term - 1.0) << std::endl;
            if(2 == minco_ptr_->getDimensions())
                gradp += -weight_c_soft_ * dist_err_soft / term * dist_grad.head(2);
            else
                gradp += -weight_c_soft_ * dist_err_soft / term * dist_grad;
        }
    }

    return ret;
}

bool RFS_EGO::optimizeRFS_Ego(const Eigen::MatrixXd &startState, const Eigen::MatrixXd &targetState, 
                              const Eigen::MatrixXd &way_points, const Eigen::VectorXd &real_time_vector, 
                              Eigen::MatrixXd &optimal_points, Eigen::VectorXd &optimal_T,
                              double &final_cost)
{
    ros::Time t0 = ros::Time::now(), t1, t2;
    int restart_nums = 0, rebound_times = 0;
    bool flag_force_return, flag_still_unsafe, flag_success;
    
    minco_ptr_->SetParam(startState, targetState, real_time_vector.rows());
    minco_ptr_->generate(way_points, real_time_vector);

    std::vector<std::vector<Eigen::Vector3d>>().swap(base_point_);
    base_point_.resize(minco_ptr_->getPieceNum() * K_ + minco_ptr_->getAllPoint().cols());
    std::vector<std::vector<Eigen::Vector3d>>().swap(direction_);
    direction_.resize(minco_ptr_->getPieceNum() * K_ + minco_ptr_->getAllPoint().cols());
    std::vector<bool>().swap(flag_temp_);
    flag_temp_.resize(minco_ptr_->getPieceNum() * K_ + minco_ptr_->getAllPoint().cols());

    std::vector<std::pair<int, int>> segments;
    if(finelyCheckAndSetConstraintPoints(segments) == CHK_RET::ERR)
    {
        return false;
    }

    // set initial value
    variable_num_ = minco_ptr_->getDimensions() * (minco_ptr_->getPieceNum() - 1) + minco_ptr_->getPieceNum();
    double x_init[variable_num_];
    // way points
    memcpy(x_init, way_points.data(), way_points.size() * sizeof(x_init[0]));
    // time vecotr
    Eigen::Map<Eigen::VectorXd> VT(x_init + way_points.size(), minco_ptr_->getPieceNum());

    // convert real time vector to virtual time vector
    minco_ptr_->RealT2VirtualT(real_time_vector.data(), VT.data());

    // 开启EGO碰撞项
    test_flag_ = false;

    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 16;
    lbfgs_params.max_iterations = 200;
    lbfgs_params.min_step = 1e-32;
    lbfgs_params.past = 3;
    lbfgs_params.delta = 1.0e-2;

    do{
        iter_num_ = 0;
        flag_force_return = false;
        force_stop_type_ = DONT_STOP;
        flag_still_unsafe = false;
        flag_success = false;

        // 开始优化
        t1 = ros::Time::now();
        int result = lbfgs::lbfgs_optimize(
            variable_num_,
            x_init,
            &final_cost,
            RFS_EGO::costFunctionCallbackRFSEgo,
            nullptr,
            RFS_EGO::earlyExitCallback,
            this,
            &lbfgs_params);
        t2 = ros::Time::now();

        double time_ns = (t2 - t1).toSec() * 1000;
        double total_time_ns = (t2 - t0).toSec() * 1000; 

        if(result == lbfgs::LBFGS_CONVERGENCE ||
           result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
           result == lbfgs::LBFGS_ALREADY_MINIMIZED||
           result == lbfgs::LBFGS_STOP)
        {
            flag_force_return = false;
            std::vector<std::pair<int, int>> segments;
            if(finelyCheckAndSetConstraintPoints(segments) == CHK_RET::OBS_FREE)
            {
                flag_success = true;
                std::cout << "\e[3;33;42mIteration number: " << iter_num_ 
                          << ", optimze once time: " << time_ns 
                          << ", total time: " << total_time_ns << "\e[0m\n";
            }
            else    // 发生碰撞
            {
                flag_still_unsafe = true;
                restart_nums++;
            }
        }
        else if(result == lbfgs::LBFGSERR_CANCELED)    // 发现新的障碍物，rebound自增
        {
            flag_force_return = true;
            rebound_times++;
            // std::cout << "reBound occur" << std::endl;
        }
    }while((flag_still_unsafe && restart_nums < 3) ||
           (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 20));
    
    Eigen::VectorXd RT(minco_ptr_->getPieceNum());
    minco_ptr_->VirtualT2RealT(VT.data(), RT.data());
    Eigen::Map<Eigen::MatrixXd> wps(x_init, way_points.rows(), way_points.cols());
    minco_ptr_->generate(wps, RT);
    return flag_success;
}

double RFS_EGO::costFunctionCallbackRFSEgo(void * ptr, const double *x, double *grad, const int n)
{
    RFS_EGO &obj = *(RFS_EGO *)ptr;
    // 路标点
    Eigen::Map<const Eigen::MatrixXd> wps(x, obj.minco_ptr_->getDimensions(), obj.minco_ptr_->getPieceNum() - 1);
    // 微分同胚映射后的时间
    Eigen::Map<const Eigen::VectorXd> VT(x + wps.size(), obj.minco_ptr_->getPieceNum());
    // 关于路标点的梯度
    Eigen::Map<Eigen::MatrixXd> grad_J_P(grad, obj.minco_ptr_->getDimensions(), obj.minco_ptr_->getPieceNum() - 1);
    // 关于时间的梯度
    Eigen::Map<Eigen::VectorXd> grad_J_VT(grad + obj.minco_ptr_->getDimensions() * (obj.minco_ptr_->getPieceNum() - 1), obj.minco_ptr_->getPieceNum());
    // 转换成真实时间
    Eigen::VectorXd RT(obj.minco_ptr_->getPieceNum());
    obj.minco_ptr_->VirtualT2RealT(VT.data(), RT.data());
    
    // 生成初值
    obj.minco_ptr_->generate(wps, RT);
    // 关于参数c的梯度
    Eigen::MatrixXd gradientJ_c(obj.minco_ptr_->getOrder() * obj.minco_ptr_->getPieceNum(), obj.minco_ptr_->getDimensions());
    gradientJ_c.setZero();
    // 关于真实时间RT的梯度
    Eigen::VectorXd gradientJ_RT(obj.minco_ptr_->getPieceNum());
    gradientJ_RT.setZero();

    // 初始化关于能量函数的梯度以及costJerk
    obj.minco_ptr_->gradientJ_E_RT(gradientJ_RT.data());
    obj.minco_ptr_->gradientJ_E_C(gradientJ_c.data());
    double costJerk = 0.0;
    costJerk = obj.minco_ptr_->getJerkCost();

    // 计算距离最小化的梯度以及costDis
    double costDis = 0.0;
    Eigen::MatrixXd cps;
    cps = obj.minco_ptr_->getSampleStates(obj.K_, 0);
    obj.minco_ptr_->gradientDistanceC_RT(obj.weight_dis_, obj.K_, cps, gradientJ_c.data(), gradientJ_RT.data(), costDis);

    double costF = 0.0;
    obj.gradientFeasiblity(gradientJ_c.data(), gradientJ_RT.data(), costF);

    // 粗略检查，若发生碰撞，则反弹
    if(obj.allowRebound(cps))
    {
        obj.roughlyCheckConstraintPoints();
    }

    // 梯度传播
    obj.minco_ptr_->grad2P_RT(gradientJ_c.data(), grad_J_P.data(), gradientJ_RT.data());
    // 将RT转换为VT
    double costTime = 0.0;
    obj.minco_ptr_->DiffphismVirtualTGrad(RT.data(), VT.data(), gradientJ_RT.data(), obj.weight_time_, grad_J_VT.data(), costTime);
    
    // 迭代次数自增
    obj.iter_num_++;

    return costJerk + costDis + costF + costTime;
}

int RFS_EGO::earlyExitCallback(void *func_data, const double *x, const double *g,
                                const double fx, const double xnorm, const double gnorm,
                                const double step, int n, int k, int ls)
{
    RFS_EGO *rfs_ego_ptr = reinterpret_cast<RFS_EGO *>(func_data); 
    return (rfs_ego_ptr->force_stop_type_ == STOP_FOR_REBOUND || rfs_ego_ptr->force_stop_type_ == STOP_FOR_ERROR);
}

bool RFS_EGO::computePointsToCheck(const int &id_cps_end, std::vector<std::vector<Eigen::Vector3d>> &pts_check)
{
    std::vector<std::vector<Eigen::Vector3d>>().swap(pts_check);
    pts_check.resize(id_cps_end);

    double resolution;
    if(sim_flag_)
        resolution = map_generate_ptr_->resolution_;
    else
        resolution = map_ptr_->mp_.resolution_;
    Eigen::VectorXd durations = minco_ptr_->getRealTimeVec();

    const double DURATION = minco_ptr_->getTotalDuration();
    double t = 0.0, t_step = std::min(resolution / v_max_, durations.minCoeff() / std::max(K_ + 1, 1) / 1.5);

    double uniform_time = DURATION / (1.0 * id_cps_end);
    Eigen::VectorXd uniform_time_vector(id_cps_end);
    for(int i = 0; i < id_cps_end; ++i)
    {
        uniform_time_vector(i) =  (i + 1.0) * uniform_time;
    }
    // std::cout << "DURATION : " << DURATION << std::endl;
    // std::cout << "id_cps_end : " << id_cps_end << std::endl;
    // std::cout << "uniform time vector is \n" << uniform_time_vector << std::endl;
    int piece = 0;
    Eigen::VectorXd tmp;
    Eigen::Vector3d pos;
    while(t < DURATION)
    {
        
        tmp = minco_ptr_->getState(t);

        if(piece > id_cps_end)
            break;

        if(2 == minco_ptr_->getDimensions())
            pos << tmp.x(), tmp.y(), a_star_ptr_->getHeight();
        else
            pos = tmp;
        
        if(t > uniform_time_vector(piece))
            piece++;
        if(piece > id_cps_end)
            piece = id_cps_end;

        pts_check[piece].emplace_back(pos);
        t += t_step;
    }
    return true;
}

RFS_EGO::CHK_RET RFS_EGO::finelyCheckAndSetConstraintPoints(std::vector<std::pair<int, int>> &segments)
{
    // 每段轨迹上有K_个采样点
    Eigen::MatrixXd init_points = minco_ptr_->getSampleStates(K_, 0);

#ifdef _VISUALIZER_
    pcl::PointCloud<pcl::PointXYZ> sample_point_test_;
    for(int i = 0; i < init_points.cols(); ++i)
    {
        pcl::PointXYZ p;
        p.x = init_points.col(i).x();
        p.y = init_points.col(i).y();
        if(2 == minco_ptr_->getDimensions())
            p.z = a_star_ptr_->getHeight();
        else
            p.z = init_points.col(i).z();
        sample_point_test_.push_back(p);
    }
    sample_point_test_.width = sample_point_test_.points.size();
    sample_point_test_.height = 1;
    sample_point_test_.is_dense = true;
    sample_point_test_.header.frame_id = "map";
    sensor_msgs::PointCloud2 sample_cloud_msg;
    pcl::toROSMsg(sample_point_test_, sample_cloud_msg);
    if(sample_point_pub_.getNumSubscribers() > 0)
        sample_point_pub_.publish(sample_cloud_msg);
#endif // _VISUALIZER_

    int in_id = -1, out_id = -1;
    std::vector<std::pair<int, int>> segment_ids;
    constexpr int ENOUGH_INTERVAL = 2;
    int same_occ_state_times = ENOUGH_INTERVAL + 1;
    bool occ, last_occ = false;
    bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
    int i_end = init_points.cols() - 1;

    double resolution;
    if(sim_flag_)
        resolution = map_generate_ptr_->resolution_;
    else
        resolution = map_ptr_->mp_.resolution_;

    std::vector<std::vector<Eigen::Vector3d>> pts_check;

    if(!computePointsToCheck(i_end, pts_check))
    {
        return CHK_RET::ERR;
    }

#ifdef _VISUALIZER_
    pcl::PointCloud<pcl::PointXYZ> pts_check_test_;
    for(auto &p1:pts_check)
    {
        for(auto &p2:p1)
        {
            pcl::PointXYZ p;
            p.x = p2.x();
            p.y = p2.y();
            p.z = p2.z();
            pts_check_test_.push_back(p);
        }
    }
    pts_check_test_.width = pts_check_test_.points.size();
    pts_check_test_.height = 1;
    pts_check_test_.is_dense = true;
    pts_check_test_.header.frame_id = "map";
    sensor_msgs::PointCloud2 pts_check_msg;
    pcl::toROSMsg(pts_check_test_, pts_check_msg);
    if(pts_check_pub_.getNumSubscribers() > 0)
        pts_check_pub_.publish(pts_check_msg);

#endif // _VISUALIZER_

    for(int i = 0; i < i_end; ++i)
    {
        for(size_t j = 0; j < pts_check[i].size(); ++j)
        {
            occ = isOccupancied(pts_check[i][j]);
            if(occ && !last_occ)    // 当前被占用，设置为起始点
            {
                if(same_occ_state_times > ENOUGH_INTERVAL)
                {
                    if(i == 0)
                        in_id = 0;
                    else
                        in_id = i - 1;
                    flag_got_start = true;
                }
                same_occ_state_times = 0;
                flag_got_end_maybe = false;
            }
            else if(!occ && last_occ)
            {
                out_id = i + 1;
                flag_got_end_maybe = true;
                same_occ_state_times = 0;
            }
            else
            {
                ++same_occ_state_times;
            }

            if(flag_got_end_maybe && (same_occ_state_times > ENOUGH_INTERVAL || (i == i_end - 1)))
            // if(flag_got_end_maybe)
            {
                flag_got_end_maybe = false;
                flag_got_end = true;
            }

            last_occ = occ;

            if(flag_got_start && flag_got_end)
            {
                flag_got_end = false;
                flag_got_start = false;
                if(in_id < 0 || out_id < 0)     // 逻辑冗余
                {
                    ROS_ERROR("Should not happen! in_id=%d, out_id=%d", in_id, out_id);
                    return CHK_RET::ERR;
                }
                segment_ids.push_back(std::pair<int, int>(in_id, out_id));  // 存入发生碰撞的起点，终点
            }
        }
    }

#ifdef _RFS_EGO_LOG_
    std::cout << "finelyCheckAndSetConstraintPoints segment_ids size " << segment_ids.size() << std::endl;
#endif // _RFS_EGO_LOG_

    if(0 == segment_ids.size()) // 未发生碰撞
        return CHK_RET::OBS_FREE;
    

    // 计算边界 该代码逻辑冗余
    int id_low_bound, id_up_bound;
    std::vector<std::pair<int, int>> bounds(segment_ids.size());
    for(size_t i = 0; i < segment_ids.size(); ++i)
    {
        if(0 == i)
        {
            id_low_bound = 1;
            if(segment_ids.size() > 1)
                id_up_bound = (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0) / 2.0);
            else    // 若只有一段碰撞或者未发生碰撞
                id_up_bound = init_points.cols() - 2; // 设置为倒数第二个点
        }
        else if(i == segment_ids.size() - 1)    // 在最后一段
        {
            id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0) / 2.0);
            id_up_bound = init_points.cols() - 2;
        }
        else
        {
            id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0) / 2.0);
            id_up_bound = (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0) / 2.0);
        }
        bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
    }

    // 避免乱序 该代码逻辑冗余
    std::vector<std::pair<int, int>> adjusted_segment_ids(segment_ids.size());
    int minimum_points = 0, num_points;
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      /*** Adjust segment length ***/
      num_points = segment_ids[i].second - segment_ids[i].first + 1;
      if (num_points < minimum_points)
      {
#ifdef _RFS_EGO_LOG_
        std::cout << "Adjust segment length" << std::endl;
#endif // _RFS_EGO_LOG_
        double add_points_each_side = (int)(((minimum_points - num_points) + 1.0f) / 2);

        adjusted_segment_ids[i].first = segment_ids[i].first - add_points_each_side >= bounds[i].first
                                            ? segment_ids[i].first - add_points_each_side
                                            : bounds[i].first;

        adjusted_segment_ids[i].second = segment_ids[i].second + add_points_each_side <= bounds[i].second
                                             ? segment_ids[i].second + add_points_each_side
                                             : bounds[i].second;
      }
      else
      {
        adjusted_segment_ids[i].first = segment_ids[i].first;
        adjusted_segment_ids[i].second = segment_ids[i].second;
      }
    }

    // 避免乱序，后一个碰撞段的起始点不大于前一个碰撞段的终点
    for(size_t i = 1; i < adjusted_segment_ids.size(); ++i)
    {
        if (adjusted_segment_ids[i - 1].second >= adjusted_segment_ids[i].first)
        {
            double middle = (double)(adjusted_segment_ids[i - 1].second + adjusted_segment_ids[i].first) / 2.0;
            adjusted_segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
            adjusted_segment_ids[i].first = static_cast<int>(middle + 1.1);
        }
    }


    // 对发生碰撞的段进行A*搜索
    double costTime = 0.0;
    std::vector<std::vector<Eigen::Vector3d>> a_star_pathes;

#ifdef _VISUALIZER_
    pcl::PointCloud<pcl::PointXYZ> a_star_start_end_point_;
#endif // _VISUALIZER_

    for(size_t i = 0; i < adjusted_segment_ids.size(); ++i)
    {
        Eigen::Vector3d in, out;
        if(2 == minco_ptr_->getDimensions())
        {
            in << init_points.col(adjusted_segment_ids[i].second).x(), init_points.col(adjusted_segment_ids[i].second).y(), a_star_ptr_->getHeight();
            out << init_points.col(adjusted_segment_ids[i].first).x(), init_points.col(adjusted_segment_ids[i].first).y(), a_star_ptr_->getHeight();
        }
        else if(3 == minco_ptr_->getDimensions())
            in = init_points.col(adjusted_segment_ids[i].second), out = init_points.col(adjusted_segment_ids[i].first);
        else
        {
            ROS_INFO("Error Dimensions!");
            ROS_BREAK();
        }

#ifdef _VISUALIZER_
        pcl::PointXYZ p1, p2;
        p1.x = in.x(), p2.x = out.x();
        p1.y = in.y(), p2.y = out.y();
        p1.z = in.z(), p2.z = out.z();
        a_star_start_end_point_.push_back(p1);
        a_star_start_end_point_.push_back(p2);
#endif  // _VISUALIZER_

        ASTAR_RET ret = a_star_ptr_->AstarGraphSearch(in, out, A_Star::AstarHeu::DIALOG_TIEBREAKER, costTime);
        if(ret == ASTAR_RET::SUCCESS)
            a_star_pathes.push_back(a_star_ptr_->getPath());
        else if(ret == ASTAR_RET::SEARCH_ERR && i + 1 < adjusted_segment_ids.size())
        {
            adjusted_segment_ids[i].second = adjusted_segment_ids[i + 1].second;
            adjusted_segment_ids.erase(adjusted_segment_ids.begin() + i + 1);
            --i;
        }
        else
        {
            ROS_ERROR("A_star error");
            adjusted_segment_ids.erase(adjusted_segment_ids.begin() + i);   // 删除该段
        }
    }

#ifdef _VISUALIZER_
    a_star_start_end_point_.width = a_star_start_end_point_.points.size();
    a_star_start_end_point_.height = 1;
    a_star_start_end_point_.is_dense = true;
    a_star_start_end_point_.header.frame_id = "map";
    sensor_msgs::PointCloud2 start_end_msg;
    pcl::toROSMsg(a_star_start_end_point_, start_end_msg);
    if(a_star_path_start_end_pub_.getNumSubscribers() > 0)
        a_star_path_start_end_pub_.publish(start_end_msg);

#endif  // _VISUALIZER_


    if(a_star_pathes.size() == 0)
    {
        return CHK_RET::ERR;
    }

#ifdef _VISUALIZER_
    pcl::PointCloud<pcl::PointXYZ> a_star_path_test;

    for(size_t i = 0 ; i < a_star_pathes.size(); ++i)
    {
        for(size_t j = 0; j < a_star_pathes[i].size(); ++j)
        {
            pcl::PointXYZ p;
            p.x = a_star_pathes[i][j].x();
            p.y = a_star_pathes[i][j].y();
            p.z = a_star_pathes[i][j].z();
            a_star_path_test.push_back(p);
        }
    }

    a_star_path_test.width = a_star_path_test.points.size();
    a_star_path_test.height = 1;
    a_star_path_test.is_dense = true;
    a_star_path_test.header.frame_id = "map";
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(a_star_path_test, cloud_msg);
    if(a_star_pub_.getNumSubscribers() > 0)
        a_star_pub_.publish(cloud_msg);
#endif // _VISUALIZER_

#ifdef _RFS_EGO_LOG_
    for(auto &p1: adjusted_segment_ids)
    {
        std::cout << '(' << p1.first << ", " << p1.second << ")\t";
    }
    std::cout << std::endl;
    for(auto &p1: segment_ids)
    {
        std::cout << '(' << p1.first << ", " << p1.second << ")\t";
    }
    std::cout << std::endl;
#endif // _RFS_EGO_LOG_

    // 为每个发生碰撞的轨迹计算{p, v}对
    std::vector<std::pair<int, int>> final_segment_ids;
    Eigen::Vector3d tmp;
    for(size_t i = 0; i < adjusted_segment_ids.size(); ++i)
    {
        // true:成功产生{p,v}对，false:生成失败
        for(int j = adjusted_segment_ids[i].first; j <= adjusted_segment_ids[i].second; ++j)
            flag_temp_[j] = false;
        
        int got_intersection_id = -1;
        for(int j = adjusted_segment_ids[i].first + 1; j < adjusted_segment_ids[i].second; ++j)
        {
            got_intersection_id = -1;
            if(i > a_star_pathes.size() - 1)
                break;

            Eigen::Vector3d ctrl_pts_law;
            Eigen::Vector3d intersection_point; // 相交点
            if(2 == minco_ptr_->getDimensions())
            {
                tmp.head(2) = init_points.col(j);
                tmp(2) = a_star_ptr_->getHeight();
                ctrl_pts_law.head(2) = init_points.col(j + 1) - init_points.col(j - 1);
                ctrl_pts_law(2) = 0;
            }
            else
            {
                tmp = init_points.col(j);
                ctrl_pts_law = init_points.col(j + 1) - init_points.col(j - 1);
            }

            int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id;  // 计算A*搜索后的路径点索引，用于计算投影
            // 若val>0，则代表着与A*搜索点的夹角小于90度，反之大于90度。投影需要找到一个与控制点成90度的交点
#ifdef _RFS_EGO_LOG_
            std::cout << "Astar id : " << Astar_id << std::endl;
            std::cout << "a_star_pathes size is " << a_star_pathes.size() << std::endl;
            std::cout << "a_star_pathes[i] size is " << a_star_pathes[i].size() << "\t  i :" << i << std::endl;
            std::cout << "a_star_pathes[i][Astar_id] " << a_star_pathes[i][Astar_id] << std::endl;
#endif // _RFS_EGO_LOG_
            double val = (a_star_pathes[i][Astar_id] - tmp).dot(ctrl_pts_law), init_val = val;

            while(true)
            {
                last_Astar_id = Astar_id;

                if(val >= 0)    // val大于0，自减，夹角会变大直到val变为负
                {
                    --Astar_id; // 自减，夹角会变大直到val变为负值
                    if(Astar_id < 0)
                    {
                        break;
                    }
                }
                else    // val小于0，自增，夹角会变小直到val变为正
                {
                    ++Astar_id; // 自增，夹角会变小直到val变为正值
                    if(Astar_id >= (int)a_star_pathes[i].size())
                    {
                        break;
                    }
                }
                // 更新val值
                val = (a_star_pathes[i][Astar_id] - tmp).dot(ctrl_pts_law);
                // std::cout << "init_val : " << init_val << std::endl;
                // std::cout << "     val : " << val << std::endl;
                if(init_val * val <= 0 && (abs(val) > 0 || abs(init_val) > 0))
                {
                    // 根据相似的原理可以得到垂直于控制点的交点
                    intersection_point =
                        a_star_pathes[i][Astar_id] +
                        ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                        (ctrl_pts_law.dot(tmp - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]))
                        );
                    got_intersection_id = j;
#ifdef _RFS_EGO_LOG_
                    std::cout << "loop intersection_point " << intersection_point << std::endl;
                    std::cout << "while got_intersection_id : " << got_intersection_id << std::endl;
#endif // _RFS_EGO_LOG_
                    break;
                }
            }

            if(got_intersection_id >= 0)
            {
                // 计算到该交点的长度，用于后续插值
#ifdef _RFS_EGO_LOG_
                std::cout << "got_intersection_id " << got_intersection_id << std::endl;
                std::cout << "intersection_point " << intersection_point << std::endl;
                std::cout << "init_points.col(j) " << init_points.col(j) << std::endl;
#endif  // _RFS_EGO_LOG_
                double length = (intersection_point - tmp).norm();
                if(length > 1e-5)   // 长度不为0，1e-5为截断误差
                {
                    flag_temp_[j] = true;
                    for(double a = length; a >= 0.0; a -= resolution)
                    {
                        bool occ = isOccupancied((a / length) * intersection_point + (1 - a / length) * tmp);    // 判断该店是否在障碍物里
                        if(occ || a < resolution)   // 若每次插值点在障碍物里，投影点往外移动一个分辨率单位；若之前的插值发现都没有在障碍物里，则直接更新投影点
                        {
                            if(occ)
                                a += resolution;
                            base_point_[j].push_back((a / length) * intersection_point + (1 - a / length) * tmp);
                            direction_[j].push_back((intersection_point - tmp).normalized());
                            break;
                        }
                    }
                }
                else
                {
                    got_intersection_id = -1;   // 获取失败
                }
            }
        }
        
        // 发生的碰撞段很短，两个控制点之间是相邻的
        if(adjusted_segment_ids[i].second - adjusted_segment_ids[i].first == 1)
        {
            got_intersection_id = -1;
            if(i > a_star_pathes.size() - 1)
                break;
            
            Eigen::Vector3d ctrl_pts_law, intersection_point, middle_point;
            if(2 == minco_ptr_->getDimensions())
            {
                ctrl_pts_law.head(2) = init_points.col(adjusted_segment_ids[i].second) - init_points.col(adjusted_segment_ids[i].first);
                ctrl_pts_law(2) = 0;
                middle_point.head(2) = (init_points.col(adjusted_segment_ids[i].second) + init_points.col(adjusted_segment_ids[i].first)) / 2.0;
                middle_point(2) = a_star_ptr_->getHeight();
            }
            else
            {
                ctrl_pts_law = init_points.col(adjusted_segment_ids[i].second) - init_points.col(adjusted_segment_ids[i].first);
                middle_point = (init_points.col(adjusted_segment_ids[i].second) + init_points.col(adjusted_segment_ids[i].first)) / 2.0;
            }
            int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
            double val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law), init_val = val;
            while (true)
            {

                last_Astar_id = Astar_id;

                if(val >= 0)    // val大于0，自减，夹角会变大直到val变为负
                {
                    --Astar_id; // 自减，夹角会变大直到val变为负值
                    if(Astar_id < 0)
                    {
                        // std::cout << "Astar_id lower than zero" << std::endl;
                        break;
                    }
                }
                else    // val小于0，自增，夹角会变小直到val变为正
                {
                    ++Astar_id; // 自增，夹角会变小直到val变为正值
                    if(Astar_id >= (int)a_star_pathes[i].size())
                    {
                        // std::cout << "Astar_id over size" << std::endl;
                        break;
                    }
                }

                val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law);

                if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0)) // val = init_val = 0.0 is not allowed
                {
                    intersection_point =
                        a_star_pathes[i][Astar_id] +
                        ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                        (ctrl_pts_law.dot(middle_point - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                        );

                    if ((intersection_point - middle_point).norm() > 0.01) // 1cm.
                    {
                        flag_temp_[adjusted_segment_ids[i].first] = true;
                        if(2 == minco_ptr_->getDimensions())
                        {
                            tmp.head(2) = init_points.col(adjusted_segment_ids[i].first);
                            tmp(2) = a_star_ptr_->getHeight();
                        }
                        else
                            tmp = init_points.col(adjusted_segment_ids[i].first);
                        base_point_[adjusted_segment_ids[i].first].push_back(tmp);
                        direction_[adjusted_segment_ids[i].first].push_back((intersection_point - middle_point).normalized());

                        got_intersection_id = adjusted_segment_ids[i].first;
                    }
                    break;
                }
            }
        }


        // 将其他未被访问过的点或者计算失败的点进行赋值{p, v}对
        if(got_intersection_id >= 0)
        {
            for(int j = got_intersection_id + 1; j <= adjusted_segment_ids[i].second; ++j)
            {
                if(!flag_temp_[j])
                {
                    base_point_[j].push_back(base_point_[j - 1].back());
                    direction_[j].push_back(direction_[j - 1].back());
                }
            }
            for(int j = got_intersection_id - 1; j >= adjusted_segment_ids[i].first; --j)
            {
                if(!flag_temp_[j])
                {
                    base_point_[j].push_back(base_point_[j + 1].back());
                    direction_[j].push_back(direction_[j + 1].back());
                }
            }
            final_segment_ids.push_back(adjusted_segment_ids[i]);
        }
    }
    segments = final_segment_ids;
    return CHK_RET::FINISH;
}

bool RFS_EGO::roughlyCheckConstraintPoints(void)
{
    int in_id = -1, out_id = -1;
    std::vector<std::pair<int, int>> segment_ids;
    bool flag_new_obs_valid = false;
    Eigen::MatrixXd points = minco_ptr_->getSampleStates(K_, 0);
    // 获得最后一个点
    int i_end = points.cols() - 1;
    bool occ;
    double resolution;
    if(sim_flag_)
        resolution = map_generate_ptr_->resolution_;
    else
        resolution = map_ptr_->mp_.resolution_;
    // 寻找碰撞点
    Eigen::Vector3d point;
    for(int i = 1; i <= i_end; ++i)
    {
        if(2 == minco_ptr_->getDimensions())
            point << points.col(i).x(), points.col(i).y(), a_star_ptr_->getHeight(); 
        else
            point = points.col(i);

        occ = isOccupancied(point);

        if(occ)
        {
            // 该点在障碍物里，若该点在新的障碍物里，则更新新的{p, v}对
            for(size_t k = 0; k < direction_[i].size(); ++k)
            {
                if((point - base_point_[i][k]).dot(direction_[i][k]) < resolution)
                {
                    occ = false;
                    break;
                }
            }
        }

        // 在障碍物内，更新
        if(occ)
        {
            flag_new_obs_valid = true;

            // 计算发生未碰撞的起始点
            int j;
            for(j = i - 1; j >= 0; --j)
            {
                occ = isOccupancied(points.col(j));
                if(!occ)
                {
                    in_id = j;
                    break;
                }
            }
            if(j < 0)   // 从后往前遍历，若一直没有发现空闲的点，则认为发生碰撞
            {
                ROS_ERROR("The vehicle is in obstacle. It means a crash in real-world.");
                in_id = 0;
            }
            // 往后遍历，找到第一个空闲的点
            for(j = i + 1; j < points.cols(); ++j)
            {
                occ = isOccupancied(points.col(j));

                if(!occ)    // 未发生碰撞，则找到
                {
                    out_id = j;
                    break;
                }
            }
            // 在往后便利的过程中，若未发现空闲的点，则目标点在障碍物里，返回false
            if(j >= points.cols())
            {
                ROS_WARN("Local target in collision, skip this planning.");
                
                force_stop_type_ = STOP_FOR_ERROR;
                return false;
            }

            i = j + 1;
            segment_ids.push_back(std::pair<int ,int>(in_id, out_id));
        }
    }

    // 若新的障碍物点是有效的
    double costTime = 0.0;
    if(flag_new_obs_valid)
    {
        std::vector<std::vector<Eigen::Vector3d>> a_star_pathes;
        for(size_t i = 0; i < segment_ids.size(); ++i)
        {
            // 进行A*搜索
            Eigen::Vector3d in, out;
            if(2 == minco_ptr_->getDimensions())
            {
                in << points.col(segment_ids[i].second).x(), points.col(segment_ids[i].second).y(), a_star_ptr_->getHeight();
                out << points.col(segment_ids[i].first).x(), points.col(segment_ids[i].first).y(), a_star_ptr_->getHeight();
            }
            else
                in = points.col(segment_ids[i].second), out = points.col(segment_ids[i].first);
            
            // Eigen::Vector3d in(points.col(segment_ids.at(i).second)), out(points.col(segment_ids.at(i).first));

            ASTAR_RET ret = a_star_ptr_->AstarGraphSearch(in, out, A_Star::AstarHeu::DIALOG_TIEBREAKER, costTime);
            if(ret == ASTAR_RET::SUCCESS)
                a_star_pathes.push_back(a_star_ptr_->getPath());
            else if(ret == ASTAR_RET::SEARCH_ERR && i + 1 < segment_ids.size())    // 搜索时间超过
            {
                segment_ids[i].second = segment_ids[i + 1].second;  // 将后面的终点作为当前段的终点
                segment_ids.erase(segment_ids.begin() + i + 1);     // 删除后面一个
                i--;
            }
            else    // 起点，终点是否在障碍物里
            {
                ROS_ERROR("A_star error");
                segment_ids.erase(segment_ids.begin() + i);   // 删除该段
            }
        }// a_star

        // 避免重复
        for(size_t i = 1; i < segment_ids.size(); ++i)
        {   
            // 乱序
            if(segment_ids[i - 1].second >= segment_ids[i].first)
            {
                double middle = (double)(segment_ids[i - 1].second + segment_ids[i].first) / 2.0;
                segment_ids[i - 1].second = static_cast<int>(middle - 0.1); // 若两个点相等，所以需要多减去0.1
                segment_ids[i].first = static_cast<int>(middle + 1.1);  // 相隔大于1
            }
        }

        // 为每个发生碰撞的轨迹计算{p, v}对
        Eigen::Vector3d tmp;
        for(size_t i = 0; i < segment_ids.size(); ++i)
        {
            // 初始化标志位
            for(int j = segment_ids[i].first; j < segment_ids[i].second ; ++j)
                flag_temp_[j] = false;

            // 计算控制点在障碍物上的投影
            int got_intersection_id = -1;
            for(int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
            {
                got_intersection_id = -1;
                if(i > a_star_pathes.size() - 1)
                    break;

                Eigen::Vector3d ctrl_pts_law;
                Eigen::Vector3d intersection_point; // 相交点

                if(2 == minco_ptr_->getDimensions())
                {
                    tmp.head(2) = points.col(j);
                    tmp(2) = a_star_ptr_->getHeight();
                    ctrl_pts_law.head(2) = points.col(j + 1) - points.col(j - 1);
                    ctrl_pts_law(2) = 0;
                }
                else
                {
                    tmp = points.col(j);
                    ctrl_pts_law = points.col(j + 1) - points.col(j - 1);
                }

                int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id;  // 计算A*搜索后的路径点索引，用于计算投影
                // 若val>0，则代表着与A*搜索点的夹角小于90度，反之大于90度。投影需要找到一个与控制点成90度的交点
                
                double val = (a_star_pathes[i][Astar_id] - tmp).dot(ctrl_pts_law), init_val = val;

                while(true)
                {
                    last_Astar_id = Astar_id;

                    if(val >= 0)    // val大于0，自减，夹角会变大直到val变为负
                    {
                        --Astar_id;
                        if(Astar_id < 0)
                        {
                            break;
                        }
                    }
                    else    // val小于0，自增，夹角会变小直到val变为非正
                    {
                        ++Astar_id; // 自增，夹角会变大直到val变为负值
                        if(Astar_id >= (int)a_star_pathes[i].size())
                        {
                            break;
                        }
                    }
                    // 更新val值
                    val = (a_star_pathes[i][Astar_id] - tmp).dot(ctrl_pts_law);

                    if(init_val * val <= 0 && (abs(val) > 0 || abs(init_val) > 0))
                    {
                        // 根据相似的原理可以得到垂直于控制点的交点
                        intersection_point =
                            a_star_pathes[i][Astar_id] +
                            ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                            (ctrl_pts_law.dot(tmp - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]))
                            );
                        got_intersection_id = j;
                        break;
                    }
                }

                // 若得到交点，进行一次插值，求得该交点投影在障碍物上的点p
                if(got_intersection_id >= 0)
                {
                    // 计算到该交点的长度，用于后续插值
                    double length = (intersection_point - tmp).norm();
                    if(length > 1e-5)   // 长度不为0，1e-5为截断误差
                    {
                        flag_temp_[j] = true;
                        for(double a = length; a >= 0.0; a -= resolution)
                        {
                            bool occ = isOccupancied((a / length) * intersection_point + (1 - a / length) * tmp);    // 判断该店是否在障碍物里
                            if(occ || a < resolution)   // 若每次插值点在障碍物里，投影点往外移动一个分辨率单位；若之前的插值发现都没有在障碍物里，则直接更新投影点
                            {
                                if(occ)
                                    a += resolution;
                                base_point_[j].push_back((a / length) * intersection_point + (1 - a / length) * tmp);
                                direction_[j].push_back ((intersection_point - tmp).normalized());
                                break;
                            }
                        }
                    }
                    else
                    {
                        got_intersection_id = -1;   // 获取失败
                    }
                }
            }

            if(got_intersection_id >= 0)
            {
                for(int j = got_intersection_id + 1; j <= segment_ids[i].second; ++j)
                {
                    if(!flag_temp_[j])
                    {
                        base_point_[j].push_back(base_point_[j - 1].back());
                        direction_[j].push_back(direction_[j - 1].back());
                    }
                }
                for(int j = got_intersection_id - 1; j >= segment_ids[i].first; --j)
                {
                    if(!flag_temp_[j])
                    {
                        base_point_[j].push_back(base_point_[j + 1].back());
                        direction_[j].push_back(direction_[j + 1].back());
                    }
                }
            }
        }

        force_stop_type_ = STOP_FOR_REBOUND;
        return true;
    }
    return false;
}

bool RFS_EGO::allowRebound(const Eigen::MatrixXd &samplePoint)
{
    if(iter_num_ < 3)
        return false;
    
    double min_product = 1;
    for(int i = 3; i <= samplePoint.cols() - 4; ++i)
    {
        double product = 
        ((samplePoint.col(i) - samplePoint.col(i - 1)).normalized()).dot((samplePoint.col(i + 1) - samplePoint.col(i)).normalized());
        if(product < min_product)
            min_product = product;
    }
    if(min_product < 0.87) // about 30 degree
        return false;

    return true;
}

void RFS_EGO::compute3D_Pose(std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> &pose)
{
    // 获取位置
    Eigen::MatrixXd wps = minco_ptr_->getSampleStates(K_, 0);
    // 获取速度
    Eigen::MatrixXd vel = minco_ptr_->getSampleStates(K_, 1);
    // 获取加速度
    Eigen::MatrixXd acc = minco_ptr_->getSampleStates(K_, 2);
    // 重力加速度
    Eigen::Vector3d gravity(0, 0, -9.81);
    // 升力方向
    Eigen::Vector3d n_f;
    // 加速度方向
    Eigen::Vector3d n_a;
    // 与升力和加速度垂直的方向，即机体坐标系下的y轴方向(FLU)
    Eigen::Vector3d n_af;
    // 计算机头朝向
    Eigen::Vector3d n_lk;
    // 当前姿态
    Eigen::Matrix3d R_wb;

    std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> ().swap(pose);
    pose.resize(acc.cols());

    // 获取姿态
    for(int i = 0; i < acc.cols(); ++i)
    {
        if(!hasZeroElement(acc.col(i)))
        {
            // 计算加速度方向
            // n_a = acc.col(i).normalized();
            n_a = vel.col(i).normalized();
            // 计算升力方向
            n_f = (acc.col(i) - gravity).normalized();
            // 计算升力和加速度垂直的方向
            n_af = n_f.cross(n_a).normalized();
            // 计算机头方向
            n_lk = n_af.cross(n_f).normalized();
        }
        else
        {
            // 悬停状态
            n_f = -1.0 * gravity.normalized();
            if(i < acc.cols() - 1)
            {
                // n_lk = acc.col(i + 1) - acc.col(i);
                n_lk = vel.col(i + 1) - vel.col(i);
            }
            else    // 最后一个点
            {
                // n_lk = acc.col(i) - acc.col(i - 1);
                n_lk = vel.col(i) - vel.col(i - 1);
            }
            n_lk(2) = 0;
            n_lk.normalize();
            n_af = n_f.cross(n_lk).normalized();
            
        }
        R_wb << n_lk, n_af, n_f;

#ifdef _RFS_EGO_LOG_
        if(!isRotationMatrix(R_wb))
        {
            std::cout << "acc is (" << acc.col(i).x() << ", " << acc.col(i).y() << ", " << acc.col(i).z() << ")\n"; 
            std::cout << "Error Rotation Matrix" << std::endl;
        }
#endif  // _RFS_EGO_LOG_

        Eigen::Quaterniond q(R_wb);

#ifdef _RFS_EGO_LOG_
        std::cout << "四元数:\n" << q.coeffs().transpose() << std::endl;
        std::cout << "q.norm() " << q.norm() << std::endl;
#endif  // _RFS_EGO_LOG_

        pose.push_back(
            std::pair<Eigen::Vector3d, Eigen::Quaterniond>(wps.col(i), q)
        );
    }
}

void RFS_EGO::compute2D_Pose(std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> &pose)
{
    // 获取位置
    Eigen::MatrixXd wps = minco_ptr_->getSampleStates(1, 0);
    // 获取速度
    Eigen::MatrixXd vel = minco_ptr_->getSampleStates(1, 1);
    Eigen::Vector3d n_z(0, 0, 1);
    Eigen::Vector3d n_x;
    Eigen::Vector3d n_y;

    // 当前姿态
    Eigen::Matrix3d R_wb;

    std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> ().swap(pose);
    pose.resize(vel.cols());
    for(int i = 0; i < vel.cols(); ++i)
    {
        n_x = vel.normalized();
        n_y = n_z.cross(n_x);
        R_wb << n_x, n_y, n_z;
        pose.push_back(
            std::pair<Eigen::Vector3d, Eigen::Quaterniond>(wps.col(i), Eigen::Quaterniond(R_wb))
        );
    }
}
