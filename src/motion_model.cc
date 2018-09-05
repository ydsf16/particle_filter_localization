// Copyright (C) 2018 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#include <particle_filter_localization/motion_model.h>

MotionModel::MotionModel ( const double& alpha1, const double& alpha2, const double& alpha3, const double& alpha4 ):
alpha1_(alpha1), alpha2_(alpha2), alpha3_(alpha3), alpha4_(alpha4)
{
    rng_ = cv::RNG(cv::getTickCount());
}

void MotionModel::sampleMotionModelOdometry ( const double& delta_rot1, const double& delta_trans, const double& delta_rot2, Pose2d& xt )
{
    /* 处理后退的问题 */
    double delta_rot1_PI = delta_rot1 - PI;
    double delta_rot2_PI = delta_rot2 - PI;
    Pose2d::NormAngle(delta_rot1_PI);
    Pose2d::NormAngle(delta_rot2_PI);
    double delta_rot1_noise = std::min(fabs(delta_rot1), fabs( delta_rot1_PI) );
    double delta_rot2_noise = std::min(fabs(delta_rot2), fabs( delta_rot2_PI) );
    
    double delta_rot1_2 = delta_rot1_noise*delta_rot1_noise;
    double delta_rot2_2 = delta_rot2_noise * delta_rot2_noise;
    double delta_trans_2 = delta_trans * delta_trans;
    
    /* 采样 */
    double delta_rot1_hat = delta_rot1 - rng_.gaussian(alpha1_ * delta_rot1_2 + alpha2_ * delta_trans_2);
    double delta_trans_hat = delta_trans - rng_.gaussian(alpha3_ * delta_trans_2 + alpha4_ * delta_rot1_2 + alpha4_ * delta_rot2_2);
    double delta_rot2_hat = delta_rot2 - rng_.gaussian(alpha1_ * delta_rot2_2 + alpha2_ * delta_trans_2);
    
    xt.x_ += delta_trans_hat * cos( xt.theta_ + delta_rot1_hat );
    xt.y_  += delta_trans_hat * sin( xt.theta_ + delta_rot1_hat );
    xt.theta_ +=  (delta_rot1_hat + delta_rot2_hat);
    xt.NormAngle(xt.theta_);
}
