// Copyright (C) 2018 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H
#include <particle_filter_localization/Pose2d.h>
#include <opencv2/opencv.hpp>

class MotionModel{
public:
    MotionModel(const double& alpha1, const double& alpha2, const double& alpha3, const double& alpha4);
    void sampleMotionModelOdometry(const double& delta_rot1, const double& delta_trans, const double& delta_rot2, Pose2d& xt);
    
private:
    double alpha1_, alpha2_, alpha3_, alpha4_;
    cv::RNG rng_;
}; // class MotionModel


#endif
