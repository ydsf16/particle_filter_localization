// Copyright (C) 2018 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#ifndef ROBOT_H
#define ROBOT_H

#include <particle_filter_localization/Pose2d.h>

class Robot{
public:
    Robot(const double& kl, const double& kr, const double& b, const Pose2d& T_r_l):
    kl_(kl), kr_(kr), b_(b), T_r_l_(T_r_l)
    {}
    
    double kl_, kr_, b_;
    Pose2d T_r_l_;
};//class Robot

#endif

