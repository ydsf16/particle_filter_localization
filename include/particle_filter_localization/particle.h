// Copyright (C) 2018 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#ifndef PARTICLE_H
#define PARTICLE_H

#include <particle_filter_localization/Pose2d.h>

class Particle{
public:
    Particle(Pose2d pose, long double weight): pose_(pose), weight_(weight){}
public:
    Pose2d pose_;
    long double weight_;
};

#endif

