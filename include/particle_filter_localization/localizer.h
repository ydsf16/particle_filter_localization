// Copyright (C) 2018 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <particle_filter_localization/robot.h>
#include <particle_filter_localization/grid_map.h>
#include <particle_filter_localization/measurement_model.h>
#include <particle_filter_localization/motion_model.h>
#include <particle_filter_localization/particle.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>

class Localizer{
public:
    Localizer(Robot* robot, GridMap* map, MotionModel* motion_model, MeasurementModel* measurement_model, size_t nParticles);
    void motionUpdate(const Pose2d& odom);
    void measurementUpdate(const sensor_msgs::LaserScanConstPtr& scan );
    void reSample();
    void particles2RosPoseArray(geometry_msgs::PoseArray& pose_array);
    void normalization();
    
private:
    Robot* robot_;
    GridMap* map_;
    MotionModel* motion_model_;
    MeasurementModel* measurement_model_;
    
    size_t nParticles_;
    std::vector<Particle> particles_; //粒子
    
    /* sys statues */
    bool is_init_;
    Pose2d last_odom_pose_;
}; //class Localizer

#endif

