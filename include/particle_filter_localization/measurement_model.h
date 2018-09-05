// Copyright (C) 2018 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#ifndef MEASUREMENT_MODEL
#define MEASUREMENT_MODEL

#include <particle_filter_localization/grid_map.h>
#include <pcl/kdtree/kdtree_flann.h>

class MeasurementModel{
public:
    MeasurementModel(GridMap* map, const double& sigma, const double& rand);
    double likelihoodFieldRangFinderModel(const double& x, const double& y); 
    
    bool getIdx(const double& x, const double& y, Eigen::Vector2i& idx);
    bool setGridLikelihood(const double& x, const double& y, const double& likelihood);
    double getGridLikelihood ( const double& x, const double& y);
    cv::Mat toCvMat(); //转换到Opencv的图片格式
private:
    double gaussion(const double& mu, const double& sigma, double x);
    
    GridMap* map_;
    double sigma_;
    double rand_;
    
    pcl::KdTreeFLANN<pcl::PointXY> kd_tree_;
    
    /* 预先计算似然域的数据 */
    Eigen::MatrixXd likelihood_data_;
    int size_x_, size_y_, init_x_, init_y_;
    double cell_size_;
    
    
}; //class MeasurementModel

#endif


