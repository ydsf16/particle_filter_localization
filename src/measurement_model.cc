// Copyright (C) 2018 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#include <particle_filter_localization/measurement_model.h>
#define PI 3.1415926

MeasurementModel::MeasurementModel ( GridMap* map, const double& sigma, const double& rand ):
map_(map), sigma_(sigma), rand_(rand)
{
    /* 初始化似然域模型, 预先计算的似然域模型的格子大小比占有栅格小1倍 */
    size_x_ = 2 * map->size_x_;
    size_y_ = 2 * map->size_y_;
    init_x_ = 2 * map->init_x_;
    init_y_ = 2 * map->init_y_;
    cell_size_ = 0.5 * map->cell_size_;
    
    likelihood_data_.resize(size_x_ , size_y_); 
    likelihood_data_.setZero();
    
    /* 构造障碍物的KD-Tree */
    pcl::PointCloud<pcl::PointXY>::Ptr cloud (new pcl::PointCloud<pcl::PointXY>);

    for(int i = 0; i < map->size_x_;  i ++)
        for(int j = 0; j < map->size_y_; j ++)
        {
            if(map->bel_data_(i, j) == 1.0) //是障碍物就加到KD数中
            {
                pcl::PointXY pt;
                pt.x = (i - map->init_x_) * map->cell_size_;
                pt.y = (j - map->init_y_) * map->cell_size_;
                cloud->push_back(pt);
            }
        }
    kd_tree_.setInputCloud(cloud); 
    
    /* 对每个格子计算likelihood */
    for(double i = 0; i < size_x_; i += 0.9)
        for(double j = 0; j < size_y_; j +=0.9)
        {
            /* 计算double x, y */
            double x = ( i - init_x_)* cell_size_;
            double y = ( j- init_y_ ) * cell_size_;
            double likelihood =  likelihoodFieldRangFinderModel(x, y);
            setGridLikelihood(x, y, likelihood);
        } 
}


double MeasurementModel::likelihoodFieldRangFinderModel ( const double& x, const double& y )
{
    /* 找到最近的距离 */
    pcl::PointXY search_point;
    search_point.x = x;
    search_point.y = y;
    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;
    int nFound =  kd_tree_.nearestKSearch(search_point, 1, k_indices, k_sqr_distances);
    double dist = k_sqr_distances.at(0);
    
    /* 高斯 + random */
    return gaussion(0.0, sigma_, dist) + rand_;
}

double MeasurementModel::gaussion ( const double& mu, const double& sigma, double x)
{
    return (1.0 / (sqrt( 2 * PI ) * sigma) ) * exp( -0.5 * (x-mu) * (x-mu) / (sigma* sigma) );
}

bool MeasurementModel::getIdx ( const double& x, const double& y, Eigen::Vector2i& idx )
{
    int xidx = cvFloor( x / cell_size_ ) + init_x_;
    int yidx  = cvFloor( y /cell_size_ )+ init_y_;
    
    if((xidx < 0) || (yidx < 0) || (xidx >= size_x_) || (yidx >= size_y_))
        return false;
    idx << xidx , yidx;
    return true;
}

double MeasurementModel::getGridLikelihood ( const double& x, const double& y)
{
    Eigen::Vector2i idx;
    if(!getIdx(x, y, idx))
        return rand_;
    return likelihood_data_(idx(0), idx(1));
}

bool MeasurementModel::setGridLikelihood ( const double& x, const double& y, const double& likelihood )
{
    Eigen::Vector2i idx;
    if(!getIdx(x, y, idx))
        return false;
    likelihood_data_(idx(0), idx(1)) = likelihood;
    return true;
}


cv::Mat MeasurementModel::toCvMat()
{
    /* 构造出opencv格式显示 */
    cv::Mat map(cv::Size(size_x_, size_y_), CV_64FC1, likelihood_data_.data(), cv::Mat::AUTO_STEP); 
    /* 翻转 */
    cv::flip(map, map, 0);
    return map;
}
