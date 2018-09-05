// Copyright (C) 2018 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#include <particle_filter_localization/localizer.h>
#include <tf/transform_broadcaster.h>

Localizer::Localizer ( Robot* robot, GridMap* map, MotionModel* motion_model, MeasurementModel* measurement_model , size_t nParticles ) :
    robot_ ( robot ), map_ ( map ), motion_model_ ( motion_model ), measurement_model_ ( measurement_model ),nParticles_ ( nParticles )
{
    is_init_ = false;

    /* 初始化粒子 */
    const double minX = map->minX();
    const double maxX = map->maxX();
    const double minY = map->minY();
    const double maxY = map->maxY();
    cv::RNG rng ( cv::getTickCount() );
    size_t N = 0;
    double weight = 1.0 / nParticles;
    while ( N < nParticles )
    {
        double x = rng.uniform ( minX, maxX );
        double y = rng.uniform ( minY, maxY );
        double th = rng.uniform ( -PI, PI );

        /* 判断是否在地图内 & 判断是否在可行区域内 */
        double bel;
        if ( !map->getGridBel ( x, y, bel ) )
        {
            continue;
        }
        if ( bel != 0.0 ) //0.0的区域是可行驶区域
        {
            continue;
        }

        /* 构造一个粒子 */
        N++;
        Particle pt ( Pose2d ( x, y, th ), weight );
        particles_.push_back ( pt );
    }
}

void Localizer::motionUpdate ( const Pose2d& odom )
{
    const double MIN_DIST = 0.02; // TODO param 如果运动太近的话就不更新了，防止出现角度计算错误
    if ( !is_init_ ) //首次
    {
        last_odom_pose_ = odom;
        is_init_ = true;
        return;
    }

    /* 计算ut */
    double dx = odom.x_ - last_odom_pose_.x_;
    double dy = odom.y_ - last_odom_pose_.y_;
    double delta_trans = sqrt ( dx * dx + dy * dy );
    double delta_rot1 = atan2 ( dy,  dx ) - last_odom_pose_.theta_;
    Pose2d::NormAngle ( delta_rot1 );
    /* 处理纯旋转 */
    if(delta_trans < 0.01) 
        delta_rot1 = 0;
    double delta_rot2 = odom.theta_ - last_odom_pose_.theta_ - delta_rot1;
    Pose2d::NormAngle ( delta_rot2 );
    
    for ( size_t i = 0; i < nParticles_; i ++ )
    {
        motion_model_->sampleMotionModelOdometry ( delta_rot1, delta_trans, delta_rot2, particles_.at ( i ).pose_ ); // for each particle
    }

    last_odom_pose_ = odom;
}


void Localizer::measurementUpdate ( const sensor_msgs::LaserScanConstPtr& scan )
{
    if ( !is_init_ )
    {
        return;
    }

    /* 获取激光的信息 */
    const double& ang_min = scan->angle_min;
    const double& ang_max = scan->angle_max;
    const double& ang_inc = scan->angle_increment;
    const double& range_max = scan->range_max;
    const double& range_min = scan->range_min;

    for ( size_t np = 0; np < nParticles_; np ++ )
    {
        Particle& pt = particles_.at ( np );
        /* for every laser beam */
        for ( size_t i = 0; i < scan->ranges.size(); i ++ )
        {
            /* 获取当前beam的距离 */
            const double& R = scan->ranges.at ( i );
            if ( R > range_max || R < range_min )
                continue;

            double angle = ang_inc * i + ang_min;
            double cangle = cos ( angle );
            double sangle = sin ( angle );
            Eigen::Vector2d p_l (
                R * cangle,
                R* sangle
            ); //在激光雷达坐标系下的坐标

            /* 转换到世界坐标系下 */
            Pose2d laser_pose = pt.pose_ * robot_->T_r_l_;
            Eigen::Vector2d p_w = laser_pose * p_l;
           
            /* 更新weight */
            double likelihood = measurement_model_->getGridLikelihood ( p_w ( 0 ), p_w ( 1 ) );
            
            /* 整合多个激光beam用的加法， 用乘法收敛的太快 */
            pt.weight_ += likelihood;
        }// for every laser beam
    } // for every particle

    /* 权重归一化 */
    normalization();
    
    /* TODO 这里最好使用书上的Argument 的重采样 + KLD重采样方法. 
     *重采样频率太高会导致粒子迅速退化*/
    static int cnt = -1;
    if  ( (cnt ++)  % 10 == 0 ) //减少重采样的次数
        reSample();    //重采样
}

void Localizer::normalization()
{
    if ( !is_init_ )
    {
        return;
    }

    long double sum = 0;
    for ( size_t i = 0; i < nParticles_; i ++ )
    {
        Particle& pt = particles_.at ( i );
        
        /* 走出地图的粒子干掉 */
        double bel = 0.0;
        if (!map_->getGridBel(pt.pose_.x_, pt.pose_.y_, bel) )
            pt.weight_ = 0.0;
//         else if (bel != 0.0) //如果走到未知区域就给一个很低的权值
//             pt.weight_ = 1.0 / nParticles_;
        
        sum += pt.weight_;
    }
    long double eta = 1.0 / sum;
    for ( size_t i = 0; i < nParticles_; i ++ )
    {
        Particle& pt = particles_.at ( i );
        pt.weight_ *= eta;
    }
}


void Localizer::reSample()
{
    if ( !is_init_ )
        return;

    /* 重采样 */
    std::vector<Particle> new_particles;
    cv::RNG rng ( cv::getTickCount() );
    long double inc = 1.0 / nParticles_;
    long double r = rng.uniform ( 0.0, inc );
    long double c = particles_.at ( 0 ).weight_;
    int i = 0;
    
    for ( size_t m = 0; m < nParticles_; m ++ )
    {
        long double U = r + m * inc;
        while ( U > c )
        {
            i = i+1;
            c = c + particles_.at ( i ).weight_;
        }
        particles_.at ( i ).weight_ = inc;
        
        /* 新采样的粒子加了高斯，稍微增加一下多样性 */
        Particle new_pt = particles_.at(i);
        new_pt.pose_.x_ += rng.gaussian(0.02);
        new_pt.pose_.y_ += rng.gaussian(0.02);
        new_pt.pose_.theta_ += rng.gaussian(0.02);
        Pose2d::NormAngle(new_pt.pose_.theta_);
        new_particles.push_back ( new_pt );
    }
    particles_ = new_particles;
}

void Localizer::particles2RosPoseArray ( geometry_msgs::PoseArray& pose_array )
{
    pose_array.header.frame_id = "odom";
    pose_array.header.stamp = ros::Time::now();
    for ( size_t i = 0; i < particles_.size(); i ++ )
    {
        Particle&  pt = particles_.at ( i );
        geometry_msgs::Pose pose;
        pose.position.x = pt.pose_.x_;
        pose.position.y = pt.pose_.y_;
        pose.position.z = 0;
        pose.orientation = tf::createQuaternionMsgFromYaw ( pt.pose_.theta_ );
        pose_array.poses.push_back ( pose );
    }
}

// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
