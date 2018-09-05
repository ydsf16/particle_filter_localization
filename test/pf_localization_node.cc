// Copyright (C) 2018 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <particle_filter_localization/grid_map.h>

#include <particle_filter_localization/localizer.h>

/* Global */
GridMap* g_map;
ros::Subscriber g_odom_suber;
ros::Publisher g_map_puber, g_particles_puber;
Localizer* g_localizer;

void odometryCallback ( const nav_msgs::OdometryConstPtr& odom );
void laserCallback ( const sensor_msgs::LaserScanConstPtr& scan );

int main ( int argc, char **argv )
{
    /***** 初始化ROS *****/
    ros::init ( argc, argv, "PF_Localization" );
    ros::NodeHandle nh;

    /* 加载地图 */
    std::string map_img_dir, map_cfg_dir;
    int initx, inity;
    double cell_size;
    /* TODO 错误处理 */
    nh.getParam ( "/pf_localization_node/map_dir", map_img_dir );
    nh.getParam ( "/pf_localization_node/map/initx", initx );
    nh.getParam ( "/pf_localization_node/map/inity", inity );
    nh.getParam ( "/pf_localization_node/map/cell_size", cell_size );
    GridMap* map = new GridMap();
    map->loadMap ( map_img_dir, initx, inity, cell_size );
    g_map = map;

    /* 初始化机器人 */
    Pose2d T_r_l;
    double x, y, theta;
    /* TODO 错误处理 */
    nh.getParam ( "/pf_localization_node/robot_laser/x", x );
    nh.getParam ( "/pf_localization_node/robot_laser/y", y );
    nh.getParam ( "/pf_localization_node/robot_laser/theta", theta );
    T_r_l = Pose2d ( x, y, theta );
    Robot* robot = new Robot ( 0.0, 0.0, 0.0, T_r_l );

    /* 初始化运动模型 */
    double a1, a2, a3, a4;
    nh.getParam ( "/pf_localization_node/motion_model/alpha1", a1 );
    nh.getParam ( "/pf_localization_node/motion_model/alpha2", a2 );
    nh.getParam ( "/pf_localization_node/motion_model/alpha3", a3 );
    nh.getParam ( "/pf_localization_node/motion_model/alpha4", a4 );
    MotionModel* motion_model = new MotionModel ( a1, a2, a3, a4 );

    /* 初始化观测模型 */
    double sigma, rand;
    nh.getParam ( "/pf_localization_node/measurement_model/sigma", sigma );
    nh.getParam ( "/pf_localization_node/measurement_model/rand", rand );
    std::cout << "\n\n正在计算似然域模型, 请稍后......\n\n";
    MeasurementModel* measurement_model = new MeasurementModel ( map, sigma, rand );
    std::cout << "\n\n似然域模型计算完毕.  开始接收数据.\n\n";

    /* 初始化定位器 */
    int n_particles;
    nh.getParam ( "/pf_localization_node/particle_filter/n_particles", n_particles );
    g_localizer = new Localizer ( robot, map, motion_model, measurement_model, n_particles );

    /* 初始Topic */
    g_odom_suber = nh.subscribe ( "/mbot/odometry", 1, odometryCallback );
    ros::Subscriber laser_suber = nh.subscribe ( "/scan", 1, laserCallback );
    g_map_puber = nh.advertise<nav_msgs::OccupancyGrid> ( "pf_localization/grid_map", 1 );
    g_particles_puber = nh.advertise<geometry_msgs::PoseArray> ( "pf_localization/particles", 1 );
    ros::spin();
}

void odometryCallback ( const nav_msgs::OdometryConstPtr& odom )
{
    /* 获取机器人姿态 */
    double x = odom->pose.pose.position.x;
    double y = odom->pose.pose.position.y;
    double theta = tf::getYaw ( odom->pose.pose.orientation );
    Pose2d rpose ( x, y, theta );

    /* 运动更新 */
    g_localizer->motionUpdate ( rpose );

    /* 发布粒子 */
    geometry_msgs::PoseArray pose_array;
    g_localizer->particles2RosPoseArray ( pose_array );
    g_particles_puber.publish ( pose_array );

}

void laserCallback ( const sensor_msgs::LaserScanConstPtr& scan )
{
    /* 测量更新 */
    g_localizer->measurementUpdate ( scan );
    
    /* 低频发布地图 */
    static int cnt = -1;
    if  ( (cnt ++)  % 100 == 0 ) 
    {
        nav_msgs::OccupancyGrid occ_map;
        g_map->toRosOccGridMap ( "odom", occ_map );
        g_map_puber.publish ( occ_map );
    }
}

// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
