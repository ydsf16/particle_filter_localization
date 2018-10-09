
## Particle Filter Localization 粒子滤波定位， 蒙特卡罗定位， 《概率机器人》第8章实现。

详细介绍请参考：https://zhuanlan.zhihu.com/p/43523632

数据集下载地址：https://pan.baidu.com/s/1j_SSEtaq7D0XwaED0Jg4Ew

## 使用方法

STEP1. 下载到自己的 ROS 工作空间：catkin_ws/src

STEP2. 编译：catkin_make

STEP3. 运行 launch: roslaunch particle_filter_localization pf_localization.launch

STEP4. 播放 rosbag: rosbag play laser1_2018-07-14-17-31-41.bag -r 2
