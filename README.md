# autoware-1.8.0-test

测试环境：
ubuntu 16.04
ros kinetic
qt5.6.2
opencv 3.4.8 
opencv安装参考：https://blog.csdn.net/CSDNZSX/article/details/104154645

autoware 1.8.0
autoware安装参考：https://gitlab.com/autowarefoundation/autoware.ai/autoware/-/wikis/Source-Build
autoware1.8.0版本源码：https://gitlab.com/autowarefoundation/autoware.ai/autoware/-/tree/1.8.0

autoware安装过程：
$ cd ~/autoware-1.8.0/ros/src
$ catkin_init_workspace
$ cd ../
$ rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
这里如果报错参考下面的bug参考
$ ./catkin_make_release

编译有bug参考：
https://blog.csdn.net/yourgreatfather/article/details/86504547

矢量地图编辑：
https://tools.tier4.jp/
全局规划，局部规划，避障都依赖矢量地图。

自己增加了cpp文件夹里面的代码，以适应每一辆小车使用。
1、speed.cpp 速度转化，将autoware里面的/twist_cmd转化为常用的/cmd_vel
2、point_test.cpp 原始点云裁剪为所需要的点云，还有聚类
3、由于聚类后还需要将聚类的点云簇框选出来，还没时间仔细看源码，暂时用网上的euclidean_cluster，参考：https://download.csdn.net/download/cquszj/12088810
4、click.cpp 将聚类后框选出来的障碍物的x,y坐标回调回来，再转化为/clicked_point主题发送，适应autoware里面的op_perception_simulator功能模块。目前只能添加一个障碍物，还不能做到多障碍物共同添加进去。

autoware本身作者修改了一个小参数，导致本地规划时候不能完全识别障碍物，将lidar_kf_contour_track_core.cpp里面的126行，nh.getParam("/lidar_kf_contour_track/enableTTC", m_Praams.bEnableTTC);原本注释掉的修改回去。实际测试时候可以识别障碍物。

修改op_perception_simulator_core.cpp的代码，增加聚类后的/detected_bounding_boxs回调，使用障碍物动态的长宽高尺寸，替换原本固定的长宽高尺寸。

实际绕行测试：
1、启动雷达
$ roslaunch rslidar_pointcloud rs_lidar_16.launch
雷达的fram_id改为/velodyne，雷达的点云主题改为/points_raw

2、启动autoware
绕行方案 总结（本地绕行）：
启动Autoware本地规划仿真：
Setup
->TF
->Vehicle Info

Map
->Point Cloud
->Vector Map
->TF

Computing
->Localization->autoware_connector->vel_pose_connector
选择Simulation Mode

->Detection->lidar_tracker->lidar_kf_contour_track

->Mission Planning->OpenPlanner-Global Planning->op_global_planner
选择Rviz Goals
这时候在RVIZ上面显示的矢量地图的道路上选择起点和终点

->Motion Planning->OpenPlanner-Local_planning里面5个模块都要勾选
->op_common_param
->op_trajectory_generator
->op_motion_predictor 选择Enable Branching
->op_trajectory_evaluator 选择Enable Prediction
->op_behavior_selector

->OpenPlanner-Simulator->op_perception_simulator

->waypoint_follower->twist_filter
->waypoint_follower->pure_pursuit小车行走

这时候在RVIZ上给仿真小车设置起点（需要在矢量地图的道路上），会出现绿色方框的小车和白色外圆圈的仿真雷达数据
然后在RVIZ上点击Op Flag，给局部路径设置障碍物位置

3、启动点云裁剪、聚类和发布障碍物坐标
$ rosrun point_test point_test
$ roslaunch euclidean_cluster euclidean_cluster.launch
$ rosrun point_test click

4、启动真实小车
$ rosrun point_test speed //这个是速度转化
$ rosrun point_test main //这个小车跟底盘通讯的/cmd_vel主题，结合自己实际写
