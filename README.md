# 激光SLAM回环检测算法demo

这里采用[LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)（半径搜索）、[Scan Context](https://github.com/gisbi-kim/SC-LIO-SAM)、[LiDAR-Iris](https://github.com/JoestarK/LiDAR-Iris)、[BoW3D](https://github.com/YungeCui/BoW3D)来做对比，效果如下：

LIO-SAM 原始基于半径范围内的ICP搜索

<img src="./img/RS.png" style="zoom:40%;" />

Scan Context

<img src="./img/Scan Context.png" style="zoom:40%;" />

LiDAR-Iris

<img src="./img/LiDAR-Iris.png" style="zoom:45%;" />

BoW3D

<img src="./img/BoW3D.png" style="zoom:40%;" />



## 运行

需要先安装LIO-SAM的环境，在Ubuntu20.04下测试

~~~shell
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone 
catkin build
roslaunch lio_sam run_garden.launch
roslaunch lidar_slam_loop_test iris_online.launch
rosbag play 1018_00_img10hz600p.bag
~~~

测试的数据集为[BotanicGarden](https://github.com/robot-pesg/BotanicGarden)