#include <ros/ros.h>
#include <string>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <visualization_msgs/MarkerArray.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <eigen3/Eigen/Dense>

#include <iostream>
#include <thread>
#include <sstream>
#include <iomanip>

#include "LidarIris/LidarIris.h"
#include <nabo/nabo.h>

class Loop_Clousre
{
public:
    Loop_Clousre(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
    ~Loop_Clousre();

    void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);

    void odometry_cb(const nav_msgs::Odometry::ConstPtr &odom_msg);

    void run_loop();

    void visual_loop();

private:
    ros::NodeHandle nh;

    ros::Subscriber pointcloud_sub;
    ros::Subscriber odom_sub;

    ros::Publisher key_point_pub;

    ros::Publisher loop_markers_pub;

    ros::Publisher path_pub;

    std::mutex mutex_lock;

    std::deque<sensor_msgs::PointCloud2> cloud_queue; // 原始点云点云队列
    std::deque<nav_msgs::Odometry> odom_queue;        // 里程计队列

    ros::Time curr_time;

    Eigen::Vector3d curr_position = Eigen::Vector3d::Zero();       // 当前位置
    Eigen::Quaterniond curr_quat = Eigen::Quaterniond::Identity(); // 当前时刻姿态四元数

    std::string lidar_topic;
    std::string odom_topic;
    std::string odom_frame;
    std::string lidar_frame;
    std::string sensor_type;

    // Iris
    int loop_event = 0;
    LidarIris iris; // 描述符生成、检索匹配器

    std::vector<LidarIris::FeatureDesc> dataset;
    Eigen::MatrixXf rowKey_mat;

    std::vector<geometry_msgs::Pose> pose_v;         // 存放历史帧的位姿
    std::vector<std::pair<int, int>> loop_pair_id_v; // 存放回环检测的帧id号

    std::vector<double> process_loop_t_v;
};

Loop_Clousre::Loop_Clousre(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
{
    nh.param<std::string>("lidar_topic", lidar_topic, "/velodyne_points");
    nh.param<std::string>("odom_topic", odom_topic, "/Odometry");
    nh.param<std::string>("odom_frame", odom_frame, "odom");        // 地图初始坐标frame_id
    nh.param<std::string>("lidar_frame", lidar_frame, "base_link"); // body系frame_id
    nh.param<std::string>("sensor_type", sensor_type, "Velodyne");  // body系frame_id

    pointcloud_sub = nh.subscribe(lidar_topic, 100, &Loop_Clousre::cloud_cb, this);
    odom_sub = nh.subscribe(odom_topic, 100, &Loop_Clousre::odometry_cb, this);

    loop_markers_pub = nh.advertise<visualization_msgs::MarkerArray>("loop_constriant", 1000, true);

    path_pub = nh.advertise<nav_msgs::Path>("/path", 1000, true);

    // 初始化描述符生成器
    iris.initial(4, 18, 1.6, 0.75, loop_event);


}

Loop_Clousre::~Loop_Clousre()
{
}

void Loop_Clousre::cloud_cb(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
{
    mutex_lock.lock();
    cloud_queue.push_back(*cloud_msg);
    mutex_lock.unlock();

}

void Loop_Clousre::odometry_cb(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    mutex_lock.lock();
    odom_queue.push_back(*odom_msg);
    mutex_lock.unlock();
}

void Loop_Clousre::run_loop()
{
    int cloudInd = 0;
    ros::Rate rate(10);
    while (ros::ok())
    {
        rate.sleep();

        if (cloud_queue.empty() || odom_queue.empty())
        {
            continue;
        }

        double t_cloud = cloud_queue.front().header.stamp.toSec();
        double t_odom = odom_queue.front().header.stamp.toSec();

        if (t_cloud != t_odom)
        {
            ROS_ERROR("Cloud and odometry messages unsync, skip the frame!");

            mutex_lock.lock();
            cloud_queue.clear();
            odom_queue.clear();
            mutex_lock.unlock();
            continue;
        }

        sensor_msgs::PointCloud2 laserCloud_curr;
        nav_msgs::Odometry odom_curr;

        mutex_lock.lock();

        laserCloud_curr = cloud_queue.front();
        odom_curr = odom_queue.front();
        curr_time = laserCloud_curr.header.stamp;

        curr_position << odom_curr.pose.pose.position.x, odom_curr.pose.pose.position.y, odom_curr.pose.pose.position.z;
        curr_quat = Eigen::Quaterniond(odom_curr.pose.pose.orientation.w, odom_curr.pose.pose.orientation.x,
                                       odom_curr.pose.pose.orientation.y, odom_curr.pose.pose.orientation.z);
        cloud_queue.pop_front();
        odom_queue.pop_front();

        mutex_lock.unlock();

        pose_v.push_back(odom_curr.pose.pose); // 将位姿加入到历史当中

        // LiDAR-Iris 回环检测
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(laserCloud_curr, *cloudXYZ);


        ros::Time t1 = ros::Time::now();

        Eigen::VectorXf rowkey;
        cv::Mat1b li1 = LidarIris::GetIris(*cloudXYZ, rowkey);

        LidarIris::FeatureDesc fd1 = iris.GetFeature(li1);
        dataset.push_back(fd1);

        if (rowKey_mat.cols() > 300)
        {
            // create a kd-tree for M, note that M must stay valid during the lifetime of the kd-tree
            Nabo::NNSearchF *rowKey_kdtree = Nabo::NNSearchF::createKDTreeLinearHeap(rowKey_mat);

            // look for the 5 nearest neighbour of a the single-point query
            const int candidates_num_ = 8;
            Eigen::VectorXi indice(candidates_num_);
            Eigen::VectorXf distance(candidates_num_);
            rowKey_kdtree->knn(rowkey, indice, distance, candidates_num_); // indices是索引，dists2是距离平方

            float min_distance = 100000.0;
            int min_index = -1;
            int min_bias = 0;
            for (int j = 0; j < int(indice.size()); j++)
            {
                if (abs(indice[j] - cloudInd) < 300)
                {
                    continue;
                }

                LidarIris::FeatureDesc fd2 = dataset[indice[j]];

                int bias;
                float candidate_distance = iris.Compare(fd1, fd2, &bias);

                if (candidate_distance < min_distance)
                {
                    min_distance = candidate_distance;
                    min_index = indice[j];
                    min_bias = bias;
                }
            }

            float distance_threshold_ = 0.35;

            ros::Time t2 = ros::Time::now();
            double time = (t2 - t1).toSec() * 1000;

            process_loop_t_v.push_back(time);
            double average_t = std::accumulate(process_loop_t_v.begin(), process_loop_t_v.end(), 0.0) / process_loop_t_v.size();


            if (min_distance < distance_threshold_ )
            {
                std::cout << "process time: " << time << "ms, average: " << average_t << std::endl;

                std::cout << "Loop detection: <" << cloudInd << ", " << min_index << ">, min_distance : " << min_distance << std::endl
                          << std::endl;

                loop_pair_id_v.push_back({cloudInd, min_index});

                //  可视化回环
                visual_loop();
            }
        }

        // 将描述子key添加到矩阵中
        if (rowKey_mat.cols() == 0)
        {
            rowKey_mat.conservativeResize(rowkey.rows(), 1);
            rowKey_mat = rowkey;
        }
        else
        {
            rowKey_mat.conservativeResize(rowKey_mat.rows(), rowKey_mat.cols() + 1);
            rowKey_mat.col(rowKey_mat.cols() - 1) = rowkey;
        }

        cloudInd++;
    }
}

void Loop_Clousre::visual_loop()
{
    visualization_msgs::MarkerArray loop_markers_msg;
    for (auto &l : loop_pair_id_v)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = odom_frame;
        marker.header.stamp = curr_time;
        marker.ns = "loop";
        marker.id = loop_markers_msg.markers.size();
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1;
        marker.color.r = 1; // 白色
        marker.color.g = 1;
        marker.color.b = 1;
        marker.pose = pose_v[l.first];
        loop_markers_msg.markers.push_back(marker);

        marker.id = loop_markers_msg.markers.size();
        marker.pose = pose_v[l.second];
        loop_markers_msg.markers.push_back(marker);

        // 连线
        visualization_msgs::Marker markerEdge;
        markerEdge.header.frame_id = odom_frame;
        markerEdge.header.stamp = curr_time;
        markerEdge.action = visualization_msgs::Marker::ADD;
        markerEdge.type = visualization_msgs::Marker::LINE_LIST;
        markerEdge.ns = "loop_edge";
        markerEdge.id = loop_markers_msg.markers.size();
        markerEdge.pose.orientation.w = 1;
        markerEdge.scale.x = 0.1;
        markerEdge.color.r = 1;
        markerEdge.color.g = 1;
        markerEdge.color.b = 0;
        markerEdge.color.a = 0.9;
        geometry_msgs::Point p;
        p = pose_v[l.first].position;
        markerEdge.points.push_back(p);
        p = pose_v[l.second].position;
        markerEdge.points.push_back(p);
        loop_markers_msg.markers.push_back(markerEdge); // 回环的连线
    }

    loop_markers_pub.publish(loop_markers_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Iris");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    Loop_Clousre lc(nh, nh_private);

    std::thread process_thread(&Loop_Clousre::run_loop, &lc);

    ros::spin();

    process_thread.join();

    return 0;
}
