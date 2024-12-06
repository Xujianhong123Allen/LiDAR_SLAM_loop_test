#include <ros/ros.h>
#include <string>
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <eigen3/Eigen/Dense>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <thread>
#include <sstream>
#include <iomanip>
#include "scancontext/Scancontext.h"

using namespace std;

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(float, time, time))

struct OusterPointXYZIRT
{
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    // uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint32_t, t, t)(uint16_t, reflectivity, reflectivity)(uint8_t, ring, ring)(uint32_t, range, range))

class Loop_Clousre
{
public:
    Loop_Clousre(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
    ~Loop_Clousre();

    void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);

    void odometry_cb(const nav_msgs::Odometry::ConstPtr &odom_msg);

    void run_loop();

    void visual_loop();

    void generate_desc(pcl::PointCloud<VelodynePointXYZIRT>::Ptr laserCloud, Eigen::MatrixXd &desc_mat);

private:
    ros::NodeHandle nh;

    ros::Subscriber pointcloud_sub;
    ros::Subscriber odom_sub;

    ros::Publisher key_point_pub;

    ros::Publisher loop_markers_pub;

    ros::Publisher descriptor_image_pub;

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

    // Parameters of LinK3D
    int nScans = 32; // Number of LiDAR scan lines
    float scanPeriod = 0.1;
    float minimumRange = 1;
    float distanceTh = 0.4;
    int matchTh = 6;

    // loop detector
    SCManager scManager;

    std::vector<geometry_msgs::Pose> pose_v;                   // 存放历史帧的位姿
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_v; // 用于存储历史帧的点云（局部坐标系下）
    std::vector<std::pair<int, int>> loop_pair_id_v;           // 存放回环检测的帧id号
};

Loop_Clousre::Loop_Clousre(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
{
    nh.param<std::string>("lidar_topic", lidar_topic, "/velodyne_points");
    nh.param<std::string>("odom_topic", odom_topic, "/Odometry");
    nh.param<std::string>("odom_frame", odom_frame, "odom");  // 地图初始坐标frame_id
    nh.param<std::string>("lidar_frame", lidar_frame, "base_link"); // body系frame_id

    pointcloud_sub = nh.subscribe(lidar_topic, 100, &Loop_Clousre::cloud_cb, this);
    odom_sub = nh.subscribe(odom_topic, 100, &Loop_Clousre::odometry_cb, this);

    loop_markers_pub = nh.advertise<visualization_msgs::MarkerArray>("loop_constriant", 1000, true);
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

        /* **************************************** 执行回环检测 ***************************************** */
        // 原始ScanContext
        pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(laserCloud_curr, *current_cloud);

        pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterSC;                           // giseop
        const float kSCFilterSize = 0.4;                                           // giseop
        downSizeFilterSC.setLeafSize(kSCFilterSize, kSCFilterSize, kSCFilterSize); // giseop

        pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud_ds(new pcl::PointCloud<pcl::PointXYZI>());

        downSizeFilterSC.setInputCloud(current_cloud);
        downSizeFilterSC.filter(*current_cloud_ds);

        cloud_v.push_back(current_cloud_ds);

        scManager.makeAndSaveScancontextAndKeys(*current_cloud_ds); // 将当前帧加入SC中

        Eigen::MatrixXd SC = scManager.getSC();

        // find keys
        auto detectResult = scManager.detectLoopClosureID(); // first: nn index, second: yaw diff
        int loopKeyCur = pose_v.size() - 1;
        int loopKeyPre = detectResult.first;
        float yawDiffRad = detectResult.second; // not use for v1 (because pcl icp withi initial somthing wrong...)
        if (loopKeyPre == -1 /* No loop found */)
            continue;

        if ((loopKeyCur - loopKeyPre) < 200)
        {
            continue;
        }

        std::cout << "SC loop found! between " << loopKeyCur << " and " << loopKeyPre << "." << std::endl; // giseop

        loop_pair_id_v.push_back(std::pair<int, int>(loopKeyCur, loopKeyPre));

        visual_loop();
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
    ros::init(argc, argv, "SC");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    Loop_Clousre lc(nh, nh_private);

    std::thread process_thread(&Loop_Clousre::run_loop, &lc);

    ros::spin();

    process_thread.join();

    return 0;
}
