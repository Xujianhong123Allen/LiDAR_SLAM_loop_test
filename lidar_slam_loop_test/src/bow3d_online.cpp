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
#include <pcl/filters/extract_indices.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <thread>
#include <sstream>
#include <iomanip>
#include "BoW3D/LinK3D_Extractor.h"
#include "BoW3D/BoW3D.h"

using namespace std;
using namespace BoW3D;

// // Parameters of LinK3D
// int nScans = 32; // Number of LiDAR scan lines
// float scanPeriod = 0.1;
// float minimumRange = 0.1;
// float distanceTh = 0.4;
// int matchTh = 6;

// // Parameters of BoW3D
// float thr = 3.5;
// int thf = 5;
// int num_add_retrieve_features = 5;

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
    float distanceTh = 0.5; // 原始0.4
    int matchTh = 6;        // 原始 6

    // Parameters of BoW3D
    float thr = 3.5; // 原始3.5
    int thf = 5;     // 原始5
    int num_add_retrieve_features = 5;

    BoW3D::LinK3D_Extractor *pLinK3dExtractor;
    BoW3D::BoW3D *pBoW3D;

    std::vector<geometry_msgs::Pose> pose_v;         // 存放历史帧的位姿
    std::vector<std::pair<int, int>> loop_pair_id_v; // 存放回环检测的帧id号
};

Loop_Clousre::Loop_Clousre(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
{
    nh.param<std::string>("lidar_topic", lidar_topic, "/velodyne_points");
    nh.param<std::string>("odom_topic", odom_topic, "/Odometry");
    nh.param<std::string>("odom_frame", odom_frame, "camera_init"); // 地图初始坐标frame_id
    nh.param<std::string>("lidar_frame", lidar_frame, "base_link"); // body系frame_id
    nh.param<std::string>("sensor_type", sensor_type, "Velodyne");  // body系frame_id

    nh.param<int>("nScans", nScans, 32);

    pointcloud_sub = nh.subscribe(lidar_topic, 100, &Loop_Clousre::cloud_cb, this);
    odom_sub = nh.subscribe(odom_topic, 100, &Loop_Clousre::odometry_cb, this);

    key_point_pub = nh.advertise<sensor_msgs::PointCloud2>("key_points", 1000, true);

    loop_markers_pub = nh.advertise<visualization_msgs::MarkerArray>("loop_constriant", 1000, true);

    // Link3D特征
    pLinK3dExtractor = new BoW3D::LinK3D_Extractor(nScans, scanPeriod, minimumRange, distanceTh, matchTh);
    // BoW3D回环检测指针
    pBoW3D = new BoW3D::BoW3D(pLinK3dExtractor, thr, thf, num_add_retrieve_features);
}

Loop_Clousre::~Loop_Clousre()
{
    std::cout << "loop count: " << loop_pair_id_v.size() << std::endl;
    delete[] pLinK3dExtractor;
    delete[] pBoW3D;
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
    int count = 0;
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
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(laserCloud_curr, *current_cloud);

        // count++;
        // if (count < 10)
        // {
        //     continue;
        // }
        // count = 0;

        // pcl::PointCloud<VelodynePointXYZIRT>::Ptr current_cloud(new pcl::PointCloud<VelodynePointXYZIRT>());

        // if (sensor_type == "Velodyne") // velodyne
        // {
        //     pcl::moveFromROSMsg(laserCloud_curr, *current_cloud);
        // }
        // else if (sensor_type == "Ouster") // ouster
        // {
        //     // Convert to Velodyne format
        //     pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn(new pcl::PointCloud<OusterPointXYZIRT>);
        //     pcl::moveFromROSMsg(laserCloud_curr, *tmpOusterCloudIn);
        //     current_cloud->points.resize(tmpOusterCloudIn->size());
        //     current_cloud->is_dense = tmpOusterCloudIn->is_dense;
        //     for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
        //     {
        //         auto &src = tmpOusterCloudIn->points[i];
        //         auto &dst = current_cloud->points[i];
        //         dst.x = src.x;
        //         dst.y = src.y;
        //         dst.z = src.z;
        //         dst.intensity = src.intensity;
        //         dst.ring = src.ring;
        //         dst.time = src.t * 1e-9f;
        //     }
        // }
        // else
        // {
        //     ROS_ERROR_STREAM("Unknown sensor type: " << sensor_type << "! Must be Velodyne or Ouster!");
        //     ros::shutdown();
        // }

        Frame *pCurrentFrame = new Frame(pLinK3dExtractor, current_cloud); // 将当前点云加入到Link3D特征中

        pcl::PointCloud<pcl::PointXYZI> keyPoints = pCurrentFrame->get_keyPoints();

        sensor_msgs::PointCloud2 key_point_msg;
        pcl::toROSMsg(keyPoints, key_point_msg);
        key_point_msg.header.stamp = curr_time;
        key_point_msg.header.frame_id = lidar_frame;
        key_point_pub.publish(key_point_msg);

        if (pCurrentFrame->mnId < 2)
        {
            pBoW3D->update(pCurrentFrame);
        }
        else
        {
            int loopFrameId = -1;
            Eigen::Matrix3d loopRelR;
            Eigen::Vector3d loopRelt;

            clock_t start, end;
            double time;
            start = clock();

            double match_score; // 类似于ICP的匹配得分
            pBoW3D->retrieve(pCurrentFrame, loopFrameId, loopRelR, loopRelt, match_score);

            end = clock();
            time = ((double)(end - start)) / CLOCKS_PER_SEC;

            pBoW3D->update(pCurrentFrame);

            if (loopFrameId == -1)
            {
                cout << "-------------------------" << endl;
                cout << "Detection Time: " << time << "s" << endl;
                cout << "Frame" << pCurrentFrame->mnId << " Has No Loop..." << endl
                     << endl;
            }
            else
            {
                cout << "--------------------------------------" << endl;
                cout << "Detection Time: " << time << "s" << endl;
                cout << "Frame" << pCurrentFrame->mnId << " Has Loop Frame" << loopFrameId << endl;

                cout << "Loop Relative R: " << endl;
                cout << loopRelR << endl;

                cout << "Loop Relative t: " << endl;
                cout << "   " << loopRelt.x() << " " << loopRelt.y() << " " << loopRelt.z() << endl
                     << endl;

                loop_pair_id_v.push_back({pCurrentFrame->mnId, loopFrameId});

                visual_loop();
            }
        }
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
    ros::init(argc, argv, "BoW3D");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    Loop_Clousre lc(nh, nh_private);

    std::thread process_thread(&Loop_Clousre::run_loop, &lc);

    ros::spin();

    process_thread.join();

    return 0;
}
