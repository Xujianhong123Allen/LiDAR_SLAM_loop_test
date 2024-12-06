#include "Frame.h"
#include <thread>

namespace BoW3D
{
    long unsigned int Frame::nNextId = 0;

    Frame::Frame(LinK3D_Extractor *pLink3dExtractor, pcl::PointCloud<pcl::PointXYZ>::Ptr pLaserCloudIn) : mpLink3dExtractor(pLink3dExtractor)
    {
        mnId = nNextId++;

        (*mpLink3dExtractor)(pLaserCloudIn, mvAggregationKeypoints, mDescriptors, mClusterEdgeKeypoints);
    }

    Frame::Frame(LinK3D_Extractor *pLink3dExtractor, pcl::PointCloud<VelodynePointXYZIRT>::Ptr pLaserCloudIn) : mpLink3dExtractor(pLink3dExtractor)
    {
        mnId = nNextId++;

        (*mpLink3dExtractor)(pLaserCloudIn, mvAggregationKeypoints, mDescriptors, mClusterEdgeKeypoints);
    }

    pcl::PointCloud<pcl::PointXYZI> Frame::get_keyPoints()
    {
        pcl::PointCloud<pcl::PointXYZI> keyPoints_cloud;

        for (auto k : mvAggregationKeypoints)
        {
            keyPoints_cloud.push_back(k);
        }

        return keyPoints_cloud;
    }

    pcl::PointCloud<pcl::PointXYZI> Frame::get_cluster_cloud()
    {
        pcl::PointCloud<pcl::PointXYZI> cluster_cloud;

        int cluster_id = 0;
        for (auto cluster : mClusterEdgeKeypoints)
        {
            for (auto c : cluster)
            {
                pcl::PointXYZI p;
                p.x = c.x;
                p.y = c.y;
                p.z = c.z;
                p.intensity = cluster_id;
                cluster_cloud.push_back(p);
            }
            cluster_id++;
        }

        return cluster_cloud;
    }

    Eigen::MatrixXd Frame::get_Des()
    {
        Eigen::MatrixXd des_mat;
        des_mat.resize(mDescriptors.rows, mDescriptors.cols);
        for (int i = 0; i < mDescriptors.rows; i++)
        {
            for (int j = 0; j < mDescriptors.cols; j++)
            {
                double d = mDescriptors.at<float>(i, j);
                des_mat(i, j) = d;
            }
        }
        return des_mat;
    }

}