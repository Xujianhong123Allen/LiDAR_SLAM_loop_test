#include "LinK3D_Extractor.h"

namespace BoW3D
{
    LinK3D_Extractor::LinK3D_Extractor(
        int nScans_,
        float scanPeriod_,
        float minimumRange_,
        float distanceTh_,
        int matchTh_) : nScans(nScans_),
                        scanPeriod(scanPeriod_),
                        minimumRange(minimumRange_),
                        distanceTh(distanceTh_),
                        matchTh(matchTh_)
    {
        scanNumTh = ceil(nScans / 6);
        ptNumTh = ceil(1.5 * scanNumTh);
    }

    void LinK3D_Extractor::removeClosedPointCloud(
        const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
        pcl::PointCloud<pcl::PointXYZ> &cloud_out)
    {
        if (&cloud_in != &cloud_out)
        {
            cloud_out.header = cloud_in.header;
            cloud_out.points.resize(cloud_in.points.size());
        }

        size_t j = 0;

        for (size_t i = 0; i < cloud_in.points.size(); ++i)
        {
            if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < minimumRange * minimumRange)
            {
                continue;
            }

            cloud_out.points[j] = cloud_in.points[i];
            j++;
        }

        if (j != cloud_in.points.size())
        {
            cloud_out.points.resize(j);
        }

        cloud_out.height = 1;
        cloud_out.width = static_cast<uint32_t>(j);
        cloud_out.is_dense = true;
    }

    // 在每个扫描帧中提取曲率较大的边缘点
    void LinK3D_Extractor::extractEdgePoint(pcl::PointCloud<pcl::PointXYZ>::Ptr pLaserCloudIn,
                                            ScanEdgePoints &edgePoints)
    {
        vector<int> scanStartInd(nScans, 0);
        vector<int> scanEndInd(nScans, 0);

        pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
        laserCloudIn = *pLaserCloudIn;
        vector<int> indices;

        pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
        removeClosedPointCloud(laserCloudIn, laserCloudIn);

        int cloudSize = laserCloudIn.points.size();
        float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
        float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y, laserCloudIn.points[cloudSize - 1].x) + 2 * M_PI;

        if (endOri - startOri > 3 * M_PI)
        {
            endOri -= 2 * M_PI;
        }
        else if (endOri - startOri < M_PI)
        {
            endOri += 2 * M_PI;
        }

        bool halfPassed = false;
        int count = cloudSize;
        pcl::PointXYZI point;
        vector<pcl::PointCloud<pcl::PointXYZI>> laserCloudScans(nScans);

        for (int i = 0; i < cloudSize; i++)
        {
            point.x = laserCloudIn.points[i].x;
            point.y = laserCloudIn.points[i].y;
            point.z = laserCloudIn.points[i].z;

            float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
            int scanID = 0;

            if (nScans == 16)
            {
                scanID = int((angle + 15) / 2 + 0.5);
                if (scanID > (nScans - 1) || scanID < 0)
                {
                    count--;
                    continue;
                }
            }
            else if (nScans == 32)
            {
                /* HDL32E */
                // scanID = int((angle + 92.0 / 3.0) * 3.0 / 4.0);
                // if (scanID > (nScans - 1) || scanID < 0)
                // {
                //     count--;
                //     continue;
                // }

                // ouster OS1-32 vfov is [-22.5, 22.5] see https://ouster.com/products/os1-lidar-sensor/
                scanID = int((angle + 22.5) / 2 + 0.5);
                if (scanID > (nScans - 1) || scanID < 0)
                {
                    count--;
                    continue;
                }
            }
            else if (nScans == 64)
            {
                if (angle >= -8.83)
                    scanID = int((2 - angle) * 3.0 + 0.5);
                else
                    scanID = nScans / 2 + int((-8.83 - angle) * 2.0 + 0.5);

                if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
                {
                    count--;
                    continue;
                }
            }
            else
            {
                printf("wrong scan number\n");
            }

            float ori = -atan2(point.y, point.x);
            if (!halfPassed)
            {
                if (ori < startOri - M_PI / 2)
                {
                    ori += 2 * M_PI;
                }
                else if (ori > startOri + M_PI * 3 / 2)
                {
                    ori -= 2 * M_PI;
                }

                if (ori - startOri > M_PI)
                {
                    halfPassed = true;
                }
            }
            else
            {
                ori += 2 * M_PI;
                if (ori < endOri - M_PI * 3 / 2)
                {
                    ori += 2 * M_PI;
                }
                else if (ori > endOri + M_PI / 2)
                {
                    ori -= 2 * M_PI;
                }
            }

            point.intensity = ori;
            laserCloudScans[scanID].points.push_back(point);
        }

        size_t scanSize = laserCloudScans.size();
        edgePoints.resize(scanSize);
        cloudSize = count;

        // edgePoints是一个二维矩阵，行代表线束数量16、32、64，每一行包含各自扫描线的边缘点

        for (int i = 0; i < nScans; i++)
        {
            int laserCloudScansSize = laserCloudScans[i].size();
            if (laserCloudScansSize >= 15)
            {
                for (int j = 10; j < laserCloudScansSize - 10; j++)
                {
                    // 曲率方式提取边缘点
                    float diffX = laserCloudScans[i].points[j - 5].x + laserCloudScans[i].points[j - 4].x + laserCloudScans[i].points[j - 3].x + laserCloudScans[i].points[j - 2].x + laserCloudScans[i].points[j - 1].x - 10 * laserCloudScans[i].points[j].x + laserCloudScans[i].points[j + 1].x + laserCloudScans[i].points[j + 2].x + laserCloudScans[i].points[j + 3].x + laserCloudScans[i].points[j + 4].x + laserCloudScans[i].points[j + 5].x;
                    float diffY = laserCloudScans[i].points[j - 5].y + laserCloudScans[i].points[j - 4].y + laserCloudScans[i].points[j - 3].y + laserCloudScans[i].points[j - 2].y + laserCloudScans[i].points[j - 1].y - 10 * laserCloudScans[i].points[j].y + laserCloudScans[i].points[j + 1].y + laserCloudScans[i].points[j + 2].y + laserCloudScans[i].points[j + 3].y + laserCloudScans[i].points[j + 4].y + laserCloudScans[i].points[j + 5].y;
                    float diffZ = laserCloudScans[i].points[j - 5].z + laserCloudScans[i].points[j - 4].z + laserCloudScans[i].points[j - 3].z + laserCloudScans[i].points[j - 2].z + laserCloudScans[i].points[j - 1].z - 10 * laserCloudScans[i].points[j].z + laserCloudScans[i].points[j + 1].z + laserCloudScans[i].points[j + 2].z + laserCloudScans[i].points[j + 3].z + laserCloudScans[i].points[j + 4].z + laserCloudScans[i].points[j + 5].z;

                    float curv = diffX * diffX + diffY * diffY + diffZ * diffZ;
                    if (curv > 10 && curv < 20000)
                    {
                        float ori = laserCloudScans[i].points[j].intensity;
                        float relTime = (ori - startOri) / (endOri - startOri);

                        PointXYZSCA tmpPt;
                        tmpPt.x = laserCloudScans[i].points[j].x;
                        tmpPt.y = laserCloudScans[i].points[j].y;
                        tmpPt.z = laserCloudScans[i].points[j].z;
                        tmpPt.scan_position = i + scanPeriod * relTime;
                        tmpPt.curvature = curv;
                        tmpPt.angle = ori;
                        edgePoints[i].emplace_back(tmpPt);
                    }

                    // PCA方式提取边缘点
                    // pcl::PointCloud<pcl::PointXYZI>::Ptr PCA_cloud(new pcl::PointCloud<pcl::PointXYZI>);
                    // for (int k = -10; k <= 10; k++)
                    // {
                    //     PCA_cloud->push_back(laserCloudScans[i].points[j + k]);
                    // }

                    // Eigen::Vector4f pcaCentroid;
                    // pcl::compute3DCentroid(*PCA_cloud, pcaCentroid); // 计算该点云的质心
                    // Eigen::Matrix3f covariance;
                    // pcl::computeCovarianceMatrixNormalized(*PCA_cloud, pcaCentroid, covariance);
                    // Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
                    // Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
                    // Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
                    // // PCA输出的特征值从小到大排列，       [e1, e2, e3]
                    // // PCA输出的特征列向量与特征值顺序对应，[v1, v2, v3]

                    // float diffX = laserCloudScans[i].points[j - 5].x + laserCloudScans[i].points[j - 4].x + laserCloudScans[i].points[j - 3].x + laserCloudScans[i].points[j - 2].x + laserCloudScans[i].points[j - 1].x - 10 * laserCloudScans[i].points[j].x + laserCloudScans[i].points[j + 1].x + laserCloudScans[i].points[j + 2].x + laserCloudScans[i].points[j + 3].x + laserCloudScans[i].points[j + 4].x + laserCloudScans[i].points[j + 5].x;
                    // float diffY = laserCloudScans[i].points[j - 5].y + laserCloudScans[i].points[j - 4].y + laserCloudScans[i].points[j - 3].y + laserCloudScans[i].points[j - 2].y + laserCloudScans[i].points[j - 1].y - 10 * laserCloudScans[i].points[j].y + laserCloudScans[i].points[j + 1].y + laserCloudScans[i].points[j + 2].y + laserCloudScans[i].points[j + 3].y + laserCloudScans[i].points[j + 4].y + laserCloudScans[i].points[j + 5].y;
                    // float diffZ = laserCloudScans[i].points[j - 5].z + laserCloudScans[i].points[j - 4].z + laserCloudScans[i].points[j - 3].z + laserCloudScans[i].points[j - 2].z + laserCloudScans[i].points[j - 1].z - 10 * laserCloudScans[i].points[j].z + laserCloudScans[i].points[j + 1].z + laserCloudScans[i].points[j + 2].z + laserCloudScans[i].points[j + 3].z + laserCloudScans[i].points[j + 4].z + laserCloudScans[i].points[j + 5].z;

                    // float ratio = eigenValuesPCA(2) / eigenValuesPCA(1); // 如果算出平面度值大于20，则认为是平面点

                    // float curv = diffX * diffX + diffY * diffY + diffZ * diffZ;
                    // if (ratio < 10)
                    // {
                    //     // std::cout << "threshold: " <<  ratio << std::endl;
                    //     float ori = laserCloudScans[i].points[j].intensity;
                    //     float relTime = (ori - startOri) / (endOri - startOri);

                    //     PointXYZSCA tmpPt;
                    //     tmpPt.x = laserCloudScans[i].points[j].x;
                    //     tmpPt.y = laserCloudScans[i].points[j].y;
                    //     tmpPt.z = laserCloudScans[i].points[j].z;
                    //     tmpPt.scan_position = i + scanPeriod * relTime;
                    //     tmpPt.curvature = curv;
                    //     // tmpPt.curvature = 1.0 / ratio;
                    //     tmpPt.angle = ori;
                    //     edgePoints[i].emplace_back(tmpPt);
                    // }
                }
            }
        }

        // pcl::PointCloud<PointXYZSCA> edge_cloud;
        // for (auto s : edgePoints)
        // {
        //     for (auto p : s)
        //     {
        //         edge_cloud.push_back(p);
        //     }
        // }

        // std::cout << "edge_cloud size: " << edge_cloud.size() << std::endl;
    }

    // 在每个扫描帧中提取曲率较大的边缘点
    void LinK3D_Extractor::extractEdgePoint(pcl::PointCloud<VelodynePointXYZIRT>::Ptr pLaserCloudIn,
                                            ScanEdgePoints &edgePoints)
    {
        // std::cout << "nScan: " << nScans << std::endl;
        vector<int> scanStartInd(nScans, 0);
        vector<int> scanEndInd(nScans, 0);

        pcl::PointCloud<VelodynePointXYZIRT>::Ptr laserCloudIn(new pcl::PointCloud<VelodynePointXYZIRT>);
        // laserCloudIn = *pLaserCloudIn;
        vector<int> indices;

        for (int i = 0; i < pLaserCloudIn->size(); i++)
        {
            VelodynePointXYZIRT p = pLaserCloudIn->points[i];
            double range = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            if (range > minimumRange && range < 100)
            {
                laserCloudIn->push_back(p);
            }
        }

        int cloudSize = laserCloudIn->points.size();
        float startOri = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
        float endOri = -atan2(laserCloudIn->points[cloudSize - 1].y, laserCloudIn->points[cloudSize - 1].x) + 2 * M_PI;

        if (endOri - startOri > 3 * M_PI)
        {
            endOri -= 2 * M_PI;
        }
        else if (endOri - startOri < M_PI)
        {
            endOri += 2 * M_PI;
        }

        bool halfPassed = false;
        int count = cloudSize;
        pcl::PointXYZI point;
        vector<pcl::PointCloud<pcl::PointXYZI>> laserCloudScans(nScans);

        for (int i = 0; i < cloudSize; i++)
        {
            point.x = laserCloudIn->points[i].x;
            point.y = laserCloudIn->points[i].y;
            point.z = laserCloudIn->points[i].z;

            float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
            int scanID = 0;

            scanID = laserCloudIn->points[i].ring;

            float ori = -atan2(point.y, point.x);
            if (!halfPassed)
            {
                if (ori < startOri - M_PI / 2)
                {
                    ori += 2 * M_PI;
                }
                else if (ori > startOri + M_PI * 3 / 2)
                {
                    ori -= 2 * M_PI;
                }

                if (ori - startOri > M_PI)
                {
                    halfPassed = true;
                }
            }
            else
            {
                ori += 2 * M_PI;
                if (ori < endOri - M_PI * 3 / 2)
                {
                    ori += 2 * M_PI;
                }
                else if (ori > endOri + M_PI / 2)
                {
                    ori -= 2 * M_PI;
                }
            }

            point.intensity = ori;
            laserCloudScans[scanID].points.push_back(point);
        }

        size_t scanSize = laserCloudScans.size();
        edgePoints.resize(scanSize);
        cloudSize = count;

        // edgePoints是一个二维矩阵，行代表线束数量16、32、64，每一行包含各自扫描线的边缘点

        for (int i = 0; i < nScans; i++)
        {
            int laserCloudScansSize = laserCloudScans[i].size();
            if (laserCloudScansSize >= 15)
            {
                for (int j = 5; j < laserCloudScansSize - 5; j++)
                {
                    float diffX = laserCloudScans[i].points[j - 5].x + laserCloudScans[i].points[j - 4].x + laserCloudScans[i].points[j - 3].x + laserCloudScans[i].points[j - 2].x + laserCloudScans[i].points[j - 1].x - 10 * laserCloudScans[i].points[j].x + laserCloudScans[i].points[j + 1].x + laserCloudScans[i].points[j + 2].x + laserCloudScans[i].points[j + 3].x + laserCloudScans[i].points[j + 4].x + laserCloudScans[i].points[j + 5].x;
                    float diffY = laserCloudScans[i].points[j - 5].y + laserCloudScans[i].points[j - 4].y + laserCloudScans[i].points[j - 3].y + laserCloudScans[i].points[j - 2].y + laserCloudScans[i].points[j - 1].y - 10 * laserCloudScans[i].points[j].y + laserCloudScans[i].points[j + 1].y + laserCloudScans[i].points[j + 2].y + laserCloudScans[i].points[j + 3].y + laserCloudScans[i].points[j + 4].y + laserCloudScans[i].points[j + 5].y;
                    float diffZ = laserCloudScans[i].points[j - 5].z + laserCloudScans[i].points[j - 4].z + laserCloudScans[i].points[j - 3].z + laserCloudScans[i].points[j - 2].z + laserCloudScans[i].points[j - 1].z - 10 * laserCloudScans[i].points[j].z + laserCloudScans[i].points[j + 1].z + laserCloudScans[i].points[j + 2].z + laserCloudScans[i].points[j + 3].z + laserCloudScans[i].points[j + 4].z + laserCloudScans[i].points[j + 5].z;

                    float curv = diffX * diffX + diffY * diffY + diffZ * diffZ;
                    if (curv > 10 && curv < 20000)
                    {
                        float ori = laserCloudScans[i].points[j].intensity;
                        float relTime = (ori - startOri) / (endOri - startOri);

                        PointXYZSCA tmpPt;
                        tmpPt.x = laserCloudScans[i].points[j].x;
                        tmpPt.y = laserCloudScans[i].points[j].y;
                        tmpPt.z = laserCloudScans[i].points[j].z;
                        tmpPt.scan_position = i + scanPeriod * relTime;
                        tmpPt.curvature = curv;
                        tmpPt.angle = ori;
                        edgePoints[i].emplace_back(tmpPt);
                    }
                }
            }
        }

        // pcl::PointCloud<PointXYZSCA> edge_cloud;
        // for (auto s : edgePoints)
        // {
        //     for (auto p : s)
        //     {
        //         edge_cloud.push_back(p);
        //     }
        // }

        // std::cout << "edge_cloud size: " << edge_cloud.size() << std::endl;
    }

    // Roughly divide the areas to save time for clustering.
    void LinK3D_Extractor::divideArea(ScanEdgePoints &scanCloud, ScanEdgePoints &sectorAreaCloud)
    {

        // 水平面部分为120份扇形区域
        sectorAreaCloud.resize(120); // The horizontal plane is divided into 120 sector area centered on LiDAR coordinate.
        int numScansPt = scanCloud.size();
        if (numScansPt == 0)
        {
            return;
        }

        // sectorAreaCloud则是代表120份扇形区域，scanCloud（这里代表边缘点）放入到各自对应的扇形区域中
        for (int i = 0; i < numScansPt; i++)
        {
            int numAScanPt = scanCloud[i].size();
            for (int j = 0; j < numAScanPt; j++)
            {
                int areaID = 0;
                float angle = scanCloud[i][j].angle;

                if (angle > 0 && angle < 2 * M_PI)
                {
                    areaID = std::floor((angle / (2 * M_PI)) * 120);
                }
                else if (angle > 2 * M_PI)
                {
                    areaID = std::floor(((angle - 2 * M_PI) / (2 * M_PI)) * 120);
                }
                else if (angle < 0)
                {
                    areaID = std::floor(((angle + 2 * M_PI) / (2 * M_PI)) * 120);
                }
                // std::cout << "areaId: " << areaID << std::endl;

                if (areaID < 0 || areaID >= 120)
                {
                    continue;
                }

                sectorAreaCloud[areaID].push_back(scanCloud[i][j]);
            }
        }
    }

    // 计算cluster点云与原点平均距离，仅用xy坐标
    float LinK3D_Extractor::computeClusterMean(vector<PointXYZSCA> &cluster)
    {
        float distSum = 0;
        int numPt = cluster.size();

        for (int i = 0; i < numPt; i++)
        {
            distSum += distXY(cluster[i]);
        }

        return (distSum / numPt);
    }

    // 计算cluster点云的均值x,y
    void LinK3D_Extractor::computeXYMean(vector<PointXYZSCA> &cluster, std::pair<float, float> &xyMeans)
    {
        int numPt = cluster.size();
        float xSum = 0;
        float ySum = 0;

        for (int i = 0; i < numPt; i++)
        {
            xSum += cluster[i].x;
            ySum += cluster[i].y;
        }

        float xMean = xSum / numPt;
        float yMean = ySum / numPt;
        xyMeans = std::make_pair(xMean, yMean);
    }

    // 对每个扇形区域的边缘点进行聚类
    void LinK3D_Extractor::getCluster(const ScanEdgePoints &sectorAreaCloud, ScanEdgePoints &clusters)
    {
        ScanEdgePoints tmpclusters;
        PointXYZSCA curvPt;
        vector<PointXYZSCA> dummy(1, curvPt);

        int numArea = sectorAreaCloud.size();

        // Cluster for each sector area.
        // 对每一个扇形区域进行聚类
        for (int i = 0; i < numArea; i++)
        {
            // 对于少于6个点的区域进行舍弃
            if (sectorAreaCloud[i].size() < 6)
            {
                continue;
            }

            int numPt = sectorAreaCloud[i].size();
            ScanEdgePoints curAreaCluster(1, dummy);
            curAreaCluster[0][0] = sectorAreaCloud[i][0];

            for (int j = 1; j < numPt; j++)
            {
                int numCluster = curAreaCluster.size();

                for (int k = 0; k < numCluster; k++)
                {
                    float mean = computeClusterMean(curAreaCluster[k]); // 平均距离
                    std::pair<float, float> xyMean;
                    computeXYMean(curAreaCluster[k], xyMean);

                    PointXYZSCA tmpPt = sectorAreaCloud[i][j];

                    if (abs(distXY(tmpPt) - mean) < distanceTh && abs(xyMean.first - tmpPt.x) < distanceTh && abs(xyMean.second - tmpPt.y) < distanceTh)
                    {
                        curAreaCluster[k].emplace_back(tmpPt);
                        break;
                    }
                    else if (abs(distXY(tmpPt) - mean) >= distanceTh && k == numCluster - 1)
                    {
                        curAreaCluster.emplace_back(dummy);
                        curAreaCluster[numCluster][0] = tmpPt;
                    }
                    else
                    {
                        continue;
                    }
                }
            }

            int numCluster = curAreaCluster.size();
            for (int j = 0; j < numCluster; j++)
            {
                int numPt = curAreaCluster[j].size();

                if (numPt < ptNumTh)
                {
                    continue;
                }
                tmpclusters.emplace_back(curAreaCluster[j]);
            }
        }

        int numCluster = tmpclusters.size();

        vector<bool> toBeMerge(numCluster, false);
        multimap<int, int> mToBeMergeInd;
        set<int> sNeedMergeInd;

        // Merge the neighbor clusters.
        for (int i = 0; i < numCluster; i++)
        {
            if (toBeMerge[i])
            {
                continue;
            }
            float means1 = computeClusterMean(tmpclusters[i]);
            std::pair<float, float> xyMeans1;
            computeXYMean(tmpclusters[i], xyMeans1);

            for (int j = 1; j < numCluster; j++)
            {
                if (toBeMerge[j])
                {
                    continue;
                }

                float means2 = computeClusterMean(tmpclusters[j]);
                std::pair<float, float> xyMeans2;
                computeXYMean(tmpclusters[j], xyMeans2);

                if (abs(means1 - means2) < 2 * distanceTh && abs(xyMeans1.first - xyMeans2.first) < 2 * distanceTh && abs(xyMeans1.second - xyMeans2.second) < 2 * distanceTh)
                {
                    mToBeMergeInd.insert(std::make_pair(i, j));
                    sNeedMergeInd.insert(i);
                    toBeMerge[i] = true;
                    toBeMerge[j] = true;
                }
            }
        }

        if (sNeedMergeInd.empty())
        {
            for (int i = 0; i < numCluster; i++)
            {
                clusters.emplace_back(tmpclusters[i]);
            }
        }
        else
        {
            for (int i = 0; i < numCluster; i++)
            {
                if (toBeMerge[i] == false)
                {
                    clusters.emplace_back(tmpclusters[i]);
                }
            }

            for (auto setIt = sNeedMergeInd.begin(); setIt != sNeedMergeInd.end(); ++setIt)
            {
                int needMergeInd = *setIt;
                auto entries = mToBeMergeInd.count(needMergeInd);
                auto iter = mToBeMergeInd.find(needMergeInd);
                vector<int> vInd;

                while (entries)
                {
                    int ind = iter->second;
                    vInd.emplace_back(ind);
                    ++iter;
                    --entries;
                }

                clusters.emplace_back(tmpclusters[needMergeInd]);
                size_t numCluster = clusters.size();

                for (size_t j = 0; j < vInd.size(); j++)
                {
                    for (size_t ptNum = 0; ptNum < tmpclusters[vInd[j]].size(); ptNum++)
                    {
                        clusters[numCluster - 1].emplace_back(tmpclusters[vInd[j]][ptNum]);
                    }
                }
            }
        }
    }

    void LinK3D_Extractor::computeDirection(pcl::PointXYZI ptFrom, pcl::PointXYZI ptTo, Eigen::Vector2f &direction)
    {
        direction(0, 0) = ptTo.x - ptFrom.x;
        direction(1, 0) = ptTo.y - ptFrom.y;
    }

    // 计算关键点（每个类的均值点）和有效类cluster
    vector<pcl::PointXYZI> LinK3D_Extractor::getMeanKeyPoint(const ScanEdgePoints &clusters, ScanEdgePoints &validCluster)
    {
        int count = 0;
        int numCluster = clusters.size();
        vector<pcl::PointXYZI> keyPoints;
        vector<pcl::PointXYZI> tmpKeyPoints;
        ScanEdgePoints tmpEdgePoints;
        map<float, int> distanceOrder; // 这里存储的是(距离，点数)

        for (int i = 0; i < numCluster; i++)
        {
            int ptCnt = clusters[i].size();
            if (ptCnt < ptNumTh)
            {
                continue;
            }

            vector<PointXYZSCA> tmpCluster;
            set<int> scans;
            float x = 0, y = 0, z = 0, intensity = 0;
            for (int ptNum = 0; ptNum < ptCnt; ptNum++)
            {
                PointXYZSCA pt = clusters[i][ptNum];
                int scan = int(pt.scan_position);
                scans.insert(scan);

                x += pt.x;
                y += pt.y;
                z += pt.z;
                intensity += pt.scan_position;
            }

            if (scans.size() < (size_t)scanNumTh)
            {
                continue;
            }

            pcl::PointXYZI pt;
            pt.x = x / ptCnt;
            pt.y = y / ptCnt;
            pt.z = z / ptCnt;
            pt.intensity = intensity / ptCnt;

            float distance = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;

            auto iter = distanceOrder.find(distance);
            if (iter != distanceOrder.end())
            {
                continue;
            }

            distanceOrder[distance] = count;
            count++;

            tmpKeyPoints.emplace_back(pt);
            tmpEdgePoints.emplace_back(clusters[i]);
        }

        // std::vector<std::pair<float, int>> map;
        // for (auto iter = distanceOrder.begin(); iter != distanceOrder.end(); iter++)
        // {
        //     map.push_back(std::make_pair((*iter).first, (*iter).second));
        // }

        // for (int i = map.size() - 1; i >= 0; i--)
        // {
        //     int index = map[i].second;
        //     pcl::PointXYZI tmpPt = tmpKeyPoints[index];

        //     keyPoints.emplace_back(tmpPt);
        //     validCluster.emplace_back(tmpEdgePoints[index]);
        // }

        // keyPoints = tmpKeyPoints;
        // validCluster = tmpEdgePoints;

        for (auto iter = distanceOrder.begin(); iter != distanceOrder.end(); iter++)
        {
            int index = (*iter).second;
            pcl::PointXYZI tmpPt = tmpKeyPoints[index];

            keyPoints.emplace_back(tmpPt);
            validCluster.emplace_back(tmpEdgePoints[index]);
        }

        return keyPoints;
    }

    // 对小数进行四舍五入圆整，保留小数后一位
    float LinK3D_Extractor::fRound(float in)
    {
        float f;
        int temp = std::round(in * 10);
        f = temp / 10.0;

        return f;
    }

    // 将关键点keyPoint转化为描述子
    void LinK3D_Extractor::getDescriptors(const vector<pcl::PointXYZI> &keyPoints,
                                          cv::Mat &descriptors)
    {
        if (keyPoints.empty())
        {
            return;
        }

        int ptSize = keyPoints.size();

        int N = 180;
        // descriptors = cv::Mat::zeros(ptSize, 180, CV_32FC1);
        descriptors = cv::Mat::zeros(ptSize, N, CV_32FC1);

        vector<vector<float>> distanceTab;
        vector<float> oneRowDis(ptSize, 0);
        distanceTab.resize(ptSize, oneRowDis);

        vector<vector<Eigen::Vector2f>> directionTab;
        Eigen::Vector2f direct(0, 0);
        vector<Eigen::Vector2f> oneRowDirect(ptSize, direct);
        directionTab.resize(ptSize, oneRowDirect);

        // Build distance and direction tables for fast descriptor generation.
        // 为了减少重复计算两点之间的距离和方向，建立表来存储距离和方向
        for (size_t i = 0; i < keyPoints.size(); i++)
        {
            for (size_t j = i + 1; j < keyPoints.size(); j++)
            {
                float dist = distPt2Pt(keyPoints[i], keyPoints[j]);
                distanceTab[i][j] = fRound(dist);
                distanceTab[j][i] = distanceTab[i][j];

                Eigen::Vector2f tmpDirection;

                tmpDirection(0, 0) = keyPoints[j].x - keyPoints[i].x;
                tmpDirection(1, 0) = keyPoints[j].y - keyPoints[i].y;

                directionTab[i][j] = tmpDirection;
                directionTab[j][i] = -tmpDirection;
            }
        }

        for (size_t i = 0; i < keyPoints.size(); i++)
        {
            vector<float> tempRow(distanceTab[i]);
            std::sort(tempRow.begin(), tempRow.end());
            int Index[3];

            // Get the closest three keypoints of current keypoint.
            // 获取当前关键点三个最近关键点的索引
            for (int k = 0; k < 3; k++)
            {
                vector<float>::iterator it1 = find(distanceTab[i].begin(), distanceTab[i].end(), tempRow[k + 1]);
                if (it1 == distanceTab[i].end())
                {
                    continue;
                }
                else
                {
                    Index[k] = std::distance(distanceTab[i].begin(), it1);
                }
            }

            // Generate the descriptor for each closest keypoint.
            // The final descriptor is based on the priority of the three closest keypoint.
            // 为每个最近点生成对应描述子，最后的描述子优先从三个描述子中排列
            for (int indNum = 0; indNum < 3; indNum++)
            {
                int index = Index[indNum];
                Eigen::Vector2f mainDirection;
                mainDirection = directionTab[i][index]; // 主方向

                vector<vector<float>> areaDis(180);
                areaDis[0].emplace_back(distanceTab[i][index]);

                for (size_t j = 0; j < keyPoints.size(); j++)
                {
                    if (j == i || (int)j == index)
                    {
                        continue;
                    }

                    Eigen::Vector2f otherDirection = directionTab[i][j];

                    Eigen::Matrix2f matrixDirect;
                    matrixDirect << mainDirection(0, 0), mainDirection(1, 0), otherDirection(0, 0), otherDirection(1, 0);
                    float deter = matrixDirect.determinant(); // 行列式

                    int areaNum = 0;
                    double cosAng = (double)mainDirection.dot(otherDirection) / (double)(mainDirection.norm() * otherDirection.norm());
                    if (abs(cosAng) - 1 > 0)
                    {
                        continue;
                    }

                    float angle = acos(cosAng) * 180 / M_PI;

                    if (angle < 0 || angle > 180)
                    {
                        continue;
                    }

                    if (deter > 0)
                    {
                        areaNum = ceil((angle - 1) / 2);
                    }
                    else
                    {
                        if (angle - 2 < 0)
                        {
                            areaNum = 0;
                        }
                        else
                        {
                            angle = 360 - angle;
                            areaNum = ceil((angle - 1) / 2);
                        }
                    }

                    // std::cout << "areaNum: " << areaNum << std::endl;

                    // if (areaNum != 0)
                    if (areaNum > 0)
                    {
                        areaDis[areaNum].emplace_back(distanceTab[i][j]);
                    }
                }

                float *descriptor = descriptors.ptr<float>(i);

                // for (int areaNum = 0; areaNum < 180; areaNum++)
                for (int areaNum = 0; areaNum < N; areaNum++)
                {
                    if (areaDis[areaNum].size() == 0)
                    {
                        continue;
                    }
                    else
                    {
                        std::sort(areaDis[areaNum].begin(), areaDis[areaNum].end());

                        if (descriptor[areaNum] == 0)
                        {
                            descriptor[areaNum] = areaDis[areaNum][0];
                        }
                    }
                }
            }
        }

        // std::cout << "descriptors: " << descriptors.rows << " * " << descriptors.cols << std::endl;
    }
    // 对两个扫描子进行匹配，获得匹配索引号
    void LinK3D_Extractor::match(vector<pcl::PointXYZI> &curAggregationKeyPt,
                                 vector<pcl::PointXYZI> &toBeMatchedKeyPt,
                                 cv::Mat &curDescriptors,
                                 cv::Mat &toBeMatchedDescriptors,
                                 vector<pair<int, int>> &vMatchedIndex)
    {
        int curKeypointNum = curAggregationKeyPt.size();
        int toBeMatchedKeyPtNum = toBeMatchedKeyPt.size();

        multimap<int, int> matchedIndexScore;
        multimap<int, int> mMatchedIndex;
        set<int> sIndex;

        for (int i = 0; i < curKeypointNum; i++)
        {
            std::pair<int, int> highestIndexScore(0, 0);
            float *pDes1 = curDescriptors.ptr<float>(i);

            for (int j = 0; j < toBeMatchedKeyPtNum; j++)
            {
                int sameDimScore = 0;
                float *pDes2 = toBeMatchedDescriptors.ptr<float>(j);

                for (int bitNum = 0; bitNum < 180; bitNum++)
                {
                    // origin
                    // if (pDes1[bitNum] != 0 && pDes2[bitNum] != 0 && abs(pDes1[bitNum] - pDes2[bitNum]) <= 0.2)
                    // {
                    //     sameDimScore += 1;
                    // }

                    // if (bitNum > 90 && sameDimScore < 3)
                    // {
                    //     break;
                    // }

                    // aerial-ground dataset, GDAUT Campus
                    if (pDes1[bitNum] != 0 && pDes2[bitNum] != 0 && abs(pDes1[bitNum] - pDes2[bitNum]) <= 0.4)
                    {
                        sameDimScore += 1;
                    }

                    if (bitNum > 40 && sameDimScore < 3)
                    {
                        break;
                    }
                }

                if (sameDimScore > highestIndexScore.second)
                {
                    highestIndexScore.first = j;
                    highestIndexScore.second = sameDimScore;
                }
            }

            // Used for removing the repeated matches.
            matchedIndexScore.insert(std::make_pair(i, highestIndexScore.second)); // Record i and its corresponding score.
            mMatchedIndex.insert(std::make_pair(highestIndexScore.first, i));      // Record the corresponding match between j and i.
            sIndex.insert(highestIndexScore.first);                                // Record the index that may be repeated matches.
        }

        // Remove the repeated matches.
        for (set<int>::iterator setIt = sIndex.begin(); setIt != sIndex.end(); ++setIt)
        {
            int indexJ = *setIt;
            auto entries = mMatchedIndex.count(indexJ);
            if (entries == 1)
            {
                auto iterI = mMatchedIndex.find(indexJ);
                auto iterScore = matchedIndexScore.find(iterI->second);
                if (iterScore->second >= matchTh)
                {
                    vMatchedIndex.emplace_back(std::make_pair(iterI->second, indexJ));
                }
            }
            else
            {
                auto iter1 = mMatchedIndex.find(indexJ);
                int highestScore = 0;
                int highestScoreIndex = -1;

                while (entries)
                {
                    int indexI = iter1->second;
                    auto iterScore = matchedIndexScore.find(indexI);
                    if (iterScore->second > highestScore)
                    {
                        highestScore = iterScore->second;
                        highestScoreIndex = indexI;
                    }
                    ++iter1;
                    --entries;
                }

                if (highestScore >= matchTh)
                {
                    vMatchedIndex.emplace_back(std::make_pair(highestScoreIndex, indexJ));
                }
            }
        }
    }

    // Remove the edge keypoints with low curvature for further edge keypoints matching.
    void LinK3D_Extractor::filterLowCurv(ScanEdgePoints &clusters, ScanEdgePoints &filtered)
    {
        int numCluster = clusters.size();
        filtered.resize(numCluster);
        for (int i = 0; i < numCluster; i++)
        {
            int numPt = clusters[i].size();
            ScanEdgePoints tmpCluster;
            vector<int> vScanID;

            for (int j = 0; j < numPt; j++)
            {
                PointXYZSCA pt = clusters[i][j];
                int scan = int(pt.scan_position);
                auto it = std::find(vScanID.begin(), vScanID.end(), scan);

                if (it == vScanID.end())
                {
                    vScanID.emplace_back(scan);
                    vector<PointXYZSCA> vPt(1, pt);
                    tmpCluster.emplace_back(vPt);
                }
                else
                {
                    int filteredInd = std::distance(vScanID.begin(), it);
                    tmpCluster[filteredInd].emplace_back(pt);
                }
            }

            for (size_t scanID = 0; scanID < tmpCluster.size(); scanID++)
            {
                if (tmpCluster[scanID].size() == 1)
                {
                    filtered[i].emplace_back(tmpCluster[scanID][0]);
                }
                else
                {
                    float maxCurv = 0;
                    PointXYZSCA maxCurvPt;
                    for (size_t j = 0; j < tmpCluster[scanID].size(); j++)
                    {
                        if (tmpCluster[scanID][j].curvature > maxCurv)
                        {
                            maxCurv = tmpCluster[scanID][j].curvature;
                            maxCurvPt = tmpCluster[scanID][j];
                        }
                    }

                    filtered[i].emplace_back(maxCurvPt);
                }
            }
        }
    }

    // Get the edge keypoint matches based on the matching results of aggregation keypoints.
    void LinK3D_Extractor::findEdgeKeypointMatch(
        ScanEdgePoints &filtered1,
        ScanEdgePoints &filtered2,
        vector<std::pair<int, int>> &vMatched,
        vector<std::pair<PointXYZSCA, PointXYZSCA>> &matchPoints)
    {
        int numMatched = vMatched.size();
        for (int i = 0; i < numMatched; i++)
        {
            pair<int, int> matchedInd = vMatched[i];

            int numPt1 = filtered1[matchedInd.first].size();
            int numPt2 = filtered2[matchedInd.second].size();

            map<int, int> mScanID_Index1;
            map<int, int> mScanID_Index2;

            for (int i = 0; i < numPt1; i++)
            {
                int scanID1 = int(filtered1[matchedInd.first][i].scan_position);
                pair<int, int> scanID_Ind(scanID1, i);
                mScanID_Index1.insert(scanID_Ind);
            }

            for (int i = 0; i < numPt2; i++)
            {
                int scanID2 = int(filtered2[matchedInd.second][i].scan_position);
                pair<int, int> scanID_Ind(scanID2, i);
                mScanID_Index2.insert(scanID_Ind);
            }

            for (auto it1 = mScanID_Index1.begin(); it1 != mScanID_Index1.end(); it1++)
            {
                int scanID1 = (*it1).first;
                auto it2 = mScanID_Index2.find(scanID1);
                if (it2 == mScanID_Index2.end())
                {
                    continue;
                }
                else
                {
                    vector<PointXYZSCA> tmpMatchPt;
                    PointXYZSCA pt1 = filtered1[matchedInd.first][(*it1).second];
                    PointXYZSCA pt2 = filtered2[matchedInd.second][(*it2).second];

                    pair<PointXYZSCA, PointXYZSCA> matchPt(pt1, pt2);
                    matchPoints.emplace_back(matchPt);
                }
            }
        }
    }

    void LinK3D_Extractor::operator()(pcl::PointCloud<pcl::PointXYZ>::Ptr pLaserCloudIn, vector<pcl::PointXYZI> &keyPoints,
                                      cv::Mat &descriptors, ScanEdgePoints &validCluster)
    {
        /* 原始BoW3d提取聚合点方式 */
        // 采用LOAM的方法在扫描帧中提取边缘点edgePoints，计算每一条扫描线上曲率较大的则为边缘点
        ScanEdgePoints edgePoints;
        extractEdgePoint(pLaserCloudIn, edgePoints);

        // 将边缘点云按照水平方向划分为120份扇形区域
        ScanEdgePoints sectorAreaCloud;
        divideArea(edgePoints, sectorAreaCloud);

        // 将扇形区域内的点云进行聚类
        ScanEdgePoints clusters;
        getCluster(sectorAreaCloud, clusters);

        // 对每个类cluster计算器关键点keyPoint(每个类均值点)和有效的类cluster
        vector<int> index;
        keyPoints = getMeanKeyPoint(clusters, validCluster);

        // 对关键点keyPoint计算描述子
        getDescriptors(keyPoints, descriptors);
    }

    void LinK3D_Extractor::operator()(pcl::PointCloud<VelodynePointXYZIRT>::Ptr pLaserCloudIn, vector<pcl::PointXYZI> &keyPoints,
                                      cv::Mat &descriptors, ScanEdgePoints &validCluster)
    {
        /* 原始BoW3d提取聚合点方式 */
        // 采用LOAM的方法在扫描帧中提取边缘点edgePoints，计算每一条扫描线上曲率较大的则为边缘点
        ScanEdgePoints edgePoints;
        extractEdgePoint(pLaserCloudIn, edgePoints);

        // 将边缘点云按照水平方向划分为120份扇形区域
        ScanEdgePoints sectorAreaCloud;
        divideArea(edgePoints, sectorAreaCloud);

        // 将扇形区域内的点云进行聚类
        ScanEdgePoints clusters;
        getCluster(sectorAreaCloud, clusters);

        // 对每个类cluster计算器关键点keyPoint(每个类均值点)和有效的类cluster
        vector<int> index;
        keyPoints = getMeanKeyPoint(clusters, validCluster);

        // 对关键点keyPoint计算描述子
        getDescriptors(keyPoints, descriptors);

        /* 采用Range深度图进行DBSCAN聚类 */
        // ScanEdgePoints clusters;
        // getCluster2(pLaserCloudIn, clusters);

        // // 对每个类cluster计算器关键点keyPoint(每个类均值点)和有效的类cluster
        // vector<int> index;
        // keyPoints = getMeanKeyPoint(clusters, validCluster);

        // std::cout << "keyPoints size: " << keyPoints.size() << std::endl;

        // // 对关键点keyPoint计算描述子
        // getDescriptors(keyPoints, descriptors);
    }

    // 新的聚类方式，对点云进行投影深度图投影，进行DBSCAN聚类
    void LinK3D_Extractor::getCluster2(pcl::PointCloud<VelodynePointXYZIRT>::Ptr pLaserCloudIn, ScanEdgePoints &clusters)
    // void LinK3D_Extractor::getCluster2(pcl::PointCloud<VelodynePointXYZIRT>::Ptr pLaserCloudIn, ScanEdgePoints &edgePoints)
    {
        pcl::PointCloud<VelodynePointXYZIRT>::Ptr laserCloudIn(new pcl::PointCloud<VelodynePointXYZIRT>);
        // copyPointCloud(*pLaserCloudIn, *laserCloudIn);

        for (int i = 0; i < pLaserCloudIn->size(); i++)
        {
            VelodynePointXYZIRT p = pLaserCloudIn->points[i];
            double range = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            if (range > minimumRange && range < 100)
            {
                laserCloudIn->push_back(p);
            }
        }

        std::cout << "laserCloudIn size:" << laserCloudIn->points.size() << std::endl;
        // pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
        // laserCloudIn = *pLaserCloudIn;
        // vector<int> indices;

        // pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
        // removeClosedPointCloud(*laserCloudIn, *laserCloudIn);

        int N_SCAN = nScans, Horizon_SCAN = 1800;

        Eigen::Matrix<PointXYZIRSCA, Eigen::Dynamic, Eigen::Dynamic> project_mat; // 投影矩阵，每个像素(x,y,z,i,r)
        project_mat.resize(N_SCAN, Horizon_SCAN);

        float startOri = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
        float endOri = -atan2(laserCloudIn->points[laserCloudIn->points.size() - 1].y,
                              laserCloudIn->points[laserCloudIn->points.size() - 1].x) +
                       2 * M_PI;

        if (endOri - startOri > 3 * M_PI)
        {
            endOri -= 2 * M_PI;
        }
        else if (endOri - startOri < M_PI)
        {
            endOri += 2 * M_PI;
        }

        bool halfPassed = false;

        cv::Mat rangeMat; // 投影深度图矩阵
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        // 1.range image projection
        for (int i = 0; i < laserCloudIn->size(); i++)
        {
            pcl::PointXYZI thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            float range = sqrt(pow(thisPoint.x, 2) + pow(thisPoint.y, 2) + pow(thisPoint.z, 2));

            // if (range < lidarMinRange || range > lidarMaxRange)
            //     continue;
            if (range < minimumRange || range > 100)
                continue;

            // 距离图像的行  与点云中ring对应,
            //  rowIdn计算出该点激光雷达是水平方向上第几线的。从下往上计数，-15度记为初始线，第0线，一共16线(N_SCAN=16
            int rowIdn = laserCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            int downsampleRate = 1;
            if (rowIdn % downsampleRate != 0)
                continue;

            // 水平角分辨率
            float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

            // Horizon_SCAN=1800,每格0.2度
            static float ang_res_x = 360.0 / float(Horizon_SCAN);
            int columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                continue;

            float ori = -atan2(thisPoint.y, thisPoint.x);
            if (!halfPassed)
            {
                if (ori < startOri - M_PI / 2)
                {
                    ori += 2 * M_PI;
                }
                else if (ori > startOri + M_PI * 3 / 2)
                {
                    ori -= 2 * M_PI;
                }

                if (ori - startOri > M_PI)
                {
                    halfPassed = true;
                }
            }
            else
            {
                ori += 2 * M_PI;
                if (ori < endOri - M_PI * 3 / 2)
                {
                    ori += 2 * M_PI;
                }
                else if (ori > endOri + M_PI / 2)
                {
                    ori -= 2 * M_PI;
                }
            }

            // point.intensity = ori;

            PointXYZIRSCA p_xyzir;
            p_xyzir.x = thisPoint.x;
            p_xyzir.y = thisPoint.y;
            p_xyzir.z = thisPoint.z;
            // p_xyzir.intensity = thisPoint.intensity;
            p_xyzir.range = range;
            p_xyzir.row_id = rowIdn;
            p_xyzir.col_id = columnIdn;

            // float ori = laserCloudScans[i].points[j].intensity;
            //    float relTime = (ori - startOri) / (endOri - startOri);

            p_xyzir.scan_position = laserCloudIn->points[i].ring + scanPeriod * (ori - startOri) / (endOri - startOri);
            p_xyzir.angle = ori;

            project_mat(rowIdn, columnIdn) = p_xyzir;

            // 转换成一维索引，存校正之后的激光点
            int index = columnIdn + rowIdn * Horizon_SCAN;
            // fullCloud->points[index] = thisPoint;
        }

        // 2. 使用BFS对前后左右的点云进行搜索，如果相邻点云之间的距离相差在d_thre之内，则认为是同一个簇
        std::vector<std::vector<bool>> vis_mat(N_SCAN, std::vector<bool>(Horizon_SCAN, false));

        std::vector<pcl::PointCloud<PointXYZIRSCA>::Ptr> clusters_v; // 聚类后的点云容器

        int d_offset[4] = {-1, 1, 2, 3};

        // float d_thre = 0.2; // D判定是否为同一类的相邻点距离阈值
        // int cluster_size = 0; // 类的数量
        double clustering_tolerance = 0.15;

        for (int i = 0; i < project_mat.rows(); i++)
        {
            for (int j = 0; j < project_mat.cols(); j++)
            {
                if (project_mat(i, j).range > minimumRange && vis_mat[i][j] == false)
                {
                    vis_mat[i][j] = true;

                    pcl::PointCloud<PointXYZIRSCA>::Ptr cluster_cloud(new pcl::PointCloud<PointXYZIRSCA>);

                    std::queue<std::pair<int, int>> q;
                    q.push({i, j});
                    while (!q.empty())
                    {
                        auto curr_ij = q.front();
                        q.pop();
                        int curr_i = curr_ij.first;
                        int curr_j = curr_ij.second;

                        cluster_cloud->push_back(project_mat(curr_i, curr_j));

                        // NCLT 数据集的点云较稀疏
                        int k_start = 0, k_end = 1;
                        // if (scan_sparse == true)
                        // if (1)
                        // {
                        //     k_start = 0, k_end = 2;
                        // }
                        for (int k = k_start; k <= k_end; k++) // 横向查找
                        {
                            int next_i = curr_i + d_offset[k];
                            if (next_i >= 0 && next_i < N_SCAN && project_mat(next_i, curr_j).range && vis_mat[next_i][curr_j] == false)
                            {
                                if (abs(project_mat(curr_i, curr_j).range - project_mat(next_i, curr_j).range) < clustering_tolerance)
                                {
                                    vis_mat[next_i][curr_j] = true;
                                    q.push({next_i, curr_j});
                                }
                            }
                        }

                        k_start = 1, k_end = 1;
                        // if (scan_sparse == true)
                        // if (1)
                        // {
                        //     k_start = 1;
                        //     k_end = 2;
                        // }
                        for (int k = k_start; k <= k_end; k++) // 纵向查找
                        {
                            int next_j = curr_j + d_offset[k];
                            if (next_j >= 0 && next_j < Horizon_SCAN && project_mat(curr_i, next_j).range && vis_mat[curr_i][next_j] == false)
                            {
                                if (abs(project_mat(curr_i, curr_j).range - project_mat(curr_i, next_j).range) < 2 * clustering_tolerance)
                                {
                                    vis_mat[curr_i][next_j] = true;
                                    q.push({curr_i, next_j});
                                }
                            }
                        }
                    }
                    int ransac_pole_min_inliers = 10;
                    if (cluster_cloud->size() > ransac_pole_min_inliers)
                    {
                        clusters_v.push_back(cluster_cloud);
                    }
                }
            }
        }

        std::cout << "clusters_v size:" << clusters_v.size() << std::endl;

        // edgePoints.resize(nScans);

        // 3.对cluster_v去除长宽比大于1的类
        for (int i = 0; i < clusters_v.size(); i++)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
            for (int j = 0; j < clusters_v[i]->points.size(); j++)
            {
                pcl::PointXYZI p;
                p.x = clusters_v[i]->points[j].x;
                p.y = clusters_v[i]->points[j].y;
                p.z = clusters_v[i]->points[j].z;
                p.intensity = clusters_v[i]->points[j].intensity;
                cluster->push_back(p);
            }

            if ((cluster->points.size() < 6))
            {
                continue;
            }

            Eigen::Vector4f pcaCentroid;
            pcl::compute3DCentroid(*cluster, pcaCentroid); // 计算该点云的质心
            Eigen::Matrix3f covariance;
            pcl::computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
            Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
            Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();

            // std::cout << eigenValuesPCA.transpose() << std::endl;

            Eigen::Vector3f l_z(0, 0, 1);
            Eigen::Vector3f e_v = eigenVectorsPCA.block<3, 1>(0, 2);
            double angle_ = acos(fabs(l_z.dot(e_v) / (l_z.norm() * e_v.norm()))) * 180 / M_PI;

            if (eigenValuesPCA(2) < 3 * eigenValuesPCA(1) || angle_ > 30)
            {
                continue;
            }

            pcl::PointXYZI minPt, maxPt;
            pcl::getMinMax3D(*cluster, minPt, maxPt);

            float h = maxPt.z - minPt.z;
            float w = sqrt(pow(maxPt.x - minPt.x, 2) + pow(maxPt.y - minPt.y, 2));

            if ((h / w < 1)) // 点云高度过小或者纵横比过大
            {
                continue;
            }

            std::vector<PointXYZSCA> final_cluster;
            for (int j = 0; j < clusters_v[i]->points.size(); j++)
            {
                PointXYZSCA p;
                int ring = clusters_v[i]->points[j].row_id;
                p.x = clusters_v[i]->points[j].x;
                p.y = clusters_v[i]->points[j].y;
                p.z = clusters_v[i]->points[j].z;
                // p.intensity = clusters_v[i]->points[j].intensity;
                p.scan_position = clusters_v[i]->points[j].scan_position;
                p.curvature = clusters_v[i]->points[j].curvature;
                p.angle = clusters_v[i]->points[j].angle;

                final_cluster.push_back(p);

                // if (ring >= 0 && ring < nScans)
                // {
                //     edgePoints[ring].push_back(p);
                // }
            }

            clusters.push_back(final_cluster);
        }
    }

}
