#include "BoW3D.h"
#include <fstream>

using namespace std;

namespace BoW3D
{
    BoW3D::BoW3D(LinK3D_Extractor *pLinK3D_Extractor, float thr_, int thf_,
                 int num_add_retrieve_features_) : mpLinK3D_Extractor(pLinK3D_Extractor),
                                                   thr(thr_),
                                                   thf(thf_),
                                                   num_add_retrieve_features(num_add_retrieve_features_)
    {
        N_nw_ofRatio = std::make_pair(0, 0);
    }

    // 将新的描述子加入到哈希表<word, place>当中,如果word找到就在场景中添加(frameId,描述子),如果没找到则添加到哈希表中
    void BoW3D::update(Frame *pCurrentFrame)
    {
        mvFrames.emplace_back(pCurrentFrame);

        cv::Mat descriptors = pCurrentFrame->mDescriptors;
        long unsigned int frameId = pCurrentFrame->mnId;

        size_t numFeature = descriptors.rows;

        if (numFeature < (size_t)num_add_retrieve_features)
        {
            for (size_t i = 0; i < numFeature; i++)
            {
                float *p = descriptors.ptr<float>(i);
                for (size_t j = 0; j < (size_t)descriptors.cols; j++)
                {
                    if (p[j] != 0)
                    {
                        unordered_map<pair<float, int>, unordered_set<pair<int, int>, pair_hash>, pair_hash>::iterator it;

                        pair<float, int> word = make_pair(p[j], j);
                        it = this->find(word);

                        if (it == this->end())
                        {
                            unordered_set<pair<int, int>, pair_hash> place;
                            place.insert(make_pair(frameId, i));
                            (*this)[word] = place;

                            N_nw_ofRatio.first++;
                            N_nw_ofRatio.second++;
                        }
                        else
                        {
                            (*it).second.insert(make_pair(frameId, i));
                            N_nw_ofRatio.second++;
                        }
                    }
                }
            }
        }
        else
        {
            for (size_t i = 0; i < (size_t)num_add_retrieve_features; i++)
            {
                float *p = descriptors.ptr<float>(i);
                for (size_t j = 0; j < (size_t)descriptors.cols; j++)
                {
                    if (p[j] != 0)
                    {
                        unordered_map<pair<float, int>, unordered_set<pair<int, int>, pair_hash>, pair_hash>::iterator it;

                        pair<float, int> word = make_pair(p[j], j);
                        it = this->find(word);

                        if (it == this->end())
                        {
                            unordered_set<pair<int, int>, pair_hash> place;
                            place.insert(make_pair(frameId, i));
                            (*this)[word] = place;

                            N_nw_ofRatio.first++;
                            N_nw_ofRatio.second++;
                        }
                        else
                        {
                            (*it).second.insert(make_pair(frameId, i));

                            N_nw_ofRatio.second++;
                        }
                    }
                }
            }
        }
    }

    void BoW3D::retrieve(Frame *pCurrentFrame, int &loopFrameId, Eigen::Matrix3d &loopRelR, Eigen::Vector3d &loopRelt, double &match_score)
    {
        int frameId = pCurrentFrame->mnId;

        cv::Mat descriptors = pCurrentFrame->mDescriptors;
        size_t rowSize = descriptors.rows;

        map<int, int> mScoreFrameID;

        if (rowSize < (size_t)num_add_retrieve_features)
        {
            for (size_t i = 0; i < rowSize; i++)
            {
                unordered_map<pair<int, int>, int, pair_hash> mPlaceScore;

                float *p = descriptors.ptr<float>(i);

                int countValue = 0;

                for (size_t j = 0; j < (size_t)descriptors.cols; j++)
                {
                    countValue++;

                    if (p[j] != 0)
                    {
                        pair<float, int> word = make_pair(p[j], j);
                        auto wordPlacesIter = this->find(word);

                        if (wordPlacesIter == this->end())
                        {
                            continue;
                        }
                        else
                        {
                            double averNumInPlaceSet = N_nw_ofRatio.second / N_nw_ofRatio.first;
                            int curNumOfPlaces = (wordPlacesIter->second).size();

                            double ratio = curNumOfPlaces / averNumInPlaceSet;

                            if (ratio > thr)
                            {
                                continue;
                            }

                            for (auto placesIter = (wordPlacesIter->second).begin(); placesIter != (wordPlacesIter->second).end(); placesIter++)
                            {
                                // The interval between the loop and the current frame should be at least 300.
                                if (frameId - (*placesIter).first < 300)
                                {
                                    continue;
                                }

                                auto placeNumIt = mPlaceScore.find(*placesIter);

                                if (placeNumIt == mPlaceScore.end())
                                {
                                    mPlaceScore[*placesIter] = 1;
                                }
                                else
                                {
                                    mPlaceScore[*placesIter]++;
                                }
                            }
                        }
                    }
                }

                for (auto placeScoreIter = mPlaceScore.begin(); placeScoreIter != mPlaceScore.end(); placeScoreIter++)
                {
                    if ((*placeScoreIter).second > thf)
                    {
                        mScoreFrameID[(*placeScoreIter).second] = ((*placeScoreIter).first).first;
                    }
                }
            }
        }
        else
        {
            for (size_t i = 0; i < (size_t)num_add_retrieve_features; i++)
            {
                unordered_map<pair<int, int>, int, pair_hash> mPlaceScore;

                float *p = descriptors.ptr<float>(i);

                int countValue = 0;

                for (size_t j = 0; j < (size_t)descriptors.cols; j++)
                {
                    countValue++;

                    if (p[j] != 0)
                    {
                        pair<float, int> word = make_pair(p[j], j);

                        auto wordPlacesIter = this->find(word);

                        if (wordPlacesIter == this->end())
                        {
                            continue;
                        }
                        else
                        {
                            double averNumInPlaceSet = (double)N_nw_ofRatio.second / N_nw_ofRatio.first;
                            int curNumOfPlaces = (wordPlacesIter->second).size();

                            double ratio = curNumOfPlaces / averNumInPlaceSet;

                            if (ratio > thr)
                            {
                                continue;
                            }

                            for (auto placesIter = (wordPlacesIter->second).begin(); placesIter != (wordPlacesIter->second).end(); placesIter++)
                            {
                                // The interval between the loop and the current frame should be at least 300.
                                if (frameId - (*placesIter).first < 300)
                                {
                                    continue;
                                }

                                auto placeNumIt = mPlaceScore.find(*placesIter);

                                if (placeNumIt == mPlaceScore.end())
                                {
                                    mPlaceScore[*placesIter] = 1;
                                }
                                else
                                {
                                    mPlaceScore[*placesIter]++;
                                }
                            }
                        }
                    }
                }

                for (auto placeScoreIter = mPlaceScore.begin(); placeScoreIter != mPlaceScore.end(); placeScoreIter++)
                {
                    if ((*placeScoreIter).second > thf)
                    {
                        mScoreFrameID[(*placeScoreIter).second] = ((*placeScoreIter).first).first;
                    }
                }
            }
        }

        if (mScoreFrameID.size() == 0)
        {
            return;
        }

        for (auto it = mScoreFrameID.rbegin(); it != mScoreFrameID.rend(); it++)
        {
            int loopId = (*it).second;

            Frame *pLoopFrame = mvFrames[loopId];
            vector<pair<int, int>> vMatchedIndex;

            mpLinK3D_Extractor->match(pCurrentFrame->mvAggregationKeypoints, pLoopFrame->mvAggregationKeypoints, pCurrentFrame->mDescriptors, pLoopFrame->mDescriptors, vMatchedIndex);

            int returnValue = 0;
            Eigen::Matrix3d loopRelativeR;
            Eigen::Vector3d loopRelativet;

            returnValue = loopCorrection(pCurrentFrame, pLoopFrame, vMatchedIndex, loopRelativeR, loopRelativet, match_score);

            // The distance between the loop and the current should less than 3m.
            if (returnValue != -1 && loopRelativet.norm() < 3 && loopRelativet.norm() > 0)
            {
                loopFrameId = (*it).second;
                loopRelR = loopRelativeR;
                loopRelt = loopRelativet;

                return;
            }
        }
    }

    int BoW3D::loopCorrection(Frame *currentFrame, Frame *matchedFrame, vector<pair<int, int>> &vMatchedIndex, Eigen::Matrix3d &R, Eigen::Vector3d &t, double &match_score)
    {
        if (vMatchedIndex.size() <= 30)
        {
            return -1;
        }

        ScanEdgePoints currentFiltered;
        ScanEdgePoints matchedFiltered;
        mpLinK3D_Extractor->filterLowCurv(currentFrame->mClusterEdgeKeypoints, currentFiltered);
        mpLinK3D_Extractor->filterLowCurv(matchedFrame->mClusterEdgeKeypoints, matchedFiltered);

        vector<std::pair<PointXYZSCA, PointXYZSCA>> matchedEdgePt;
        mpLinK3D_Extractor->findEdgeKeypointMatch(currentFiltered, matchedFiltered, vMatchedIndex, matchedEdgePt);

        pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>());

        pcl::CorrespondencesPtr corrsPtr(new pcl::Correspondences());

        for (int i = 0; i < (int)matchedEdgePt.size(); i++)
        {
            std::pair<PointXYZSCA, PointXYZSCA> matchPoint = matchedEdgePt[i];

            pcl::PointXYZ sourcePt(matchPoint.first.x, matchPoint.first.y, matchPoint.first.z);
            pcl::PointXYZ targetPt(matchPoint.second.x, matchPoint.second.y, matchPoint.second.z);

            source->push_back(sourcePt);
            target->push_back(targetPt);

            pcl::Correspondence correspondence(i, i, 0);
            corrsPtr->push_back(correspondence);
        }

        pcl::Correspondences corrs;
        pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> Ransac_based_Rejection;
        Ransac_based_Rejection.setInputSource(source);
        Ransac_based_Rejection.setInputTarget(target);
        double sac_threshold = 0.4;
        Ransac_based_Rejection.setInlierThreshold(sac_threshold);
        Ransac_based_Rejection.getRemainingCorrespondences(*corrsPtr, corrs);

        if (corrs.size() <= 100)
        {
            return -1;
        }

        Eigen::Vector3d p1 = Eigen::Vector3d::Zero();
        Eigen::Vector3d p2 = p1;
        int corrSize = (int)corrs.size();
        for (int i = 0; i < corrSize; i++)
        {
            pcl::Correspondence corr = corrs[i];
            p1(0) += source->points[corr.index_query].x;
            p1(1) += source->points[corr.index_query].y;
            p1(2) += source->points[corr.index_query].z;

            p2(0) += target->points[corr.index_match].x;
            p2(1) += target->points[corr.index_match].y;
            p2(2) += target->points[corr.index_match].z;
        }

        Eigen::Vector3d center1 = Eigen::Vector3d(p1(0) / corrSize, p1(1) / corrSize, p1(2) / corrSize);
        Eigen::Vector3d center2 = Eigen::Vector3d(p2(0) / corrSize, p2(1) / corrSize, p2(2) / corrSize);

        vector<Eigen::Vector3d> vRemoveCenterPt1, vRemoveCenterPt2;
        for (int i = 0; i < corrSize; i++)
        {
            pcl::Correspondence corr = corrs[i];
            pcl::PointXYZ sourcePt = source->points[corr.index_query];
            pcl::PointXYZ targetPt = target->points[corr.index_match];

            Eigen::Vector3d removeCenterPt1 = Eigen::Vector3d(sourcePt.x - center1(0), sourcePt.y - center1(1), sourcePt.z - center1(2));
            Eigen::Vector3d removeCenterPt2 = Eigen::Vector3d(targetPt.x - center2(0), targetPt.y - center2(1), targetPt.z - center2(2));

            vRemoveCenterPt1.emplace_back(removeCenterPt1);
            vRemoveCenterPt2.emplace_back(removeCenterPt2);
        }

        Eigen::Matrix3d w = Eigen::Matrix3d::Zero();

        for (int i = 0; i < corrSize; i++)
        {
            w += vRemoveCenterPt1[i] * vRemoveCenterPt2[i].transpose();
        }

        Eigen::JacobiSVD<Eigen::Matrix3d> svd(w, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d U = svd.matrixU();
        Eigen::Matrix3d V = svd.matrixV();

        R = V * U.transpose();
        t = center2 - R * center1;

        // 计算的得分：最临近点距离 之和 除以 点数
        // https://zhuanlan.zhihu.com/p/651000456#:~:text=%E4%B8%80%E3%80%81PCL%E4%B8%ADICP%E9%85%8D%E5%87%86%E7%9A%84getFitnessScore%20%28%29%E5%87%BD%E6%95%B0%E8%BF%94%E5%9B%9E%E5%80%BC%E6%9C%80%E8%BF%91%E7%9C%8B%E5%88%B0%E5%BE%88%E5%A4%9A%E8%A7%A3%E9%87%8A%E8%AF%B4ICP%E9%85%8D%E5%87%86%E7%9A%84%E6%9C%80%E5%90%8E%E9%85%8D%E5%87%86%E5%88%86%E6%95%B0%E6%98%AF%E5%9D%87%E6%96%B9%E8%AF%AF%E5%B7%AEMSE%EF%BC%8C%E6%98%AF%E4%B8%8D%E5%AF%B9%E7%9A%84%E3%80%82%20%E7%9B%B4%E6%8E%A5%E8%AF%B4%E7%BB%93%E8%AE%BA%EF%BC%8C%E6%98%AF%20%E6%9C%80%E8%BF%91%E7%82%B9%E8%B7%9D%E7%A6%BB%E7%9A%84%E5%B9%B3%E5%9D%87%E5%80%BC%EF%BC%81%20%E5%8D%B3%EF%BC%9A,ICP_FitnessScore%3D%E6%9C%80%E4%B8%B4%E8%BF%91%E7%82%B9%E8%B7%9D%E7%A6%BB%20%E4%B9%8B%E5%92%8C%20%E9%99%A4%E4%BB%A5%20%E7%82%B9%E6%95%B0%20%3D%E2%88%91%E2%80%A6
        double aver_d = 0;
        for (int i = 0; i < corrSize; i++)
        {
            pcl::Correspondence corr = corrs[i];
            Eigen::Vector3d sourcePt(source->points[corr.index_query].x, source->points[corr.index_query].y, source->points[corr.index_query].z);
            Eigen::Vector3d targetPt(target->points[corr.index_match].x, target->points[corr.index_match].y, target->points[corr.index_match].z);

            Eigen::Vector3d transPt = R * sourcePt + t;
            double distance = (transPt - targetPt).norm();
            aver_d += distance;
        }
        aver_d /= corrSize;

        match_score = aver_d;

        // std::cout << "aver_d: " << aver_d << std::endl;

        return 1;
    }

    int BoW3D::get_frame_size()
    {
        return mvFrames.size();
    }

}
