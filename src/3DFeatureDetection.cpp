//
// Created by Jianping on 2019/10/14.
//



#include "3DFeatureDetection.h"
#include "PCA.h"
#include <set>
#include <pcl/kdtree/kdtree_flann.h>

using namespace BNF;

void TrDimFeatureDetector::pruneUnstablePoints(
        const std::vector<PCAFeature> &pcaFeatures,
        std::vector<PCAFeature> &pcaFeatures_stage1) {
    for (int i = 0; i < pcaFeatures.size(); ++i)
    {
        float ratio1, ratio2;
        ratio1 = pcaFeatures[i].values.lamada2 / pcaFeatures[i].values.lamada1;
        ratio2 = pcaFeatures[i].values.lamada3 / pcaFeatures[i].values.lamada2;

        if (    ratio1 < _option.ratioMax &&
                ratio2 < _option.ratioMax &&
                pcaFeatures[i].valid &&
                pcaFeatures[i].ptNum > _option.minPtNum
                )
        {
            pcaFeatures_stage1.push_back(pcaFeatures[i]);
        }
    }
}

bool cmpBasedOnCurvature(PCAFeature &a, PCAFeature &b)
{
    return  a.curvature > b.curvature;
}

void TrDimFeatureDetector::nonMaximaSuppression(std::vector<PCAFeature> &pcaFeatures_stage1,
                                                std::vector<PCAFeature> &pcaFeatures_stage2) {

    sort(pcaFeatures_stage1.begin(), pcaFeatures_stage1.end(), cmpBasedOnCurvature);
    pcl::PointCloud<pcl::PointXYZ> pointCloud;

    /*建立UnSegment以及UnSegment的迭代器,存储未分割的点号;*/
    std::set<size_t, std::less<size_t>> unVisitedPtId;
    std::set<size_t, std::less<size_t>>::iterator iterUnseg;
    LOG(INFO) << "stable points size"  << pcaFeatures_stage1.size();
    for (size_t i = 0; i < pcaFeatures_stage1.size(); ++i)
    {
        unVisitedPtId.insert(i);
        pointCloud.points.push_back(pcaFeatures_stage1[i].pt);
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(pointCloud.makeShared());

    //邻域搜索所用变量
    std::vector<int> search_indices;
    std::vector<float> distances;

    size_t keypointNum = 0;
    do
    {
        keypointNum++;
        std::vector<int>().swap(search_indices);
        std::vector<float>().swap(distances);

        size_t id;
        iterUnseg = unVisitedPtId.begin();
        id = *iterUnseg;
        pcaFeatures_stage2.push_back(pcaFeatures_stage1[id]);
        unVisitedPtId.erase(id);

        tree.radiusSearch(pcaFeatures_stage1[id].pt, _option.radiusNonMax, search_indices, distances);

        for (size_t i = 0; i < search_indices.size(); ++i)
        {
            unVisitedPtId.erase(search_indices[i]);
        }

    } while (!unVisitedPtId.empty());

}

void TrDimFeatureDetector::detectionBasedOnCurvature(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        const pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree,
        std::vector<int>& indexes) {

    std::vector<BNF::PCAFeature> pcaFeatures;
    pcaFeatures.resize(cloud->points.size());

#pragma omp parallel for num_threads(6)
    for (int i = 0; i < cloud->points.size(); i++)
    {
        std::vector<int> indices; //Vector
        std::vector<float> distances;    //Vector

        kdtree->radiusSearch(i, _option.radiusFeatureCalculation, indices, distances);

        if(indices.size() < 5){
            pcaFeatures[i].valid = false;
            continue;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        for (int i = 0; i < indices.size(); ++i) {
            pcl::PointXYZ &pt = cloud->points[indices[i]];
            tmp_cloud->push_back(pt);
        }

        BNF::PCAAnalyzer pcaAnalyzer(tmp_cloud);
        pcaAnalyzer.estimateFeature();
        pcaFeatures[i] = pcaAnalyzer.getPCAFeature();

        //important !! pay attention
        pcaFeatures[i].pt = cloud->points[i];
        pcaFeatures[i].idx = i;
    }
    LOG(INFO)<<"PCA calculate done" ;

    std::vector<BNF::PCAFeature> pcaFeatures_stage1;
    pruneUnstablePoints(pcaFeatures,pcaFeatures_stage1);

    std::vector<BNF::PCAFeature> pcaFeatures_stage2;
    nonMaximaSuppression(pcaFeatures_stage1,pcaFeatures_stage2);

    //copy results
    for (int i = 0; i < pcaFeatures_stage2.size(); ++i) {
        indexes.push_back(pcaFeatures_stage2[i].idx);
    }
    LOG(INFO)<<"we found key point size: " << indexes.size();
}

