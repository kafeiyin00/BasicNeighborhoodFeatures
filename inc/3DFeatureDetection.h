//
// Created by Jianping on 2019/10/14.
//

#ifndef BASICNEIGHBORHOODFEATURES_3DFEATUREDETECTION_H
#define BASICNEIGHBORHOODFEATURES_3DFEATUREDETECTION_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/kdtree/kdtree_flann.h>
#include "PCA.h"

namespace BNF{

    struct featureDetectionOption
    {
        float radiusFeatureCalculation;
        float ratioMax;
        size_t minPtNum;
        float radiusNonMax;
    };


    class TrDimFeatureDetector{
    public:
        TrDimFeatureDetector(const featureDetectionOption& option):
                _option(option){

        }

        void detectionBasedOnCurvature(
                const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                const pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree,
                std::vector<int>& indexes);

    private:
        void pruneUnstablePoints(const std::vector<PCAFeature> &pcaFeatures,
                std::vector<PCAFeature> &pcaFeatures_stage1);

        void nonMaximaSuppression(std::vector<PCAFeature> &pcaFeatures_stage1,
                std::vector<PCAFeature> &pcaFeatures_stage2);

    private:
        const featureDetectionOption& _option;
    };

}


#endif //BASICNEIGHBORHOODFEATURES_3DFEATUREDETECTION_H
