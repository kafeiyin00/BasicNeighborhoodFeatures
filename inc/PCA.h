//
// Created by 86384 on 2019/10/14.
//

#ifndef BASICNEIGHBORHOODFEATURES_PCA_H
#define BASICNEIGHBORHOODFEATURES_PCA_H

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


namespace BNF{

    struct EigenValues  // lamada1 > lamada2 > lamada3;
    {
        double lamada1;
        double lamada2;
        double lamada3;
    };

    struct EigenVectors  //
    {
        Eigen::Vector3f principalDirection;
        Eigen::Vector3f middleDirection;
        Eigen::Vector3f normalDirection;
    };

    struct PCAFeature  //
    {
        EigenValues values;  //
        EigenVectors vectors;//
        double curvature;   //
        double linear;      //
        double planar;      //
        double spherical;
        double linear_2;
        double planar_2;
        double spherical_2;
        pcl::PointXYZ pt;
        size_t ptId;
        size_t ptNum;
    };

    class PCAAnalyzer{
    public:
        PCAAnalyzer(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud):
        _cloud(cloud){
            LOG(INFO) << "PCA input point size :" << _cloud->points.size();
        }

        void estimateFeature();
    private:
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud;
        PCAFeature _pcaFeature;
    };

    inline void PCAAnalyzer::estimateFeature() {
        Eigen::MatrixXf raw_points(3,_cloud->size());
        Eigen::Vector3f average(0,0,0);
        for (int i = 0; i < _cloud->size(); ++i) {
            pcl::PointXYZ &pt = _cloud->points[i];
            raw_points.col(i) = Eigen::Vector3f(pt.x,pt.y,pt.z);
            average += Eigen::Vector3f(pt.x,pt.y,pt.z);
        }
        average /= _cloud->size();
        for (int i = 0; i < _cloud->size(); ++i){
            raw_points.col(i) = raw_points.col(i) - average;
        }

        Eigen::Matrix3f cov_matrix = raw_points*raw_points.transpose();
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig; // default constructor
        eig.computeDirect(cov_matrix); // works for 2x2 and 3x3 matrices, does not require loops
        Eigen::Vector3f D = eig.eigenvalues();

        Eigen::Vector3f sD = Eigen::Vector3f(sqrt(abs(D(0))), sqrt(abs(D(1))), sqrt(abs(D(2))));
        float line_feature = (sD(2) - sD(1))/ sD(2);

        _pcaFeature.linear = line_feature;

    }
}

#endif //BASICNEIGHBORHOODFEATURES_PCA_H
