//
// Created by 86384 on 2019/10/14.
//

#ifndef BASICNEIGHBORHOODFEATURES_PCA_H
#define BASICNEIGHBORHOODFEATURES_PCA_H

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <glog/logging.h>


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
        size_t ptNum;
        bool valid;
        pcl::PointXYZ pt;
        int idx;
    };

    class PCAAnalyzer{
    public:
        PCAAnalyzer(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud):
        _cloud(cloud){
            LOG(INFO) << "PCA input point size :" << _cloud->points.size();
        }

        void estimateFeature();

        const PCAFeature getPCAFeature();

    private:
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud;
        PCAFeature _pcaFeature;
    };

    inline void PCAAnalyzer::estimateFeature() {

        CHECK(_cloud->size() > 3) << "selected points is fewer than 3";

        _pcaFeature.ptNum = _cloud->size();

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
        Eigen::Vector3f eigenValues = eig.eigenvalues();
        Eigen::Matrix3f eigenVectors = eig.eigenvectors();

        _pcaFeature.valid = true;

        _pcaFeature.vectors.principalDirection = eigenVectors.col(2);
        _pcaFeature.vectors.middleDirection = eigenVectors.col(1);
        _pcaFeature.vectors.normalDirection = eigenVectors.col(0);

        _pcaFeature.values.lamada1 = eigenValues(2);
        _pcaFeature.values.lamada2 = eigenValues(1);
        _pcaFeature.values.lamada3 = eigenValues(0);

        if ((_pcaFeature.values.lamada1 + _pcaFeature.values.lamada2 + _pcaFeature.values.lamada3) < 1e-6)
        {
            _pcaFeature.curvature = 0;
        }
        else
        {
            _pcaFeature.curvature = _pcaFeature.values.lamada3 / (_pcaFeature.values.lamada1 + _pcaFeature.values.lamada2 + _pcaFeature.values.lamada3);
        }

        _pcaFeature.linear = (sqrt(_pcaFeature.values.lamada1) - sqrt(_pcaFeature.values.lamada2)) / sqrt(_pcaFeature.values.lamada1);
        _pcaFeature.planar = (sqrt(_pcaFeature.values.lamada2) - sqrt(_pcaFeature.values.lamada3)) / sqrt(_pcaFeature.values.lamada1);
        _pcaFeature.spherical = sqrt(_pcaFeature.values.lamada3) / sqrt(_pcaFeature.values.lamada1);
        _pcaFeature.linear_2 = ((_pcaFeature.values.lamada1) - (_pcaFeature.values.lamada2)) / (_pcaFeature.values.lamada1);
        _pcaFeature.planar_2 = ((_pcaFeature.values.lamada2) - (_pcaFeature.values.lamada3)) / (_pcaFeature.values.lamada1);
        _pcaFeature.spherical_2 = (_pcaFeature.values.lamada3) / (_pcaFeature.values.lamada1);

    }

    inline const PCAFeature PCAAnalyzer::getPCAFeature() {
        return _pcaFeature;
    }
}

#endif //BASICNEIGHBORHOODFEATURES_PCA_H
