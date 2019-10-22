//
// Created by Jianping on 2019/10/16.
//

#include "3DFeatureDetection.h"
#include "PangoVis.h"

#include <glog/logging.h>
#include "Config.h"

#include <pcl/io/ply_io.h>

#include <pcl/kdtree/kdtree_flann.h>

int main(){
    //load data
    std::string testDataFilename = DATA_FOLDER_PATH"dempointcloud.ply";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPLYFile<pcl::PointXYZ>(testDataFilename, *cloud) == -1) //* load the file
    {
        LOG(INFO)<<"bad file";
        return (-1);
    }

    LOG(INFO) << "loaded points :" <<cloud->size();

    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>());  //建立kdtree对象
    kdtree->setInputCloud(cloud);
    
    BNF::featureDetectionOption option;
    option.ratioMax = 0.9;
    option.radiusFeatureCalculation = 10;
    option.minPtNum = 20;
    option.radiusNonMax = 10;

    BNF::TrDimFeatureDetector featureDetector(option);

    std::vector<int> indexes;
    featureDetector.detectionBasedOnCurvature(cloud,kdtree,indexes);

    //TODO: Weitong: add 3d debugger for the feature detection and parameter setting
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypointrgb(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (int k = 0; k < cloud->points.size() ; ++k) {
        pcl::PointXYZRGB pt;
        pt.x = cloud->points[k].x;
        pt.y = cloud->points[k].y;
        pt.z = cloud->points[k].z;
        pt.r= 0;
        pt.g= 255;
        pt.b= 0;
        cloudrgb->push_back(pt);
    }
    for (int j = 0; j < indexes.size(); ++j) {
        pcl::PointXYZRGB pt;
        pt.x = cloud->points[indexes[j]].x;
        pt.y = cloud->points[indexes[j]].y;
        pt.z = cloud->points[indexes[j]].z;
        pt.r= 0;
        pt.g= 0;
        pt.b= 255;
        keypointrgb->push_back(pt);
    }
    int pointsize[2] = {1,7};
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudrgbvector;
    cloudrgbvector.push_back(cloudrgb);
    cloudrgbvector.push_back(keypointrgb);
    PangoVis Vis;
    Vis.process(cloudrgbvector,pointsize);

}