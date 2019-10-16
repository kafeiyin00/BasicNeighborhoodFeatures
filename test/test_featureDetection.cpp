//
// Created by Jianping on 2019/10/16.
//

#include "3DFeatureDetection.h"

#include <glog/logging.h>
#include "Config.h"

#include <pcl/io/ply_io.h>

#include <pcl/octree/octree_search.h>

int main(){
    //load data
    std::string testDataFilename = DATA_FOLDER_PATH"pts.ply";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPLYFile<pcl::PointXYZ>(testDataFilename, *cloud) == -1) //* load the file
    {
        LOG(INFO)<<"bad file";
        return (-1);
    }

    LOG(INFO) << "loaded points :" <<cloud->size();

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr octree(
            new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(5.0));
    octree->setInputCloud(cloud);
    octree->addPointsFromInputCloud();
    
    BNF::featureDetectionOption option;
    option.ratioMax = 0.1;
    option.radiusFeatureCalculation = 3;
    option.minPtNum = 50;
    option.radiusNonMax = 3;

    BNF::TrDimFeatureDetector featureDetector(option);

    std::vector<int> indexes;
    featureDetector.detectionBasedOnCurvature(cloud,octree,indexes);

    //TODO: Weitong: add 3d debugger for the feature detection and parameter setting
}