//
// Created by Jianping on 2019/10/14.
//

#include <glog/logging.h>
#include "Config.h"
#include "PCA.h"

#include <pcl/io/ply_io.h>

#include <pcl/octree/octree_search.h>

#include "PangoVis.h"

int main(){
    LOG(INFO)<<"#-- test env";

    //load data
    std::string testDataFilename = DATA_FOLDER_PATH"pts.ply";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPLYFile<pcl::PointXYZ>(testDataFilename, *cloud) == -1) //* load the file
    {
        LOG(INFO)<<"bad file";
        return (-1);
    }

    LOG(INFO) << "loaded points :" <<cloud->size();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb(new pcl::PointCloud<pcl::PointXYZRGB>());

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (2.0);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::DepthFirstIterator it = octree.depth_begin();

    while(*it != nullptr){
        //LOG(INFO) << "depth" <<it.getCurrentOctreeDepth();

        pcl::octree::OctreeNode* node = it.getCurrentOctreeNode();
        if (node->getNodeType() == pcl::octree::LEAF_NODE){
            std::vector<int> indices;
            it.getLeafContainer().getPointIndices(indices);

            //LOG(INFO) << "point size:" << indices.size();
            if(indices.size() < 20){
                it++;
                continue;
            }

            //extract points
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

            for (int i = 0; i < indices.size(); ++i) {
                pcl::PointXYZ &pt = cloud->points[indices[i]];
                tmp_cloud->push_back(pt);
            }

            BNF::PCAAnalyzer pcaAnalyzer(tmp_cloud);
            pcaAnalyzer.estimateFeature();

//            LOG(INFO) << "planar feature: " << pcaAnalyzer.getPCAFeature().planar;

            //TODO: Weitong: add 3d debugger for the pcafeature
            for (int j = 0; j < indices.size(); ++j) {
                pcl::PointXYZRGB pt;
                pt.x = cloud->points[indices[j]].x;
                pt.y = cloud->points[indices[j]].y;
                pt.z = cloud->points[indices[j]].z;
                pt.g= int(255*pcaAnalyzer.getPCAFeature().planar);
                pt.r= 0;
                pt.b= 0;
                cloudrgb->push_back(pt);
            }
        }
        it++;
    }
    int pointsize[1] = {1};
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudrgbvector;
    cloudrgbvector.push_back(cloudrgb);
    //可视化test
    PangoVis Vis;
    Vis.process(cloudrgbvector,pointsize);

    //BNF::PCAAnalyzer pcaAnalyzer(cloud);


}