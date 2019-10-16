//
// Created by weitongwu on 2019/10/16.
//

#ifndef BASICNEIGHBORHOODFEATURES_PANGOVIS_H
#define BASICNEIGHBORHOODFEATURES_PANGOVIS_H

#include "PangoCloud.h"
#include <pcl/common/centroid.h>


class PangoVis
{
public:
    PangoVis();
    bool process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb);

};
#endif //BASICNEIGHBORHOODFEATURES_PANGOVIS_H
