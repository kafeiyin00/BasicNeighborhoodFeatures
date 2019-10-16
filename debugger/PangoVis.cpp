//
// Created by weitongwu on 2019/10/16.
//

#include "PangoVis.h"
PangoVis::PangoVis()
{

}


bool  PangoVis::process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb)
{
    //可视化
    pangolin::CreateWindowAndBind("Main",640,480);
    glEnable(GL_DEPTH_TEST);

    // 点云重心
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloudrgb, centroid);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,500),
            pangolin::ModelViewLookAt(centroid[0],centroid[1],centroid[2], 0,0,0, pangolin::AxisY));

    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
            .SetHandler(&handler);


    while( !pangolin::ShouldQuit() )
    {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        PangoCloud cloudrgb1(cloudrgb.get());
        cloudrgb1.drawPoints();

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }

    return true;
}