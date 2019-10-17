//
// Created by Jianping on 2019/10/16.
//

#include "BSCDescription.h"
#include <glog/logging.h>
//#include <pcl\registration\correspondence_rejection.h>
//#include <pcl\registration\transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/correspondence_rejection.h>

using namespace BNF;

unsigned char BNF::BSCDescriptor::byteBitsLookUp(unsigned char b) {
    static const unsigned char table[256]  = {
            /* 0 */ 0, /* 1 */ 1, /* 2 */ 1, /* 3 */ 2,
            /* 4 */ 1, /* 5 */ 2, /* 6 */ 2, /* 7 */ 3,
            /* 8 */ 1, /* 9 */ 2, /* a */ 2, /* b */ 3,
            /* c */ 2, /* d */ 3, /* e */ 3, /* f */ 4,
            /* 10 */ 1, /* 11 */ 2, /* 12 */ 2, /* 13 */ 3,
            /* 14 */ 2, /* 15 */ 3, /* 16 */ 3, /* 17 */ 4,
            /* 18 */ 2, /* 19 */ 3, /* 1a */ 3, /* 1b */ 4,
            /* 1c */ 3, /* 1d */ 4, /* 1e */ 4, /* 1f */ 5,
            /* 20 */ 1, /* 21 */ 2, /* 22 */ 2, /* 23 */ 3,
            /* 24 */ 2, /* 25 */ 3, /* 26 */ 3, /* 27 */ 4,
            /* 28 */ 2, /* 29 */ 3, /* 2a */ 3, /* 2b */ 4,
            /* 2c */ 3, /* 2d */ 4, /* 2e */ 4, /* 2f */ 5,
            /* 30 */ 2, /* 31 */ 3, /* 32 */ 3, /* 33 */ 4,
            /* 34 */ 3, /* 35 */ 4, /* 36 */ 4, /* 37 */ 5,
            /* 38 */ 3, /* 39 */ 4, /* 3a */ 4, /* 3b */ 5,
            /* 3c */ 4, /* 3d */ 5, /* 3e */ 5, /* 3f */ 6,
            /* 40 */ 1, /* 41 */ 2, /* 42 */ 2, /* 43 */ 3,
            /* 44 */ 2, /* 45 */ 3, /* 46 */ 3, /* 47 */ 4,
            /* 48 */ 2, /* 49 */ 3, /* 4a */ 3, /* 4b */ 4,
            /* 4c */ 3, /* 4d */ 4, /* 4e */ 4, /* 4f */ 5,
            /* 50 */ 2, /* 51 */ 3, /* 52 */ 3, /* 53 */ 4,
            /* 54 */ 3, /* 55 */ 4, /* 56 */ 4, /* 57 */ 5,
            /* 58 */ 3, /* 59 */ 4, /* 5a */ 4, /* 5b */ 5,
            /* 5c */ 4, /* 5d */ 5, /* 5e */ 5, /* 5f */ 6,
            /* 60 */ 2, /* 61 */ 3, /* 62 */ 3, /* 63 */ 4,
            /* 64 */ 3, /* 65 */ 4, /* 66 */ 4, /* 67 */ 5,
            /* 68 */ 3, /* 69 */ 4, /* 6a */ 4, /* 6b */ 5,
            /* 6c */ 4, /* 6d */ 5, /* 6e */ 5, /* 6f */ 6,
            /* 70 */ 3, /* 71 */ 4, /* 72 */ 4, /* 73 */ 5,
            /* 74 */ 4, /* 75 */ 5, /* 76 */ 5, /* 77 */ 6,
            /* 78 */ 4, /* 79 */ 5, /* 7a */ 5, /* 7b */ 6,
            /* 7c */ 5, /* 7d */ 6, /* 7e */ 6, /* 7f */ 7,
            /* 80 */ 1, /* 81 */ 2, /* 82 */ 2, /* 83 */ 3,
            /* 84 */ 2, /* 85 */ 3, /* 86 */ 3, /* 87 */ 4,
            /* 88 */ 2, /* 89 */ 3, /* 8a */ 3, /* 8b */ 4,
            /* 8c */ 3, /* 8d */ 4, /* 8e */ 4, /* 8f */ 5,
            /* 90 */ 2, /* 91 */ 3, /* 92 */ 3, /* 93 */ 4,
            /* 94 */ 3, /* 95 */ 4, /* 96 */ 4, /* 97 */ 5,
            /* 98 */ 3, /* 99 */ 4, /* 9a */ 4, /* 9b */ 5,
            /* 9c */ 4, /* 9d */ 5, /* 9e */ 5, /* 9f */ 6,
            /* a0 */ 2, /* a1 */ 3, /* a2 */ 3, /* a3 */ 4,
            /* a4 */ 3, /* a5 */ 4, /* a6 */ 4, /* a7 */ 5,
            /* a8 */ 3, /* a9 */ 4, /* aa */ 4, /* ab */ 5,
            /* ac */ 4, /* ad */ 5, /* ae */ 5, /* af */ 6,
            /* b0 */ 3, /* b1 */ 4, /* b2 */ 4, /* b3 */ 5,
            /* b4 */ 4, /* b5 */ 5, /* b6 */ 5, /* b7 */ 6,
            /* b8 */ 4, /* b9 */ 5, /* ba */ 5, /* bb */ 6,
            /* bc */ 5, /* bd */ 6, /* be */ 6, /* bf */ 7,
            /* c0 */ 2, /* c1 */ 3, /* c2 */ 3, /* c3 */ 4,
            /* c4 */ 3, /* c5 */ 4, /* c6 */ 4, /* c7 */ 5,
            /* c8 */ 3, /* c9 */ 4, /* ca */ 4, /* cb */ 5,
            /* cc */ 4, /* cd */ 5, /* ce */ 5, /* cf */ 6,
            /* d0 */ 3, /* d1 */ 4, /* d2 */ 4, /* d3 */ 5,
            /* d4 */ 4, /* d5 */ 5, /* d6 */ 5, /* d7 */ 6,
            /* d8 */ 4, /* d9 */ 5, /* da */ 5, /* db */ 6,
            /* dc */ 5, /* dd */ 6, /* de */ 6, /* df */ 7,
            /* e0 */ 3, /* e1 */ 4, /* e2 */ 4, /* e3 */ 5,
            /* e4 */ 4, /* e5 */ 5, /* e6 */ 5, /* e7 */ 6,
            /* e8 */ 4, /* e9 */ 5, /* ea */ 5, /* eb */ 6,
            /* ec */ 5, /* ed */ 6, /* ee */ 6, /* ef */ 7,
            /* f0 */ 4, /* f1 */ 5, /* f2 */ 5, /* f3 */ 6,
            /* f4 */ 5, /* f5 */ 6, /* f6 */ 6, /* f7 */ 7,
            /* f8 */ 5, /* f9 */ 6, /* fa */ 6, /* fb */ 7,
            /* fc */ 6, /* fd */ 7, /* fe */ 7, /* ff */ 8
    };
    return table[b];
}

int BNF::BSCDescriptor::hammingDistance(const BSCDescriptor& bsdf1, const BSCDescriptor& bsdf2) {

    CHECK(bsdf1._size==bsdf2._size);
    //以位运算形式计算hamming distance
    //首先将对应byte上异或
    int one_count = 0;
    for(int i=0;i<bsdf1._bytes;i++)
    {
        one_count+=byteBitsLookUp(bsdf1._feature[i]^bsdf1._feature[i]);
    }
    return one_count;
}

float Comput3DDistanceBetweenPoints(const pcl::PointXYZ &pt1, const pcl::PointXYZ &pt2)
{
    float dertax, dertay, dertaz, dis;
    dertax = pt1.x - pt2.x;
    dertay = pt1.y - pt2.y;
    dertaz = pt1.z - pt2.z;
    dis = (dertax)*(dertax)+(dertay)*(dertay)+(dertaz)*(dertaz);
    dis = sqrt(dis);
    return dis;
}

bool BNF::BSCDescriptor::computeEigenVectorsByWeightPCA(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                                                        const std::vector<int> &search_indices, int test_index,
                                                        Eigen::Vector3f &principalDirection,
                                                        Eigen::Vector3f &middleDirection,
                                                        Eigen::Vector3f &normalDirection) {

    if (search_indices.size() < 3)
        return false;


    double radius;
    radius = sqrt(2.0)*extract_radius_;

    //计算中心点坐标,以及(radius-d)的和,其中radius是邻域半径,d是邻域点到当前点的距离;
    double center_x(0.0), center_y(0.0), center_z(0.0), dis_all(0.0);
    for (size_t it = 0; it < search_indices.size(); ++it)
    {
        center_x += input_cloud->points[search_indices[it]].x;
        center_y += input_cloud->points[search_indices[it]].y;
        center_z += input_cloud->points[search_indices[it]].z;

        dis_all += (radius - Comput3DDistanceBetweenPoints(input_cloud->points[search_indices[it]], input_cloud->points[test_index]));
    }
    center_x /= search_indices.size();
    center_y /= search_indices.size();
    center_z /= search_indices.size();

    //计算协方差矩阵
    Eigen::Matrix<float, 3, 3> covariance;
    covariance(0, 0) = 0.0f;
    covariance(0, 1) = 0.0f;
    covariance(0, 2) = 0.0f;

    covariance(1, 0) = 0.0f;
    covariance(1, 1) = 0.0f;
    covariance(1, 2) = 0.0f;

    covariance(2, 0) = 0.0f;
    covariance(2, 1) = 0.0f;
    covariance(2, 2) = 0.0f;


    for (size_t it = 0; it < search_indices.size(); ++it)
    {
        float dis, weight;
        dis = Comput3DDistanceBetweenPoints(input_cloud->points[search_indices[it]], input_cloud->points[test_index]);
        weight = radius - dis;
        covariance(0, 0) += weight*(input_cloud->points[search_indices[it]].x - center_x)*(input_cloud->points[search_indices[it]].x - center_x);
        covariance(0, 1) += weight*(input_cloud->points[search_indices[it]].x - center_x)*(input_cloud->points[search_indices[it]].y - center_y);
        covariance(0, 2) += weight*(input_cloud->points[search_indices[it]].x - center_x)*(input_cloud->points[search_indices[it]].z - center_z);

        covariance(1, 0) += weight*(input_cloud->points[search_indices[it]].y - center_y)*(input_cloud->points[search_indices[it]].x - center_x);
        covariance(1, 1) += weight*(input_cloud->points[search_indices[it]].y - center_y)*(input_cloud->points[search_indices[it]].y - center_y);
        covariance(1, 2) += weight*(input_cloud->points[search_indices[it]].y - center_y)*(input_cloud->points[search_indices[it]].z - center_z);


        covariance(2, 0) += weight*(input_cloud->points[search_indices[it]].z - center_z)*(input_cloud->points[search_indices[it]].x - center_x);
        covariance(2, 1) += weight*(input_cloud->points[search_indices[it]].z - center_z)*(input_cloud->points[search_indices[it]].y - center_y);
        covariance(2, 2) += weight*(input_cloud->points[search_indices[it]].z - center_z)*(input_cloud->points[search_indices[it]].z - center_z);

    }
    covariance(0, 0) /= dis_all;
    covariance(0, 1) /= dis_all;
    covariance(0, 2) /= dis_all;

    covariance(1, 0) /= dis_all;
    covariance(1, 1) /= dis_all;
    covariance(1, 2) /= dis_all;

    covariance(2, 0) /= dis_all;
    covariance(2, 1) /= dis_all;
    covariance(2, 2) /= dis_all;

    //求特征值
    Eigen::EigenSolver<Eigen::Matrix3f> es(covariance, true);
    //求得特征值和特征向量
    Eigen::EigenSolver<Eigen::Matrix3f>::EigenvalueType ev = es.eigenvalues();
    Eigen::EigenSolver<Eigen::Matrix3f>::EigenvectorsType evc = es.eigenvectors();

    if (es.info() == Eigen::Success)
    {
        float MaxEigenValue = ev(0).real();
        float MinEigenValue = ev(0).real();
        unsigned int MaxEigenValueId(0), MinEigenValueId(0);

        for (int i = 0; i < 3; i++)
        {
            if (ev(i).real() > MaxEigenValue)
            {
                MaxEigenValueId = i;
                MaxEigenValue = ev(i).real();
            }

            if (ev(i).real() < MinEigenValue)
            {
                MinEigenValueId = i;
                MinEigenValue = ev(i).real();
            }
        }

        principalDirection.x() = evc.col(MaxEigenValueId).x().real();
        principalDirection.y() = evc.col(MaxEigenValueId).y().real();
        principalDirection.z() = evc.col(MaxEigenValueId).z().real();

        normalDirection.x() = evc.col(MinEigenValueId).x().real();
        normalDirection.y() = evc.col(MinEigenValueId).y().real();
        normalDirection.z() = evc.col(MinEigenValueId).z().real();

        middleDirection = principalDirection.cross(normalDirection);
    }
    else
    {
        LOG(INFO) << "未能正确计算该点的坐标系";
        return false;
    }
    return true;
}


void BNF::BSCDescriptor::computeTranformationMatrixBetweenCoordinateSystems(
        const BNF::BSCDescriptor::CoordinateSystem &coordinate_src,
        const BNF::BSCDescriptor::CoordinateSystem &coordinate_traget, Eigen::Matrix4f &tragetToSource) {

    pcl::PointCloud<pcl::PointXYZ> cloud_src, cloud_target;
    pcl::PointXYZ pt;
    //获得cloud_src点云;
    pt.x = coordinate_src.xAxis.x();
    pt.y = coordinate_src.xAxis.y();
    pt.z = coordinate_src.xAxis.z();
    cloud_src.points.push_back(pt);

    pt.x = coordinate_src.yAxis.x();
    pt.y = coordinate_src.yAxis.y();
    pt.z = coordinate_src.yAxis.z();
    cloud_src.points.push_back(pt);

    pt.x = coordinate_src.zAxis.x();
    pt.y = coordinate_src.zAxis.y();
    pt.z = coordinate_src.zAxis.z();
    cloud_src.points.push_back(pt);

    //获得cloud_target点云;
    pt.x = coordinate_traget.xAxis.x();
    pt.y = coordinate_traget.xAxis.y();
    pt.z = coordinate_traget.xAxis.z();
    cloud_target.points.push_back(pt);

    pt.x = coordinate_traget.yAxis.x();
    pt.y = coordinate_traget.yAxis.y();
    pt.z = coordinate_traget.yAxis.z();
    cloud_target.points.push_back(pt);

    pt.x = coordinate_traget.zAxis.x();
    pt.y = coordinate_traget.zAxis.y();
    pt.z = coordinate_traget.zAxis.z();
    cloud_target.points.push_back(pt);


    /*创建点对的对应关系correspondences;*/
    pcl::Correspondences  correspondences;
    pcl::Correspondence   correspondence;
    for (size_t i = 0; i < 3; i++)
    {
        correspondence.index_match = (int)i;
        correspondence.index_query = (int)i;
        correspondences.push_back(correspondence);
    }

    //根据对应关系correspondences计算旋转平移矩阵;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> trans_est;
    Eigen::Matrix4f trans;
    trans_est.estimateRigidTransformation(cloud_src, cloud_target, correspondences, trans);
    tragetToSource = trans.inverse();
}

void BNF::BSCDescriptor::computeEigenVectorsBy2Dpca(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                                                    const std::vector<int> &search_indices, int test_index,
                                                    Eigen::Vector3f &principalDirection) {
    if (search_indices.size() < 3)
        return;

    double radius;
    radius = sqrt(2.0)*extract_radius_;

    //计算中心点坐标,以及(R-d)的和,其中R是邻域半径,d是邻域点到当前点的距离;
    double center_x(0.0), center_y(0.0), center_z(0.0),dis_all(0.0);
    for (size_t it = 0; it < search_indices.size(); ++it)
    {
        center_x += input_cloud->points[search_indices[it]].x;
        center_y += input_cloud->points[search_indices[it]].y;
        center_z += input_cloud->points[search_indices[it]].z;

        dis_all += (radius-Comput3DDistanceBetweenPoints(input_cloud->points[search_indices[it]], input_cloud->points[test_index]));
    }
    center_x /= search_indices.size();
    center_y /= search_indices.size();
    center_z /= search_indices.size();

    //计算协方差矩阵
    Eigen::Matrix<float, 2, 2> covariance;
    covariance(0, 0) = 0.0f;
    covariance(0, 1) = 0.0f;
    covariance(1, 0) = 0.0f;
    covariance(1, 1) = 0.0f;


    for (size_t it = 0; it < search_indices.size(); ++it)
    {
        float dis,weight;
        dis = Comput3DDistanceBetweenPoints(input_cloud->points[search_indices[it]], input_cloud->points[test_index]);
        weight = radius - dis;
        covariance(0, 0) += weight*(input_cloud->points[search_indices[it]].x - center_x)*(input_cloud->points[search_indices[it]].x - center_x);
        covariance(0, 1) += weight*(input_cloud->points[search_indices[it]].x - center_x)*(input_cloud->points[search_indices[it]].y - center_y);
        covariance(1, 0) += weight*(input_cloud->points[search_indices[it]].x - center_x)*(input_cloud->points[search_indices[it]].y - center_y);
        covariance(1, 1) += weight*(input_cloud->points[search_indices[it]].y - center_y)*(input_cloud->points[search_indices[it]].y - center_y);
    }
    covariance(0, 0) /= dis_all;
    covariance(0, 1) /= dis_all;
    covariance(1, 0) /= dis_all;
    covariance(1, 1) /= dis_all;

    //求特征值
    Eigen::EigenSolver<Eigen::Matrix2f> es(covariance, true);
    //求得特征值和特征向量
    Eigen::EigenSolver<Eigen::Matrix2f>::EigenvalueType ev = es.eigenvalues();
    Eigen::EigenSolver<Eigen::Matrix2f>::EigenvectorsType evc = es.eigenvectors();

    if (ev(0).real() > ev(1).real())
    {
        principalDirection.x() = evc.col(0).x().real();
        principalDirection.y() = evc.col(0).y().real();
        principalDirection.z() = 0.0f;
    }
    else
    {
        principalDirection.x() = evc.col(1).x().real();
        principalDirection.y() = evc.col(1).y().real();
        principalDirection.z() = 0.0f;
    }
}

bool BNF::BSCDescriptor::computeLocalCoordinateSystem(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                                                      int test_index, const std::vector<int> &search_indices,
                                                      BNF::BSCDescriptor::CoordinateSystem &localCoordinateSystem) {
    //利用PCA计算关键点局部邻域分布的特征向量;
    Eigen::Vector3f  principalDirectionPca, normalDirectionPca;
    computeEigenVectorsBy2Dpca(input_cloud, search_indices, test_index, principalDirectionPca);
    //得到局部坐标系;
    localCoordinateSystem.zAxis.x() = 0.0f;
    localCoordinateSystem.zAxis.y() = 0.0f;
    localCoordinateSystem.zAxis.z() = 1.0f;

    localCoordinateSystem.xAxis = principalDirectionPca;
    localCoordinateSystem.yAxis = localCoordinateSystem.zAxis.cross(localCoordinateSystem.xAxis);

    localCoordinateSystem.origin.x() = input_cloud->points[test_index].x;
    localCoordinateSystem.origin.y() = input_cloud->points[test_index].y;
    localCoordinateSystem.origin.z() = input_cloud->points[test_index].z;

    //对坐标轴的长度进行归一化;
    localCoordinateSystem.xAxis.normalize();
    localCoordinateSystem.yAxis.normalize();

    return true;
}

void BNF::BSCDescriptor::transformPointCloudToLocalSystem(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                                                          int test_index, const std::vector<int> &search_indices,
                                                          const BNF::BSCDescriptor::CoordinateSystem &localCoordinateSystem,
                                                          pcl::PointCloud<pcl::PointXYZ>::Ptr &result_cloud) {
    //计算局部坐标系到场景坐标系的转换矩阵;
    CoordinateSystem  coordinate_scene;
    coordinate_scene.xAxis.x() = 1.0f; coordinate_scene.xAxis.y() = 0.0f; coordinate_scene.xAxis.z() = 0.0f;
    coordinate_scene.yAxis.x() = 0.0f; coordinate_scene.yAxis.y() = 1.0f; coordinate_scene.yAxis.z() = 0.0f;
    coordinate_scene.zAxis.x() = 0.0f; coordinate_scene.zAxis.y() = 0.0f; coordinate_scene.zAxis.z() = 1.0f;


    Eigen::Matrix4f   tragetToSource;
    computeTranformationMatrixBetweenCoordinateSystems(coordinate_scene, localCoordinateSystem, tragetToSource);

    //对search_indices中的点进行坐标转换,并将以上的点压入reuslt_cloud中;
    //清理内存
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    result_cloud->resize(search_indices.size());
    for (int i = 0; i < search_indices.size(); i++)
    {
        pcl::PointXYZ pt;
        pt.x = input_cloud->points[search_indices[i]].x - input_cloud->points[test_index].x;
        pt.y = input_cloud->points[search_indices[i]].y - input_cloud->points[test_index].y;
        pt.z = input_cloud->points[search_indices[i]].z - input_cloud->points[test_index].z;

        temp_cloud->points.push_back(pt);
    }

    pcl::transformPointCloud(*temp_cloud, *result_cloud, tragetToSource);
}

void BNF::BSCDescriptor::constructCubicGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr &rotated_cloud,
                                            std::vector<GridVoxel> &grid) {
    //邻域搜索所用变量
    pcl::KdTreeFLANN<pcl::PointXY> tree;
    std::vector<int> search_indices;
    std::vector<float> distances;

    /*遍历所有点往XOY面投影,利用高斯距离加权计算落入每个格子中的点个数;*/
    pcl::PointCloud<pcl::PointXY>::Ptr cloud_xy(new  pcl::PointCloud<pcl::PointXY>);
    //将3维点云投影到XOY平面;
    for(size_t i=0;i<rotated_cloud->size();i++)
    {
        pcl::PointXY pt;
        pt.x = rotated_cloud->points[i].x;
        pt.y = rotated_cloud->points[i].y;
        cloud_xy->points.push_back(pt);
    }
    tree.setInputCloud(cloud_xy);

    //遍历每一个格网,找到格网中心点半径1.5*unit_side_length_范围内的点,用这些点加权计算落入格网中的点个数;
    for (size_t i = 0; i < voxel_side_num_; i++)
    {
        for (size_t j = 0; j < voxel_side_num_; j++)
        {
            pcl::PointXY pt;
            float derta;
            derta = unit_side_length_ / 2;
            //计算格网中心点坐标;
            pt.x = (i + 0.5)*unit_side_length_ - extract_radius_;
            pt.y = (j + 0.5)*unit_side_length_ - extract_radius_;
            //搜索邻域内的点;
            std::vector<int>().swap(search_indices);
            std::vector<float>().swap(distances);
//            search_indices.swap(std::vector<int>());
//            distances.swap(std::vector<float>());
            tree.radiusSearch(pt, 1.5*unit_side_length_, search_indices, distances);

            if (distances.empty() != true)
            {
                //邻域内的点加权计算落入格网中的点个数,权值为距离高斯函数exp(-(x-u)*(x-u)* / (2 * derta * derta));
                for (size_t n = 0; n < search_indices.size(); n++)
                {
                    grid[i + j*voxel_side_num_].point_num += exp(-distances[n] / (2 * derta * derta));
                    float depth;
                    depth = rotated_cloud->points[search_indices[n]].z + extract_radius_;
                    grid[i + j*voxel_side_num_].average_depth += depth*exp(-distances[n] / (2 * derta * derta));
                }
            }
        }
    }


    //遍历所有点往XOZ面投影,计算落入每个格子中的点个数;
    pcl::PointCloud<pcl::PointXY>::Ptr cloud_xz(new  pcl::PointCloud<pcl::PointXY>);
    for (size_t i = 0; i < rotated_cloud->size(); i++)
    {
        pcl::PointXY pt;
        pt.x = rotated_cloud->points[i].x;
        pt.y = rotated_cloud->points[i].z;
        cloud_xz->points.push_back(pt);
    }
    tree.setInputCloud(cloud_xz);

    for (size_t i = 0; i < voxel_side_num_; i++)
    {
        for (size_t j = 0; j < voxel_side_num_; j++)
        {
            pcl::PointXY pt;
            float derta;
            derta = unit_side_length_ / 2;
            pt.x = (i + 0.5)*unit_side_length_ - extract_radius_;
            pt.y = (j + 0.5)*unit_side_length_ - extract_radius_;
            tree.radiusSearch(pt, 1.5*unit_side_length_, search_indices, distances);
            if (distances.empty() != true)
            {
                for (size_t n = 0; n < search_indices.size(); n++)
                {
                    grid[i + j*voxel_side_num_ + voxel_side_num_*voxel_side_num_].point_num += exp(-distances[n] / (2 * derta * derta));
                    float depth;
                    depth = rotated_cloud->points[search_indices[n]].y + extract_radius_;
                    grid[i + j*voxel_side_num_ + voxel_side_num_*voxel_side_num_].average_depth += depth*exp(-distances[n] / (2 * derta * derta));
                }
            }
        }
    }


    //遍历所有点往YOZ面投影,计算落入每个格子中的点个数;
    pcl::PointCloud<pcl::PointXY>::Ptr cloud_yz(new  pcl::PointCloud<pcl::PointXY>);
    for (size_t i = 0; i < rotated_cloud->size(); i++)
    {
        pcl::PointXY pt;
        pt.x = rotated_cloud->points[i].y;
        pt.y = rotated_cloud->points[i].z;
        cloud_yz->points.push_back(pt);
    }
    tree.setInputCloud(cloud_yz);

    for (size_t i = 0; i < voxel_side_num_; i++)
    {
        for (size_t j = 0; j < voxel_side_num_; j++)
        {
            pcl::PointXY pt;
            float derta;
            derta = unit_side_length_ / 2;
            pt.x = (i + 0.5)*unit_side_length_ - extract_radius_;
            pt.y = (j + 0.5)*unit_side_length_ - extract_radius_;
            tree.radiusSearch(pt, 1.5*unit_side_length_, search_indices, distances);
            if (distances.empty() != true)
            {
                for (size_t n = 0; n < search_indices.size(); n++)
                {
                    grid[i + j*voxel_side_num_ + 2 * voxel_side_num_*voxel_side_num_].point_num += exp(-distances[n] / (2 * derta * derta));
                    float depth;
                    depth = rotated_cloud->points[search_indices[n]].x + extract_radius_;
                    grid[i + j*voxel_side_num_ + 2 * voxel_side_num_*voxel_side_num_].average_depth += depth*exp(-distances[n] / (2 * derta * derta));
                }
            }
        }
    }


    /*未考虑边缘效应的情况;*/
    /*for (size_t i = 0; i < rotated_cloud->size(); i++)
    {
        x = getVoxelNum(rotated_cloud->points[i].x);
        y = getVoxelNum(rotated_cloud->points[i].y);
        index = x + y*voxel_side_num_ ;
        grid[index].point_num++;
    }
    for (size_t i = 0; i < rotated_cloud->size(); i++)
    {
        x = getVoxelNum(rotated_cloud->points[i].x);
        z = getVoxelNum(rotated_cloud->points[i].z);
        index = x + z*voxel_side_num_ + voxel_side_num_*voxel_side_num_;
        grid[index].point_num++;
    }
    for (size_t i = 0; i < rotated_cloud->size(); i++)
    {
        y = getVoxelNum(rotated_cloud->points[i].y);
        z = getVoxelNum(rotated_cloud->points[i].z);
        index = y + z*voxel_side_num_ + 2 * voxel_side_num_*voxel_side_num_;
        grid[index].point_num++;
    }*/

    //遍历所有格子,计算每个格子的归一化权值(Ng/Vg)/(Nn/Vn);
    //Ng为每个格子中的点个数,Vg为每个格子的面积;Nn为局部邻域内的所有点,Vn为局部邻域的面积;
    float grid_density, grid_area, neighbourhood_density, neighbourhood_area;
    //计算局部邻域内的点密度;
    neighbourhood_area = M_PI*extract_radius_*extract_radius_;
    neighbourhood_density = rotated_cloud->size() / neighbourhood_area;

    for (size_t i = 0; i < grid.size(); i++)
    {
        //计算每个格子点的平均深度;
        if (grid[i].point_num == 0.0)
        {
            grid[i].average_depth = 0.0f;
        }
        else
        {
            grid[i].average_depth /= grid[i].point_num;
        }

        //计算每个格网的点密度;
        grid_area = unit_side_length_*unit_side_length_;
        grid_density = grid[i].point_num / grid_area;
        //计算每个格网的归一化权值;
        if (neighbourhood_density != 0.0f)
        {
            grid[i].normalized_point_weight = grid_density / neighbourhood_density;
        }
        else
        {
            grid[i].normalized_point_weight = 0.0f;
        }
    }
}

bool BNF::BSCDescriptor::contain2DPair(int pair1, int pair2)
{
    if (grid_index_pairs_2d_.empty() == true)
    {
        return false;
    }
    else
    {
        for (int i = 0; i < grid_index_pairs_2d_.size(); i++)
        {
            if (((pair1 == grid_index_pairs_2d_[i].first) && (pair2 == grid_index_pairs_2d_[i].second))
                || ((pair1 == grid_index_pairs_2d_[i].second) && (pair2 == grid_index_pairs_2d_[i].first)))
            {
                return true;
            }
        }
        return false;
    }

}

void BNF::BSCDescriptor::randomSamplePointPairs() {
    //使用128对随机格网对
    for (int i = 0; i < voxel_side_num_*voxel_side_num_ * 2; i++)
    {
        int pair1 = rand() % voxel_side_num_*voxel_side_num_;
        int pair2 = rand() % voxel_side_num_*voxel_side_num_;
        while (pair1 == pair2 || contain2DPair(pair1, pair2))
        {
            pair1 = rand() % voxel_side_num_*voxel_side_num_;
            pair2 = rand() % voxel_side_num_*voxel_side_num_;
        }
        grid_index_pairs_2d_.push_back(std::pair<int, int>(pair1, pair2));
    }
}

std::vector<char> BNF::BSCDescriptor::computeFeatureProjectedGridAndCompareFeature(
        const std::vector<GridVoxel> &grid) {
    float normalized_point_weightT(0.1);
    std::vector<char> result_feature;
    result_feature.resize(gridFeatureDimension_ + compairFeatureDimension_);

    //计算每个格网是否为空,得到二进制特征;
    for (int i = 0; i<grid.size(); i++)
    {
        if (grid[i].normalized_point_weight > normalized_point_weightT)
        {
            int bit_num = i % 8;
            int byte_num = i / 8;
            char test_num = 1 << bit_num;
            result_feature[byte_num] |= test_num;
        }
    }

    /*计算平面格网点密度两两比较的特征;*/
    //遍历所有的点对,计算他们之间点密度变化的均值和标准差;
    double average(0.0), variance(0.0), standardDeviation(0.0), x(0.0);
    for (int i = 0; i < grid_index_pairs_2d_.size(); i++)
    {
        x = grid[grid_index_pairs_2d_[i].first].normalized_point_weight - grid[grid_index_pairs_2d_[i].second].normalized_point_weight;
        average += x;
    }
    average /= grid_index_pairs_2d_.size();

    for (int i = 0; i < grid_index_pairs_2d_.size(); i++)
    {
        x = grid[grid_index_pairs_2d_[i].first].normalized_point_weight - grid[grid_index_pairs_2d_[i].second].normalized_point_weight;
        variance += (x - average)*(x - average);
    }
    variance /= grid_index_pairs_2d_.size();
    standardDeviation = sqrt(variance);

    for (int i = 0; i<grid_index_pairs_2d_.size(); i++)
    {
        //如果比较的两个格子为空,则取值为0;如果一个格子为空,另一个为非空,则取值为1;
        if ((grid[grid_index_pairs_2d_[i].first].normalized_point_weight > normalized_point_weightT
             &&grid[grid_index_pairs_2d_[i].second].normalized_point_weight < normalized_point_weightT)
            || (grid[grid_index_pairs_2d_[i].first].normalized_point_weight < normalized_point_weightT
                &&grid[grid_index_pairs_2d_[i].second].normalized_point_weight > normalized_point_weightT))
        {
            x = grid[grid_index_pairs_2d_[i].first].normalized_point_weight - grid[grid_index_pairs_2d_[i].second].normalized_point_weight;
            int k = i + (int)grid.size();
            int bit_num = k % 8;
            int byte_num = k / 8;
            char test_num = 1 << bit_num;
            result_feature[byte_num] |= test_num;
        }

        // 如果两个格子都为非空, 则进行比较;
        if (grid[grid_index_pairs_2d_[i].first].normalized_point_weight > normalized_point_weightT
            &&grid[grid_index_pairs_2d_[i].second].normalized_point_weight > normalized_point_weightT)
        {
            x = grid[grid_index_pairs_2d_[i].first].normalized_point_weight - grid[grid_index_pairs_2d_[i].second].normalized_point_weight;
            if (abs(x - average) > standardDeviation)
            {
                int k = i + (int)grid.size();
                int bit_num = k % 8;
                int byte_num = k / 8;
                char test_num = 1 << bit_num;
                result_feature[byte_num] |= test_num;
            }
        }

    }
    return result_feature;
}

std::vector<char> BNF::BSCDescriptor::computeFeatureProjectedGridAndCompareFeature2D(
        const std::vector<GridVoxel> &grid) {
    float normalized_point_weightT(0.1);
    std::vector<char> result_feature(gridFeatureDimension_ + compairFeatureDimension_);
    int featureDimenshions(0);

    //计算每个格网是否为空,得到二进制特征;
    for (int i = 0; i<grid.size(); i++)
    {
        if (grid[i].normalized_point_weight > normalized_point_weightT)
        {
            int bit_num = i % 8;
            int byte_num = i / 8;
            char test_num = 1 << bit_num;
            result_feature[byte_num] |= test_num;
        }
        featureDimenshions++;
    }



    //XOY,XOZ,YOZ 面特征计算,遍历所有的点对,计算他们之间深度变化的均值和标准差;
    double average_depth(0.0), variance_depth(0.0);
    double standardDeviation_depth(0.0), depth(0.0);
    double average_density(0.0), variance_density(0.0);
    double standardDeviation_density(0.0), density(0.0);
    int offset(0);

    for (int nn = 0; nn < 3; nn++)
    {
        average_depth = 0.0; variance_depth = 0.0; standardDeviation_depth = 0.0; depth = 0.0;
        average_density = 0.0; variance_density = 0.0; standardDeviation_density = 0.0; density = 0.0;


        for (int i = 0; i < grid_index_pairs_2d_.size(); i++)
        {
            depth = grid[grid_index_pairs_2d_[i].first + offset].average_depth - grid[grid_index_pairs_2d_[i].second + offset].average_depth;
            average_depth += depth;

            density = grid[grid_index_pairs_2d_[i].first + offset].normalized_point_weight - grid[grid_index_pairs_2d_[i].second + offset].normalized_point_weight;
            average_density += density;

        }
        average_depth /= grid_index_pairs_2d_.size();
        average_density /= grid_index_pairs_2d_.size();


        for (int i = 0; i < grid_index_pairs_2d_.size(); i++)
        {
            depth = grid[grid_index_pairs_2d_[i].first + offset].average_depth - grid[grid_index_pairs_2d_[i].second + offset].average_depth;
            variance_depth += (depth - average_depth)*(depth - average_depth);

            density = grid[grid_index_pairs_2d_[i].first + offset].normalized_point_weight - grid[grid_index_pairs_2d_[i].second + offset].normalized_point_weight;
            variance_density += (density - average_density)*(density - average_density);

        }
        variance_depth /= grid_index_pairs_2d_.size();
        standardDeviation_depth = sqrt(variance_depth);

        variance_density /= grid_index_pairs_2d_.size();
        standardDeviation_density = sqrt(variance_density);



        for (int i = 0; i<grid_index_pairs_2d_.size(); i++)
        {
            // 进行两个格子的平均深度差;
            depth = grid[grid_index_pairs_2d_[i].first + offset].average_depth - grid[grid_index_pairs_2d_[i].second + offset].average_depth;
            if (abs(depth - average_depth) > standardDeviation_depth)
            {
                int k = featureDimenshions;
                int bit_num = k % 8;
                int byte_num = k / 8;
                char test_num = 1 << bit_num;
                result_feature[byte_num] |= test_num;
                //cout << 1 << endl;
            }
            featureDimenshions++;

            // 进行两个格子的平均密度差;
            if (grid[grid_index_pairs_2d_[i].first].normalized_point_weight < normalized_point_weightT
                &&grid[grid_index_pairs_2d_[i].second].normalized_point_weight < normalized_point_weightT)
            {
                //如果两个格子都为空,则该位置的特征为0;
            }
            else
            {
                density = grid[grid_index_pairs_2d_[i].first + offset].normalized_point_weight - grid[grid_index_pairs_2d_[i].second + offset].normalized_point_weight;
                if (abs(density - average_density) > standardDeviation_density)
                {
                    int k = featureDimenshions;
                    int bit_num = k % 8;
                    int byte_num = k / 8;
                    char test_num = 1 << bit_num;
                    result_feature[byte_num] |= test_num;
                }
            }
            featureDimenshions++;
        }

        offset += (voxel_side_num_*voxel_side_num_);
    }
    return result_feature;
}

bool BNF::BSCDescriptor::extractBinaryFeatureOfKeypoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                                                        int ptIndex, const std::vector<int> &searchIndexes,
                                                        std::vector<std::vector<char>> &features) {
    std::vector<char> feature;

    if (searchIndexes.empty())
    {
        LOG(INFO) << "no points";
        features.push_back(feature);
        features.push_back(feature);
        return false;
    }

    //存储格网结构
    std::vector<GridVoxel> grid(gridFeatureDimension_);

    CoordinateSystem localSystem1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZ>);


    computeLocalCoordinateSystem(input_cloud, ptIndex, searchIndexes, localSystem1);
    transformPointCloudToLocalSystem(input_cloud, ptIndex, searchIndexes, localSystem1, result_cloud);

    //构建格网
    constructCubicGrid(result_cloud, grid);

    //从格网生成特征
    feature = computeFeatureProjectedGridAndCompareFeature2D(grid);
//    feature.localSystem_.xAxis = localSystem1.xAxis;//给局部邻域的2D主方向赋值;
//    feature.localSystem_.yAxis = localSystem1.yAxis;//给局部邻域的2D主方向赋值;
//    feature.localSystem_.zAxis = localSystem1.zAxis;//给局部邻域的2D主方向赋值;
//    feature.localSystem_.origin = localSystem1.origin;//给局部邻域的2D主方向赋值;
    features.push_back(feature);

    //计算完成清理grid和result_cloud 进行下一次计算
    std::vector<GridVoxel>().swap(grid);
//    grid.swap(std::vector<GridVoxel>(gridFeatureDimension_));
    result_cloud->points.clear();

    /*Z轴不变,X轴和Y轴反向;*/
    CoordinateSystem localSystem2;
    localSystem2.xAxis = -localSystem1.xAxis;
    localSystem2.yAxis = -localSystem1.yAxis;
    localSystem2.zAxis = localSystem1.zAxis;
    localSystem2.origin = localSystem1.origin;
    transformPointCloudToLocalSystem(input_cloud, ptIndex, searchIndexes, localSystem2, result_cloud);
    //构建格网
    constructCubicGrid(result_cloud, grid);

    //从格网生成特征
    feature = computeFeatureProjectedGridAndCompareFeature2D(grid);
//    feature.localSystem_.xAxis = localSystem2.xAxis;//给局部邻域的2D主方向赋值;
//    feature.localSystem_.yAxis = localSystem2.yAxis;//给局部邻域的2D主方向赋值;
//    feature.localSystem_.zAxis = localSystem2.zAxis;//给局部邻域的2D主方向赋值;
//    feature.localSystem_.origin = localSystem2.origin;//给局部邻域的2D主方向赋值;
    features.push_back(feature);

    //计算完成清理grid和result_cloud 进行下一次计算
    std::vector<GridVoxel>().swap(grid);
//    grid.swap(std::vector<GridVoxel>(gridFeatureDimension_));
    result_cloud->points.clear();

    return true;
}