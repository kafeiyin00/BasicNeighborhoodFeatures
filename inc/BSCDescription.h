//
// Created by Jianping on 2019/10/16.
//

#ifndef BASICNEIGHBORHOODFEATURES_BSCDESCRIPTION_H
#define BASICNEIGHBORHOODFEATURES_BSCDESCRIPTION_H

#include <vector>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace BNF{

class BSCDescriptor{
public:
    struct CoordinateSystem
    {
        Eigen::Vector3f  xAxis;
        Eigen::Vector3f  yAxis;
        Eigen::Vector3f  zAxis;
        Eigen::Vector3f  origin;
    };

    struct GridVoxel
    {
        double point_num;
        float point_weight;
        float intensity;
        float density;
        float normalized_point_weight;
        float average_depth;
        GridVoxel() :point_num(0.0), point_weight(0.0f), intensity(0.0f), density(0.0f),
                     normalized_point_weight(0.0f), average_depth(0.0f)
        {

        }
    };

public:
    BSCDescriptor(int size):
    _size(size),_bytes(size/8)
    {
        _feature.resize(_bytes);
        memset(_feature.data(),0,sizeof(char)*_bytes);
    }

    BSCDescriptor(const std::vector<char>& feature){
        _size = feature.size()*8;
        _bytes = feature.size();
        _feature.resize(_bytes);
        memcpy(_feature.data(),feature.data(), sizeof(char)*_bytes);
    }

    int hammingDistance(const BSCDescriptor& bsdf1, const BSCDescriptor& bsdf2);

    std::vector<char> getFeature();



private:
    unsigned char byteBitsLookUp(unsigned char b);

    /// ---bsc core

    bool computeEigenVectorsByWeightPCA(const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud,const std::vector<int> & search_indices,int test_index,
                                        Eigen::Vector3f  &principalDirection,
                                        Eigen::Vector3f  &middleDirection,
                                        Eigen::Vector3f  &normalDirection);

    void computeTranformationMatrixBetweenCoordinateSystems(const CoordinateSystem & coordinate_src,
                                                            const CoordinateSystem & coordinate_traget,
                                                            Eigen::Matrix4f  & tragetToSource);

    void computeEigenVectorsBy2Dpca(const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud,//输入点云
                                    const std::vector<int> & search_indices, //关键点的邻域点索引号
                                    int test_index,
                                    Eigen::Vector3f  &principalDirection);
    /*建立每个关键点的局部坐标系（主方向为Z轴正向,法方向为X轴正向）,并将变换到该局部坐标系下的点压入result_cloud中;*/
    bool computeLocalCoordinateSystem(const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud,//输入点云
                                 int test_index,    //关键点的索引号
                                 const std::vector<int> & search_indices, //关键点的邻域点索引号
                                 CoordinateSystem &localCoordinateSystem);

    void transformPointCloudToLocalSystem(const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud,//输入点云
                                     int test_index,    //关键点的索引号
                                     const std::vector<int> & search_indices, //关键点的邻域点索引号
                                     const CoordinateSystem &localCoordinateSystem,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr & result_cloud);//输出的已经获得旋转不变性的点云

    /*按照点密度计算的点大小来生成立体格网*/
    void constructCubicGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr & rotated_cloud, std::vector<GridVoxel> & grid);//结果格网

    //对2维平面格网进行随机采样;
    void randomSamplePointPairs();

    bool contain2DPair(int pair1, int pair2);

    //根据立体格网计算三个投影面的投影特征和两两比较的特征;//useless
    std::vector<char> computeFeatureProjectedGridAndCompareFeature(const std::vector<GridVoxel> & grid);

    //根据立体格网计算三个投影面的投影特征和两两比较的特征(每个投影面分别比较);
    std::vector<char> computeFeatureProjectedGridAndCompareFeature2D(const std::vector<GridVoxel> & grid);

    bool extractBinaryFeatureOfKeypoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud,
            int ptIndex,
            const std::vector<int> &searchIndexes,
            std::vector<std::vector<char>> & features);

private:
    int _size;
    int _bytes;
    std::vector<char> _feature;

    /// --- bsc
    float extract_radius_;				//
    unsigned int voxel_side_num_;		//
    float unit_side_length_;			//
    std::vector<float> side_length_thresh_;	//
    std::vector<std::pair<int, int>> grid_index_pairs_2d_;

    int gridFeatureDimension_;
    int compairFeatureDimensionInEachPlane_;//
    int compairFeatureDimension_;


    CoordinateSystem localSystem_;
};

inline std::vector<char> BSCDescriptor::getFeature() {
    return _feature;
}

}

#endif //BASICNEIGHBORHOODFEATURES_BSCDESCRIPTION_H
