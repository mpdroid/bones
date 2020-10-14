/*
* Segments depth image point cloud using cilantro library
*/
#ifndef CLUSTERIZER_H
#define CLUSTERIZER_H

#include <iostream>
#include <k4a/k4a.h>
#include <vector>
#include <cilantro/clustering/connected_component_extraction.hpp>
#include <cilantro/utilities/point_cloud.hpp>
using namespace cilantro;
using namespace std;

namespace clusterizer {
    
struct BoundingCube
{
    Eigen::Vector3f u;
    Eigen::Vector3f v;
    Eigen::Vector3f ftl;
    Eigen::Vector3f ftr;
    Eigen::Vector3f fbr;
    Eigen::Vector3f fbl;
    Eigen::Vector3f btl;
    Eigen::Vector3f btr;
    Eigen::Vector3f bbr;
    Eigen::Vector3f bbl;
    k4a_float3_t u_k4;
    k4a_float3_t v_k4;
    k4a_float2_t u2d_k4;
    k4a_float2_t v2d_k4;
    
    vector<float> color;
    Eigen::MatrixXf points;
    Eigen::MatrixXi pointColors;
    vector<k4a_float3_t> pointer;
    size_t label;
};


class Clusterizer
{
public:
    Clusterizer(cilantro::VectorSet3f *cilantroPoints, cilantro::VectorSet3f *cilantroColors);
    ~Clusterizer();
    vector<BoundingCube> Clusterize(cilantro::PointCloud3f* cloud_seg);
    cilantro::PointCloud3f cloud;
private:
    void write_point_cloud(const char *file_name, cilantro::PointCloud3f pc);

};
}

#endif