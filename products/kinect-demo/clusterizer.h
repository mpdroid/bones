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
#include "common.h"
using namespace cilantro;
using namespace std;

namespace clusterizer {
    
struct BoundingCube
{
    Eigen::Vector3f u; // corner closest to depth camera origin 
    Eigen::Vector3f v; // corner furthest from depth camera origin

    // same as above but in kinect structure
    k4a_float3_t u_k4; 
    k4a_float3_t v_k4;

    k4a_float2_t u2d_k4; // corner closest to color camera origin
    k4a_float2_t v2d_k4; // corner furthest from color camera origin

    Eigen::Vector3f ftl; // front top left of bounding cube
    Eigen::Vector3f ftr; // front top right
    Eigen::Vector3f fbr; // front bottom right
    Eigen::Vector3f fbl; // and so on ...
    Eigen::Vector3f btl;
    Eigen::Vector3f btr;
    Eigen::Vector3f bbr;
    Eigen::Vector3f bbl;
    
    vector<float> color;         // segment color 
    Eigen::MatrixXf points;      // coordinates of points inside the cube
    Eigen::MatrixXi pointColors; // colors of points inside the cube
    // vector<k4a_float3_t> pointer;
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