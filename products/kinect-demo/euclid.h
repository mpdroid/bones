#ifndef __EUCLID_H
#define __EUCLID_H
#include <iostream>
#include <vector>
#include <map>
using namespace std;
#include <k4abt.h>
#include <k4a/k4a.h>

#include "linmath.h"
using namespace linmath;

#include "clusterizer.h"
using namespace clusterizer;

#include "widget.h"

#define PRINT_VECTOR(name, v) \
    cout << name << " " << v[0] << "," << v[1] << "," << v[2] << endl;

const auto epsilon = std::numeric_limits<float>::epsilon();
const auto SMOOTHING_FACTOR = 5.f;
const auto NUM_DATA_POINTS = 20.f;
const auto MULTIPLIER = SMOOTHING_FACTOR / ( 1 + NUM_DATA_POINTS);

struct TransformationMatrix {
    mat4x4 model;
};

struct JointCoordinates {
    k4a_float3_t position;
    k4a_quaternion_t orientation;
    bool is_set = false;
};

struct Ray {
    Eigen::Vector3f origin;
    Eigen::Vector3f direction;
};

struct Detection {
    Ray ray;
    BoundingCube cube;
    std::string name;
    int frame_number;
};

struct BoundingPlane {
    Eigen::Matrix<float, 4,3> corners;
    Eigen::Vector3f normal;
    Eigen::Vector3f center;
};

class BodyGeometry
{
public:
    BodyGeometry(
        k4a_calibration_t *sensor_calibration,
        const float window_origin,
        const ImVec2 window_size,
        int color_image_width,
        int color_image_height,
        vector<JointCoordinates> *moving_average,
        k4abt_joint_id_t pointer_joint_id,
        vector<k4abt_body_t> bodies,
        vector<k4abt_joint_id_t> joint_ids);
    ~BodyGeometry();
    k4a_float3_t joint_to_global(uint32_t body_id, k4abt_joint_id_t joint_id, k4a_float3_t target);
    ImVec2 joint_target_to_window(uint32_t body_id, k4abt_joint_id_t joint_id, k4a_float3_t target);
    ImVec2 joint_to_window(uint32_t body_id, k4abt_joint_id_t joint_id);
    Ray joint_to_ray(uint32_t body_id, k4abt_joint_id_t joint_id, k4a_float3_t target);
    Ray joints_to_ray(uint32_t body_id, k4abt_joint_id_t from_joint_id, k4abt_joint_id_t to_joint_id, float delta);
    Ray joints_to_ray(uint32_t body_id, k4abt_joint_id_t from_joint_id, k4abt_joint_id_t to_joint_id, k4abt_joint_id_t tip_joint_id);
    k4abt_joint_t get_joint(uint32_t body_id, k4abt_joint_id_t joint_id);
    bool is_thumb_pointing_upwards(uint32_t body_id);
    JointInfo get_joint_information(uint32_t body_id, k4abt_joint_id_t joint_id, int *valid);
    Eigen::Vector3f ray_to_joint_ortho(Ray ray, uint32_t body_id, k4abt_joint_id_t joint_id);
    k4a_float2_t window_to_point2d(ImVec2 window_point);
    k4a_float3_t window_to_point(ImVec2 window_point);
    ImVec2 point_to_window(k4a_float3_t target);
    ImVec2 vector_to_window(Eigen::Vector3f point);
    ImVec2 vector2f_to_window(Eigen::Vector2f target);
    k4a_float2_t point_to_color(k4a_float3_t point, int *valid);
    int get_body_count();
    int model_count();
    bool is_point_in_forward_space(k4a_float3_t point, k4abt_joint_id_t joint_id, bool is_x_positive) ;
    bool is_point_in_forward_space(k4a_float3_t point);
    bool is_point_in_forward_space(k4a_float3_t point, vector<Ray> rays);
    k4a_calibration_t *get_sensor_calibration();
    int get_color_image_width();
    int get_color_image_height();

private:
    k4a_calibration_t *sensor_calibration;
    float window_origin;
    ImVec2 window_size;
    ImVec2 actual_image_size;
    int color_image_width;
    int color_image_height;
    int rendered_height;
    vector<k4abt_body_t> bodies;
    map<uint32_t, int> body_id_map;
    map<k4abt_joint_id_t, int> joint_id_map;
    vector<vector<TransformationMatrix>> joint_to_world;
    vector<vector<TransformationMatrix>> world_to_joint;
    void update_moving_average(JointCoordinates *moving_average, k4abt_joint_t joint);
    ImVec2 fittedVector(int x, int y);
};


bool find_intersecting_cube(Ray ray, vector<BoundingCube> *pointCubes, int *pointee);
k4a_float3_t vector_to_point(Eigen::Vector3f aVector);
void point_to_color2d(k4a_float3_t *point, k4a_calibration_t *sensor_calibration, int w, int h, k4a_float2_t *point_2d);
void vector_to_color2d(Vector3f vec3, k4a_calibration_t *sensor_calibration, int w, int h, k4a_float2_t *point_2d);
bool compareBodies(k4abt_body_t first, k4abt_body_t second);

#endif