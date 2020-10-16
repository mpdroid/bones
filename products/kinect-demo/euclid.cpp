#include "euclid.h"

k4a_float3_t vector_to_point(Eigen::Vector3f aVector)
{
    return {aVector[0], aVector[1], aVector[2]};
}

bool is_point_inside_bounded_plane(Eigen::Vector3f intercept, BoundingPlane plane)
{
    Eigen::Vector3f maxVal = plane.corners.colwise().maxCoeff();
    Eigen::Vector3f minVal = plane.corners.colwise().minCoeff();

    return abs(maxVal(0) - intercept[0]) <= abs(maxVal(0) - minVal(0)) + 0.00001f &&
           abs(maxVal(1) - intercept[1]) <= abs(maxVal(1) - minVal(1)) + 0.00001f &&
           abs(maxVal(2) - intercept[2]) <= abs(maxVal(2) - minVal(2)) + 0.00001f;
}

bool does_ray_intersect_plane(Ray ray, BoundingPlane plane, k4a_float3_t *intercept, float *distanceToPlane)
{
    bool doesIntercept = false;
    float denominator = plane.normal.dot(ray.direction);
    if (abs(denominator) > epsilon)
    {
        Eigen::Vector3f difference = plane.center - ray.origin;
        float t = difference.dot(plane.normal) / denominator;
        if (t >= 0.0001f)
        {
            Eigen::Vector3f intercept3f = ray.origin + ray.direction * t;
            if (is_point_inside_bounded_plane(intercept3f, plane))
            {
                doesIntercept = true;
                *distanceToPlane = (intercept3f - ray.origin).norm();
                k4a_float3_t k4_intercept = {intercept3f[0], intercept3f[1], intercept3f[2]};
                *intercept = k4_intercept;
            }
        }
        else
        {
            doesIntercept = false;
        }
    }
    else
    {
        doesIntercept = false;
    }
    return doesIntercept;
}

// move to Geometry class
Eigen::Vector3f compute_normal(Eigen::Matrix<float, 4, 3> corners)
{
    Eigen::Vector3f first = corners.row(1) - corners.row(0);
    Eigen::Vector3f second = corners.row(2) - corners.row(0);
    auto normal = first.cross(second);
    auto l2norm = normal.norm();
    normal = normal / l2norm;
    // printf(" normal %f, %f, %f\n", normal[0], normal[1], normal[2] );
    return normal;
}

bool does_ray_intersect_cube(Ray ray, BoundingCube cube, k4a_float3_t *intercept, float *distanceToPointee)
{
    vector<BoundingPlane> boundingPlanes;
    bool doesIntersect = false;
    float distanceToNearest = std::numeric_limits<float>::infinity();
    BoundingPlane nearestPlane;
    ImVec2 nearestIntersect;
    k4a_float3_t k4_intercept;
    bool interceptflag = false;
    BoundingPlane plane1;
    plane1.corners.row(0) << cube.u[0], cube.u[1], cube.u[2];
    plane1.corners.row(1) << cube.u[0], cube.u[1], cube.v[2];
    plane1.corners.row(2) << cube.u[0], cube.v[1], cube.u[2];
    plane1.corners.row(3) << cube.u[0], cube.v[1], cube.v[2];
    plane1.normal = compute_normal(plane1.corners);

    plane1.center << cube.u[0], (cube.u[1] + cube.v[1]) / 2.f, (cube.u[2] + cube.v[2]) / 2.f;
    boundingPlanes.push_back(plane1);

    BoundingPlane plane2;
    plane2.corners.row(0) << cube.v[0], cube.u[1], cube.u[2];
    plane2.corners.row(1) << cube.v[0], cube.u[1], cube.v[2];
    plane2.corners.row(2) << cube.v[0], cube.v[1], cube.u[2];
    plane2.corners.row(3) << cube.v[0], cube.v[1], cube.v[2];
    plane2.normal = compute_normal(plane2.corners);

    plane2.center << cube.v[0], (cube.u[1] + cube.v[1]) / 2.f, (cube.u[2] + cube.v[2]) / 2.f;
    boundingPlanes.push_back(plane2);

    BoundingPlane plane3;
    plane3.corners.row(0) << cube.u[0], cube.u[1], cube.u[2];
    plane3.corners.row(1) << cube.u[0], cube.u[1], cube.v[2];
    plane3.corners.row(2) << cube.v[0], cube.u[1], cube.u[2];
    plane3.corners.row(3) << cube.v[0], cube.u[1], cube.v[2];
    plane3.normal = compute_normal(plane3.corners);

    plane3.center << (cube.u[0] + cube.v[0]) / 2.f, cube.u[1], (cube.u[2] + cube.v[2]) / 2.f;
    boundingPlanes.push_back(plane3);

    BoundingPlane plane4;
    plane4.corners.row(0) << cube.u[0], cube.v[1], cube.u[2];
    plane4.corners.row(1) << cube.u[0], cube.v[1], cube.v[2];
    plane4.corners.row(2) << cube.v[0], cube.v[1], cube.u[2];
    plane4.corners.row(3) << cube.v[0], cube.v[1], cube.v[2];
    plane4.normal = compute_normal(plane4.corners);

    plane4.center << (cube.u[0] + cube.v[0]) / 2.f, cube.v[1], (cube.u[2] + cube.v[2]) / 2.f;
    boundingPlanes.push_back(plane4);

    BoundingPlane plane5;
    plane5.corners.row(0) << cube.u[0], cube.u[1], cube.u[2];
    plane5.corners.row(1) << cube.u[0], cube.v[1], cube.u[2];
    plane5.corners.row(2) << cube.v[0], cube.u[1], cube.u[2];
    plane5.corners.row(3) << cube.v[0], cube.v[1], cube.u[2];
    plane5.normal = compute_normal(plane5.corners);

    plane5.center << (cube.u[0] + cube.v[0]) / 2.f, (cube.u[1] + cube.v[1]) / 2.f, cube.u[2];
    boundingPlanes.push_back(plane5);

    BoundingPlane plane6;
    plane6.corners.row(0) << cube.u[0], cube.u[1], cube.v[2];
    plane6.corners.row(1) << cube.u[0], cube.v[1], cube.v[2];
    plane6.corners.row(2) << cube.v[0], cube.u[1], cube.v[2];
    plane6.corners.row(3) << cube.v[0], cube.v[1], cube.v[2];
    plane6.normal = compute_normal(plane6.corners);

    plane6.center << (cube.u[0] + cube.v[0]) / 2.f, (cube.u[1] + cube.v[1]) / 2.f, cube.v[2];
    boundingPlanes.push_back(plane6);

    int interceptCount = 0;
    for (int p = 0; p < boundingPlanes.size(); p++)
    {
        BoundingPlane plane = boundingPlanes[p];
        float distance;
        k4a_float3_t planeInterceptVector;
        bool planeIntersects = does_ray_intersect_plane(ray, plane, &planeInterceptVector, &distance);
        if (planeIntersects == true && (distance < distanceToNearest))
        {
            interceptCount++;
            doesIntersect = true;
            nearestPlane = plane;
            distanceToNearest = distance;
            *intercept = planeInterceptVector;
            interceptflag = true;
        }
    }
    // *distanceToPointee = distanceToNearest;

    if (interceptflag == true)
    {
        doesIntersect = true;
        *distanceToPointee = distanceToNearest;
    }
    else
    {
        doesIntersect = false;
        *distanceToPointee = std::numeric_limits<float>::infinity();
    }
    return doesIntersect;
}

bool find_intersecting_cube(Ray ray, vector<BoundingCube> *pointCubes, int *pointee)
{
    float distanceToNearest = std::numeric_limits<float>::infinity();
    k4a_float3_t intercept;
    bool found = false;
    for (int b = 0; b < (*pointCubes).size(); b++)
    {
        BoundingCube *cube = &((*pointCubes)[b]);
        k4a_float3_t intersectionVector;
        float distance;
        bool isPointee = does_ray_intersect_cube(ray, *cube, &intersectionVector, &distance);
        if (isPointee == true && distance < distanceToNearest)
        {
            *pointee = b;
            // cube->pointer.clear();
            // cube->pointer.push_back({ray.origin[0], ray.origin[1], ray.origin[2]});
            // cube->pointer.push_back(intersectionVector);
            distanceToNearest = distance;
            found = true;
        }
    }

    return found;
}

void point_to_color2d(k4a_float3_t *point, k4a_calibration_t *sensor_calibration, int w, int h, k4a_float2_t *point_2d)
{

    int valid = 0;
    k4a_calibration_3d_to_2d(sensor_calibration,
                             point,
                             K4A_CALIBRATION_TYPE_DEPTH,
                             K4A_CALIBRATION_TYPE_COLOR,
                             point_2d,
                             &valid);

    if (valid == 0)
    {
        *point_2d = {0.f, 0.f};
    }
    else
    {
        if (point_2d->v[0] < 0.f)
        {
            point_2d->v[0] = 0.f;
        }
        if (point_2d->v[0] > (float)w)
        {
            point_2d->v[0] = (float)w;
        }
        if (point_2d->v[1] < 0.f)
        {
            point_2d->v[1] = 0.f;
        }
        if (point_2d->v[1] > (float)h)
        {
            point_2d->v[1] = (float)h;
        }
    }
}

// TODO move to geometry
void vector_to_color2d(Vector3f vec3, k4a_calibration_t *sensor_calibration, int w, int h, k4a_float2_t *point_2d)
{
    k4a_float3_t point = vector_to_point(vec3);
    point_to_color2d(&point, sensor_calibration, w, h, point_2d);
}

ImVec2 computeActualImageSize(const ImVec2 window_size,
                              int color_image_width,
                              int color_image_height)
{
    float iw = (float)color_image_width;
    float ih = (float)color_image_height;
    float aw = window_size.x;
    float ah = window_size.y;

    float vertical_scale = ah / ih;
    float horizontal_scale = aw / iw;
    float scale = min(vertical_scale, horizontal_scale);
    aw = scale * iw;
    ah = scale * ih;

    return ImVec2((int)aw, (int)ah);
}

Euclid::Euclid(k4a_calibration_t *sensor_calibration,
                           const float window_origin,
                           const ImVec2 window_size,
                           int color_image_width,
                           int color_image_height,
                           vector<JointCoordinates> *moving_average,
                           k4abt_joint_id_t pointer_joint_id,
                           vector<k4abt_body_t> bodies,
                           vector<k4abt_joint_id_t> joint_ids)
{
    this->sensor_calibration = sensor_calibration;
    this->window_origin = window_origin;
    this->window_size = window_size;
    this->color_image_height = color_image_height;
    this->color_image_width = color_image_width;
    this->actual_image_size = computeActualImageSize(window_size, color_image_width, color_image_height);

    // this->rendered_height = (int)((float)this->color_image_height * (float)window_size.x / (float)this->color_image_width);
    this->rendered_height = this->actual_image_size.y;
    this->bodies = bodies;
    for (int j = 0; j < joint_ids.size(); j++)
    {
        this->joint_id_map.insert(pair<k4abt_joint_id_t, int>(joint_ids[j], j));
    }
    for (int b = 0; b < bodies.size(); b++)
    {
        k4abt_body_t body = bodies[b];
        this->body_id_map.insert(pair<uint32_t, int>(body.id, b));
        vector<TransformationMatrix> j2w_models, w2j_models;
        for (int j = 0; j < joint_ids.size(); j++)
        {
            k4abt_joint_id_t joint_id = joint_ids[j];
            k4abt_joint_t joint = body.skeleton.joints[joint_id];

            k4a_float3_t jointPosition = joint.position;
            k4a_quaternion_t jointOrientation = joint.orientation;
            if (joint_id == pointer_joint_id && b < moving_average->size())
            {
                if ((*moving_average)[b].is_set == false)
                {
                    (*moving_average)[b].position = joint.position;
                    (*moving_average)[b].orientation = joint.orientation;
                    (*moving_average)[b].is_set = true;
                }
                update_moving_average(&(*moving_average)[b], joint);
                jointPosition = (*moving_average)[b].position;
                jointOrientation = (*moving_average)[b].orientation;
            }

            TransformationMatrix matrix, inverted;
            mat4x4 translation, rotation;
            mat4x4_translate(translation, jointPosition.v[0], jointPosition.v[1], jointPosition.v[2]);
            quaternion_to_mat4x4(rotation,
                                 {jointOrientation.v[0],
                                  jointOrientation.v[1],
                                  jointOrientation.v[2],
                                  jointOrientation.v[3]});
            mat4x4_mul(matrix.model, translation, rotation);
            mat4x4_invert(inverted.model, matrix.model);

            j2w_models.push_back(matrix);
            w2j_models.push_back(inverted);
        }
        this->joint_to_world.push_back(j2w_models);
        this->world_to_joint.push_back(w2j_models);
    }
}

Euclid::~Euclid()
{
    for (vector<TransformationMatrix> v : this->joint_to_world)
    {
        v.clear();
    }
    this->joint_to_world.clear();
    for (vector<TransformationMatrix> v : this->world_to_joint)
    {
        v.clear();
    }
    this->world_to_joint.clear();
}

int Euclid::get_body_count()
{
    return this->bodies.size();
}

void Euclid::update_moving_average(JointCoordinates *moving_average, k4abt_joint_t joint)
{
    moving_average->position.v[0] = moving_average->position.v[0] * (1.f - MULTIPLIER) + joint.position.v[0] * MULTIPLIER;
    moving_average->position.v[1] = moving_average->position.v[1] * (1.f - MULTIPLIER) + joint.position.v[1] * MULTIPLIER;
    moving_average->position.v[2] = moving_average->position.v[2] * (1.f - MULTIPLIER) + joint.position.v[2] * MULTIPLIER;
    moving_average->orientation.v[0] = moving_average->orientation.v[0] * (1.f - MULTIPLIER) + joint.orientation.v[0] * MULTIPLIER;
    moving_average->orientation.v[1] = moving_average->orientation.v[1] * (1.f - MULTIPLIER) + joint.orientation.v[1] * MULTIPLIER;
    moving_average->orientation.v[2] = moving_average->orientation.v[2] * (1.f - MULTIPLIER) + joint.orientation.v[2] * MULTIPLIER;
    moving_average->orientation.v[3] = moving_average->orientation.v[3] * (1.f - MULTIPLIER) + joint.orientation.v[3] * MULTIPLIER;
}

k4a_float3_t Euclid::joint_to_global(uint32_t body_id, k4abt_joint_id_t joint_id, k4a_float3_t target)
{
    int body_num = body_id_map[body_id];
    int joint_num = joint_id_map[joint_id];

    vector<TransformationMatrix> models = this->joint_to_world[body_num];
    vec4 transformee = {target.v[0], target.v[1], target.v[2], 1.0f};
    vec4 transformed;
    mat4x4_mul_vec4(transformed, models[joint_num].model, transformee);
    k4a_float3_t k4a_transformed = {transformed[0], transformed[1], transformed[2]};
    return k4a_transformed;
}

ImVec2 Euclid::joint_target_to_window(uint32_t body_id, k4abt_joint_id_t joint_id, k4a_float3_t target)
{
    int valid = 0;
    int x = (int)window_origin, y = 0;
    Eigen::Vector3f v;
    v << target.v[0], target.v[1], target.v[2];
    float norm = v.norm();
    float k = norm;
    while (k > 0.00001f)
    {
        k4a_float3_t t = {v[0], v[1], v[2]};
        k4a_float3_t transformed = joint_to_global(body_id, joint_id, t);
        k4a_float2_t k4a_point_2d;
        k4a_calibration_3d_to_2d(this->sensor_calibration,
                                 &transformed,
                                 K4A_CALIBRATION_TYPE_DEPTH,
                                 K4A_CALIBRATION_TYPE_COLOR,
                                 &k4a_point_2d,
                                 &valid);
        x = x + (int)((k4a_point_2d.v[0] / (float)this->color_image_width) * this->actual_image_size.x);
        y = y + (int)(k4a_point_2d.v[1] / (float)this->color_image_height * (float)rendered_height);
        if (valid == 1)
            break;
        k = k - norm / 5.f;
        v = v * k;
    }

    return ImVec2(x, y);
}

k4a_float2_t Euclid::point_to_color(k4a_float3_t point, int *valid)
{
    k4a_float2_t pt2d;
    k4a_calibration_3d_to_2d(sensor_calibration,
                             &point,
                             K4A_CALIBRATION_TYPE_DEPTH,
                             K4A_CALIBRATION_TYPE_COLOR,
                             &pt2d,
                             valid);
    return pt2d;
}

ImVec2 Euclid::joint_to_window(uint32_t body_id, k4abt_joint_id_t joint_id)
{
    int body_num = body_id_map[body_id];
    k4abt_body_t body = bodies[body_num];

    k4a_float3_t target = body.skeleton.joints[joint_id].position;
    k4a_float2_t k4a_point_2d;
    int valid = 0;
    k4a_calibration_3d_to_2d(sensor_calibration,
                             &target,
                             K4A_CALIBRATION_TYPE_DEPTH,
                             K4A_CALIBRATION_TYPE_COLOR,
                             &k4a_point_2d,
                             &valid);
    int x = (int)window_origin, y = 0;
    if (valid == 1)
    {
        x = x + (int)((k4a_point_2d.v[0] / (float)this->color_image_width) * this->actual_image_size.x);
        y = y + (int)(k4a_point_2d.v[1] / (float)this->color_image_height * (float)rendered_height);
    }
    return fittedVector(x, y);
}

ImVec2 Euclid::fittedVector(int x, int y)
{
    int x1 = x, y1 = y;
    if (x1 < (int)window_origin)
    {
        x1 = (int)window_origin;
    }
    if (x1 > (int)window_origin + (int)actual_image_size.x)
    {
        x1 = (int)window_origin + (int)actual_image_size.x;
    }
    if (y1 < 0)
    {
        y1 = 0;
    }
    if (y1 > (int)actual_image_size.y)
    {
        y1 = (int)actual_image_size.y;
    }

    return ImVec2(x1, y1);
}

k4abt_joint_t Euclid::get_joint(uint32_t body_id, k4abt_joint_id_t joint_id)
{
    int body_num = this->body_id_map[body_id];
    k4abt_body_t body = this->bodies[body_num];
    return body.skeleton.joints[joint_id];
}

JointInfo Euclid::get_joint_information(uint32_t body_id, k4abt_joint_id_t joint_id, int *valid)
{
    JointInfo jointInfo;
    int body_num = this->body_id_map[body_id];
    k4abt_body_t body = this->bodies[body_num];
    k4abt_joint_t joint = body.skeleton.joints[joint_id];
    k4a_float2_t pt2d = point_to_color(joint.position, valid);
    if (*valid == 1)
    {

        jointInfo.depthCoordinates = {joint.position.v[0], joint.position.v[1], joint.position.v[2]};
        jointInfo.orientation = {joint.orientation.v[0], joint.orientation.v[1], joint.orientation.v[2], joint.orientation.v[3]};
        jointInfo.imageCoordinates = {pt2d.v[0], pt2d.v[0]};
        ImVec2 origin = point_to_window(joint.position);
        jointInfo.x_direction.push_back(origin);
        jointInfo.x_direction.push_back(joint_target_to_window(body_id, joint_id, {100.f, 0.f, 0.f}));
        jointInfo.y_direction.push_back(origin);
        jointInfo.y_direction.push_back(joint_target_to_window(body_id, joint_id, {0.f, 100.f, 0.f}));
        jointInfo.z_direction.push_back(origin);
        jointInfo.z_direction.push_back(joint_target_to_window(body_id, joint_id, {0.f, 0.f, 100.f}));
        jointInfo.textCoordinates = origin;
    }
    return jointInfo;
}

bool Euclid::is_thumb_pointing_upwards(uint32_t body_id)
{

    k4abt_joint_t thumb = this->get_joint(body_id, K4ABT_JOINT_THUMB_RIGHT);
    k4abt_joint_t hand = this->get_joint(body_id, K4ABT_JOINT_HANDTIP_RIGHT);

    return thumb.position.v[1] - hand.position.v[1] < 0.f;
}

k4a_float2_t Euclid::window_to_point2d(ImVec2 window_point)
{
    float x = (window_point.x - this->window_origin) * (float)this->color_image_width / this->actual_image_size.x;
    float y = (window_point.y) * (float)this->color_image_height / this->actual_image_size.y;
    int valid = 0;
    k4a_float2_t pt2d = {x, y};
    return pt2d;
}

k4a_float3_t Euclid::window_to_point(ImVec2 window_point)
{
    k4a_float2_t pt2d = this->window_to_point2d(window_point);
    k4a_float3_t pt3d;
    int valid = 0;
    k4a_calibration_2d_to_3d(
        this->sensor_calibration, &pt2d, 15.f, K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_DEPTH, &pt3d, &valid);
    return pt3d;
}

ImVec2 Euclid::point_to_window(k4a_float3_t target)
{

    k4a_float2_t k4a_point_2d;

    int valid = 0;
    k4a_calibration_3d_to_2d(this->sensor_calibration,
                             &target,
                             K4A_CALIBRATION_TYPE_DEPTH,
                             K4A_CALIBRATION_TYPE_COLOR,
                             &k4a_point_2d,
                             &valid);

    int x = (int)window_origin, y = 0;
    if (valid == 1)
    {
        x = (int)window_origin + (int)((k4a_point_2d.v[0] / (float)this->color_image_width) * actual_image_size.x);
        y = (int)(k4a_point_2d.v[1] / (float)this->color_image_height * (float)rendered_height);
    }
    return fittedVector(x, y);
}

ImVec2 Euclid::vector2f_to_window(Eigen::Vector2f point)
{
    int x = point[0], y = point[1];
    x = x + (int)((point[0] / (float)this->color_image_width) * actual_image_size.x);
    y = y + (int)(point[1] / (float)this->color_image_height * (float)rendered_height);
    return fittedVector(x, y);
}

ImVec2 Euclid::vector_to_window(Eigen::Vector3f point)
{

    k4a_float3_t target = {point[0], point[1], point[2]};
    k4a_float2_t k4a_point_2d;

    int valid = 0;
    k4a_calibration_3d_to_2d(sensor_calibration,
                             &target,
                             K4A_CALIBRATION_TYPE_DEPTH,
                             K4A_CALIBRATION_TYPE_COLOR,
                             &k4a_point_2d,
                             &valid);

    int x = (int)window_origin, y = 0;
    if (valid == 1)
    {
        x = x + (int)((k4a_point_2d.v[0] / (float)this->color_image_width) * actual_image_size.x);
        y = y + (int)(k4a_point_2d.v[1] / (float)this->color_image_height * (float)rendered_height);
    }
    return fittedVector(x, y);
}

Ray Euclid::joint_to_ray(uint32_t body_id, k4abt_joint_id_t joint_id, k4a_float3_t target)
{
    int body_num = this->body_id_map[body_id];
    int joint_num = this->joint_id_map[joint_id];
    k4abt_body_t body = this->bodies[body_num];
    k4abt_joint_t joint = body.skeleton.joints[joint_id];
    k4a_float3_t position = joint.position;
    vector<TransformationMatrix> models = this->joint_to_world[body_num];
    vec4 transformee = {target.v[0], target.v[1], target.v[2], 1.0f};
    vec4 transformed;
    mat4x4_mul_vec4(transformed, models[joint_num].model, transformee);
    Ray ray;
    ray.direction << transformed[0] - position.v[0], transformed[1] - position.v[1], transformed[2] - position.v[2];
    ray.direction = ray.direction / ray.direction.norm();
    ray.origin << position.v[0], position.v[1], position.v[2];
    return ray;
}

Ray Euclid::joints_to_ray(uint32_t body_id, k4abt_joint_id_t from_joint_id, k4abt_joint_id_t to_joint_id, float delta)
{
    int body_num = this->body_id_map[body_id];
    int from_joint_num = this->joint_id_map[from_joint_id];
    int to_joint_num = this->joint_id_map[to_joint_id];
    k4abt_body_t body = this->bodies[body_num];
    k4abt_joint_t from_joint = body.skeleton.joints[from_joint_id];
    k4abt_joint_t to_joint = body.skeleton.joints[to_joint_id];
    k4a_float3_t from_position = from_joint.position;
    k4a_float3_t to_position = to_joint.position;
    Ray ray;
    ray.direction << to_position.v[0] - from_position.v[0], to_position.v[1] - from_position.v[1], to_position.v[2] - from_position.v[2];
    ray.direction = ray.direction / ray.direction.norm();
    ray.origin << to_position.v[0], to_position.v[1], to_position.v[2];
    ray.origin = ray.origin + delta * ray.direction;
    return ray;
}

Ray Euclid::joints_to_ray(uint32_t body_id, k4abt_joint_id_t from_joint_id, k4abt_joint_id_t to_joint_id, k4abt_joint_id_t tip_joint_id)
{
    int body_num = this->body_id_map[body_id];
    int from_joint_num = this->joint_id_map[from_joint_id];
    int to_joint_num = this->joint_id_map[to_joint_id];
    int tip_joint_num = this->joint_id_map[tip_joint_id];
    k4abt_body_t body = this->bodies[body_num];
    k4abt_joint_t from_joint = body.skeleton.joints[from_joint_id];
    k4abt_joint_t to_joint = body.skeleton.joints[to_joint_id];
    k4abt_joint_t tip_joint = body.skeleton.joints[tip_joint_id];
    k4a_float3_t from_position = from_joint.position;
    k4a_float3_t to_position = to_joint.position;
    k4a_float3_t tip_position = tip_joint.position;
    Ray ray;
    ray.direction << to_position.v[0] - from_position.v[0], to_position.v[1] - from_position.v[1], to_position.v[2] - from_position.v[2];
    ray.direction = ray.direction / ray.direction.norm();
    ray.origin << tip_position.v[0], tip_position.v[1], tip_position.v[2];
    ray.origin = ray.origin;
    return ray;
}

Eigen::Vector3f Euclid::ray_to_joint_ortho(Ray ray, uint32_t body_id, k4abt_joint_id_t joint_id)
{
    int body_num = this->body_id_map[body_id];
    k4abt_body_t body = this->bodies[body_num];
    k4abt_joint_t joint = body.skeleton.joints[joint_id];
    Eigen::Vector3f p, diff;
    p << joint.position.v[0], joint.position.v[1], joint.position.v[2];
    diff = p - ray.origin;

    Eigen::Vector3f ortho = diff - ray.direction * (diff.dot(ray.direction));
    ortho = ortho / ortho.norm();
    return ortho;
}

int Euclid::model_count()
{
    return joint_to_world.size();
}

bool Euclid::is_point_in_forward_space(k4a_float3_t point, vector<Ray> rays)
{

    for (Ray ray : rays)
    {
        Eigen::Vector3f point_direction;
        point_direction << point.v[0] - ray.origin[0], point.v[1] - ray.origin[1], point.v[2] - ray.origin[2];
        float dotp = ray.direction.dot(point_direction);
        float denom = point_direction.norm();
        float dotp_normalized = dotp / denom;
        float angle = acos(dotp_normalized);
        if (dotp > 0.0001f && angle <= M_PI / 4.f)
            return true;
    }
    return false;
}

bool Euclid::is_point_in_forward_space(k4a_float3_t point)
{
    k4abt_joint_id_t from_joint_id = K4ABT_JOINT_ELBOW_RIGHT;
    k4abt_joint_id_t to_joint_id = K4ABT_JOINT_HAND_RIGHT;
    for (int body_num = 0; body_num < bodies.size(); body_num++)
    {
        int from_joint_num = this->joint_id_map[from_joint_id];
        int to_joint_num = this->joint_id_map[to_joint_id];
        k4abt_body_t body = this->bodies[body_num];
        k4abt_joint_t from_joint = body.skeleton.joints[from_joint_id];
        k4abt_joint_t to_joint = body.skeleton.joints[to_joint_id];
        k4a_float3_t from_position = from_joint.position;
        k4a_float3_t to_position = to_joint.position;
        Ray ray;
        ray.direction << to_position.v[0] - from_position.v[0], to_position.v[1] - from_position.v[1], to_position.v[2] - from_position.v[2];
        ray.direction = ray.direction / ray.direction.norm();
        ray.origin << from_position.v[0], from_position.v[1], from_position.v[2];
        Eigen::Vector3f point_direction;
        point_direction << point.v[0] - to_position.v[0], point.v[1] - to_position.v[1], point.v[2] - to_position.v[2];
        float dotp = ray.direction.dot(point_direction);
        float denom = point_direction.norm();
        float dotp_normalized = dotp / denom;
        float angle = acos(dotp_normalized);
        if (dotp > 0.0001f && angle <= M_PI / 4.f)
            return true;
    }
    return false;
}

bool Euclid::is_point_in_forward_space(k4a_float3_t point, k4abt_joint_id_t joint_id, bool is_x_positive)
{
    int joint_num = this->joint_id_map[joint_id];

    float eps = (is_x_positive == true) ? epsilon : -epsilon;
    for (vector<TransformationMatrix> models : this->world_to_joint)
    {
        vec4 pointPrime;
        vec4 pointVector = {
            point.v[0],
            point.v[1],
            point.v[2],
            1.0f};
        mat4x4_mul_vec4(pointPrime, models[joint_num].model, pointVector);

        if (is_x_positive == true && pointPrime[0] > epsilon)
        {
            return true;
        }
        if (is_x_positive == false && pointPrime[0] < -epsilon)
        {
            return true;
        }
    }
    return false;
}

k4a_calibration_t *Euclid::get_sensor_calibration()
{
        return this->sensor_calibration;
}
int Euclid::get_color_image_width()
{
    return this->color_image_width;
}
int Euclid::get_color_image_height()
{
    return this->color_image_height;
}
