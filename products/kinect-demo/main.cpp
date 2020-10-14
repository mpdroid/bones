/*
* Demonstrates kinect features.
* Pulled together from various public repos:
* ....
* TODO
* - list repos
* - stick to common method and variable naming convention
* Actors
*   - main
*   - viewer
*   - controller
        - abstract
        - starWars
        - writing
        - objectDetection
*   - kinect
*   - 
*/


#include <iostream>
#include <vector>
#include <future>
#include <thread>
#include <algorithm>
#include <string>
#include <algorithm>
#include <cctype>
#include <stdio.h>
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

using namespace std;

#include "k4adepthpixelcolorizer.h"
#include "k4apixel.h"
#include "k4astaticimageproperties.h"
#include "texture.h"
#include "viewerwindow.h"
#include "azure_vision_request.h"

#include "linmath.h"
using namespace linmath;
#include "body_geometry.h"
#include <k4abt.h>


#define VERIFY(result, error)                                                                            \
    if (result != K4A_RESULT_SUCCEEDED)                                                                  \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    }

#define DEMO_MODE_OBJECT_DETECTION 'O'
#define DEMO_MODE_XRAY_VISION 'X'
#define DEMO_MODE_LIGHT_SABERS 'L'
#define DEMO_MODE_JOINT_INFO 'J'
#define DEMO_MODE_WRITING 'W'

// TODO move global defines, structs and constants to common header
struct Globals
{
    bool time_to_go = false;
    bool show_depth_image = false;
    char demo_mode = DEMO_MODE_OBJECT_DETECTION;
    bool writing = false;
    int writing_mode = 0; // 0 - stopped, 1 - writing, 2 - stopping
    bool erase = true;
    bool use_point_cloud_image_for_detection = false;
} globals;

const auto FRAME_INTERVAL = 60;
const auto WRITE_INTERVAL = 1;

using namespace viewer;

// Mutex to acquire and release locks before performing synchronized ops
// TODO Remove if not needed
std::mutex mtx;

//TOD Delete this
void NoOpDeallocator(void *data, size_t a, void *b) {}


// Move Depth image color functions to separate class
void ColorizeDepthImage(const k4a::image &depthImage,
                        DepthPixelVisualizationFunction visualizationFn,
                        std::pair<uint16_t, uint16_t> expectedValueRange,
                        std::vector<BgraPixel> *buffer);

void ColorizeFilteredDepthImage(const k4a::image &depthImage,
                                k4a_image_t transformed_color_image,
                                k4a_image_t xy_table,
                                BodyGeometry *body_geometry,
                                vector<Ray> rays,
                                DepthPixelVisualizationFunction visualizationFn,
                                std::pair<uint16_t, uint16_t> expectedValueRange,
                                std::vector<BgraPixel> *buffer);

void ColorizeClusteredDepthImage(const k4a::image &depthImage,
                                 k4a_image_t transformed_color_image,
                                 std::vector<BgraPixel> *buffer,
                                 cilantro::PointCloud3f *cloud,
                                 vector<Detection> detections,
                                 k4a_calibration_t sensor_calibration);


//TODO Remove if not needed
void error_handler(int sig)
{
    void *array[10];
    size_t size;

    // get void*'s for all entries on the stack
    size = backtrace(array, 10);

    // print out all the frames to stderr
    fprintf(stderr, "Error: signal %d:\n", sig);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    exit(1);
}

// TODO add comments
static void create_xy_table(const k4a_calibration_t *calibration, k4a_image_t xy_table)
{
    k4a_float2_t *table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_table);

    int width = calibration->depth_camera_calibration.resolution_width;
    int height = calibration->depth_camera_calibration.resolution_height;

    int camerawidth = calibration->color_camera_calibration.resolution_width;
    int cameraheight = calibration->color_camera_calibration.resolution_height;
    cout << "Depth camera size" << width << " " << height << endl;
    cout << "Color camera size" << camerawidth << " " << cameraheight << endl;
    k4a_float2_t p;
    k4a_float3_t ray;
    int valid;

    for (int y = 0, idx = 0; y < height; y++)
    {
        p.xy.y = (float)y;
        for (int x = 0; x < width; x++, idx++)
        {
            p.xy.x = (float)x;

            k4a_calibration_2d_to_3d(
                calibration, &p, 1.f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid);

            if (valid)
            {
                table_data[idx].xy.x = ray.xyz.x;
                table_data[idx].xy.y = ray.xyz.y;
            }
            else
            {
                table_data[idx].xy.x = nanf("");
                table_data[idx].xy.y = nanf("");
            }
        }
    }
}

// TODO add comments
// TODO build Clusterizer inside
// TODO add point count to clusterizer
static void generate_point_cloud(const k4a_image_t depth_image,
                                 k4a_image_t transformed_color_image,
                                 const k4a_image_t xy_table,
                                 BodyGeometry *body_geometry,
                                 vector<Ray> rays,
                                 cilantro::VectorSet3f &cilantroPoints,
                                 cilantro::VectorSet3f &cilantroColors,
                                 int *point_count)
{
    int width = k4a_image_get_width_pixels(depth_image);
    int height = k4a_image_get_height_pixels(depth_image);
    uint16_t *depth_data = (uint16_t *)(void *)k4a_image_get_buffer(depth_image);
    k4a_float2_t *xy_table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_table);
    const uint8_t *buf = (uint8_t *)(void *)k4a_image_get_buffer(transformed_color_image);

    // TODO 3 lines possibly remove
    int len = static_cast<unsigned int>(k4a_image_get_size(transformed_color_image));
    uint8_t *bufcpy = new uint8_t[len];
    memcpy(bufcpy, buf, len);

    viewer::BgraPixel *pixels = reinterpret_cast<viewer::BgraPixel *>(bufcpy);
    vector<k4a_float3_t> pointsOfInterest;
    vector<k4a_float3_t> colors;
    int total = 0;
    for (int i = 0; i < width * height; i++)
    {
        if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
        {
            total++;
            float x = (xy_table_data[i].xy.x * (float)depth_data[i]);
            float y = (xy_table_data[i].xy.y * (float)depth_data[i]);
            float z = (float)depth_data[i];
            k4a_float3_t point = {x, y, z};
            if (!body_geometry->is_point_in_forward_space(point, rays))
            {
                continue;
            }
            pointsOfInterest.push_back(point);
            colors.push_back({(float)pixels[i].Red, (float)pixels[i].Green, (float)pixels[i].Blue});
            *point_count += 1;
        }
    }

    cout << "selected points " << *point_count << "/" << total << endl;
    if (*point_count > 0)
    {

        cilantroPoints.resize(3, *point_count);
        cilantroColors.resize(3, *point_count);
        for (int i = 0; i < *point_count; i++)
        {
            k4a_float3_t point = pointsOfInterest[i];
            k4a_float3_t color = colors[i];
            cilantroPoints(0, i) = point.v[0] / 1000.f;
            cilantroPoints(1, i) = point.v[1] / 1000.f;
            cilantroPoints(2, i) = point.v[2] / 1000.f;
            cilantroColors(0, i) = color.v[0] / 255.f;
            cilantroColors(1, i) = color.v[1] / 255.f;
            cilantroColors(2, i) = color.v[2] / 255.f;
        }
    }
}

static bool colorize_point_cloud(k4a_transformation_t transformation_handle,
                                 const k4a_image_t depth_image,
                                 const k4a_image_t color_image,
                                 k4a_image_t *transformed_color_image)
{
    int depth_image_width_pixels = k4a_image_get_width_pixels(depth_image);
    int depth_image_height_pixels = k4a_image_get_height_pixels(depth_image);
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                                                 depth_image_width_pixels,
                                                 depth_image_height_pixels,
                                                 depth_image_width_pixels * 4 * (int)sizeof(uint8_t),
                                                 transformed_color_image))
    {
        printf("Failed to create transformed color image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_color_image_to_depth_camera(transformation_handle,
                                                                               depth_image,
                                                                               color_image,
                                                                               *transformed_color_image))
    {
        printf("Failed to compute transformed color image\n");
        return false;
    }
    return true;
}

// move to Geometry class
bool contains(BoundingPlane plane, Eigen::Vector3f intercept)
{
    Eigen::Vector3f maxVal = plane.corners.colwise().maxCoeff();
    Eigen::Vector3f minVal = plane.corners.colwise().minCoeff();

    return abs(maxVal(0) - intercept[0]) <= abs(maxVal(0) - minVal(0)) + 0.00001f &&
           abs(maxVal(1) - intercept[1]) <= abs(maxVal(1) - minVal(1)) + 0.00001f &&
           abs(maxVal(2) - intercept[2]) <= abs(maxVal(2) - minVal(2)) + 0.00001f;
}

// move to Geometry class
void checkIntersects(BoundingPlane plane, Ray ray, bool *doesIntercept, k4a_float3_t *intercept, float *distanceToPlane)
{
    float denominator = plane.normal.dot(ray.direction);
    if (abs(denominator) > epsilon)
    {
        Eigen::Vector3f difference = plane.center - ray.origin;
        float t = difference.dot(plane.normal) / denominator;
        if (t >= 0.0001f)
        {
            // cout << " evaluating plane t value " << t << endl;
            Eigen::Vector3f intercept3f = ray.origin + ray.direction * t;
            if (contains(plane, intercept3f))
            {
                *doesIntercept = true;
                *distanceToPlane = (intercept3f - ray.origin).norm();
                // cout << " distance to plane " << *distanceToPlane << endl;
                k4a_float3_t k4_intercept = {intercept3f[0], intercept3f[1], intercept3f[2]};
                *intercept = k4_intercept;
            }
        }
        else
        {
            *doesIntercept = false;
        }
    }
    else
    {
        *doesIntercept = false;
    }
}

// move to Geometry class
Eigen::Vector3f computeNormal(Eigen::Matrix<float, 4, 3> corners)
{
    Eigen::Vector3f first = corners.row(1) - corners.row(0);
    Eigen::Vector3f second = corners.row(2) - corners.row(0);
    auto normal = first.cross(second);
    auto l2norm = normal.norm();
    normal = normal / l2norm;
    // printf(" normal %f, %f, %f\n", normal[0], normal[1], normal[2] );
    return normal;
}

// move to Geometry class
void checkPointee(BoundingCube cube, Ray ray, bool *doesIntersect, k4a_float3_t *intercept, float *distanceToPointee)
{
    vector<BoundingPlane> boundingPlanes;
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
    plane1.normal = computeNormal(plane1.corners);

    plane1.center << cube.u[0], (cube.u[1] + cube.v[1]) / 2.f, (cube.u[2] + cube.v[2]) / 2.f;
    boundingPlanes.push_back(plane1);

    BoundingPlane plane2;
    plane2.corners.row(0) << cube.v[0], cube.u[1], cube.u[2];
    plane2.corners.row(1) << cube.v[0], cube.u[1], cube.v[2];
    plane2.corners.row(2) << cube.v[0], cube.v[1], cube.u[2];
    plane2.corners.row(3) << cube.v[0], cube.v[1], cube.v[2];
    plane2.normal = computeNormal(plane2.corners);

    plane2.center << cube.v[0], (cube.u[1] + cube.v[1]) / 2.f, (cube.u[2] + cube.v[2]) / 2.f;
    boundingPlanes.push_back(plane2);

    BoundingPlane plane3;
    plane3.corners.row(0) << cube.u[0], cube.u[1], cube.u[2];
    plane3.corners.row(1) << cube.u[0], cube.u[1], cube.v[2];
    plane3.corners.row(2) << cube.v[0], cube.u[1], cube.u[2];
    plane3.corners.row(3) << cube.v[0], cube.u[1], cube.v[2];
    plane3.normal = computeNormal(plane3.corners);

    plane3.center << (cube.u[0] + cube.v[0]) / 2.f, cube.u[1], (cube.u[2] + cube.v[2]) / 2.f;
    boundingPlanes.push_back(plane3);

    BoundingPlane plane4;
    plane4.corners.row(0) << cube.u[0], cube.v[1], cube.u[2];
    plane4.corners.row(1) << cube.u[0], cube.v[1], cube.v[2];
    plane4.corners.row(2) << cube.v[0], cube.v[1], cube.u[2];
    plane4.corners.row(3) << cube.v[0], cube.v[1], cube.v[2];
    plane4.normal = computeNormal(plane4.corners);

    plane4.center << (cube.u[0] + cube.v[0]) / 2.f, cube.v[1], (cube.u[2] + cube.v[2]) / 2.f;
    boundingPlanes.push_back(plane4);

    BoundingPlane plane5;
    plane5.corners.row(0) << cube.u[0], cube.u[1], cube.u[2];
    plane5.corners.row(1) << cube.u[0], cube.v[1], cube.u[2];
    plane5.corners.row(2) << cube.v[0], cube.u[1], cube.u[2];
    plane5.corners.row(3) << cube.v[0], cube.v[1], cube.u[2];
    plane5.normal = computeNormal(plane5.corners);

    plane5.center << (cube.u[0] + cube.v[0]) / 2.f, (cube.u[1] + cube.v[1]) / 2.f, cube.u[2];
    boundingPlanes.push_back(plane5);

    BoundingPlane plane6;
    plane6.corners.row(0) << cube.u[0], cube.u[1], cube.v[2];
    plane6.corners.row(1) << cube.u[0], cube.v[1], cube.v[2];
    plane6.corners.row(2) << cube.v[0], cube.u[1], cube.v[2];
    plane6.corners.row(3) << cube.v[0], cube.v[1], cube.v[2];
    plane6.normal = computeNormal(plane6.corners);

    plane6.center << (cube.u[0] + cube.v[0]) / 2.f, (cube.u[1] + cube.v[1]) / 2.f, cube.v[2];
    boundingPlanes.push_back(plane6);

    int interceptCount = 0;
    for (int p = 0; p < boundingPlanes.size(); p++)
    {
        BoundingPlane plane = boundingPlanes[p];
        float distance;
        bool planeIntersects = false;
        k4a_float3_t planeInterceptVector;
        checkIntersects(plane, ray, &planeIntersects, &planeInterceptVector, &distance);
        if (planeIntersects == true && (distance < distanceToNearest))
        {
            interceptCount++;
            // cout << "found intersecting plane " << p << " at " << distance << endl;
            *doesIntersect = true;
            nearestPlane = plane;
            distanceToNearest = distance;
            *intercept = planeInterceptVector;
            // k4_intercept = {planeInterceptVector.v[0], plane.center[1], plane.center[2]};
            interceptflag = true;
        }
    }
    // *distanceToPointee = distanceToNearest;

    if (interceptflag == true)
    {
        *doesIntersect = true;
        *distanceToPointee = distanceToNearest;
    }
    else
    {
        *doesIntersect = false;
        *distanceToPointee = std::numeric_limits<float>::infinity();
    }
}


// TODO move to geometry
bool getPointee(Ray ray, vector<BoundingCube> *pointCubes, int *pointee)
{
    float distanceToNearest = std::numeric_limits<float>::infinity();
    k4a_float3_t intercept;
    bool found = false;
    for (int b = 0; b < (*pointCubes).size(); b++)
    {
        BoundingCube *cube = &((*pointCubes)[b]);
        bool isPointee = false;
        k4a_float3_t intersectionVector;
        float distance;
        checkPointee(*cube, ray, &isPointee, &intersectionVector, &distance);
        if (isPointee == true && distance < distanceToNearest)
        {
            *pointee = b;
            cube->pointer.clear();
            cube->pointer.push_back({ray.origin[0], ray.origin[1], ray.origin[2]});
            cube->pointer.push_back(intersectionVector);
            distanceToNearest = distance;
            found = true;
        }
    }

    return found;
}

// TODO Common helper / rename
k4a_float3_t vec2k4a(Eigen::Vector3f aVector)
{
    return {aVector[0], aVector[1], aVector[2]};
}

// TODO move to geometry
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
    k4a_float3_t point = vec2k4a(vec3);
    point_to_color2d(&point, sensor_calibration, w, h, point_2d);
}

// TODO move to common / #define function
void to_upper(std::string &str)
{
    std::transform(str.begin(), str.end(), str.begin(), ::toupper);
}

// TODO move to other class
// Pass clusterizer and body_geometry
void detect_objects(int frame_number,
                    cv::Mat cvImageMatrix,
                    cilantro::VectorSet3f *cilantroPoints,
                    cilantro::VectorSet3f *cilantroColors,
                    cilantro::PointCloud3f *cloud_seg,
                    vector<Ray> rays,
                    vector<Detection> *detections,
                    k4a_calibration_t *sensor_calibration,
                    int w,
                    int h)
{
    mtx.lock();
    // detections->clear();
    mtx.unlock();
    Clusterizer clusterizer(cilantroPoints, cilantroColors);
    vector<BoundingCube> pointCubes = clusterizer.Clusterize(cloud_seg);
    for (int c = 0; c < pointCubes.size(); c++)
    {
        BoundingCube *cube = &(pointCubes[c]);
        vector<k4a_float2_t> points2d;
        vector<Vector3f> vectors = {cube->ftl, cube->ftr, cube->fbr, cube->fbl, cube->btl, cube->btr, cube->bbr, cube->bbl};
        float minx = INFINITY, miny = INFINITY;
        float maxx = -INFINITY, maxy = -INFINITY;

        for (Vector3f vector : vectors)
        {
            k4a_float2_t pt2d;
            vector_to_color2d(vector, sensor_calibration, w, h, &pt2d);
            if (pt2d.v[0] < minx)
            {
                minx = pt2d.v[0];
            }
            else if (pt2d.v[0] > maxx)
            {
                maxx = pt2d.v[0];
            }
            if (pt2d.v[1] < miny)
            {
                miny = pt2d.v[1];
            }
            else if (pt2d.v[1] > maxy)
            {
                maxy = pt2d.v[1];
            }
        }
        cube->u2d_k4.v[0] = minx - 20.f;
        cube->u2d_k4.v[1] = miny - 20.f;
        cube->v2d_k4.v[0] = maxx + 20.f;
        cube->v2d_k4.v[1] = maxy + 20.f;
        // TODO - set color image u,v while creating cube

    }
    if (pointCubes.size() > 0)
    {
        for (Ray ray : rays)
        {
            int pointee;
            bool found = getPointee(ray, &pointCubes, &pointee);
            if (found == true)
            {
                Detection detection;
                detection.cube = pointCubes[pointee];
                detection.ray = ray;
                detection.frame_number = frame_number;

                size_t pointeeLabel = detection.cube.label;
                int x = (int)detection.cube.u2d_k4.v[0];
                int y = (int)detection.cube.u2d_k4.v[1];
                int x1 = (int)detection.cube.v2d_k4.v[0];
                int y1 = (int)detection.cube.v2d_k4.v[1];

                if (x1 < x)
                {
                    cout << " bad u v x in frame " << frame_number << " " << x << "," << x1 << endl;
                    int t = x;
                    x = x1;
                    x1 = t;
                }
                if (y1 < y)
                {
                    cout << " bad u v y in frame " << frame_number << " " << y << "," << y1 << endl;
                    int t = y;
                    y = y1;
                    y1 = t;
                }
                if (x1 < cvImageMatrix.size().width && y1 < cvImageMatrix.size().height)
                {
                    // TODO eliminate option
                    if (globals.use_point_cloud_image_for_detection == true)
                    {

                        uint8_t *bufcpy = new uint8_t[w * 4 * h];
                        for (int n = 0; n < w * h * 4; n++)
                        {
                            bufcpy[n] = 255;
                        }

                        cv::Mat blank = cv::Mat::zeros(cv::Size(cvImageMatrix.cols, cvImageMatrix.rows), CV_8UC1);
                        int pointsset = 0;
                        vector<BgraPixel> pixelBuffer(w * h);
                        for (int p = 0; p < w * h; p++)
                        {
                            pixelBuffer[p] = {0, 0, 0, 255};
                        }
                        for (int c = 0; c < detection.cube.points.rows(); c++)
                        {

                            Eigen::Vector3f p = detection.cube.points.row(c);
                            Eigen::Vector4i rgba = detection.cube.pointColors.row(c);
                            k4a_float3_t pt3d = vec2k4a(p);
                            BgraPixel pixel = {(uint8_t)rgba[0], (uint8_t)rgba[1], (uint8_t)rgba[2], (uint8_t)rgba[3]};
                            k4a_float2_t pt2d;
                            point_to_color2d(&pt3d, sensor_calibration, w, h, &pt2d);
                            int k = ((int)pt2d.v[1] * w * 4 + (int)pt2d.v[0] * 4);
                            if (k < w * 4 * h)
                            {
                                pointsset++;
                                // pixelBuffer[k] = pixel;
                                bufcpy[k++] = rgba[2];
                                bufcpy[k++] = rgba[1];
                                bufcpy[k++] = rgba[0];
                                bufcpy[k] = rgba[3];
                            }
                        }
                        cv::Mat fullImage = cv::Mat(h, w, CV_8UC4, (void *)bufcpy);
                        if (pointsset > 0)
                        {
                            auto filename = "test" + to_string(frame_number) + ".jpg";
                            cv::imwrite(filename, fullImage);
                        }

                        cv::Rect region_of_interest = cv::Rect(x, y, (x1 - x), (y1 - y));
                        try
                        {
                            cv::Mat roi = fullImage(region_of_interest);
                            AzureVisionRequest *vq = new AzureVisionRequest(0);

                            std::vector<uchar> buff;
                            std::vector<int> param(2);
                            param[0] = cv::IMWRITE_JPEG_QUALITY;
                            param[1] = 95; //default(95) 0-100
                            cv::imencode(".jpg", roi, buff, param);
                            auto filename = "obj_roi" + to_string(frame_number) + ".jpg";
                            // cv::imwrite(filename, roi);
                            const char *azureVisionKey = getenv("AZURE_VISION_KEY");
                            vq->SendAnalysisRequest(azureVisionKey, buff);
                            string name = vq->getTopObject();
                            std::transform(name.begin(), name.end(), name.begin(), ::toupper);
                            cout << "detected " << name << " in frame " << detection.frame_number << endl;
                            detection.name = name;
                            detections->push_back(detection);
                        }
                        catch (const std::exception &e)
                        {
                            std::cerr << e.what() << std::endl;
                        }
                    }
                    else
                    {
                        cv::Rect region_of_interest = cv::Rect(x, y, (x1 - x), (y1 - y));
                        try
                        {
                            cv::Mat roi = cvImageMatrix(region_of_interest);
                            AzureVisionRequest *vq = new AzureVisionRequest(0);

                            std::vector<uchar> buff;
                            std::vector<int> param(2);
                            param[0] = cv::IMWRITE_JPEG_QUALITY;
                            param[1] = 95; //default(95) 0-100
                            cv::imencode(".jpg", roi, buff, param);
                            auto filename = "obj_roi" + to_string(frame_number) + ".jpg";
                            // cv::imwrite(filename, roi);

                            vq->SendAnalysisRequest("ac1549fb73224a7386db9bf1ed2a9a09", buff);
                            string name = vq->getTopObject();
                            to_upper(name);
                            cout << "detected " << name << " in frame " << detection.frame_number << endl;
                            detection.name = name;

                            detections->push_back(detection);
                        }
                        catch (const std::exception &e)
                        {
                            std::cerr << e.what() << std::endl;
                        }
                    }
                }
            }
        }
    }
}

void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    {
        printf("Escape key pressed\n");
        globals.time_to_go = true;
    }
    if (key == GLFW_KEY_D && action == GLFW_PRESS)
    {
        printf("Toggling depth image\n");
        globals.show_depth_image = !globals.show_depth_image;
    }
    if (key == GLFW_KEY_L && action == GLFW_PRESS)
    {
        printf("Switching to light saber display\n");
        globals.demo_mode = DEMO_MODE_LIGHT_SABERS;
    }
    if (key == GLFW_KEY_O && action == GLFW_PRESS)
    {
        printf("Switching to object detection\n");
        globals.demo_mode = DEMO_MODE_OBJECT_DETECTION;
    }
    if (key == GLFW_KEY_X && action == GLFW_PRESS)
    {
        printf("Switching to x-ray vision\n");
        globals.demo_mode = DEMO_MODE_XRAY_VISION;
    }
    if (key == GLFW_KEY_J && action == GLFW_PRESS)
    {
        printf("Switching to joint information\n");
        globals.demo_mode = DEMO_MODE_JOINT_INFO;
    }
    if (key == GLFW_KEY_W && action == GLFW_PRESS)
    {
        printf("Switching to writing mode\n");
        globals.demo_mode = DEMO_MODE_WRITING;
    }
    if (key == GLFW_KEY_E && action == GLFW_PRESS)
    {
        printf("Erase\n");
        globals.erase = true;
    }
    if (key == GLFW_KEY_LEFT_SHIFT && action == GLFW_PRESS)
    {
        printf("Writing on\n");
        globals.writing_mode = 1;
    }
    if (key == GLFW_KEY_LEFT_SHIFT && action == GLFW_RELEASE)
    {
        printf("Writing off\n");
        globals.writing_mode = 2;
    }
}


// TODO move to body geometry
bool compareBodies(k4abt_body_t first, k4abt_body_t second)
{
    float h1 = first.skeleton.joints[K4ABT_JOINT_HEAD].position.v[1];
    float h2 = second.skeleton.joints[K4ABT_JOINT_HEAD].position.v[1];
    return (h1 < h2);
}

// TODO common
void print_vector(string name, Vector3f v)
{
    cout << name << " " << v[0] << "," << v[1] << "," << v[2] << endl;
}

int main()
{
    signal(SIGSEGV, error_handler);
    try
    {
        k4a_image_t xy_table = NULL;
        k4a_image_t point_cloud = NULL;
        int num_frames = 0;
        vector<BoundingCube *> pointeeList;
        vector<Detection> detections;
        vector<LetterWidget> letterPoints;
        vector<PointerWidget> lightSabers;
        LetterWidget currentLetter;
        letterPoints.push_back(currentLetter);
        vector<JointCoordinates> moving_average;
        int left_hand_raise_state = 0;

        // Check for devices
        //
        const uint32_t deviceCount = k4a::device::get_installed_count();
        if (deviceCount == 0)
        {
            throw std::runtime_error("No Azure Kinect devices detected!");
        }

        // Start the device
        //
        k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        config.camera_fps = K4A_FRAMES_PER_SECOND_30;
        config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        config.color_resolution = K4A_COLOR_RESOLUTION_720P;

        // This means that we'll only get captures that have both color and
        // depth images, so we don't need to check if the capture contains
        // a particular type of image.
        //
        config.synchronized_images_only = true;

        k4a::device dev = k4a::device::open(K4A_DEVICE_DEFAULT);
        dev.start_cameras(&config);

        // Create the viewer window.
        //
        ViewerWindow &window = ViewerWindow::Instance();
        window.Initialize("My Favorite Things", 1280, 720);

        window.SetKeyCallBackHandler(key_callback);

        // Textures we can give to OpenGL / the viewer window to render.
        //
        Texture depthTexture = window.CreateTexture(GetDepthDimensions(config.depth_mode));
        Texture colorTexture = window.CreateTexture(GetColorDimensions(config.color_resolution));

        int colorWindowWidth = (globals.show_depth_image == true) ? window.GetWidth() / 2 : window.GetWidth();

        const ImVec2 windowSize(window.GetWidth(), static_cast<float>(window.GetHeight()));
        ImVec2 colorWindowSize(colorWindowWidth, static_cast<float>(window.GetHeight()));
        ImVec2 depthWindowSize(colorWindowWidth, static_cast<float>(window.GetHeight()));
        float colorWindowOrigin = 0.f;
        float depthWindowWidth = (globals.show_depth_image == true) ? window.GetWidth() / 2 : window.GetWidth();

        // A buffer containing a BGRA color representation of the depth image.
        // This is what we'll end up giving to depthTexture as an image source.
        // We don't need a similar buffer for the color image because the color
        // image already comes to us in BGRA32 format.
        //

        k4a_calibration_t sensor_calibration;
        VERIFY(k4a_device_get_calibration(dev.handle(),
                                          config.depth_mode,
                                          config.color_resolution,
                                          &sensor_calibration),
               "Get depth camera calibration failed!");
        k4a_transformation_t transformation = k4a_transformation_create(&sensor_calibration);

        k4abt_tracker_t tracker = NULL;
        k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
        VERIFY(k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker),
               "Body tracker initialization failed!");

        k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                         sensor_calibration.depth_camera_calibration.resolution_width,
                         sensor_calibration.depth_camera_calibration.resolution_height,
                         sensor_calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
                         &xy_table);

        create_xy_table(&sensor_calibration, xy_table);

        k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                         sensor_calibration.depth_camera_calibration.resolution_width,
                         sensor_calibration.depth_camera_calibration.resolution_height,
                         sensor_calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float3_t),
                         &point_cloud);

        while (window.BeginFrame() && globals.time_to_go == false)
        {
            ImVec2 *nose;
            vector<ImVec2> lineOfSight(2);
            ImVec2 *head;
            vector<ImVec2> eyes(2);
            vector<int> body_ids;
            cilantro::VectorSet3f cilantroPoints;
            cilantro::VectorSet3f cilantroColors;
            vector<ImVec2> directionImVectors;
            std::vector<BgraPixel> depthTextureBuffer;
            bool cilantroAvailable = false;
            k4a_image_t transformed_color_image = NULL;

            cilantro::PointCloud3f cloud_seg;
            BodyGeometry *body_geometry;
            vector<PointerWidget> pointers;
            vector<CubeWidget> cubeWidgets;
            vector<AxisWidget> axes;
            vector<JointWidget> jointWidgets;
            // cout << "Current frame " << (num_frames) << endl;
            // cout << "Erasing detections from " << (num_frames - 20 * FRAME_INTERVAL) << endl;

            colorWindowWidth = (globals.show_depth_image == true) ? window.GetWidth() / 2 : window.GetWidth();
            colorWindowOrigin = (globals.show_depth_image == true) ? 1440 / 2.f : 0.f;
            depthWindowWidth = 1440 / 2.f;

            colorWindowSize = {(float)colorWindowWidth, static_cast<float>(window.GetHeight())};
            depthWindowSize = {(float)depthWindowWidth, static_cast<float>(window.GetHeight())};

            detections.erase(
                std::remove_if(detections.begin(), detections.end(),
                               [&](const Detection detection) { return (num_frames - 20 * FRAME_INTERVAL) > detection.frame_number; }),
                detections.end());
            lightSabers.erase(
                std::remove_if(lightSabers.begin(), lightSabers.end(),
                               [&](const PointerWidget lightSaber) { return (num_frames - 5) > lightSaber.frameNumber; }),
                lightSabers.end());

            if (globals.demo_mode != DEMO_MODE_WRITING)
            {
                letterPoints.clear();
            }
            if (globals.erase == true)
            {
                globals.erase = false;
                letterPoints.clear();
            }
            k4a::capture capture;
            vector<k4abt_body_t> bodies;
            uint32_t num_bodies = 0;
            if (dev.get_capture(&capture, std::chrono::milliseconds(0)))
            {
                const k4a::image depthImage = capture.get_depth_image();
                const k4a::image colorImage = capture.get_color_image();

                const uint8_t *buf = colorImage.get_buffer();
                const int len = colorImage.get_size();
                if (len == 0)
                {
                    break;
                }
                const int w = colorImage.get_width_pixels();
                const int h = colorImage.get_height_pixels();
                uint8_t *bufcpy = new uint8_t[len];
                memcpy(bufcpy, buf, len);

                BgraPixel *pixels = reinterpret_cast<BgraPixel *>(bufcpy);
                colorize_point_cloud(transformation,
                                     depthImage.handle(),
                                     colorImage.handle(),
                                     &transformed_color_image);

                if (1 == 1)
                {

                    k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker,
                                                                                           capture.handle(),
                                                                                           K4A_WAIT_INFINITE);

                    if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
                    {
                        // It should never hit timeout when K4A_WAIT_INFINITE is set.
                        printf("Error! Add capture to tracker process queue timeout!\n");
                        break;
                    }
                    else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
                    {
                        printf("Error! Add capture to tracker process queue failed!\n");
                        break;
                    }
                    k4abt_frame_t body_frame = NULL;
                    k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
                    if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
                    {

                        num_bodies = k4abt_frame_get_num_bodies(body_frame);
                        if (num_bodies > 0)
                        {

                            for (int b = 0; b < (int)num_bodies; b++)
                            {
                                k4abt_body_t bd;
                                VERIFY(k4abt_frame_get_body_skeleton(body_frame, b, &bd.skeleton),
                                       "Get body from body frame failed!");
                                // bd.id = k4abt_frame_get_body_id(body_frame, b);
                                bodies.push_back(bd);
                            }
                            sort(bodies.begin(), bodies.end(), compareBodies);
                            for (int b = 0; b < (int)num_bodies; b++)
                            {
                                bodies[b].id = b;
                            }

                            vector<k4abt_joint_id_t> joints_of_interest = {
                                K4ABT_JOINT_EYE_LEFT,
                                K4ABT_JOINT_EYE_RIGHT,
                                K4ABT_JOINT_NOSE,
                                K4ABT_JOINT_HAND_LEFT,
                                K4ABT_JOINT_HAND_RIGHT,
                                K4ABT_JOINT_ELBOW_RIGHT,
                                K4ABT_JOINT_SPINE_CHEST,
                                K4ABT_JOINT_HANDTIP_LEFT,
                                K4ABT_JOINT_HANDTIP_RIGHT,
                                K4ABT_JOINT_THUMB_RIGHT};

                            if (moving_average.size() < (int)num_bodies)
                            {
                                moving_average.resize((int)num_bodies);
                            }
                            if (globals.demo_mode == DEMO_MODE_LIGHT_SABERS)
                            {
                                body_geometry = new BodyGeometry(&sensor_calibration, colorWindowOrigin, colorWindowSize, w, h, &moving_average, K4ABT_JOINT_THUMB_RIGHT, bodies, joints_of_interest);
                            }
                            else
                            {
                                if (globals.demo_mode == DEMO_MODE_OBJECT_DETECTION)
                                {
                                    body_geometry = new BodyGeometry(&sensor_calibration, colorWindowOrigin, colorWindowSize, w, h, &moving_average, K4ABT_JOINT_HANDTIP_RIGHT, bodies, joints_of_interest);
                                }
                                else
                                {
                                    body_geometry = new BodyGeometry(&sensor_calibration, colorWindowOrigin, colorWindowSize, w, h, &moving_average, K4ABT_JOINT_KNEE_LEFT, bodies, joints_of_interest);
                                }
                            }

                            vector<Ray> rays;
                            vector<PointerWidget> newSabers;
                            for (int k = 0; k < (int)num_bodies; k++)
                            {
                                body_ids.push_back(k);
                                if (globals.demo_mode == DEMO_MODE_LIGHT_SABERS)
                                {
                                    int lastSaberIndex = -1;
                                    for (int l = lightSabers.size() - 1; l >= 0; l--)
                                    {
                                        if (lightSabers[l].bodyId == k)
                                        {
                                            lastSaberIndex = l;
                                            break;
                                        }
                                    }

                                    cout << "last saber for body " << k << " " << lastSaberIndex << endl;

                                    Ray ray = body_geometry->joints_to_ray(k, K4ABT_JOINT_ELBOW_RIGHT, K4ABT_JOINT_HAND_RIGHT, K4ABT_JOINT_HAND_RIGHT);

                                    Vector3f ortho = body_geometry->ray_to_joint_ortho(ray, k, K4ABT_JOINT_THUMB_RIGHT);
                                    // rays.push_back(ray);
                                    // ImVec2 pointerBeginObj = body_geometry->vector_to_window(ray.origin);
                                    ImVec2 pointerBeginObj = body_geometry->joint_to_window(k, K4ABT_JOINT_HAND_RIGHT);

                                    PointerWidget lightSaber;
                                    // lightSaber.boxType = 4;
                                    lightSaber.bodyId = k;
                                    lightSaber.frameNumber = num_frames;
                                    lightSaber.alpha = 200;

                                    lightSaber.arrow.push_back(pointerBeginObj);

                                    ImVec2 pointerEndObj = body_geometry->vector_to_window(ray.origin + ortho * 600.f);
                                    lightSaber.arrow.push_back(pointerEndObj);

                                    if (lastSaberIndex > -1)
                                    {
                                        PointerWidget lastSaber = lightSabers[lastSaberIndex];
                                        float x1_0 = lastSaber.arrow[0].x;
                                        float x2_0 = lastSaber.arrow[1].x;
                                        float x1_1 = lightSaber.arrow[0].x;
                                        float x2_1 = lightSaber.arrow[1].x;
                                        float y1_0 = lastSaber.arrow[0].y;
                                        float y2_0 = lastSaber.arrow[1].y;
                                        float y1_1 = lightSaber.arrow[0].y;
                                        float y2_1 = lightSaber.arrow[1].y;
                                        float dx1 = (x1_1 - x1_0) / 20.f;
                                        float dy1 = (y1_1 - y1_0) / 20.f;
                                        float dx2 = (x2_1 - x2_0) / 20.f;
                                        float dy2 = (y2_1 - y2_0) / 20.f;
                                        for (int dn = 1; dn <= 20; dn++)
                                        {
                                            PointerWidget interpolation;
                                            interpolation.boxType = 4;
                                            interpolation.bodyId = k;
                                            interpolation.frameNumber = num_frames;
                                            interpolation.alpha = dn * 20;
                                            interpolation.arrow.push_back(ImVec2(x1_0 + dx1 * (float)dn, y1_0 + dy1 * (float)dn));
                                            interpolation.arrow.push_back(ImVec2(x2_0 + dx2 * (float)dn, y2_0 + dy2 * (float)dn));
                                            newSabers.push_back(interpolation);
                                        }
                                    }
                                    else
                                    {
                                        newSabers.push_back(lightSaber);
                                    }
                                }

                                if (globals.demo_mode == DEMO_MODE_WRITING)
                                {

                                    k4a_float3_t left_hand = body_geometry->joint_to_global(k, K4ABT_JOINT_HAND_LEFT, {0.f, 0.f, 0.f});
                                    k4a_float3_t left_hand_act = body_geometry->get_joint(k, K4ABT_JOINT_HAND_LEFT).position;
                                    k4a_float3_t nose = body_geometry->joint_to_global(k, K4ABT_JOINT_NOSE, {0.f, 0.f, 0.f});
                                    k4a_float3_t tip = body_geometry->joint_to_global(k, K4ABT_JOINT_THUMB_RIGHT, {0.f, 0.f, 0.f});
                                    k4a_float3_t hand = body_geometry->joint_to_global(k, K4ABT_JOINT_HAND_RIGHT, {0.f, 0.f, 0.f});
                                    ImVec2 tip_vec = body_geometry->point_to_window(tip);

                                    Vector3f lh, rh, diff;
                                    lh << left_hand.v[0], left_hand.v[1], left_hand.v[2];
                                    rh << hand.v[0], hand.v[1], hand.v[2];
                                    diff = lh - rh;
                                    if (diff.norm() < 100.f)
                                    {
                                        cout << "Erasing" << endl;
                                        letterPoints.clear();
                                        if (globals.writing_mode == 1)
                                        {
                                            globals.writing_mode = 2;
                                        }
                                        else
                                        {
                                            globals.writing_mode = 0;
                                        }
                                    }
                                    if (letterPoints.size() == 0)
                                    {
                                        LetterWidget nextLetter;
                                        letterPoints.push_back(nextLetter);
                                    }
                                    int currentLetterIndex = letterPoints.size() - 1;
                                    currentLetter = letterPoints[currentLetterIndex];

                                    // cout << nose.v[1] << " " << left_hand.v[1] << " " << left_hand_act.v[1] << endl;
                                    if (nose.v[1] > left_hand.v[1])
                                    {
                                        globals.writing_mode = 1;
                                    }
                                    else
                                    {
                                        if (globals.writing_mode == 1)
                                        {
                                            globals.writing_mode = 2;
                                        }
                                        else
                                        {
                                            globals.writing_mode = 0;
                                        }
                                    }
                                    if (globals.writing_mode == 1)
                                    {
                                        if (num_frames % WRITE_INTERVAL == 0)
                                        {
                                            letterPoints[currentLetterIndex].letterPath.push_back(tip_vec);
                                            cout << "num points for letter " << letterPoints[currentLetterIndex].letterPath.size() << endl;
                                        }
                                    }
                                    if (globals.writing_mode == 2)
                                    {
                                        globals.writing_mode = 0;
                                        if (letterPoints[currentLetterIndex].letterPath.size() > 0)
                                        {
                                            cout << "total num points for letter " << letterPoints[currentLetterIndex].letterPath.size() << endl;
                                            cout << "total number  of letters " << letterPoints.size() << endl;
                                            LetterWidget nextLetter;
                                            nextLetter.boxType = 2;
                                            letterPoints.push_back(nextLetter);
                                        }
                                    }
                                }

                                if (globals.demo_mode == DEMO_MODE_OBJECT_DETECTION)
                                {

                                    Ray ray = body_geometry->joints_to_ray(k, K4ABT_JOINT_ELBOW_RIGHT, K4ABT_JOINT_HAND_RIGHT, 120.f);
                                    rays.push_back(ray);
                                    ImVec2 pointerBeginObj = body_geometry->vector_to_window(ray.origin);
                                    directionImVectors.push_back(pointerBeginObj);

                                    float delta = 200.f;
                                    k4a_float3_t target = {ray.origin[0] + ray.direction[0] * delta, ray.origin[1] + ray.direction[1] * delta,
                                                           ray.origin[2] + ray.direction[2] * delta};
                                    ImVec2 pointerEndObj = body_geometry->point_to_window(target);
                                    if (pointerEndObj.x == 0 && pointerEndObj.y == 0)
                                    {
                                        directionImVectors.push_back(pointerBeginObj);
                                    }
                                    else
                                    {
                                        directionImVectors.push_back(pointerEndObj);
                                    }
                                }
                                if (globals.demo_mode == DEMO_MODE_JOINT_INFO)
                                {
                                    vector<k4abt_joint_id_t> joint_ids = {
                                        K4ABT_JOINT_HANDTIP_LEFT,
                                        K4ABT_JOINT_HANDTIP_RIGHT,
                                        K4ABT_JOINT_SPINE_CHEST
                                    };
                                    for (k4abt_joint_id_t joint_id : joint_ids)
                                    {
                                        int valid = 0;
                                        JointInfo jointInfo = body_geometry->get_joint_information(k, joint_id, &valid);
                                        if (valid == 1)
                                        {
                                            JointWidget jointDisplay;
                                            jointDisplay.bodyId = k;
                                            jointDisplay.frameNumber = num_bodies;
                                            jointDisplay.joints.push_back(jointInfo);
                                            jointWidgets.push_back(jointDisplay);
                                        }
                                    }
                                }
                            }
                            if (globals.demo_mode == DEMO_MODE_JOINT_INFO)
                            {
                                AxisWidget axisItem;
                                axisItem.name = "Depth Camera";
                                axisItem.bodyId = -1;
                                axisItem.boxType = 5;
                                axisItem.frameNumber = num_frames;
                                axisItem.textCoordinates = ImVec2(100.f, 75.f);
                                k4a_float3_t origin = body_geometry->window_to_point(ImVec2(100.f, 100.f));
                                Eigen::Vector3f depth_zero, depth_x, depth_y, depth_z;
                                depth_zero << origin.v[0], origin.v[1], origin.v[2];
                                depth_x << origin.v[0] + 3.f, origin.v[1], origin.v[2];
                                depth_y << origin.v[0], origin.v[1] + 3.f, origin.v[2];
                                depth_z << origin.v[0], origin.v[1], origin.v[2] + 3.f;
                                ImVec2 pointerBeginObj = body_geometry->vector_to_window(depth_zero);
                                axisItem.x.push_back(pointerBeginObj);
                                axisItem.x.push_back(body_geometry->vector_to_window(depth_x));
                                axisItem.y.push_back(pointerBeginObj);
                                axisItem.y.push_back(body_geometry->vector_to_window(depth_y));
                                axisItem.z.push_back(pointerBeginObj);
                                axisItem.z.push_back(body_geometry->vector_to_window(depth_z));
                                axisItem.color = {0, 0, 0};
                                axisItem.textShadowColor = {255, 255, 255};
                                axes.push_back(axisItem);

                            }

                            lightSabers.insert(lightSabers.end(), newSabers.begin(), newSabers.end());
                            // bboxItems.insert(bboxItems.end(), lightSabers.begin(), lightSabers.end());

                            if (globals.demo_mode == DEMO_MODE_OBJECT_DETECTION)
                            {
                                if (directionImVectors.size() > 0)
                                {
                                    PointerWidget pointer;
                                    // directionItem.boxType = 1;
                                    pointer.frameNumber = num_frames;
                                    pointer.arrow = directionImVectors;
                                    pointers.push_back(pointer);
                                }
                            }

                            if (num_frames % FRAME_INTERVAL == 0 && globals.demo_mode == DEMO_MODE_OBJECT_DETECTION)
                            {
                                int point_count = 0;

                                generate_point_cloud(depthImage.handle(), transformed_color_image, xy_table, body_geometry, rays, cilantroPoints, cilantroColors, &point_count);
                                if (point_count > 100)
                                {

                                    cv::Mat cvImageMatrix(h, w, CV_8UC4, (void *)bufcpy, cv::Mat::AUTO_STEP);
                                    std::thread th1(detect_objects, num_frames, cvImageMatrix, &cilantroPoints, &cilantroColors, &cloud_seg, rays, &detections, &sensor_calibration, w, h);
                                    th1.detach();

                                    cilantroAvailable = true;
                                }
                            }
                            k4abt_frame_release(body_frame);
                            // cout << "processing detections  in frame " << num_frames << endl;

                            // cout << "num detections " << detections.size() << endl;
                            for (Detection detection : detections)
                            {
                                vector<ImVec2> bboxes;
                                Vector3f ftl = detection.cube.ftl;
                                Vector3f ftr = detection.cube.ftr;
                                Vector3f fbr = detection.cube.fbr;
                                Vector3f fbl = detection.cube.fbl;
                                Vector3f btl = detection.cube.btl;
                                Vector3f btr = detection.cube.btr;
                                Vector3f bbr = detection.cube.bbr;
                                Vector3f bbl = detection.cube.bbl;

                                CubeWidget cubeWidget;
                                cubeWidget.frameNumber = detection.frame_number;
                                cubeWidget.color = {detection.cube.color[0], detection.cube.color[1], detection.cube.color[2]};
                                cubeWidget.ftl = body_geometry->vector_to_window(ftl);
                                cubeWidget.ftr = body_geometry->vector_to_window(ftr);
                                cubeWidget.fbr = body_geometry->vector_to_window(fbr);
                                cubeWidget.fbl = body_geometry->vector_to_window(fbl);
                                cubeWidget.btl = body_geometry->vector_to_window(btl);
                                cubeWidget.btr = body_geometry->vector_to_window(btr);
                                cubeWidget.bbr = body_geometry->vector_to_window(bbr);
                                cubeWidget.bbl = body_geometry->vector_to_window(bbl);
                                cubeWidget.name = detection.name;
                                cubeWidgets.push_back(cubeWidget);

                                ImVec2 top_left = body_geometry->vector_to_window(detection.cube.u);
                                ImVec2 bottom_right = body_geometry->vector_to_window(detection.cube.v);
                                ImVec2 rayStart = body_geometry->vector_to_window(detection.ray.origin);
                                ImVec2 rayEnd = body_geometry->point_to_window(detection.cube.pointer[1]);
                            }
                            // cout << "num of bbox items " << bboxItems.size() << endl;
                            if (globals.show_depth_image == true)
                            {

                                ColorizeFilteredDepthImage(depthImage,
                                                           transformed_color_image,
                                                           xy_table,
                                                           body_geometry,
                                                           rays,
                                                           K4ADepthPixelColorizer::ColorizeBlueToRed,
                                                           GetDepthModeRange(config.depth_mode),
                                                           &depthTextureBuffer);
                                // if (detections.size() > 0)
                                // {
                                //     ColorizeClusteredDepthImage(depthImage, transformed_color_image, &depthTextureBuffer, &cloud_seg,
                                //                                 detections,
                                //                                 sensor_calibration);
                                // }
                                depthTexture.Update(&depthTextureBuffer[0]);
                            }
                            delete body_geometry;
                            delete[] bufcpy;
                        }
                    }
                    else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
                    {
                        //  It should never hit timeout when K4A_WAIT_INFINITE is set.
                        printf("Error! Pop body frame result timeout!\n");
                        break;
                    }
                    else
                    {
                        printf("Pop body frame result failed!\n");
                        break;
                    }
                }
                colorTexture.Update(pixels);
                k4a_image_release(transformed_color_image);
                // k4a_capture_release(capture.handle());
            }
            if (globals.show_depth_image == true)
            {
                window.ShowTexture("Depth", depthTexture, ImVec2(0.f, 0.f), depthWindowSize);
            }
            if (num_bodies == 0)
            {
                window.ShowTexture("Color", colorTexture, ImVec2(colorWindowOrigin, 0.f), colorWindowSize);
            }
            else
            {
                window.ShowTextureWithPersistentBoundingBoxes(
                    "Color", colorTexture, ImVec2(colorWindowOrigin, 0.f), colorWindowSize, 
                    &lightSabers,
                    &pointers,
                    &axes,
                    &jointWidgets,
                    &letterPoints,
                    &cubeWidgets,
                    body_ids, num_frames, FRAME_INTERVAL);
            }
            // This will tell ImGui/OpenGL to render the frame, and will block until the next vsync.
            //

            window.EndFrame();
            num_frames++;
            cloud_seg.clear();
        }
    }

    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "Press [enter] to exit." << std::endl;
        std::cin.get();
        return 1;
    }
    return 0;
}

// Given a depth image, output a BGRA-formatted color image into buffer, using
// expectedValueRange to define what min/max values the depth image should have.
// Low values are blue, high values are red.
//
void ColorizeDepthImage(const k4a::image &depthImage,
                        DepthPixelVisualizationFunction visualizationFn,
                        std::pair<uint16_t, uint16_t> expectedValueRange,
                        std::vector<BgraPixel> *buffer)
{
    // This function assumes that the image is made of depth pixels (i.e. uint16_t's),
    // which is only true for IR/depth images.
    //
    const k4a_image_format_t imageFormat = depthImage.get_format();
    if (imageFormat != K4A_IMAGE_FORMAT_DEPTH16 && imageFormat != K4A_IMAGE_FORMAT_IR16)

    {
        throw std::logic_error("Attempted to colorize a non-depth image!");
    }

    const int width = depthImage.get_width_pixels();
    const int height = depthImage.get_height_pixels();

    buffer->resize(static_cast<size_t>(width * height));

    const uint16_t *depthData = reinterpret_cast<const uint16_t *>(depthImage.get_buffer());
    for (int h = 0; h < height; ++h)
    {
        for (int w = 0; w < width; ++w)
        {
            const size_t currentPixel = static_cast<size_t>(h * width + w);
            (*buffer)[currentPixel] = visualizationFn(depthData[currentPixel],
                                                      expectedValueRange.first,
                                                      expectedValueRange.second);
        }
    }
}

void ColorizeFilteredDepthImage(const k4a::image &depthImage,
                                k4a_image_t transformed_color_image,
                                k4a_image_t xy_table,
                                BodyGeometry *body_geometry,
                                vector<Ray> rays,
                                DepthPixelVisualizationFunction visualizationFn,
                                std::pair<uint16_t, uint16_t> expectedValueRange,
                                std::vector<BgraPixel> *buffer)
{
    // This function assumes that the image is made of depth pixels (i.e. uint16_t's),
    // which is only true for IR/depth images.
    //
    const k4a_image_format_t imageFormat = depthImage.get_format();
    if (imageFormat != K4A_IMAGE_FORMAT_DEPTH16 && imageFormat != K4A_IMAGE_FORMAT_IR16)

    {
        throw std::logic_error("Attempted to colorize a non-depth image!");
    }

    const int width = depthImage.get_width_pixels();
    const int height = depthImage.get_height_pixels();

    buffer->resize(static_cast<size_t>(width * height));

    const uint16_t *depthData = reinterpret_cast<const uint16_t *>(depthImage.get_buffer());
    k4a_float2_t *xy_table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_table);

    const uint8_t *buf = (uint8_t *)(void *)k4a_image_get_buffer(transformed_color_image);
    int len = static_cast<unsigned int>(k4a_image_get_size(transformed_color_image));
    uint8_t *bufcpy = new uint8_t[len];
    memcpy(bufcpy, buf, len);
    viewer::BgraPixel *pixels = reinterpret_cast<viewer::BgraPixel *>(bufcpy);

    for (int h = 0; h < height; ++h)
    {
        for (int w = 0; w < width; ++w)
        {
            const size_t i = static_cast<size_t>(h * width + w);
            if (depthData[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
            {
                float x = (xy_table_data[i].xy.x * (float)depthData[i]);
                float y = (xy_table_data[i].xy.y * (float)depthData[i]);
                float z = (float)depthData[i];
                k4a_float3_t point = {x, y, z};
                // if (body_geometry->is_point_in_forward_space(point, K4ABT_JOINT_HANDTIP_RIGHT, false))
                if (body_geometry->is_point_in_forward_space(point, rays))
                {
                    // (*buffer)[i] = visualizationFn(depthData[i],
                    //                                expectedValueRange.first,
                    //                                expectedValueRange.second);
                    if (i >= len)
                    {
                        (*buffer)[i] = visualizationFn(depthData[i],
                                                       expectedValueRange.first,
                                                       expectedValueRange.second);
                        // printf("transformed color image point out of depth image buffer %ld, %d\n", i, len);
                    }
                    else
                    {
                        (*buffer)[i] = pixels[i];
                    }
                    // continue;
                }
                else
                {
                    // (*buffer)[i] = visualizationFn(depthData[i],
                    //                                expectedValueRange.first,
                    //                                expectedValueRange.second);
                }
            }
        }
    }
}

void ColorizeClusteredDepthImage(const k4a::image &depthImage,
                                 k4a_image_t transformed_color_image,
                                 std::vector<BgraPixel> *buffer,
                                 cilantro::PointCloud3f *cloud,
                                 vector<Detection> detections,
                                 k4a_calibration_t sensor_calibration)
{
    // This function assumes that the image is made of depth pixels (i.e. uint16_t's),
    // which is only true for IR/depth images.
    //
    const k4a_image_format_t imageFormat = depthImage.get_format();
    if (imageFormat != K4A_IMAGE_FORMAT_DEPTH16 && imageFormat != K4A_IMAGE_FORMAT_IR16)
    {
        throw std::logic_error("Attempted to colorize a non-depth image!");
    }

    const int width = depthImage.get_width_pixels();
    const int height = depthImage.get_height_pixels();
    int w = k4a_image_get_width_pixels(transformed_color_image);
    int h = k4a_image_get_height_pixels(transformed_color_image);

    int num_points = cloud->size();
    vector<vector<float>> label_colors;
    for (Detection detection : detections)
    {
        BoundingCube b = detection.cube;
        label_colors.push_back(b.color);
        Ray ray = detection.ray;
        if (b.pointer.size() == 0)
        {
            continue;
        }

        // k4a_float3_t pt1 = {b.u[0], b.u[1], b.u[2]};
        // k4a_float3_t pt2 = {b.v[0], b.v[1], b.v[2]};
        k4a_float3_t pt1 = b.pointer[0];
        k4a_float3_t pt2 = b.pointer[1];
        k4a_float2_t pt1_2d, pt2_2d;
        int vld1, vld2;
        k4a_calibration_3d_to_2d(&sensor_calibration,
                                 &pt1,
                                 K4A_CALIBRATION_TYPE_DEPTH,
                                 K4A_CALIBRATION_TYPE_DEPTH,
                                 &pt1_2d,
                                 &vld1);
        k4a_calibration_3d_to_2d(&sensor_calibration,
                                 &pt2,
                                 K4A_CALIBRATION_TYPE_DEPTH,
                                 K4A_CALIBRATION_TYPE_DEPTH,
                                 &pt2_2d,
                                 &vld2);
        if (vld1 == 1 && vld2 == 1)
        {

            int x1 = (int)pt1_2d.v[0];
            int y1 = (int)pt1_2d.v[1];
            int x2 = (int)pt2_2d.v[0];
            int y2 = (int)pt2_2d.v[1];
            float m = (float)(y2 - y1) / (float)(x2 - x1);
            // printf("drawing line %f \n", m);
            for (int x = x1; x < x2; x++)
            {
                int y = y1 + (int)(m * (float)(x - x1));
                int k = y * width + x;
                BgraPixel pixel = {(uint8_t)(b.color[0]), (uint8_t)(b.color[1]), (uint8_t)(b.color[2]), 255};
                (*buffer)[k] = pixel;
            }
        }
    }
    const uint8_t *buf = (uint8_t *)(void *)k4a_image_get_buffer(transformed_color_image);
    int len = static_cast<unsigned int>(k4a_image_get_size(transformed_color_image));
    uint8_t *bufcpy = new uint8_t[len];
    memcpy(bufcpy, buf, len);
    viewer::BgraPixel *pixels = reinterpret_cast<viewer::BgraPixel *>(bufcpy);

    for (int i = 0; i < num_points; i++)
    {
        float x = cloud->points(0, i) * 1000.f;
        float y = cloud->points(1, i) * 1000.f;
        float z = cloud->points(2, i) * 1000.f;
        k4a_float3_t pt3d = {x, y, z};
        k4a_float2_t pt2d;
        int valid = 0;

        k4a_calibration_3d_to_2d(&sensor_calibration,
                                 &pt3d,
                                 K4A_CALIBRATION_TYPE_DEPTH,
                                 K4A_CALIBRATION_TYPE_DEPTH,
                                 &pt2d,
                                 &valid);
        if (valid == 1)
        {
            int k = (int)pt2d.v[1] * width + (int)pt2d.v[0];
            if (k < width * height)
            {

                BgraPixel pixel = {(uint8_t)(cloud->colors(0, i) * 255.f), (uint8_t)(cloud->colors(1, i) * 255.f), (uint8_t)(cloud->colors(2, i) * 255.f), 255};
                (*buffer)[k] = pixel;
                // for (vector<float> label_color : label_colors)
                // {
                //     // printf(" label color %f %f %f \n", label_color[0], label_color[1], label_color[2]);
                //     // printf(" cloud color %f %f %f \n", cloud->colors(0, i) * 255.f, cloud->colors(1, i) * 255.f, cloud->colors(2, i) * 255.f);
                //     if (abs(label_color[0] - cloud->colors(0, i) * 255.f) < 1.f && abs(label_color[1] - cloud->colors(1, i) * 255.f) < 1.f && abs(label_color[2] - cloud->colors(2, i) * 255.f) < 1.f)
                //     {
                //         BgraPixel pixel = {(uint8_t)(cloud->colors(0, i) * 255.f), (uint8_t)(cloud->colors(1, i) * 255.f), (uint8_t)(cloud->colors(2, i) * 255.f), 255};
                //         if (k < len)
                //         {
                //             (*buffer)[k] = pixels[k];
                //             //(*buffer)[k] = pixel;
                //         }
                //     }
                // }
            }
            else
            {
                printf("out of buffer %f, %f\n", pt2d.v[0], pt2d.v[1]);
            }
        }
    }
}
