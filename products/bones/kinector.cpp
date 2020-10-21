#include "kinector.h"
using namespace kinector;
using namespace viewer;

Kinector::Kinector(
    k4a_device_configuration_t *kinect_config,
    k4a::device *device)
{
    this->kinect_config = kinect_config;
    k4a_result_t result = k4a_device_get_calibration(device->handle(),
                                                     kinect_config->depth_mode,
                                                     kinect_config->color_resolution,
                                                     &sensor_calibration);
    if (result != K4A_RESULT_SUCCEEDED)
    {
        printf("depth camera calibration failed!\n");
        exit(1);
    }
    this->calibration = &sensor_calibration;
    transformation = k4a_transformation_create(calibration);

    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    result = k4abt_tracker_create(calibration, tracker_config, &tracker);
    if (result != K4A_RESULT_SUCCEEDED)
    {
        printf("tracker init failed!\n");
        exit(1);
    }

    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                     calibration->depth_camera_calibration.resolution_width,
                     calibration->depth_camera_calibration.resolution_height,
                     calibration->depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
                     &xy_table);

    createXYTable(xy_table);

    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                     calibration->depth_camera_calibration.resolution_width,
                     calibration->depth_camera_calibration.resolution_height,
                     calibration->depth_camera_calibration.resolution_width * (int)sizeof(k4a_float3_t),
                     &point_cloud);

    is_valid = true;
}

// Compares bodies based on height
bool compareBodies(k4abt_body_t first, k4abt_body_t second)
{
    float h1 = first.skeleton.joints[K4ABT_JOINT_HEAD].position.v[1];
    float h2 = second.skeleton.joints[K4ABT_JOINT_HEAD].position.v[1];
    return (h1 < h2);
}

void Kinector::initializeFrame(k4a::capture capture)
{
    is_valid = false;
    bodies.clear();
    bodyIds.clear();
    depthPixelBuffer.clear();
    depthImage = capture.get_depth_image();
    colorImage = capture.get_color_image();
    colorImageBuffer = colorImage.get_buffer();
    colorImageHeight = colorImage.get_height_pixels();
    colorImageWidth = colorImage.get_width_pixels();
    int len = static_cast<unsigned int>(k4a_image_get_size(colorImage.handle()));
    uint8_t *bufcpy = new uint8_t[len];
    memcpy(bufcpy, colorImageBuffer, len);
    cvImage = cv::Mat(
        colorImageHeight,
        colorImageWidth,
        CV_8UC4, (void *)bufcpy, cv::Mat::AUTO_STEP);
    pixels = reinterpret_cast<BgraPixel *>(colorImageBuffer);
    colorizePointCloud(
        depthImage.handle(),
        colorImage.handle(),
        &colorizedDepthImage);

    k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(
        tracker,
        capture.handle(),
        K4A_WAIT_INFINITE);
    if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
    {
        printf("Error! Add capture to tracker process queue timeout!\n");
        return;
    }
    else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
    {
        printf("Error! Add capture to tracker process queue failed!\n");
        return;
    }
    k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
    if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
    {
        int num_bodies = k4abt_frame_get_num_bodies(body_frame);
        for (int b = 0; b < (int)num_bodies; b++)
        {
            k4abt_body_t bd;
            VERIFY(k4abt_frame_get_body_skeleton(body_frame, b, &bd.skeleton),
                   "Get body from body frame failed!");
            bodies.push_back(bd);
        }
        sort(bodies.begin(), bodies.end(), compareBodies);
        for (int b = 0; b < (int)num_bodies; b++)
        {
            bodyIds.push_back(b);
            bodies[b].id = b;
        }

        is_valid = true;
    }
    else
    {
        printf("failed to pop body frame\n");
    }
}

bool Kinector::isValid()
{
    return is_valid;
}

k4a_calibration_t *Kinector::getCalibration()
{
    return calibration;
}

k4a_image_t Kinector::getXYTable()
{
    return xy_table;
}
BgraPixel *Kinector::getColorPixels()
{
    return pixels;
}

BgraPixel *Kinector::getDepthPixels()
{
    return &(depthPixelBuffer[0]);
}
int Kinector::getColorImageHeight()
{
    return colorImageHeight;
}

int Kinector::getColorImageWidth()
{
    return colorImageWidth;
}

k4a::image Kinector::getDepthImage()
{
    return depthImage;
}

int Kinector::getDepthImageHeight()
{
    return GetDepthDimensions(kinect_config->depth_mode).second;
}

int Kinector::getDepthImageWidth()
{
    return GetDepthDimensions(kinect_config->depth_mode).first;
}

cv::Mat Kinector::getCVImage()
{
    return cvImage;
}
k4a_image_t Kinector::getColorizedDepthImage()
{
    return colorizedDepthImage;
}

float Kinector::getColorWindowOrigin()
{
    return colorWindowOrigin;
}
void Kinector::setColorWindowOrigin(float origin)
{
    this->colorWindowOrigin = origin;
}
ImVec2 Kinector::getColorWindowSize()
{
    return colorWindowSize;
}
void Kinector::setColorWindowSize(ImVec2 size)
{
    this->colorWindowSize = size;
}

void Kinector::releaseFrame()
{
    if (is_valid)
    {
        if (bodyIds.size() > 0)
        {
            k4abt_frame_release(body_frame);
        }
        k4a_image_release(colorizedDepthImage);
    }
}
vector<k4abt_body_t> Kinector::getBodies()
{
    return bodies;
}
vector<int> Kinector::getBodyIds()
{
    return bodyIds;
}

void Kinector::createXYTable(k4a_image_t xy_table)
{
    k4a_float2_t *table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_table);

    int width = calibration->depth_camera_calibration.resolution_width;
    int height = calibration->depth_camera_calibration.resolution_height;

    int camerawidth = calibration->color_camera_calibration.resolution_width;
    int cameraheight = calibration->color_camera_calibration.resolution_height;
    TRACE("Depth camera size", width, " ", height);
    TRACE("Color camera size", camerawidth, " ", cameraheight);
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

int Kinector::generatePointCloud(
    Euclid *euclid,
    vector<Ray> rays,
    cilantro::VectorSet3f *cilantroPoints,
    cilantro::VectorSet3f *cilantroColors)
{
    int width = k4a_image_get_width_pixels(depthImage.handle());
    int height = k4a_image_get_height_pixels(depthImage.handle());
    uint16_t *depth_data = (uint16_t *)(void *)k4a_image_get_buffer(depthImage.handle());
    k4a_float2_t *xy_table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_table);
    const uint8_t *buf = (uint8_t *)(void *)k4a_image_get_buffer(colorizedDepthImage);

    // TODO 3 lines possibly remove
    int len = static_cast<unsigned int>(k4a_image_get_size(colorizedDepthImage));
    uint8_t *bufcpy = new uint8_t[len];
    memcpy(bufcpy, buf, len);

    viewer::BgraPixel *pixels = reinterpret_cast<viewer::BgraPixel *>(bufcpy);
    vector<k4a_float3_t> pointsOfInterest;
    vector<k4a_float3_t> colors;
    int total_point_count = 0;
    int filtered_point_count = 0;
    for (int i = 0; i < width * height; i++)
    {
        if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
        {
            total_point_count++;
            float x = (xy_table_data[i].xy.x * (float)depth_data[i]);
            float y = (xy_table_data[i].xy.y * (float)depth_data[i]);
            float z = (float)depth_data[i];
            k4a_float3_t point = {x, y, z};
            if (!euclid->isPointInFieldOfView(point, rays))
            {
                continue;
            }
            pointsOfInterest.push_back(point);
            colors.push_back({(float)pixels[i].Red, (float)pixels[i].Green, (float)pixels[i].Blue});
            filtered_point_count++;
        }
    }
    TRACE("selected points ", filtered_point_count, "/", total_point_count);
    if (filtered_point_count > 0)
    {

        cilantroPoints->resize(3, filtered_point_count);
        cilantroColors->resize(3, filtered_point_count);
        for (int i = 0; i < filtered_point_count; i++)
        {
            k4a_float3_t point = pointsOfInterest[i];
            k4a_float3_t color = colors[i];
            (*cilantroPoints)(0, i) = point.v[0] / 1000.f;
            (*cilantroPoints)(1, i) = point.v[1] / 1000.f;
            (*cilantroPoints)(2, i) = point.v[2] / 1000.f;
            (*cilantroColors)(0, i) = color.v[0] / 255.f;
            (*cilantroColors)(1, i) = color.v[1] / 255.f;
            (*cilantroColors)(2, i) = color.v[2] / 255.f;
        }
    }
    return filtered_point_count;
}

bool Kinector::colorizePointCloud(const k4a_image_t depth_image,
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
        ERROR("Failed to create transformed color image");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_color_image_to_depth_camera(transformation,
                                                                               depth_image,
                                                                               color_image,
                                                                               *transformed_color_image))
    {
        ERROR("Failed to compute transformed color image");
        return false;
    }
    return true;
}

// Given a depth image, output a BGRA-formatted color image into buffer, using
// expectedValueRange to define what min/max values the depth image should have.
// Low values are blue, high values are red.
//
void Kinector::colorizeDepthImage()
{
    std::pair<uint16_t, uint16_t> expectedValueRange = GetDepthModeRange(kinect_config->depth_mode);
    const k4a_image_format_t imageFormat = depthImage.get_format();
    if (imageFormat != K4A_IMAGE_FORMAT_DEPTH16 && imageFormat != K4A_IMAGE_FORMAT_IR16)

    {
        throw std::logic_error("Attempted to colorize a non-depth image!");
    }

    const int width = depthImage.get_width_pixels();
    const int height = depthImage.get_height_pixels();

    depthPixelBuffer.resize(static_cast<size_t>(width * height));

    const uint16_t *depthData = reinterpret_cast<const uint16_t *>(depthImage.get_buffer());
    for (int h = 0; h < height; ++h)
    {
        for (int w = 0; w < width; ++w)
        {
            const size_t currentPixel = static_cast<size_t>(h * width + w);
            depthPixelBuffer[currentPixel] = K4ADepthPixelColorizer::ColorizeBlueToRed(depthData[currentPixel],
                                                      expectedValueRange.first,
                                                      expectedValueRange.second);
        }
    }
}


void Kinector::colorizeFilteredDepthImage(
                                                  Euclid *euclid,
                                                  vector<Ray> rays)
{
    std::pair<uint16_t, uint16_t> expectedValueRange = GetDepthModeRange(kinect_config->depth_mode);
    const k4a_image_format_t imageFormat = depthImage.get_format();
    if (imageFormat != K4A_IMAGE_FORMAT_DEPTH16 && imageFormat != K4A_IMAGE_FORMAT_IR16)

    {
        throw std::logic_error("Attempted to colorize a non-depth image!");
    }

    const int width = depthImage.get_width_pixels();
    const int height = depthImage.get_height_pixels();

    depthPixelBuffer.resize(static_cast<size_t>(width * height));

    const uint16_t *depthData = reinterpret_cast<const uint16_t *>(depthImage.get_buffer());
    k4a_float2_t *xy_table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_table);

    const uint8_t *buf = (uint8_t *)(void *)k4a_image_get_buffer(colorizedDepthImage);
    int len = static_cast<unsigned int>(k4a_image_get_size(colorizedDepthImage));
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
                if (euclid->isPointInFieldOfView(point, rays))
                {
                    if (i >= len)
                    {
                        depthPixelBuffer[i] = K4ADepthPixelColorizer::ColorizeBlueToRed(depthData[i],
                                                       expectedValueRange.first,
                                                       expectedValueRange.second);
                    }
                    else
                    {
                        depthPixelBuffer[i] = pixels[i];
                    }
                }
                else
                {
                    depthPixelBuffer[i] = K4ADepthPixelColorizer::ColorizeBlueToRed(depthData[i],
                                                   expectedValueRange.first,
                                                   expectedValueRange.second);
                }
            }
        }
    }
}