#include "rendor.h"
using namespace kdviewer;

Rendor &Rendor::Instance()
{
    static Rendor viewerWindow;
    return viewerWindow;
}

void Rendor::Initialize(const char *windowTitle,
                        int defaultWidth,
                        int defaultHeight,
                        GLFWkeyfun keyStrokeHandler,
                        RunTimeConfig *runtimeConfig,
                        k4a_device_configuration_t kinect_config)
{
    ViewerWindow::Initialize(windowTitle, defaultWidth, defaultHeight);
    INFO("initialized kinect viewer");
    this->runtimeConfig = runtimeConfig;
    glfwSetKeyCallback(m_window, keyStrokeHandler);
    ComputeDimensions();
}

void Rendor::ComputeDimensions()
{

    colorWindowWidth = (this->runtimeConfig->show_depth_image == true) ? GetWidth() / 2 : GetWidth();
    windowSize = ImVec2(GetWidth(), static_cast<float>(GetHeight()));
    colorWindowSize = ImVec2(colorWindowWidth, static_cast<float>(GetHeight()));
    depthWindowWidth = (this->runtimeConfig->show_depth_image == true) ? GetWidth() / 2 : GetWidth();
    depthWindowSize = ImVec2(depthWindowWidth, static_cast<float>(GetHeight()));
    colorWindowOrigin = (this->runtimeConfig->show_depth_image == true) ? GetWidth() / 2.f : 0.f;
}

float Rendor::GetColorWindowOrigin()
{
    return colorWindowOrigin;
}
ImVec2 Rendor::GetColorWindowSize()
{
    return colorWindowSize;
}


void Rendor::ShowTextures(const Texture &colorTexture,
                          const Texture &depthTexture, 
                          Kinector *kinector, 
                          AbstractScene *scene, 
                          bool showDepthImage)
{
    ImGui::SetNextWindowPos(ImVec2(colorWindowOrigin, 0.f));
    ImGui::SetNextWindowSize(colorWindowSize);

    if (ImGui::Begin("Color", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse))
    {
        ImVec2 imageSize(static_cast<float>(colorTexture.Width()), static_cast<float>(colorTexture.Height()));
        ImVec2 imageMaxSize = colorWindowSize;
        imageMaxSize.y -= GetTitleBarHeight();

        imageSize = GetMaxImageSize(imageSize, imageMaxSize);

        ImGui::Image(reinterpret_cast<ImTextureID>(static_cast<intptr_t>(colorTexture.Name())), imageSize);

        scene->annotate(ImGui::GetWindowDrawList(), kinector->GetBodyIds(), GetTitleBarHeight());
    }
    ImGui::End();

    if (showDepthImage == true)
    {
        ViewerWindow::ShowTexture("Depth", depthTexture, ImVec2(0.f, 0.f), depthWindowSize);
    }
}


// void KinectDemoViewer::ColorizeClusteredDepthImage(const k4a::image &depthImage,
//                                                    k4a_image_t transformed_color_image,
//                                                    std::vector<BgraPixel> *buffer,
//                                                    cilantro::PointCloud3f *cloud,
//                                                    vector<Detection> detections,
//                                                    k4a_calibration_t sensor_calibration)
// {
//     // This function assumes that the image is made of depth pixels (i.e. uint16_t's),
//     // which is only true for IR/depth images.
//     //
//     const k4a_image_format_t imageFormat = depthImage.get_format();
//     if (imageFormat != K4A_IMAGE_FORMAT_DEPTH16 && imageFormat != K4A_IMAGE_FORMAT_IR16)
//     {
//         throw std::logic_error("Attempted to colorize a non-depth image!");
//     }

//     const int width = depthImage.get_width_pixels();
//     const int height = depthImage.get_height_pixels();
//     int w = k4a_image_get_width_pixels(transformed_color_image);
//     int h = k4a_image_get_height_pixels(transformed_color_image);

//     int num_points = cloud->size();
//     vector<vector<float>> label_colors;
//     for (Detection detection : detections)
//     {
//         BoundingCube b = detection.cube;
//         label_colors.push_back(b.color);
//         Ray ray = detection.ray;
//         if (b.pointer.size() == 0)
//         {
//             continue;
//         }

//         // k4a_float3_t pt1 = {b.u[0], b.u[1], b.u[2]};
//         // k4a_float3_t pt2 = {b.v[0], b.v[1], b.v[2]};
//         k4a_float3_t pt1 = b.pointer[0];
//         k4a_float3_t pt2 = b.pointer[1];
//         k4a_float2_t pt1_2d, pt2_2d;
//         int vld1, vld2;
//         k4a_calibration_3d_to_2d(&sensor_calibration,
//                                  &pt1,
//                                  K4A_CALIBRATION_TYPE_DEPTH,
//                                  K4A_CALIBRATION_TYPE_DEPTH,
//                                  &pt1_2d,
//                                  &vld1);
//         k4a_calibration_3d_to_2d(&sensor_calibration,
//                                  &pt2,
//                                  K4A_CALIBRATION_TYPE_DEPTH,
//                                  K4A_CALIBRATION_TYPE_DEPTH,
//                                  &pt2_2d,
//                                  &vld2);
//         if (vld1 == 1 && vld2 == 1)
//         {

//             int x1 = (int)pt1_2d.v[0];
//             int y1 = (int)pt1_2d.v[1];
//             int x2 = (int)pt2_2d.v[0];
//             int y2 = (int)pt2_2d.v[1];
//             float m = (float)(y2 - y1) / (float)(x2 - x1);
//             // printf("drawing line %f \n", m);
//             for (int x = x1; x < x2; x++)
//             {
//                 int y = y1 + (int)(m * (float)(x - x1));
//                 int k = y * width + x;
//                 BgraPixel pixel = {(uint8_t)(b.color[0]), (uint8_t)(b.color[1]), (uint8_t)(b.color[2]), 255};
//                 (*buffer)[k] = pixel;
//             }
//         }
//     }
//     const uint8_t *buf = (uint8_t *)(void *)k4a_image_get_buffer(transformed_color_image);
//     int len = static_cast<unsigned int>(k4a_image_get_size(transformed_color_image));
//     uint8_t *bufcpy = new uint8_t[len];
//     memcpy(bufcpy, buf, len);
//     viewer::BgraPixel *pixels = reinterpret_cast<viewer::BgraPixel *>(bufcpy);

//     for (int i = 0; i < num_points; i++)
//     {
//         float x = cloud->points(0, i) * 1000.f;
//         float y = cloud->points(1, i) * 1000.f;
//         float z = cloud->points(2, i) * 1000.f;
//         k4a_float3_t pt3d = {x, y, z};
//         k4a_float2_t pt2d;
//         int valid = 0;

//         k4a_calibration_3d_to_2d(&sensor_calibration,
//                                  &pt3d,
//                                  K4A_CALIBRATION_TYPE_DEPTH,
//                                  K4A_CALIBRATION_TYPE_DEPTH,
//                                  &pt2d,
//                                  &valid);
//         if (valid == 1)
//         {
//             int k = (int)pt2d.v[1] * width + (int)pt2d.v[0];
//             if (k < width * height)
//             {

//                 BgraPixel pixel = {(uint8_t)(cloud->colors(0, i) * 255.f), (uint8_t)(cloud->colors(1, i) * 255.f), (uint8_t)(cloud->colors(2, i) * 255.f), 255};
//                 (*buffer)[k] = pixel;
//                 // for (vector<float> label_color : label_colors)
//                 // {
//                 //     // printf(" label color %f %f %f \n", label_color[0], label_color[1], label_color[2]);
//                 //     // printf(" cloud color %f %f %f \n", cloud->colors(0, i) * 255.f, cloud->colors(1, i) * 255.f, cloud->colors(2, i) * 255.f);
//                 //     if (abs(label_color[0] - cloud->colors(0, i) * 255.f) < 1.f && abs(label_color[1] - cloud->colors(1, i) * 255.f) < 1.f && abs(label_color[2] - cloud->colors(2, i) * 255.f) < 1.f)
//                 //     {
//                 //         BgraPixel pixel = {(uint8_t)(cloud->colors(0, i) * 255.f), (uint8_t)(cloud->colors(1, i) * 255.f), (uint8_t)(cloud->colors(2, i) * 255.f), 255};
//                 //         if (k < len)
//                 //         {
//                 //             (*buffer)[k] = pixels[k];
//                 //             //(*buffer)[k] = pixel;
//                 //         }
//                 //     }
//                 // }
//             }
//             else
//             {
//                 printf("out of buffer %f, %f\n", pt2d.v[0], pt2d.v[1]);
//             }
//         }
//     }
// }
