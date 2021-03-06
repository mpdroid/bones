#ifndef __KINECTOR_H
#define __KINECTOR_H
#include <iostream>
#include <math.h>
using namespace std;
#include "k4adepthpixelcolorizer.h"
#include "k4apixel.h"
#include "k4astaticimageproperties.h"
#include "euclid.h"
#include <k4abt.h>
#include "k4apixel.h"
#include "common.h"
#include <opencv2/opencv.hpp>

namespace kinector
{
    class Kinector
    {
    public:
        Kinector(
            k4a_device_configuration_t *kinect_config,
            k4a::device *device);
        bool isValid();
        void createXYTable(k4a_image_t xy_table);
        void initializeFrame(k4a::capture capture);
        k4a_calibration_t *getCalibration();
        viewer::BgraPixel *getColorPixels();
        viewer::BgraPixel *getDepthPixels();
        cv::Mat getCVImage();
        k4a::image getDepthImage();
        int getDepthImageHeight();
        int getDepthImageWidth();
        int getColorImageHeight();
        int getColorImageWidth();
        k4a_image_t getColorizedDepthImage();
        k4a_image_t getXYTable();
        float getColorWindowOrigin();
        void setColorWindowOrigin(float origin);
        ImVec2 getColorWindowSize();
        void setColorWindowSize(ImVec2 size);

        vector<k4abt_body_t> getBodies();
        vector<int> getBodyIds();
        void releaseFrame();
        bool colorizePointCloud(
            const k4a_image_t depth_image,
            const k4a_image_t color_image,
            k4a_image_t *transformed_color_image);
        int generatePointCloud(Euclid *euclid,
                               vector<Ray> rays,
                               cilantro::VectorSet3f *cilantroPoints,
                               cilantro::VectorSet3f *cilantroColors);
        void colorizeDepthImage();
        void colorizeFilteredDepthImage(Euclid *euclid,
                                        vector<Ray> rays);

    private:
        Kinector();
        float colorWindowOrigin;
        ImVec2 colorWindowSize;
        k4a_transformation_t transformation;
        k4a_image_t xy_table = NULL;
        k4a_image_t point_cloud = NULL;
        k4a_device_configuration_t *kinect_config;
        k4a_calibration_t sensor_calibration;
        k4a_calibration_t *calibration;
        k4abt_tracker_t tracker = NULL;
        k4a::image depthImage;
        k4a::image colorImage;
        k4a_image_t colorizedDepthImage;
        int colorImageHeight;
        int colorImageWidth;
        uint8_t *colorImageBuffer;
        viewer::BgraPixel *pixels;
        vector<viewer::BgraPixel> depthPixelBuffer;
        k4abt_frame_t body_frame = NULL;
        cv::Mat cvImage;
        vector<int> bodyIds;
        vector<k4abt_body_t> bodies;
        bool is_valid = false;
    };
} // namespace kinector

#endif