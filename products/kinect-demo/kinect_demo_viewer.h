#ifndef __VIEWER_H
#define __VIEWER_H

#include <iostream>
#include <vector>
#include <map>
#include <math.h>
#include "k4adepthpixelcolorizer.h"
#include "k4apixel.h"
#include "k4astaticimageproperties.h"
#include <k4abt.h>

#include "euclid.h"
#include "viewerutil.h"
using namespace std;
#include "viewerwindow.h"
using namespace viewer;
#include "common.h"
#include "scene/scene.h"
using namespace scene;
template <typename Base, typename T>
inline bool instanceof (const T *)
{
    return is_base_of<Base, T>::value;
}

namespace kdviewer
{
    class KinectDemoViewer : public ViewerWindow
    {
    public:
        static KinectDemoViewer &Instance();

        void Initialize(const char *windowTitle,
                        int defaultWidth,
                        int defaultHeight,
                        GLFWkeyfun keyHandler,
                        RunTimeConfig *runtimeConfig,
                        k4a_device_configuration_t kinect_config);

        void ComputeDimensions();

        void ShowDepthTexture(const Texture &texture);

        void ShowColorTexture(const Texture &texture);

        void ShowTextureWithPersistentBoundingBoxes(const Texture &texture,
                                                    AbstractScene *scene,
                                                    vector<int> bodies);

        void ColorizeDepthImage(const k4a::image &depthImage,
                                DepthPixelVisualizationFunction visualizationFn,
                                std::pair<uint16_t, uint16_t> expectedValueRange,
                                std::vector<BgraPixel> *buffer);

        void ColorizeFilteredDepthImage(const k4a::image &depthImage,
                                        k4a_image_t transformed_color_image,
                                        k4a_image_t xy_table,
                                        Euclid *euclid,
                                        vector<Ray> rays,
                                        DepthPixelVisualizationFunction visualizationFn,
                                        std::pair<uint16_t, uint16_t> expectedValueRange,
                                        std::vector<BgraPixel> *buffer);

        // void ColorizeClusteredDepthImage(const k4a::image &depthImage,
        //                                  k4a_image_t transformed_color_image,
        //                                  std::vector<BgraPixel> *buffer,
        //                                  cilantro::PointCloud3f *cloud,
        //                                  vector<Detection> detections,
        //                                  k4a_calibration_t sensor_calibration);

        float GetColorWindowOrigin();

        ImVec2 GetColorWindowSize();

    protected:
        KinectDemoViewer() = default;

    private:
        RunTimeConfig *runtimeConfig;

        int colorWindowWidth;
        ImVec2 windowSize;
        ImVec2 colorWindowSize;
        ImVec2 depthWindowSize;
        float colorWindowOrigin = 0.f;
        float depthWindowWidth;
    };
} // namespace kdviewer

#endif