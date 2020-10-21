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
    class Rendor : public ViewerWindow
    {
    public:
        static Rendor &Instance();

        void Initialize(const char *windowTitle,
                        int defaultWidth,
                        int defaultHeight,
                        GLFWkeyfun keyHandler,
                        RunTimeConfig *runtimeConfig,
                        k4a_device_configuration_t kinect_config);
        void showTextures(const Texture &colorTexture,
                          const Texture &depthTexture,
                          Kinector *kinector,
                          AbstractScene *scene,
                          bool showDepthImage);
        void computeDimensions();
        float getColorWindowOrigin();
        ImVec2 getColorWindowSize();

    protected:
        Rendor() = default;

    private:
        RunTimeConfig *runtimeConfig;
        int colorWindowWidth;
        ImVec2 windowSize;
        ImVec2 colorWindowSize;
        ImVec2 depthWindowSize;
        float colorWindowOrigin = 0.f;
        float depthWindowWidth;
        Texture *depthTexture;
        Texture *colorTexture;
    };
} // namespace kdviewer

#endif