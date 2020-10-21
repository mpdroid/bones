#ifndef __THINGFINDER_H
#define __THINGFINDER_H
#include <iostream>
#include "azure_vision_request.h"
#include "scene.h"
#include "kinector.h"
#include <thread>
using namespace kinector;
using namespace scene;
namespace scene {
    class ThingFinderScene : public AbstractScene
    {
    public:
        ThingFinderScene();
        ~ThingFinderScene();
        void onLoopStart(int frame_number);
        void comprehend(Kinector *kinector, int frame_number);
        void annotate(ImDrawList *drawList, vector<int> bodies, float y_shift);
        void onLoopEnd();
        vector<Ray> getRays();

    private:
        vector<Detection> detections;
        vector<CubeWidget> cubeWidgets;
        vector<PointerWidget> objectPointers;
        vector<Ray> rays;
        int currentFrameNumber = 0;
    };

}

#endif