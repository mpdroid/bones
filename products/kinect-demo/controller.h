#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <iostream>
#include <vector>
#include <thread>
#include <algorithm>
#include <string>
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
#include "rendor.h"
using namespace kdviewer;
#include "azure_vision_request.h"

#include "linmath.h"
using namespace linmath;
#include <k4abt.h>
#include "kinector.h"
using namespace kinector;

#include "scene/scene.h"
#include "scene/thingfinder.h"
using namespace scene;



static RunTimeConfig *RUNTIMECONFIG;

const auto FRAME_INTERVAL = 60;
// Mutex to acquire and release locks before performing synchronized ops
// TODO Remove if not needed
// std::mutex mtx;

namespace controller
{

    class Controller
    {
    public:
        static Controller *getInstance();
        int runLoop();

    private:
        static Controller *inst_;
        Controller() {}
        Controller(const Controller &);
        Controller &operator=(const Controller &);
        ~Controller();
        RunTimeConfig runtimeConfig;
        AbstractScene *scene;
    };
} // namespace controller

#endif