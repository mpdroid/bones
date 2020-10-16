#ifndef __SCENE_H
#define __SCENE_H
#include <iostream>
#include <vector>
#include <algorithm>
#include "k4aimgui_all.h"
#include "common.h"
#include "euclid.h"
#include "widget.h"
using namespace std;
#include "kinector.h"
using namespace kinector;

namespace scene
{
    class AbstractScene
    {
    public:
        static AbstractScene *getInstance(char demo_mode);
        virtual void onLoopStart(int frame_number){};
        virtual void capture(Kinector *kinector, Euclid *euclid, int frame_number){};
        virtual void render(ImDrawList *drawList, vector<int> bodies, float y_shift){};
        virtual void onLoopEnd(){};
    };

    class LightSaberScene : public AbstractScene
    {
    public:
        LightSaberScene();
        ~LightSaberScene();
        void onLoopStart(int frame_number);
        void capture(Kinector *kinector, Euclid *euclid, int frame_number);
        void render(ImDrawList *drawList, vector<int> bodies, float y_shift);
        void onLoopEnd();

    private:
        vector<PointerWidget> lightSabers;
    };

    class JointInfoScene : public AbstractScene
    {
    public:
        JointInfoScene();
        ~JointInfoScene();
        void onLoopStart(int frame_number);
        void capture(Kinector *kinector, Euclid *euclid, int frame_number);
        void render(ImDrawList *drawList, vector<int> bodies, float y_shift);
        void onLoopEnd();

    private:
        vector<AxisWidget> axisWidgets;
        vector<JointWidget> jointWidgets;
    };

    class WriteAirScene : public AbstractScene
    {
    public:
        WriteAirScene();
        ~WriteAirScene();
        void onLoopStart(int frame_number);
        void capture(Kinector *kinector, Euclid *euclid, int frame_number);
        void render(ImDrawList *drawList, vector<int> bodies, float y_shift);
        void onLoopEnd();

    private:
        vector<LetterWidget> letterWidgets;
        LetterWidget currentLetter;
        int writeMode = 0;
    };

} // namespace scene

#endif