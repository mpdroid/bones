#ifndef __WIDGET_H
#define __WIDGET_H
#include <imgui.h>
#include <vector>
#include <string>
using namespace std;

struct Widget
{
    int frameNumber;
    int boxType;
    int bodyId;
    std::string name;
    ImVec2 topLeft;
    ImVec2 bottomRight;
    vector<ImVec2> letterPath;
    vector<ImVec2> arrow;
    int alpha;
    vector<float> color;
};

struct PointerWidget : Widget {
    vector<ImVec2> arrow;    
};

struct CubeWidget : Widget
{
    std::string name;
    ImVec2 ftl; // front-top-left
    ImVec2 ftr;
    ImVec2 fbr;
    ImVec2 fbl;
    ImVec2 btl;
    ImVec2 btr;
    ImVec2 bbr;
    ImVec2 bbl;
};

struct LetterWidget : Widget
{
    vector<ImVec2> letterPath;
};

struct JointInfo
{
    vector<ImVec2> x_direction;
    vector<ImVec2> y_direction;
    vector<ImVec2> z_direction;
    vector<float> depthCoordinates;
    vector<float> orientation;
    vector<float> imageCoordinates;
    ImVec2 textCoordinates;
    k4abt_joint_id_t jointId;
};

struct JointWidget : Widget
{
    vector<JointInfo> joints;
};

struct AxisWidget : Widget
{
    ImVec2 textCoordinates;
    vector<float> textShadowColor;
    vector<ImVec2> x;
    vector<ImVec2> y;
    vector<ImVec2> z;
};

#endif