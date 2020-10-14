// TODO keep original and move custom to different file
// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef K4AIMGUI_ALL_H
#define K4AIMGUI_ALL_H

// Some of the dear imgui implementation headers must be included in a specific order, which can
// lead to really odd build breaks if you include one of those headers to get something out of it
// without including the rest in a header whose includes don't already include all of those
// dependencies from some other include they already have.
//
// This file includes all dear imgui-related headers that have ordering constraints; if you need
// access to symbols from any of these headers, include this header rather than directly including
// those headers.
//

// Some of these headers end up including windows.h, which defines the min/max macros, which conflict
// with std::min and std::max, which we're using because they're portable.  This macro modifies the
// behavior of windows.h to not define those macros so we can avoid the conflict.
//
#define NOMINMAX

// Clang parses doxygen-style comments in your source and checks for doxygen syntax errors.
// Unfortunately, some of our external dependencies have doxygen syntax errors in their
// headers and clang looks at them when we include them here, so we need to shut off those
// warnings on these headers.
//
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#pragma clang diagnostic ignored "-Wdocumentation-unknown-command"
#endif

#include <GL/gl3w.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <vector>
#include <string>
using namespace std;

#ifdef __clang__
#pragma clang diagnostic pop
#endif

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
