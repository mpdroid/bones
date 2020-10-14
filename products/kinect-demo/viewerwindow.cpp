//TODO Extend original instead of modifying

// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "viewerwindow.h"

#include <iostream>

#include "viewerutil.h"
using namespace std;
#include <algorithm>
#include <math.h>

using namespace viewer;

namespace
{

    // A callback that we give to OpenGL so we can get notified about any errors that might
    // occur when they happen.  Notifications relevant to performance tuning also come in
    // through this callback.
    //
    void APIENTRY glDebugOutput(GLenum source,
                                GLenum type,
                                GLuint id,
                                GLenum severity,
                                GLsizei length,
                                const GLchar *message,
                                void *userParam)
    {
        (void)userParam;

        // Some of the performance messages are a bit noisy, so we want to drop those
        // to reduce noise in the log.
        //
        constexpr GLuint noisyMessages[] = {
            131185, // Event that says a texture was loaded into memory
            131169, // Event that says a buffer was allocated
        };

        for (GLuint noisyMessageId : noisyMessages)
        {
            if (id == noisyMessageId)
            {
                return;
            }
        }

        std::cerr << "OpenGL debug message:" << std::endl
                  << "  source: " << source << std::endl
                  << "  type:   " << type << std::endl
                  << "  id:     " << id << std::endl
                  << "  sev:    " << severity << std::endl
                  << "  len:    " << length << std::endl
                  << "  msg:    " << message << std::endl
                  << "---------------------------" << std::endl;
    }
} // namespace

ViewerWindow &ViewerWindow::Instance()
{
    static ViewerWindow viewerWindow;
    return viewerWindow;
}

void ViewerWindow::Initialize(const char *windowTitle, int defaultWidth, int defaultHeight)
{
    std::cout << "Started initializing OpenGL..." << std::endl;

    if (m_window != nullptr)
    {
        throw std::logic_error("Attempted to double-initialize the window!");
    }

    if (!glfwInit())
    {
        throw std::runtime_error("Failed to initialize GLFW!");
    }

    m_windowWidth = defaultWidth;
    m_windowHeight = defaultHeight;

    // Start the window
    //
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_DECORATED, false);
    m_window = glfwCreateWindow(m_windowWidth, m_windowHeight, windowTitle, nullptr, nullptr);

    glfwMakeContextCurrent(m_window);

    // Enable vsync (cap framerate at the display's refresh rate)
    //
    glfwSwapInterval(1);

    // Initialize OpenGL
    //
    if (gl3wInit())
    {
        throw std::runtime_error("Failed to initialize OpenGL!");
    }

    // Turn on OpenGL debugging.  While not strictly necessary, this makes it much easier
    // to track down OpenGL errors when they occur.
    //
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);
    glEnable(GL_DEBUG_OUTPUT);
    glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
    glDebugMessageCallback(glDebugOutput, nullptr);
    glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_TRUE);

    // Initialize ImGui
    //
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(m_window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    // ImGui style settings
    //
    ImGui::StyleColorsDark();
    ImGui::GetStyle().WindowRounding = 0.0f;

    // By default, ImGui tries to save the previous window layout to disk.
    // That doesn't really make sense for this application, so we want to
    // disable saving the window layout.
    //
    ImGui::GetIO().IniFilename = nullptr;

    CheckOpenGLErrors();

    std::cout << "Finished initializing OpenGL." << std::endl;
}

ViewerWindow::~ViewerWindow()
{
    // ImGui will assert if we throw without having called Render, which
    // obscures the actual error message, so we call it here just in case.
    //
    ImGui::Render();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(m_window);
    glfwTerminate();
}

bool ViewerWindow::BeginFrame()
{
    if (m_window == nullptr)
    {
        // You need to call ViewerWindow::Initialize first.
        //
        throw std::logic_error("Attempted to use uninitialized window!");
    }

    // glfwWindowShouldClose will start returning true when the user
    // clicks the close button on the title bar.
    //
    if (glfwWindowShouldClose(m_window))
    {
        return false;
    }

    glfwPollEvents();

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    return true;
}

void ViewerWindow::EndFrame()
{
    ImGui::Render();

    glfwMakeContextCurrent(m_window);
    glfwGetFramebufferSize(m_window, &m_windowWidth, &m_windowHeight);
    glViewport(0, 0, m_windowWidth, m_windowHeight);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(m_window);
    glfwPollEvents();

    CheckOpenGLErrors();
}

Texture ViewerWindow::CreateTexture(int width, int height)
{
    return Texture(width, height);
}

Texture ViewerWindow::CreateTexture(std::pair<int, int> dimensions)
{
    return Texture(dimensions.first, dimensions.second);
}

void ViewerWindow::ShowTexture(const char *name, const Texture &texture, const ImVec2 &position, const ImVec2 &maxSize)
{
    ImGui::SetNextWindowPos(position);
    ImGui::SetNextWindowSize(maxSize);

    if (ImGui::Begin(name, nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse))
    {
        // Figure out how big we can make the image
        //
        ImVec2 imageSize(static_cast<float>(texture.Width()), static_cast<float>(texture.Height()));
        ImVec2 imageMaxSize = maxSize;
        imageMaxSize.y -= GetTitleBarHeight();

        imageSize = GetMaxImageSize(imageSize, imageMaxSize);

        ImGui::Image(reinterpret_cast<ImTextureID>(static_cast<intptr_t>(texture.Name())), imageSize);
    }
    ImGui::End();
}

float getBezierValue(float n1, float n2, float perc)
{
    float diff = n2 - n1;

    return n1 + (diff * perc);
}

void ViewerWindow::ShowTextureWithPersistentBoundingBoxes(const char *name, const Texture &texture, const ImVec2 &position, const ImVec2 &maxSize,
                                                          vector<PointerWidget> *lightSabers,
                                                          vector<PointerWidget> *objectPointers,
                                                          vector<AxisWidget> *axes,
                                                          vector<JointWidget> *jointWidgets,
                                                          vector<LetterWidget> *letterWidgets,
                                                          vector<CubeWidget> *cubeWidgets,
                                                          vector<int> bodies, int currentFrameNumber,
                                                          int frameInterval)
{
    ImGui::SetNextWindowPos(position);
    ImGui::SetNextWindowSize(maxSize);

    if (ImGui::Begin(name, nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse))
    {
        // Figure out how big we can make the image
        //
        ImVec2 imageSize(static_cast<float>(texture.Width()), static_cast<float>(texture.Height()));
        ImVec2 imageMaxSize = maxSize;
        imageMaxSize.y -= GetTitleBarHeight();

        imageSize = GetMaxImageSize(imageSize, imageMaxSize);

        vector<ImU32> colors = {IM_COL32(255, 69, 0, 255), IM_COL32(128, 255, 0, 255)};

        ImGui::Image(reinterpret_cast<ImTextureID>(static_cast<intptr_t>(texture.Name())), imageSize);

        if (letterWidgets->size() > 0)
        {
            for (int i = 0; i < letterWidgets->size(); i++)
            {
                if ((*letterWidgets)[i].letterPath.size() < 3)
                    continue;
                for (int k = 0; k < (*letterWidgets)[i].letterPath.size() - 2; k += 2)
                {
                    ImVec2 first = (*letterWidgets)[i].letterPath[k];
                    ImVec2 second = (*letterWidgets)[i].letterPath[k + 1];
                    ImVec2 third = (*letterWidgets)[i].letterPath[k + 2];
                    for (float bez = 0; bez < 1; bez += 0.01)
                    {
                        float xa = getBezierValue(first.x, second.x, bez);
                        float ya = getBezierValue(first.y, second.y, bez);
                        float xb = getBezierValue(second.x, third.x, bez);
                        float yb = getBezierValue(second.y, third.y, bez);

                        float xmid = getBezierValue(xa, xb, bez);
                        float ymid = getBezierValue(ya, yb, bez);
                        ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(xmid, ymid), 5, IM_COL32(255, 69, 0, 255), 20.0f);
                    }
                }
            }
        }
        if (jointWidgets->size() > 0)
        {
            for (int i = 0; i < jointWidgets->size(); i++)
            {
                JointWidget widget = (*jointWidgets)[i];
                for (JointInfo joint : widget.joints)
                {
                    auto color = IM_COL32(255, 255, 255, 255);
                    auto shadowColor = IM_COL32(0, 255, 0, 255);
                    float fontSize = ((float)maxSize.y) * 15.f / 720.f;
                    float shadowFontSize = ((float)maxSize.y) * 15.02f / 720.f;

                    ImVec2 textStart(joint.textCoordinates.x + 5.f, joint.textCoordinates.y + 2.f);
                    textStart.y += GetTitleBarHeight();
                    ImVec2 shadowTextStart(joint.textCoordinates.x + 3.f, joint.textCoordinates.y + 0.f);

                    ostringstream depthStream, orientStream;
                    depthStream << "Depth: " << (int)joint.depthCoordinates[0] << ", " << (int)joint.depthCoordinates[1] << ", " << (int)joint.depthCoordinates[2];
                    string depth_string = depthStream.str();
                    ImGui::GetWindowDrawList()->AddText(NULL, fontSize, textStart, color, depth_string.c_str());

                    ImVec2 textStartOrientation(joint.textCoordinates.x + 5.f, joint.textCoordinates.y + 18.f);
                    textStartOrientation.y += GetTitleBarHeight();
                    orientStream.precision(2);
                    orientStream << "Orientation: " << joint.orientation[0]
                                 << ", " << joint.orientation[1]
                                 << ", " << joint.orientation[2]
                                 << ", " << joint.orientation[3];
                    string orientString = orientStream.str();
                    ImGui::GetWindowDrawList()->AddText(NULL, fontSize, textStartOrientation, color, orientString.c_str());

                    ImVec2 start = joint.x_direction[0];
                    start.y += GetTitleBarHeight();
                    ImVec2 end = joint.x_direction[1];
                    end.y += GetTitleBarHeight();
                    ImGui::GetWindowDrawList()->AddLine(start, end, IM_COL32(255, 0, 0, 255), 4.0f);

                    ImVec2 starty = joint.y_direction[0];
                    starty.y += GetTitleBarHeight();
                    ImVec2 endy = joint.y_direction[1];
                    endy.y += GetTitleBarHeight();
                    ImGui::GetWindowDrawList()->AddLine(starty, endy, IM_COL32(0, 255, 0, 255), 4.0f);

                    ImVec2 startz = joint.z_direction[0];
                    startz.y += GetTitleBarHeight();
                    ImVec2 endz = joint.z_direction[1];
                    endz.y += GetTitleBarHeight();
                    ImGui::GetWindowDrawList()->AddLine(startz, endz, IM_COL32(0, 0, 255, 255), 4.0f);
                }
            }
        }

        if (cubeWidgets->size() > 0)
        {
            for (int i = 0; i < cubeWidgets->size(); i++)
            {
                CubeWidget widget = (*cubeWidgets)[i];
                widget.ftl.y += GetTitleBarHeight();
                widget.ftr.y += GetTitleBarHeight();
                widget.fbr.y += GetTitleBarHeight();
                widget.fbl.y += GetTitleBarHeight();
                widget.btl.y += GetTitleBarHeight();
                widget.btr.y += GetTitleBarHeight();
                widget.bbr.y += GetTitleBarHeight();
                widget.bbl.y += GetTitleBarHeight();

                int frameDiff = currentFrameNumber - widget.frameNumber;
                int boxDiff = currentFrameNumber - widget.frameNumber;
                float alpha = max(55, 200 - 10 * (int)((float)frameDiff));
                float boxalpha = max(0, 255 - (int)((float)frameDiff));
                if (frameDiff <= 20 * frameInterval)
                {
                    auto color = widget.color;

                    if (widget.name.length() > 0)
                    {
                        ImVec2 textStart(widget.ftl.x + 5.f, widget.ftl.y - 50.f);

                        float fontSize = ((float)maxSize.y) * 20.f / 720.f;
                        ImGui::GetWindowDrawList()->AddText(NULL, fontSize, textStart, IM_COL32(255, 255, 255, boxalpha), widget.name.c_str());
                    }

                    ImVec2 lines[4] = {widget.ftl, widget.ftr, widget.fbr, widget.fbl};

                    ImGui::GetWindowDrawList()->AddPolyline(lines, 4,
                                                            IM_COL32(255, 255, 255, boxalpha), true, 2.f);
                    ImVec2 lines_back[4] = {widget.btl, widget.btr, widget.bbr, widget.bbl};
                    ImGui::GetWindowDrawList()->AddPolyline(lines_back, 4,
                                                            IM_COL32(255, 255, 255, boxalpha), true, 2.f);
                    ImVec2 lines_left[4] = {widget.ftl, widget.btl, widget.bbl, widget.fbl};
                    ImGui::GetWindowDrawList()->AddPolyline(lines_left, 4,
                                                            IM_COL32(255, 255, 255, boxalpha), true, 2.f);
                    ImVec2 lines_right[4] = {widget.ftr, widget.btr, widget.bbr, widget.fbr};
                    ImGui::GetWindowDrawList()->AddPolyline(lines_right, 4,
                                                            IM_COL32(255, 255, 255, boxalpha), true, 2.f);
                }
            }
        }

        if (axes->size() > 0)
        {
            for (int i = 0; i < axes->size(); i++)
            {
                AxisWidget axis = (*axes)[i];
                ImVec2 textStart(axis.textCoordinates.x + 5.f, axis.textCoordinates.y + 2.f);
                ImVec2 shadowTextStart(axis.textCoordinates.x + 3.f, axis.textCoordinates.y + 0.f);
                auto color = IM_COL32(255, 255, 255, 255);
                auto shadowColor = IM_COL32(axis.textShadowColor[0], axis.textShadowColor[1], axis.textShadowColor[2], 255);
                float fontSize = ((float)maxSize.y) * 20.f / 720.f;
                float shadowFontSize = ((float)maxSize.y) * 20.05f / 720.f;
                // ImGui::GetWindowDrawList()->AddText(NULL, shadowFontSize, shadowTextStart, shadowColor, axis.name.c_str());
                ImGui::GetWindowDrawList()->AddText(NULL, fontSize, textStart, color, axis.name.c_str());
                ImVec2 start = axis.x[0];
                start.y += GetTitleBarHeight();
                ImVec2 end = axis.x[1];
                end.y += GetTitleBarHeight();
                ImGui::GetWindowDrawList()->AddLine(start, end, IM_COL32(255, 0, 0, 255), 3.0f);
                // ImVec2 xlabel_s(end.x + 3.f, end.y - 18.f);
                // ImGui::GetWindowDrawList()->AddText(NULL, shadowFontSize, xlabel_s, shadowColor, "+x");
                ImVec2 xlabel(end.x + 5.f, end.y - 20.f);
                ImGui::GetWindowDrawList()->AddText(NULL, fontSize, xlabel, IM_COL32(255, 0, 0, 255), "+X");

                ImVec2 starty = axis.y[0];
                starty.y += GetTitleBarHeight();
                ImVec2 endy = axis.y[1];
                endy.y += GetTitleBarHeight();
                ImGui::GetWindowDrawList()->AddLine(starty, endy, IM_COL32(0, 255, 0, 255), 3.0f);
                // ImVec2 ylabel_s(endy.x - 20.f, endy.y + 3.f);
                // ImGui::GetWindowDrawList()->AddText(NULL, shadowFontSize, ylabel_s, shadowColor, "+y");
                ImVec2 ylabel(endy.x - 20.f, endy.y + 5.f);
                ImGui::GetWindowDrawList()->AddText(NULL, fontSize, ylabel, IM_COL32(0, 255, 0, 255), "+Y");

                if (axis.z.size() > 0)
                {
                    ImVec2 startz = axis.z[0];
                    startz.y += GetTitleBarHeight();
                    ImVec2 endz = axis.z[1];
                    endz.y += GetTitleBarHeight();
                    ImGui::GetWindowDrawList()->AddLine(startz, endz, IM_COL32(0, 0, 255, 255), 3.0f);
                    // ImVec2 zlabel_s(endz.x + 3.f, endz.y + 3.f);
                    // ImGui::GetWindowDrawList()->AddText(NULL, shadowFontSize, zlabel_s, shadowColor, "+z");
                    ImVec2 zlabel(endz.x + 5.f, endz.y + 5.f);
                    ImGui::GetWindowDrawList()->AddText(NULL, fontSize, zlabel, IM_COL32(0, 0, 255, 255), "+Z");
                }
            }
        }
        if (lightSabers->size() > 0)
        {
            map<int, vector<PointerWidget>> lightSabersByBody;
            for (int bodyId : bodies)
            {
                vector<PointerWidget> sabers;
                for (int i = 0; i < lightSabers->size(); i++)
                {
                    if ((*lightSabers)[i].bodyId == bodyId)
                    {
                        sabers.push_back((*lightSabers)[i]);
                    }
                }
                lightSabersByBody.insert(pair<int, vector<PointerWidget>>(bodyId, sabers));
            }
            for (int bodyId : bodies)
            {
                vector<PointerWidget> sabers = lightSabersByBody[bodyId];

                int n = sabers.size();
                float b = pow(200.f, 1.f / (float)n);
                for (int i = 0; i < n; i++)
                {
                    int alpha = (int)(pow(b, i) - 1.f);
                    vector<ImU32> colors = {IM_COL32(255, 69, 0, alpha), IM_COL32(128, 255, 0, alpha)};
                    // cout << " num vectors for saber " << i << " " << sabers[i].directionVectors.size() << alpha << endl;
                    for (int j = 0; j < sabers[i].arrow.size(); j++)
                    {
                        sabers[i].arrow[j].y += GetTitleBarHeight();
                    }
                    vector<ImVec2> directionVectors = sabers[i].arrow;
                    for (int j = 0; j < (int)directionVectors.size() - 1; j += 2)
                    {
                        ImVec2 start = directionVectors[j];
                        ImVec2 end = directionVectors[j + 1];
                        ImGui::GetWindowDrawList()->AddLine(start, end, colors[bodyId], 20.0f);
                    }
                }
            }
        }

        if (objectPointers->size() > 0)
        {

            for (int i = 0; i < objectPointers->size(); i++)
            {
                vector<ImU32> colors = {IM_COL32(255, 69, 0, 255), IM_COL32(128, 255, 0, 255)};
                for (int j = 0; j < (*objectPointers)[i].arrow.size(); j++)
                {
                    (*objectPointers)[i].arrow[j].y += GetTitleBarHeight();
                }
                vector<ImVec2> arrow = (*objectPointers)[i].arrow;
                for (int k = 0; k < (int)arrow.size() - 1; k += 2)
                {
                    ImVec2 start = arrow[k];
                    ImVec2 end = arrow[k + 1];
                    // int bodyId = bodies[i / 2] % colors.size();
                    ImGui::GetWindowDrawList()->AddLine(start, end, IM_COL32(255, 255, 255, 255), 5.0f);
                }
            }
        }

   }
    ImGui::End();
}

void ViewerWindow::SetKeyCallBackHandler(GLFWkeyfun cbfun)
{
    glfwSetKeyCallback(m_window, cbfun);
}