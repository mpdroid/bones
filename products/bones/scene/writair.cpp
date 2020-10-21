#include "scene.h"
using namespace scene;
const auto WRITE_INTERVAL = 1;

/*****************************************************************/
/******** WRITING ON AIR *****************************************/
/*****************************************************************/
WriteAirScene::WriteAirScene()
{
    joints_of_interest = {
        K4ABT_JOINT_HAND_LEFT,
        K4ABT_JOINT_SPINE_CHEST,
        K4ABT_JOINT_THUMB_RIGHT,
        K4ABT_JOINT_HAND_RIGHT};
}

WriteAirScene::~WriteAirScene()
{
}

void WriteAirScene::onLoopStart(int frame_number)
{
}

void WriteAirScene::comprehend(Kinector *kinector, int frame_number)
{
    TRACE("capturing write air");
    Euclid *euclid = new Euclid(kinector->getCalibration(),
                                kinector->getColorWindowOrigin(),
                                kinector->getColorWindowSize(),
                                kinector->getColorImageWidth(),
                                kinector->getColorImageHeight(),
                                &moving_average,
                                K4ABT_JOINT_COUNT,
                                kinector->getBodies(),
                                joints_of_interest);
    for (int k = 0; k < euclid->getBodyCount(); k++)
    {

        k4a_float3_t left_hand = euclid->jointToGlobal(k, K4ABT_JOINT_HAND_LEFT, {0.f, 0.f, 0.f});
        k4a_float3_t left_hand_act = euclid->getJoint(k, K4ABT_JOINT_HAND_LEFT).position;
        k4a_float3_t chest = euclid->jointToGlobal(k, K4ABT_JOINT_SPINE_CHEST, {0.f, 0.f, 0.f});
        k4a_float3_t tip = euclid->jointToGlobal(k, K4ABT_JOINT_THUMB_RIGHT, {0.f, 0.f, 0.f});
        k4a_float3_t hand = euclid->jointToGlobal(k, K4ABT_JOINT_HAND_RIGHT, {0.f, 0.f, 0.f});
        ImVec2 tip_vec = euclid->pointToWindow(tip);

        Vector3f lh, rh, diff;
        lh << left_hand.v[0], left_hand.v[1], left_hand.v[2];
        rh << hand.v[0], hand.v[1], hand.v[2];
        diff = lh - rh;
        if (diff.norm() < 100.f)
        {
            letterWidgets.clear();
            if (writeMode == 1)
            {
                writeMode = 2;
            }
            else
            {
                writeMode = 0;
            }
        }
        if (letterWidgets.size() == 0)
        {
            LetterWidget nextLetter;
            letterWidgets.push_back(nextLetter);
        }
        int currentLetterIndex = letterWidgets.size() - 1;
        currentLetter = letterWidgets[currentLetterIndex];

        if (chest.v[1] > left_hand.v[1])
        {
            writeMode = 1;
        }
        else
        {
            if (writeMode == 1)
            {
                writeMode = 2;
            }
            else
            {
                writeMode = 0;
            }
        }
        if (writeMode == 1)
        {
            if (frame_number % WRITE_INTERVAL == 0)
            {
                letterWidgets[currentLetterIndex].letterPath.push_back(tip_vec);
            }
        }
        if (writeMode == 2)
        {
            writeMode = 0;
            if (letterWidgets[currentLetterIndex].letterPath.size() > 0)
            {
                LetterWidget nextLetter;
                nextLetter.boxType = 2;
                letterWidgets.push_back(nextLetter);
            }
        }
    }
    kinector->colorizeDepthImage();
}

float getBezierValue(float n1, float n2, float perc)
{
    float diff = n2 - n1;
    return n1 + (diff * perc);
}

void WriteAirScene::annotate(ImDrawList *drawList, vector<int> bodies, float y_shift)
{
    for (int i = 0; i < letterWidgets.size(); i++)
    {
        if (letterWidgets[i].letterPath.size() < 3)
            continue;
        for (int k = 0; k < letterWidgets[i].letterPath.size() - 2; k += 2)
        {
            ImVec2 first = letterWidgets[i].letterPath[k];
            ImVec2 second = letterWidgets[i].letterPath[k + 1];
            ImVec2 third = letterWidgets[i].letterPath[k + 2];
            for (float bez = 0; bez < 1; bez += 0.01)
            {
                float xa = getBezierValue(first.x, second.x, bez);
                float ya = getBezierValue(first.y, second.y, bez);
                float xb = getBezierValue(second.x, third.x, bez);
                float yb = getBezierValue(second.y, third.y, bez);

                float xmid = getBezierValue(xa, xb, bez);
                float ymid = getBezierValue(ya, yb, bez);
                drawList->AddCircleFilled(ImVec2(xmid, ymid), 5, IM_COL32(255, 69, 0, 255), 20.0f);
            }
        }
    }
}

void WriteAirScene::onLoopEnd()
{
}