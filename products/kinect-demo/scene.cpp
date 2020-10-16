#include "scene.h"
#include "thingfinder.h"
using namespace scene;

AbstractScene *AbstractScene::getInstance(char demo_mode)
{
    switch (demo_mode)
    {
    case DEMO_MODE_LIGHT_SABERS:
        return new LightSaberScene();
        break;
    case DEMO_MODE_JOINT_INFO:
        return new JointInfoScene();
        break;
    case DEMO_MODE_WRITING:
        return new WriteAirScene();
        break;
    case DEMO_MODE_OBJECT_DETECTION:
        return new ThingFinderScene();
        break;
    default:
        return new LightSaberScene();
        break;
    }
}

/*****************************************************************/
/******** LIGHT SABER ********************************************/
/*****************************************************************/
LightSaberScene::LightSaberScene()
{
}

LightSaberScene::~LightSaberScene()
{
    cout << "destroying " << endl;
}

void LightSaberScene::onLoopStart(int frame_number)
{
    cout << "started loop" << endl;
    lightSabers.erase(
        std::remove_if(lightSabers.begin(), lightSabers.end(),
                       [&](const PointerWidget lightSaber) { return (frame_number - 5) > lightSaber.frameNumber; }),
        lightSabers.end());
    return;
}

void LightSaberScene::capture(Kinector *kinector, BodyGeometry *body_geometry, int frame_number)
{
        cout << "started capture" <<  body_geometry->get_body_count() << endl;
    vector<PointerWidget> newSabers;
    for (int k = 0; k < body_geometry->get_body_count(); k++)
    {
        cout << "started capture" << endl;
        int lastSaberIndex = -1;
        for (int l = lightSabers.size() - 1; l >= 0; l--)
        {
            if (lightSabers[l].bodyId == k)
            {
                lastSaberIndex = l;
                break;
            }
        }
        cout << "last saber for body " << k << " " << lastSaberIndex << endl;

        Ray ray = body_geometry->joints_to_ray(k, K4ABT_JOINT_ELBOW_RIGHT, K4ABT_JOINT_HAND_RIGHT, K4ABT_JOINT_HAND_RIGHT);

        Vector3f ortho = body_geometry->ray_to_joint_ortho(ray, k, K4ABT_JOINT_THUMB_RIGHT);
        // rays.push_back(ray);
        // ImVec2 pointerBeginObj = body_geometry->vector_to_window(ray.origin);
        ImVec2 pointerBeginObj = body_geometry->joint_to_window(k, K4ABT_JOINT_HAND_RIGHT);

        PointerWidget lightSaber;
        // lightSaber.boxType = 4;
        lightSaber.bodyId = k;
        lightSaber.frameNumber = frame_number;
        lightSaber.alpha = 200;

        lightSaber.arrow.push_back(pointerBeginObj);

        ImVec2 pointerEndObj = body_geometry->vector_to_window(ray.origin + ortho * 800.f);
        lightSaber.arrow.push_back(pointerEndObj);

        if (lastSaberIndex > -1)
        {
            PointerWidget lastSaber = lightSabers[lastSaberIndex];
            float x1_0 = lastSaber.arrow[0].x;
            float x2_0 = lastSaber.arrow[1].x;
            float x1_1 = lightSaber.arrow[0].x;
            float x2_1 = lightSaber.arrow[1].x;
            float y1_0 = lastSaber.arrow[0].y;
            float y2_0 = lastSaber.arrow[1].y;
            float y1_1 = lightSaber.arrow[0].y;
            float y2_1 = lightSaber.arrow[1].y;
            float dx1 = (x1_1 - x1_0) / 20.f;
            float dy1 = (y1_1 - y1_0) / 20.f;
            float dx2 = (x2_1 - x2_0) / 20.f;
            float dy2 = (y2_1 - y2_0) / 20.f;
            for (int dn = 1; dn <= 20; dn++)
            {
                PointerWidget interpolation;
                interpolation.boxType = 4;
                interpolation.bodyId = k;
                interpolation.frameNumber = frame_number;
                interpolation.alpha = dn * 20;
                interpolation.arrow.push_back(ImVec2(x1_0 + dx1 * (float)dn, y1_0 + dy1 * (float)dn));
                interpolation.arrow.push_back(ImVec2(x2_0 + dx2 * (float)dn, y2_0 + dy2 * (float)dn));
                newSabers.push_back(interpolation);
            }
        }
        else
        {
            newSabers.push_back(lightSaber);
        }
    }
    lightSabers.insert(lightSabers.end(), newSabers.begin(), newSabers.end());
    cout << "num sabers" << lightSabers.size();
    return;
}

void LightSaberScene::render(ImDrawList *drawList, vector<int> bodies, float y_shift)
{
    if (lightSabers.size() > 0)
    {
        map<int, vector<PointerWidget>> lightSabersByBody;
        for (int bodyId : bodies)
        {
            vector<PointerWidget> sabers;
            for (int i = 0; i < lightSabers.size(); i++)
            {
                if (lightSabers[i].bodyId == bodyId)
                {
                    sabers.push_back(lightSabers[i]);
                }
            }
            lightSabersByBody.insert(pair<int, vector<PointerWidget>>(bodyId, sabers));
        }
        for (int bodyId : bodies)
        {
            vector<PointerWidget> sabers = lightSabersByBody[bodyId];
            cout << "body id" << bodyId << "num sabers" << sabers.size() << endl;
            ;
            int n = sabers.size();
            float b = pow(200.f, 1.f / (float)n);
            for (int i = 0; i < n; i++)
            {
                int alpha = (int)(pow(b, i) - 1.f);
                vector<ImU32> colors = {IM_COL32(255, 69, 0, alpha), IM_COL32(128, 255, 0, alpha)};
                // cout << " num vectors for saber " << i << " " << sabers[i].directionVectors.size() << alpha << endl;
                cout << "saber id" << i << "num arrows" << sabers[i].arrow.size() << endl;
                ;
                for (int j = 0; j < sabers[i].arrow.size(); j++)
                {
                    sabers[i].arrow[j].y += y_shift;
                }
                vector<ImVec2> directionVectors = sabers[i].arrow;
                for (int j = 0; j < (int)directionVectors.size() - 1; j += 2)
                {
                    ImVec2 start = directionVectors[j];
                    ImVec2 end = directionVectors[j + 1];
                    drawList->AddLine(start, end, colors[bodyId], 20.0f);
                }
            }
        }
    }
}

void LightSaberScene::onLoopEnd()
{
    cout << "ended loop" << endl;
}

/*****************************************************************/
/******** JOINT INFORMATION **************************************/
/*****************************************************************/

JointInfoScene::JointInfoScene()
{
}

JointInfoScene::~JointInfoScene()
{
    cout << "destroying " << endl;
}

void JointInfoScene::onLoopStart(int frame_number)
{
    cout << "started loop" << endl;
    jointWidgets.clear();
    axisWidgets.clear();
}

void JointInfoScene::capture(Kinector *kinector, BodyGeometry *body_geometry, int frame_number)
{
    AxisWidget axisItem;
    axisItem.name = "Depth Camera";
    axisItem.bodyId = -1;
    axisItem.boxType = 5;
    axisItem.frameNumber = frame_number;
    axisItem.textCoordinates = ImVec2(100.f, 75.f);
    k4a_float3_t origin = body_geometry->window_to_point(ImVec2(100.f, 100.f));
    Eigen::Vector3f depth_zero, depth_x, depth_y, depth_z;
    depth_zero << origin.v[0], origin.v[1], origin.v[2];
    depth_x << origin.v[0] + 3.f, origin.v[1], origin.v[2];
    depth_y << origin.v[0], origin.v[1] + 3.f, origin.v[2];
    depth_z << origin.v[0], origin.v[1], origin.v[2] + 3.f;
    ImVec2 pointerBeginObj = body_geometry->vector_to_window(depth_zero);
    axisItem.x.push_back(pointerBeginObj);
    axisItem.x.push_back(body_geometry->vector_to_window(depth_x));
    axisItem.y.push_back(pointerBeginObj);
    axisItem.y.push_back(body_geometry->vector_to_window(depth_y));
    axisItem.z.push_back(pointerBeginObj);
    axisItem.z.push_back(body_geometry->vector_to_window(depth_z));
    axisItem.color = {0, 0, 0};
    axisItem.textShadowColor = {255, 255, 255};
    axisWidgets.push_back(axisItem);

    for (int k = 0; k < body_geometry->get_body_count(); k++)
    {
        vector<k4abt_joint_id_t> joint_ids = {
            K4ABT_JOINT_HANDTIP_LEFT,
            K4ABT_JOINT_HANDTIP_RIGHT,
            K4ABT_JOINT_SPINE_CHEST};
        for (k4abt_joint_id_t joint_id : joint_ids)
        {
            int valid = 0;
            JointInfo jointInfo = body_geometry->get_joint_information(k, joint_id, &valid);
            if (valid == 1)
            {
                JointWidget jointDisplay;
                jointDisplay.bodyId = k;
                jointDisplay.frameNumber = frame_number;
                jointDisplay.joints.push_back(jointInfo);
                jointWidgets.push_back(jointDisplay);
            }
        }
    }
}

void JointInfoScene::render(ImDrawList *drawList, vector<int> bodies, float y_shift)
{
    cout << "started render" << endl;
    for (int i = 0; i < axisWidgets.size(); i++)
    {
        AxisWidget axis = axisWidgets[i];
        ImVec2 textStart(axis.textCoordinates.x + 5.f, axis.textCoordinates.y + 2.f);
        ImVec2 shadowTextStart(axis.textCoordinates.x + 3.f, axis.textCoordinates.y + 0.f);
        auto color = IM_COL32(255, 255, 255, 255);
        //auto shadowColor = IM_COL32(axis.textShadowColor[0], axis.textShadowColor[1], axis.textShadowColor[2], 255);
        float fontSize = 25.f;
        //float shadowFontSize = 25.05f;
        // drawList->AddText(NULL, shadowFontSize, shadowTextStart, shadowColor, axis.name.c_str());
        drawList->AddText(NULL, fontSize, textStart, color, axis.name.c_str());
        ImVec2 start = axis.x[0];
        start.y += y_shift;
        ImVec2 end = axis.x[1];
        end.y += y_shift;
        drawList->AddLine(start, end, IM_COL32(255, 0, 0, 255), 3.0f);
        ImVec2 xlabel(end.x + 5.f, end.y - 20.f);
        drawList->AddText(NULL, fontSize, xlabel, IM_COL32(255, 0, 0, 255), "+X");

        ImVec2 starty = axis.y[0];
        starty.y += y_shift;
        ImVec2 endy = axis.y[1];
        endy.y += y_shift;
        drawList->AddLine(starty, endy, IM_COL32(0, 255, 0, 255), 3.0f);
        ImVec2 ylabel(endy.x - 20.f, endy.y + 5.f);
        drawList->AddText(NULL, fontSize, ylabel, IM_COL32(0, 255, 0, 255), "+Y");

        if (axis.z.size() > 0)
        {
            ImVec2 startz = axis.z[0];
            startz.y += y_shift;
            ImVec2 endz = axis.z[1];
            endz.y += y_shift;
            drawList->AddLine(startz, endz, IM_COL32(0, 0, 255, 255), 3.0f);
            ImVec2 zlabel(endz.x + 5.f, endz.y + 5.f);
            drawList->AddText(NULL, fontSize, zlabel, IM_COL32(0, 0, 255, 255), "+Z");
        }
    }

    for (int i = 0; i < jointWidgets.size(); i++)
    {
        JointWidget widget = jointWidgets[i];
        for (JointInfo joint : widget.joints)
        {
            auto color = IM_COL32(255, 255, 255, 255);
            auto shadowColor = IM_COL32(0, 255, 0, 255);
            float fontSize = 15.f;
            float shadowFontSize = 15.02f;

            ImVec2 textStart(joint.textCoordinates.x + 5.f, joint.textCoordinates.y + 2.f);
            textStart.y += y_shift;
            ImVec2 shadowTextStart(joint.textCoordinates.x + 3.f, joint.textCoordinates.y + 0.f);

            ostringstream depthStream, orientStream;
            depthStream << "Depth: " << (int)joint.depthCoordinates[0] << ", " << (int)joint.depthCoordinates[1] << ", " << (int)joint.depthCoordinates[2];
            string depth_string = depthStream.str();
            drawList->AddText(NULL, fontSize, textStart, color, depth_string.c_str());

            ImVec2 textStartOrientation(joint.textCoordinates.x + 5.f, joint.textCoordinates.y + 18.f);
            textStartOrientation.y += y_shift;
            orientStream.precision(2);
            orientStream << "Orientation: " << joint.orientation[0]
                         << ", " << joint.orientation[1]
                         << ", " << joint.orientation[2]
                         << ", " << joint.orientation[3];
            string orientString = orientStream.str();
            drawList->AddText(NULL, fontSize, textStartOrientation, color, orientString.c_str());

            ImVec2 start = joint.x_direction[0];
            start.y += y_shift;
            ImVec2 end = joint.x_direction[1];
            end.y += y_shift;
            drawList->AddLine(start, end, IM_COL32(255, 0, 0, 255), 4.0f);

            ImVec2 starty = joint.y_direction[0];
            starty.y += y_shift;
            ImVec2 endy = joint.y_direction[1];
            endy.y += y_shift;
            drawList->AddLine(starty, endy, IM_COL32(0, 255, 0, 255), 4.0f);

            ImVec2 startz = joint.z_direction[0];
            startz.y += y_shift;
            ImVec2 endz = joint.z_direction[1];
            endz.y += y_shift;
            drawList->AddLine(startz, endz, IM_COL32(0, 0, 255, 255), 4.0f);
        }
    }
}

void JointInfoScene::onLoopEnd()
{
    cout << "ended loop" << endl;
    jointWidgets.clear();
    axisWidgets.clear();
}