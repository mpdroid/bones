#include "thingfinder.h"
using namespace scene;

const auto FRAME_INTERVAL = 60;

/*****************************************************************/
/******** THING FINDER *******************************************/
/*****************************************************************/
ThingFinderScene::ThingFinderScene()
{
}

ThingFinderScene::~ThingFinderScene()
{
}

void ThingFinderScene::onLoopStart(int frame_number)
{
    // TRACE("num of dets before", detections.size());
    detections.erase(
        std::remove_if(detections.begin(), detections.end(),
                       [&](const Detection detection) { return (frame_number -  10 * FRAME_INTERVAL) > detection.frame_number; }),
        detections.end());
    // TRACE("num of dets after", detections.size());
    currentFrameNumber = frame_number;
    rays.clear();
    objectPointers.clear();
    cubeWidgets.clear();
}

void detect_objects(int frame_number,
                    cv::Mat cvImageMatrix,
                    cilantro::VectorSet3f cilantroPoints,
                    cilantro::VectorSet3f cilantroColors,
                    vector<Ray> rays,
                    vector<Detection> *detections,
                    k4a_calibration_t *sensor_calibration,
                    int w,
                    int h)
{
    Clusterizer clusterizer(&cilantroPoints, &cilantroColors);
    cilantro::PointCloud3f cloud_seg;

    vector<BoundingCube> pointCubes = clusterizer.clusterize(&cloud_seg);
    for (int c = 0; c < pointCubes.size(); c++)
    {
        BoundingCube *cube = &(pointCubes[c]);
        vector<k4a_float2_t> points2d;
        vector<Vector3f> vectors = {cube->ftl, cube->ftr, cube->fbr, cube->fbl, cube->btl, cube->btr, cube->bbr, cube->bbl};
        float minx = INFINITY, miny = INFINITY;
        float maxx = -INFINITY, maxy = -INFINITY;

        for (Vector3f vector : vectors)
        {
            k4a_float2_t pt2d;
            vector_to_color2d(vector, sensor_calibration, w, h, &pt2d);
            if (pt2d.v[0] < minx)
            {
                minx = pt2d.v[0];
            }
            else if (pt2d.v[0] > maxx)
            {
                maxx = pt2d.v[0];
            }
            if (pt2d.v[1] < miny)
            {
                miny = pt2d.v[1];
            }
            else if (pt2d.v[1] > maxy)
            {
                maxy = pt2d.v[1];
            }
        }
        cube->u2d_k4.v[0] = minx - 20.f;
        cube->u2d_k4.v[1] = miny - 20.f;
        cube->v2d_k4.v[0] = maxx + 20.f;
        cube->v2d_k4.v[1] = maxy + 20.f;
        // TODO - set color image u,v while creating cube
    }
    if (pointCubes.size() > 0)
    {
        for (Ray ray : rays)
        {
            int pointee;
            bool found = find_intersecting_cube(ray, &pointCubes, &pointee);
            if (found == true)
            {
                Detection detection;
                detection.cube = pointCubes[pointee];
                detection.ray = ray;
                detection.frame_number = frame_number;

                size_t pointeeLabel = detection.cube.label;
                int x = (int)detection.cube.u2d_k4.v[0];
                int y = (int)detection.cube.u2d_k4.v[1];
                int x1 = (int)detection.cube.v2d_k4.v[0];
                int y1 = (int)detection.cube.v2d_k4.v[1];

                if (x1 < x)
                {
                    ERROR(" bad u v x in frame ", frame_number, " ", x, ",", x1);
                    int t = x;
                    x = x1;
                    x1 = t;
                }
                if (y1 < y)
                {
                    ERROR(" bad u v y in frame ", frame_number, " ", y, ",", y1);
                    int t = y;
                    y = y1;
                    y1 = t;
                }
                if (x1 < cvImageMatrix.size().width && y1 < cvImageMatrix.size().height)
                {
                    cv::Rect region_of_interest = cv::Rect(x, y, (x1 - x), (y1 - y));
                    try
                    {
                        cv::Mat roi = cvImageMatrix(region_of_interest);
                        AzureVisionRequest *vq = new AzureVisionRequest(0);

                        std::vector<uchar> buff;
                        std::vector<int> param(2);
                        param[0] = cv::IMWRITE_JPEG_QUALITY;
                        param[1] = 95; //default(95) 0-100
                        cv::imencode(".jpg", roi, buff, param);
                        auto filename = "obj_roi" + to_string(frame_number) + ".jpg";
                        // cv::imwrite(filename, roi);

                        vq->SendAnalysisRequest(getenv("AZURE_VISION_KEY"), buff);
                        string name = vq->getTopObject();
                        TO_UPPER(name);
                        TRACE("detected ", name, " in frame ", detection.frame_number);
                        if (name != "")
                        {
                            detections->clear();
                        }
                        detection.name = name;
                        detections->push_back(detection);
                    }
                    catch (const std::exception &e)
                    {
                        ERROR(e.what());
                    }
                }
            }
        }
    }
}

void ThingFinderScene::comprehend(Kinector *kinector, int frame_number)
{
    Euclid *euclid = new Euclid(kinector->getCalibration(),
                                kinector->getColorWindowOrigin(),
                                kinector->getColorWindowSize(),
                                kinector->getColorImageWidth(),
                                kinector->getColorImageHeight(),
                                &moving_average,
                                K4ABT_JOINT_COUNT,
                                kinector->getBodies(),
                                joints_of_interest);
    cilantro::VectorSet3f cilantroPoints;
    cilantro::VectorSet3f cilantroColors;
    cilantro::PointCloud3f cloud_seg;

    vector<ImVec2> directionImVectors;
    for (int k = 0; k < euclid->getBodyCount(); k++)
    {

        Ray ray = euclid->jointsToRay(k, K4ABT_JOINT_ELBOW_RIGHT, K4ABT_JOINT_HAND_RIGHT, 120.f);
        rays.push_back(ray);
        ImVec2 pointerBeginObj = euclid->vectorToWindow(ray.origin);
        directionImVectors.push_back(pointerBeginObj);

        float delta = 200.f;
        k4a_float3_t target = {ray.origin[0] + ray.direction[0] * delta, ray.origin[1] + ray.direction[1] * delta,
                               ray.origin[2] + ray.direction[2] * delta};
        ImVec2 pointerEndObj = euclid->pointToWindow(target);
        if (pointerEndObj.x == 0 && pointerEndObj.y == 0)
        {
            directionImVectors.push_back(pointerBeginObj);
        }
        else
        {
            directionImVectors.push_back(pointerEndObj);
        }
    }

    if (directionImVectors.size() > 0)
    {
        PointerWidget pointer;
        pointer.frameNumber = frame_number;
        pointer.arrow = directionImVectors;
        objectPointers.push_back(pointer);
    }

    if (frame_number % FRAME_INTERVAL == 0)
    {
        int point_count = kinector->generatePointCloud(euclid,
                                                       rays,
                                                       &cilantroPoints,
                                                       &cilantroColors);
        if (point_count > 100)
        {
            std::thread th1(detect_objects,
                            frame_number,
                            kinector->getCVImage(),
                            cilantroPoints, cilantroColors,
                            rays, &detections,
                            kinector->getCalibration(),
                            euclid->getColorImageWidth(),
                            euclid->getColorImageHeight());
            th1.detach();
        }
    }
    for (Detection detection : detections)
    {
        vector<ImVec2> bboxes;
        Vector3f ftl = detection.cube.ftl;
        Vector3f ftr = detection.cube.ftr;
        Vector3f fbr = detection.cube.fbr;
        Vector3f fbl = detection.cube.fbl;
        Vector3f btl = detection.cube.btl;
        Vector3f btr = detection.cube.btr;
        Vector3f bbr = detection.cube.bbr;
        Vector3f bbl = detection.cube.bbl;

        CubeWidget cubeWidget;
        cubeWidget.frameNumber = detection.frame_number;
        cubeWidget.color = {detection.cube.color[0], detection.cube.color[1], detection.cube.color[2]};
        cubeWidget.ftl = euclid->vectorToWindow(ftl);
        cubeWidget.ftr = euclid->vectorToWindow(ftr);
        cubeWidget.fbr = euclid->vectorToWindow(fbr);
        cubeWidget.fbl = euclid->vectorToWindow(fbl);
        cubeWidget.btl = euclid->vectorToWindow(btl);
        cubeWidget.btr = euclid->vectorToWindow(btr);
        cubeWidget.bbr = euclid->vectorToWindow(bbr);
        cubeWidget.bbl = euclid->vectorToWindow(bbl);
        cubeWidget.name = detection.name;
        cubeWidgets.push_back(cubeWidget);
    }
    kinector->colorizeFilteredDepthImage(euclid, rays);
}

void ThingFinderScene::annotate(ImDrawList *drawList, vector<int> bodies, float y_shift)
{
    for (int i = 0; i < cubeWidgets.size(); i++)
    {
        CubeWidget widget = cubeWidgets[i];
        widget.ftl.y += y_shift;
        widget.ftr.y += y_shift;
        widget.fbr.y += y_shift;
        widget.fbl.y += y_shift;
        widget.btl.y += y_shift;
        widget.btr.y += y_shift;
        widget.bbr.y += y_shift;
        widget.bbl.y += y_shift;

        int frameDiff = currentFrameNumber - widget.frameNumber;
        int boxDiff = currentFrameNumber - widget.frameNumber;
        float alpha = max(55, 200 - 10 * (int)((float)frameDiff));
        float boxalpha = max(0, 255 - (int)((float)frameDiff));
        if (frameDiff <= 20 * FRAME_INTERVAL)
        {
            auto color = widget.color;

            if (widget.name.length() > 0)
            {
                ImVec2 textStart(widget.ftl.x + 5.f, widget.ftl.y - 50.f);

                float fontSize = 25.f;
                float rectLength = 20.f * widget.name.length();
                ImVec2 rectStart(widget.ftl.x - 5.f, widget.ftl.y - 52.f);
                ImVec2 rectEnd(widget.ftl.x + rectLength + 5.f, widget.ftl.y - 23.f);
                drawList->AddRectFilled(rectStart, rectEnd, IM_COL32(255, 255, 255, boxalpha));
                drawList->AddText(NULL, fontSize, textStart, IM_COL32(0, 0, 0, boxalpha), widget.name.c_str());
            }

            ImVec2 lines[4] = {widget.ftl, widget.ftr, widget.fbr, widget.fbl};

            drawList->AddPolyline(lines, 4,
                                  IM_COL32(255, 255, 255, boxalpha), true, 2.f);
            ImVec2 lines_back[4] = {widget.btl, widget.btr, widget.bbr, widget.bbl};
            drawList->AddPolyline(lines_back, 4,
                                  IM_COL32(255, 255, 255, boxalpha), true, 2.f);
            ImVec2 lines_left[4] = {widget.ftl, widget.btl, widget.bbl, widget.fbl};
            drawList->AddPolyline(lines_left, 4,
                                  IM_COL32(255, 255, 255, boxalpha), true, 2.f);
            ImVec2 lines_right[4] = {widget.ftr, widget.btr, widget.bbr, widget.fbr};
            drawList->AddPolyline(lines_right, 4,
                                  IM_COL32(255, 255, 255, boxalpha), true, 2.f);
        }
    }

    for (int i = 0; i < objectPointers.size(); i++)
    {
        vector<ImU32> colors = {IM_COL32(255, 69, 0, 255), IM_COL32(128, 255, 0, 255)};
        for (int j = 0; j < objectPointers[i].arrow.size(); j++)
        {
            objectPointers[i].arrow[j].y += y_shift;
        }
        vector<ImVec2> arrow = objectPointers[i].arrow;
        for (int k = 0; k < (int)arrow.size() - 1; k += 2)
        {
            ImVec2 start = arrow[k];
            ImVec2 end = arrow[k + 1];
            // int bodyId = bodies[i / 2] % colors.size();
            drawList->AddLine(start, end, IM_COL32(255, 255, 255, 255), 5.0f);
        }
    }
}

void ThingFinderScene::onLoopEnd()
{
}

vector<Ray> ThingFinderScene::getRays()
{
    return this->rays;
}