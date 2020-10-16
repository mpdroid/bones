#include "controller.h"
using namespace controller;

Controller *Controller::inst_ = NULL;

Controller *Controller::getInstance()
{
    if (inst_ == NULL)
    {
        inst_ = new Controller();
        RUNTIMECONFIG = &(inst_->runtimeConfig);
    }
    return (inst_);
}

static char SCENE_SWITCH_INDICATOR = 'Z';

void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    {
        printf("Escape key pressed\n");
        (*RUNTIMECONFIG).time_to_go = true;
    }
    if (key == GLFW_KEY_D && action == GLFW_PRESS)
    {
        printf("Toggling depth image\n");
        (*RUNTIMECONFIG).show_depth_image = !(*RUNTIMECONFIG).show_depth_image;
    }
    if (key == GLFW_KEY_L && action == GLFW_PRESS)
    {
        printf("Switching to light saber display\n");
        (*RUNTIMECONFIG).demo_mode = DEMO_MODE_LIGHT_SABERS;
        SCENE_SWITCH_INDICATOR = GLFW_KEY_L;
    }
    if (key == GLFW_KEY_O && action == GLFW_PRESS)
    {
        printf("Switching to object detection\n");
        (*RUNTIMECONFIG).demo_mode = DEMO_MODE_OBJECT_DETECTION;
        SCENE_SWITCH_INDICATOR = GLFW_KEY_O;
    }
    if (key == GLFW_KEY_J && action == GLFW_PRESS)
    {
        printf("Switching to joint information\n");
        (*RUNTIMECONFIG).demo_mode = DEMO_MODE_JOINT_INFO;
        SCENE_SWITCH_INDICATOR = GLFW_KEY_J;
    }
    if (key == GLFW_KEY_W && action == GLFW_PRESS)
    {
        printf("Switching to writing mode\n");
        (*RUNTIMECONFIG).demo_mode = DEMO_MODE_WRITING;
        SCENE_SWITCH_INDICATOR = GLFW_KEY_W;
    }
}

int Controller::runLoop()
{
    try
    {
        k4a_image_t xy_table = NULL;
        k4a_image_t point_cloud = NULL;
        int frame_number = 0;
        vector<Detection> detections;
        vector<JointCoordinates> moving_average;
        int left_hand_raise_state = 0;

        // Check for devices
        //
        const uint32_t deviceCount = k4a::device::get_installed_count();
        if (deviceCount == 0)
        {
            throw std::runtime_error("No Azure Kinect devices detected!");
        }

        k4a_device_configuration_t kinect_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        kinect_config.camera_fps = K4A_FRAMES_PER_SECOND_30;
        kinect_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        kinect_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        kinect_config.color_resolution = K4A_COLOR_RESOLUTION_720P;
        kinect_config.synchronized_images_only = true;

        k4a::device dev = k4a::device::open(K4A_DEVICE_DEFAULT);
        dev.start_cameras(&kinect_config);

        KinectDemoViewer window = KinectDemoViewer::Instance();
        cout << "initializing window" << endl;
        window.Initialize(
            "Kinect DK Demo",
            1280,
            720,
            key_callback,
            RUNTIMECONFIG,
            kinect_config);

        Texture depthTexture = window.CreateTexture(GetDepthDimensions(kinect_config.depth_mode));
        Texture colorTexture = window.CreateTexture(GetColorDimensions(kinect_config.color_resolution));

        Kinector kinector(&kinect_config, &dev);
        if (kinector.isValid() == false)
        {
            printf("Kinect init failed!\n");
            exit(1);
        }


        this->scene = AbstractScene::getInstance(runtimeConfig.demo_mode);
        while (window.BeginFrame() && (*RUNTIMECONFIG).time_to_go == false)
        {
            if (SCENE_SWITCH_INDICATOR != 'Z')
            {
                cout << "scene switched " << endl;
                delete this->scene;
                this->scene = AbstractScene::getInstance(SCENE_SWITCH_INDICATOR);
                SCENE_SWITCH_INDICATOR = 'Z';
            }
            this->scene->onLoopStart(frame_number);
            std::vector<BgraPixel> depthTextureBuffer;
            BodyGeometry *body_geometry;

            window.ComputeDimensions();

            k4a::capture capture;
            vector<k4abt_body_t> bodies;
            uint32_t num_bodies = 0;
            if (dev.get_capture(&capture, std::chrono::milliseconds(0)))
            {
                kinector.InitializeFrame(capture);
                if (kinector.isValid() == false)
                {
                    printf("Failed to initialize kinect in frame\n");
                    break;
                }
                vector<k4abt_joint_id_t> joints_of_interest = {
                    K4ABT_JOINT_NOSE,
                    K4ABT_JOINT_HAND_LEFT,
                    K4ABT_JOINT_HAND_RIGHT,
                    K4ABT_JOINT_ELBOW_RIGHT,
                    K4ABT_JOINT_SPINE_CHEST,
                    K4ABT_JOINT_HANDTIP_LEFT,
                    K4ABT_JOINT_HANDTIP_RIGHT,
                    K4ABT_JOINT_THUMB_RIGHT};
                num_bodies = kinector.GetBodyIds().size();
                if (moving_average.size() < (int)num_bodies)
                {
                    moving_average.resize((int)num_bodies);
                }
                body_geometry = new BodyGeometry(kinector.GetCalibration(), window.GetColorWindowOrigin(),
                                                 window.GetColorWindowSize(), kinector.GetColorImageWidth(), kinector.GetColorImageHeight(),
                                                 &moving_average, K4ABT_JOINT_KNEE_LEFT,
                                                 kinector.GetBodies(),
                                                 joints_of_interest);
                if ((*RUNTIMECONFIG).demo_mode == DEMO_MODE_OBJECT_DETECTION)
                {
                    body_geometry = new BodyGeometry(kinector.GetCalibration(), window.GetColorWindowOrigin(),
                                                     window.GetColorWindowSize(), kinector.GetColorImageWidth(), kinector.GetColorImageHeight(), &moving_average, K4ABT_JOINT_HANDTIP_RIGHT,
                                                     kinector.GetBodies(),
                                                     joints_of_interest);
                }
                this->scene->capture(&kinector, body_geometry, frame_number);

                vector<Ray> rays;
                if ((*RUNTIMECONFIG).demo_mode == DEMO_MODE_OBJECT_DETECTION)
                {
                    rays = ((ThingFinderScene *)(this->scene))->getRays();
                }

                if ((*RUNTIMECONFIG).show_depth_image == true)
                {
                    if (rays.size() == 0)
                    {
                        window.ColorizeDepthImage(kinector.GetDepthImage(),
                                                  K4ADepthPixelColorizer::ColorizeBlueToRed,
                                                  GetDepthModeRange(kinect_config.depth_mode),
                                                  &depthTextureBuffer);
                    }
                    else
                    {

                        window.ColorizeFilteredDepthImage(kinector.GetDepthImage(),
                                                          kinector.GetColorizedDepthImage(),
                                                          kinector.GetXYTable(),
                                                          body_geometry,
                                                          rays,
                                                          K4ADepthPixelColorizer::ColorizeBlueToRed,
                                                          GetDepthModeRange(kinect_config.depth_mode),
                                                          &depthTextureBuffer);
                        // if (detections.size() > 0)
                        // {
                        //     window.ColorizeClusteredDepthImage(depthImage, colorized_depth_image, &depthTextureBuffer, &cloud_seg,
                        //                                 detections,
                        //                                 sensor_calibration);
                        // }
                    }
                    depthTexture.Update(&(depthTextureBuffer[0]));
                }


                delete body_geometry;
            }
            colorTexture.Update(kinector.GetPixels());
            kinector.ReleaseFrame();
            if ((*RUNTIMECONFIG).show_depth_image == true)
            {
                window.ShowDepthTexture(depthTexture);
            }
            if (kinector.GetBodyIds().size() == 0)
            {
                window.ShowColorTexture(colorTexture);
            }
            else
            {
                window.ShowTextureWithPersistentBoundingBoxes(
                    colorTexture,
                    this->scene,
                    kinector.GetBodyIds());
            }

            this->scene->onLoopEnd();
            window.EndFrame();
            frame_number++;
            // cloud_seg.clear();
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "Press [enter] to exit." << std::endl;
        std::cin.get();
        return 1;
    }
    return 0;
}
