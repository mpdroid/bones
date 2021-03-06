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
        (*RUNTIMECONFIG).time_to_go = true;
    }
    if (key == GLFW_KEY_D && action == GLFW_PRESS)
    {
        (*RUNTIMECONFIG).show_depth_image = !(*RUNTIMECONFIG).show_depth_image;
    }
    if (key == GLFW_KEY_L && action == GLFW_PRESS)
    {
        (*RUNTIMECONFIG).demo_mode = DEMO_MODE_LIGHT_SABERS;
        SCENE_SWITCH_INDICATOR = GLFW_KEY_L;
    }
    if (key == GLFW_KEY_O && action == GLFW_PRESS)
    {
        (*RUNTIMECONFIG).demo_mode = DEMO_MODE_OBJECT_DETECTION;
        SCENE_SWITCH_INDICATOR = GLFW_KEY_O;
    }
    if (key == GLFW_KEY_J && action == GLFW_PRESS)
    {
        (*RUNTIMECONFIG).demo_mode = DEMO_MODE_JOINT_INFO;
        SCENE_SWITCH_INDICATOR = GLFW_KEY_J;
    }
    if (key == GLFW_KEY_W && action == GLFW_PRESS)
    {
        (*RUNTIMECONFIG).demo_mode = DEMO_MODE_WRITING;
        SCENE_SWITCH_INDICATOR = GLFW_KEY_W;
    }
}

int Controller::runLoop()
{
    try
    {
        int frame_number = 0;

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

        Rendor window = Rendor::Instance();
        INFO("initializing window");
        window.Initialize(
            "Kinect DK Demo",
            GetColorDimensions(kinect_config.color_resolution).first,
            GetColorDimensions(kinect_config.color_resolution).second,
            key_callback,
            RUNTIMECONFIG,
            kinect_config);

        Texture depthTexture = window.CreateTexture(GetDepthDimensions(kinect_config.depth_mode));
        Texture colorTexture = window.CreateTexture(GetColorDimensions(kinect_config.color_resolution));

        Kinector kinector(&kinect_config, &dev);
        if (kinector.isValid() == false)
        {
            ERROR("Kinect init failed!");
            exit(1);
        }


        this->scene = AbstractScene::getInstance(runtimeConfig.demo_mode);
        while (window.BeginFrame() && (*RUNTIMECONFIG).time_to_go == false)
        {
            if (SCENE_SWITCH_INDICATOR != 'Z')
            {
                delete this->scene;
                this->scene = AbstractScene::getInstance(SCENE_SWITCH_INDICATOR);
                SCENE_SWITCH_INDICATOR = 'Z';
            }
            this->scene->onLoopStart(frame_number);
            // std::vector<BgraPixel> depthTextureBuffer;

            window.computeDimensions();
            kinector.setColorWindowOrigin(window.getColorWindowOrigin());
            kinector.setColorWindowSize(window.getColorWindowSize());

            k4a::capture capture;
            vector<k4abt_body_t> bodies;
            uint32_t num_bodies = 0;
            if (dev.get_capture(&capture, std::chrono::milliseconds(0)))
            {
                kinector.initializeFrame(capture);
                if (kinector.isValid() == false)
                {
                    ERROR("Failed to initialize kinect in frame");
                    break;
                }

                this->scene->comprehend(&kinector, frame_number);
            }
            colorTexture.Update(kinector.getColorPixels());
            depthTexture.Update(kinector.getDepthPixels());
            window.showTextures(colorTexture, depthTexture, &kinector, this->scene, (*RUNTIMECONFIG).show_depth_image);
            this->scene->onLoopEnd();
            kinector.releaseFrame();
            window.EndFrame();
            frame_number++;
            // cloud_seg.clear();
        }
    }
    catch (const std::exception &e)
    {
        ERROR(e.what());
        std::cerr << "Press [enter] to exit." << std::endl;
        std::cin.get();
        return 1;
    }
    return 0;
}
