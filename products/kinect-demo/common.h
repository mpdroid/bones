#ifndef __COMMON_H
#define __COMMON_H
#define DEMO_MODE_OBJECT_DETECTION 'O'
#define DEMO_MODE_LIGHT_SABERS 'L'
#define DEMO_MODE_JOINT_INFO 'J'
#define DEMO_MODE_WRITING 'W'
#define VERIFY(result, error)                                                                            \
    if (result != K4A_RESULT_SUCCEEDED)                                                                  \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    }

struct RunTimeConfig
{
    bool time_to_go = false;
    bool show_depth_image = false;
    char demo_mode = DEMO_MODE_LIGHT_SABERS;
    bool writing = false;
    bool use_point_cloud_image_for_detection = false;
};

#define TO_UPPER(str) \
    std::transform(str.begin(), str.end(), str.begin(), ::toupper);

#endif