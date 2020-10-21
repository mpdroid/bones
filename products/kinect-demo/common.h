#ifndef __COMMON_H
#define __COMMON_H

#include <iostream>
#include <string>
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
    char demo_mode = DEMO_MODE_JOINT_INFO;
    bool writing = false;
    bool use_point_cloud_image_for_detection = false;
};

#define TO_UPPER(str) \
    std::transform(str.begin(), str.end(), str.begin(), ::toupper);

// Tracing based on https://stackoverflow.com/questions/29326460/how-to-make-a-variadic-macro-for-stdcout
#define TRACING true

template <typename... Args>
void trace(const Args &... args)
{
    (std::cout << ... << args);
}

#define TRACE(...)                  \
    if (TRACING == true)            \
    {                               \
        trace("TRACE", ": ",__FILE__, ",",__FUNCTION__,",",__LINE__ ,": "); \
        trace( __VA_ARGS__); \
        trace("\n"); \
    }

#define INFO(...) \
    {                               \
        trace("INFO", ": ",__FILE__, ",",__FUNCTION__,",",__LINE__ ,": "); \
        trace( __VA_ARGS__); \
        trace("\n"); \
    }

#define ERROR(...) \
    {                               \
        trace("ERROR", ": ",__FILE__, ",",__FUNCTION__,",",__LINE__ ,": "); \
        trace( __VA_ARGS__); \
        trace("\n"); \
    }

#endif