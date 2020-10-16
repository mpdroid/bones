# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

# find_package(OpenGL REQUIRED)
find_package(k4abt REQUIRED)
find_package(cilantro REQUIRED)
find_package(cpprestsdk REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "    include path: ${EIGEN3_INCLUDE_DIRS}")

include_directories(
    .
    ${OPENGL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(favorite-things
    main.cpp
    texture.cpp
    clusterizer.cpp
    viewerwindow.cpp
    kinect_demo_viewer.cpp
    azure_vision_request.cpp
    euclid.cpp
    controller.cpp
    kinector.cpp
    scene/scene.cpp
    scene/writair.cpp
    scene/thingfinder.cpp)


target_link_libraries(favorite-things PRIVATE
    k4a::k4a
    k4abt::k4abt
    glfw::glfw
    imgui::imgui
    cpprestsdk::cpprest
    cilantro
    ${OPENGL_LIBRARIES}
    ${OpenCV_LIBS})