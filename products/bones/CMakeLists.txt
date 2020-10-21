# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

# find_package(OpenGL REQUIRED)
find_package(k4abt REQUIRED)
find_package(cilantro REQUIRED)
find_package(cpprestsdk REQUIRED)
find_package(OpenCV REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

include_directories(
    .
    ${OPENGL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(bones
    main.cpp
    texture.cpp
    clusterizer.cpp
    viewerwindow.cpp
    renderor.cpp
    azure_vision_request.cpp
    euclid.cpp
    controller.cpp
    kinector.cpp
    scene/scene.cpp
    scene/writair.cpp
    scene/thingfinder.cpp)


target_link_libraries(bones PRIVATE
    k4a::k4a
    k4abt::k4abt
    glfw::glfw
    imgui::imgui
    cpprestsdk::cpprest
    cilantro
    ${OPENGL_LIBRARIES}
    ${OpenCV_LIBS})