cmake_minimum_required(VERSION 3.15)
project(MocapApiCppDemo LANGUAGES CXX)

add_executable(mocap_demo
    src/main.cpp
    src/MocapClient.cpp
    src/JointData.cpp
    src/SkeletonViewer.cpp
)

find_package(OpenGL REQUIRED)
find_package(glfw3 CONFIG REQUIRED)

target_include_directories(mocap_demo PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

target_link_directories(mocap_demo PRIVATE
    ${CMAKE_SOURCE_DIR}/lib/win32/x64
)

target_link_libraries(mocap_demo PRIVATE
    MocapApi
    glfw
    OpenGL::GL
)

target_compile_features(mocap_demo PRIVATE cxx_std_17)
