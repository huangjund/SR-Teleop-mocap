cmake_minimum_required(VERSION 3.15)
project(MocapApiCppDemo LANGUAGES CXX)

add_executable(mocap_demo
    src/main.cpp
    src/MocapClient.cpp
    src/JointData.cpp
    src/SkeletonViewer.cpp
)

add_executable(mocap_teleop
    src/teleop_main.cpp
    src/MocapClient.cpp
    src/JointData.cpp
    src/TeleopMapping.cpp
)

find_package(OpenGL REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(pinocchio CONFIG REQUIRED)

target_include_directories(mocap_demo PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

target_include_directories(mocap_teleop PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

target_link_directories(mocap_demo PRIVATE
    ${CMAKE_SOURCE_DIR}/lib/win32/x64
)

target_link_directories(mocap_teleop PRIVATE
    ${CMAKE_SOURCE_DIR}/lib/win32/x64
)

target_link_libraries(mocap_demo PRIVATE
    MocapApi
    glfw
    OpenGL::GL
)

target_compile_features(mocap_demo PRIVATE cxx_std_17)

target_link_libraries(mocap_teleop PRIVATE
    MocapApi
    pinocchio::pinocchio
)

file(TO_CMAKE_PATH "${CMAKE_SOURCE_DIR}" PROJECT_SOURCE_DIR_NORMALIZED)
target_compile_definitions(mocap_teleop PRIVATE PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR_NORMALIZED}")

if (WIN32)
    target_link_libraries(mocap_teleop PRIVATE Ws2_32)
endif()

target_compile_features(mocap_teleop PRIVATE cxx_std_17)
