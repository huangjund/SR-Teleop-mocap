cmake_minimum_required(VERSION 3.15)
project(MocapApiCppDemo LANGUAGES CXX)

# Your executable source
add_executable(mocap_demo
    src/main.cpp
)
# Viewer dependencies
find_package(OpenGL REQUIRED)
find_package(glfw3 CONFIG REQUIRED)

# Include the SDK headers
target_include_directories(mocap_demo PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

# Link against the MocapApi library
target_link_directories(mocap_demo PRIVATE
    ${CMAKE_SOURCE_DIR}/lib/win32/x64
)

target_link_libraries(mocap_demo PRIVATE
    MocapApi
    glfw
    OpenGL::GL
)

# On Windows, use a recent C++ standard
target_compile_features(mocap_demo PRIVATE cxx_std_17)
