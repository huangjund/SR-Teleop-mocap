cmake_minimum_required(VERSION 3.15)
project(MocapApiCppDemo LANGUAGES CXX)

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

# Your executable source
add_executable(mocap_demo
    src/main.cpp
    src/HandMocapReceiver.cpp
)

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
)

add_executable(mocap_hands_viewer
    src/main_viewer.cpp
    src/HandMocapReceiver.cpp
    src/HandVisualizer.cpp
)

target_include_directories(mocap_hands_viewer PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

target_link_directories(mocap_hands_viewer PRIVATE
    ${CMAKE_SOURCE_DIR}/lib/win32/x64
)

target_link_libraries(mocap_hands_viewer PRIVATE
    MocapApi
    glfw
    OpenGL::GL
)

# On Windows, use a recent C++ standard
target_compile_features(mocap_demo PRIVATE cxx_std_17)
target_compile_features(mocap_hands_viewer PRIVATE cxx_std_17)

if (WIN32 AND EXISTS "${CMAKE_SOURCE_DIR}/bin/win32/x64/MocapApi.dll")
    add_custom_command(TARGET mocap_demo POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/bin/win32/x64/MocapApi.dll"
        "$<TARGET_FILE_DIR:mocap_demo>")

    add_custom_command(TARGET mocap_hands_viewer POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/bin/win32/x64/MocapApi.dll"
        "$<TARGET_FILE_DIR:mocap_hands_viewer>")
endif()
