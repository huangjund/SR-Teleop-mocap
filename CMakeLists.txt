cmake_minimum_required(VERSION 3.15)
project(MocapApiCppDemo LANGUAGES CXX)

find_package(OpenGL REQUIRED)
# Prefer a CMake config package for GLFW if available; otherwise fall back to
# pkg-config (common on Linux distributions).
find_package(glfw3 QUIET)
if (NOT glfw3_FOUND)
    find_package(PkgConfig QUIET)
    if (PkgConfig_FOUND)
        pkg_check_modules(GLFW3 glfw3)
    endif()
endif()

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
    OpenGL::GL
)

if (TARGET glfw)
    target_link_libraries(mocap_hands_viewer PRIVATE glfw)
elseif (TARGET glfw3)
    target_link_libraries(mocap_hands_viewer PRIVATE glfw3)
elseif (TARGET PkgConfig::GLFW3)
    target_link_libraries(mocap_hands_viewer PRIVATE PkgConfig::GLFW3)
    target_include_directories(mocap_hands_viewer PRIVATE ${GLFW3_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "GLFW dependency not found. Provide glfw3Config.cmake or install glfw3 via pkg-config.")
endif()

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
