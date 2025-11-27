#pragma once

#include <vector>

#include <GLFW/glfw3.h>

#include "JointData.h"

class SkeletonViewer {
public:
    SkeletonViewer() = default;
    ~SkeletonViewer();

    bool Create(int width, int height);
    bool Alive() const;
    void Draw(const std::vector<JointSample>& joints);

    void SetViewAngles(float pitchDegrees, float yawDegrees);
    void SetViewDistance(float distance);

private:
    float pitchDegrees_ = -20.0f;
    float yawDegrees_   = 0.0f;
    float distance_     = 6.0f;
    GLFWwindow* window_ = nullptr;
};
