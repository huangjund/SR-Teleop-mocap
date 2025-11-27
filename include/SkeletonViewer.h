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

private:
    GLFWwindow* window_ = nullptr;
};
