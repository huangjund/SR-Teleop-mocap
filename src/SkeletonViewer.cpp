#include "SkeletonViewer.h"

#include <iostream>
#include <unordered_map>

SkeletonViewer::~SkeletonViewer() {
    if (window_) {
        glfwDestroyWindow(window_);
        glfwTerminate();
    }
}

bool SkeletonViewer::Create(int width, int height) {
    if (!glfwInit()) {
        std::cerr << "Failed to init GLFW (viewer disabled).\n";
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    window_ = glfwCreateWindow(width, height, "Mocap Skeleton Viewer", nullptr, nullptr);
    if (!window_) {
        std::cerr << "Failed to create GLFW window (viewer disabled).\n";
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    glEnable(GL_DEPTH_TEST);
    glLineWidth(2.0f);

    return true;
}

bool SkeletonViewer::Alive() const {
    return !window_ || !glfwWindowShouldClose(window_);
}

void SkeletonViewer::Draw(const std::vector<JointSample>& joints) {
    if (!window_) return;

    int fbWidth = 0, fbHeight = 0;
    glfwGetFramebufferSize(window_, &fbWidth, &fbHeight);
    glViewport(0, 0, fbWidth, fbHeight);

    glClearColor(0.05f, 0.05f, 0.08f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float aspect      = fbHeight > 0 ? static_cast<float>(fbWidth) / fbHeight : 1.0f;
    const float scale = 2.5f;
    glOrtho(-scale * aspect, scale * aspect, -scale, scale, -20.0f, 20.0f);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0.0f, -0.5f, -6.0f);
    glRotatef(-20.0f, 1.0f, 0.0f, 0.0f);
    glRotatef(static_cast<float>(glfwGetTime()) * 10.0f, 0.0f, 1.0f, 0.0f);

    std::unordered_map<int, size_t> tagToIndex;
    tagToIndex.reserve(joints.size());
    for (size_t i = 0; i < joints.size(); ++i) {
        tagToIndex[static_cast<int>(joints[i].tag)] = i;
    }

    glBegin(GL_LINES);
    glColor3f(0.1f, 0.9f, 0.6f);
    for (const auto& j : joints) {
        if (j.parentTag == MocapApi::JointTag_Invalid) continue;
        auto it = tagToIndex.find(static_cast<int>(j.parentTag));
        if (it == tagToIndex.end()) continue;

        const Vec3& a = j.worldPos;
        const Vec3& b = joints[it->second].worldPos;
        const float s = 0.01f;
        glVertex3f(a.x * s, a.y * s, a.z * s);
        glVertex3f(b.x * s, b.y * s, b.z * s);
    }
    glEnd();

    glfwSwapBuffers(window_);
    glfwPollEvents();
}
