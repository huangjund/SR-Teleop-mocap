#include "SkeletonViewer.h"

#include <iostream>
#include <unordered_map>

namespace {
bool IsHandJoint(MocapApi::EMCPJointTag tag) {
    switch (tag) {
        case MocapApi::JointTag_RightHand:
        case MocapApi::JointTag_RightHandThumb1:
        case MocapApi::JointTag_RightHandThumb2:
        case MocapApi::JointTag_RightHandThumb3:
        case MocapApi::JointTag_RightInHandIndex:
        case MocapApi::JointTag_RightHandIndex1:
        case MocapApi::JointTag_RightHandIndex2:
        case MocapApi::JointTag_RightHandIndex3:
        case MocapApi::JointTag_RightInHandMiddle:
        case MocapApi::JointTag_RightHandMiddle1:
        case MocapApi::JointTag_RightHandMiddle2:
        case MocapApi::JointTag_RightHandMiddle3:
        case MocapApi::JointTag_RightInHandRing:
        case MocapApi::JointTag_RightHandRing1:
        case MocapApi::JointTag_RightHandRing2:
        case MocapApi::JointTag_RightHandRing3:
        case MocapApi::JointTag_RightInHandPinky:
        case MocapApi::JointTag_RightHandPinky1:
        case MocapApi::JointTag_RightHandPinky2:
        case MocapApi::JointTag_RightHandPinky3:
        case MocapApi::JointTag_LeftHand:
        case MocapApi::JointTag_LeftHandThumb1:
        case MocapApi::JointTag_LeftHandThumb2:
        case MocapApi::JointTag_LeftHandThumb3:
        case MocapApi::JointTag_LeftInHandIndex:
        case MocapApi::JointTag_LeftHandIndex1:
        case MocapApi::JointTag_LeftHandIndex2:
        case MocapApi::JointTag_LeftHandIndex3:
        case MocapApi::JointTag_LeftInHandMiddle:
        case MocapApi::JointTag_LeftHandMiddle1:
        case MocapApi::JointTag_LeftHandMiddle2:
        case MocapApi::JointTag_LeftHandMiddle3:
        case MocapApi::JointTag_LeftInHandRing:
        case MocapApi::JointTag_LeftHandRing1:
        case MocapApi::JointTag_LeftHandRing2:
        case MocapApi::JointTag_LeftHandRing3:
        case MocapApi::JointTag_LeftInHandPinky:
        case MocapApi::JointTag_LeftHandPinky1:
        case MocapApi::JointTag_LeftHandPinky2:
        case MocapApi::JointTag_LeftHandPinky3:
            return true;
        default:
            return false;
    }
}
}  // namespace

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

void SkeletonViewer::SetViewAngles(float pitchDegrees, float yawDegrees) {
    pitchDegrees_ = pitchDegrees;
    yawDegrees_   = yawDegrees;
}

void SkeletonViewer::SetViewDistance(float distance) {
    distance_ = distance;
}

void SkeletonViewer::Draw(const std::vector<JointSample>& joints) {
    if (!window_) return;

    const bool drawHands = filter_ != SkeletonFilter::BodyOnly;
    const bool drawBody  = filter_ != SkeletonFilter::HandsOnly;

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
    glTranslatef(0.0f, -0.5f, -distance_);
    glRotatef(pitchDegrees_, 1.0f, 0.0f, 0.0f);
    glRotatef(yawDegrees_, 0.0f, 1.0f, 0.0f);

    std::unordered_map<int, size_t> tagToIndex;
    tagToIndex.reserve(joints.size());
    for (size_t i = 0; i < joints.size(); ++i) {
        tagToIndex[static_cast<int>(joints[i].tag)] = i;
    }

    glBegin(GL_LINES);
    glColor3f(0.1f, 0.9f, 0.6f);
    for (const auto& j : joints) {
        if (j.parentTag == MocapApi::JointTag_Invalid) continue;

        const bool isHand      = IsHandJoint(j.tag);
        const bool parentIsHand = IsHandJoint(j.parentTag);

        if ((isHand && !drawHands) || (!isHand && !drawBody)) continue;
        if ((parentIsHand && !drawHands) || (!parentIsHand && !drawBody)) continue;

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
