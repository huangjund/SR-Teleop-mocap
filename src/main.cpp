// main.cpp
//
// Simple MocapApi BVH subscriber.
// Splits rendering and mocap logic into lightweight classes while keeping behaviour compact.

#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "MocapClient.h"
#include "SkeletonViewer.h"

int main() {
    const char* SERVER_IP = "127.0.0.1";
    const uint16_t PORT   = 7012;

    const float VIEW_PITCH_DEG = -20.0f;
    const float VIEW_YAW_DEG   = -30.0f;
    const float VIEW_DISTANCE  = 6.0f;
    const SkeletonViewer::SkeletonFilter VIEW_FILTER = SkeletonViewer::SkeletonFilter::FullSkeleton;

    const std::vector<std::string> apiDataTypes = {
        "Avatar motion frames (MCPEvent_AvatarUpdated)",
        "Rigid body transforms (MCPEvent_RigidBodyUpdated)",
        "Sensor module telemetry (MCPEvent_SensorModulesUpdated)",
        "Tracker transforms (MCPEvent_TrackerUpdated)",
        "Marker positions (MCPEvent_MarkerData)",
        "PWR data (MCPEvent_PWRData)",
        "Command replies (MCPEvent_CommandReply)",
        "Notifications (MCPEvent_Notify)"};

    std::cout << "API exposes " << apiDataTypes.size() << " data categories via events:" << std::endl;
    for (const auto& type : apiDataTypes) {
        std::cout << "  - " << type << std::endl;
    }

    MocapClient client(SERVER_IP, PORT);
    if (!client.Initialize()) return 1;

    SkeletonViewer viewer;
    if (!viewer.Create(960, 720)) {
        std::cout << "Continuing without viewer (GLFW unavailable).\n";
    }
    viewer.SetViewAngles(VIEW_PITCH_DEG, VIEW_YAW_DEG);
    viewer.SetViewDistance(VIEW_DISTANCE);
    viewer.SetSkeletonFilter(VIEW_FILTER);

    auto lastPrintTime = std::chrono::steady_clock::now();
    while (viewer.Alive()) {
        client.Poll();
        const auto& joints = client.LatestJoints();

        if (!joints.empty()) viewer.Draw(joints);

        auto now = std::chrono::steady_clock::now();
        auto dt  = std::chrono::duration_cast<std::chrono::seconds>(now - lastPrintTime).count();
        if (dt >= 2) {
            lastPrintTime = now;
            if (joints.empty()) {
                std::cout << "[Info] No avatars yet (check Axis Studio is streaming BVH)...\n";
            } else {
                std::cout << "\n-- Latest joint world positions (computed via FK) --\n";
                for (const auto& joint : joints) {
                    std::cout << "  " << joint.jointName << " [tag " << static_cast<int>(joint.tag) << "]" << std::endl;
                    std::cout << "    World Pos : (" << joint.worldPos.x << ", " << joint.worldPos.y << ", "
                              << joint.worldPos.z << ")\n";
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return 0;
}
