// main.cpp
//
// Simple MocapApi BVH subscriber.
// - Connects to Axis Studio BVH broadcast at 127.0.0.1:7012
// - Every 2 seconds (0.5 Hz), prints full BVH joint data (local + world pose)
//
// Build:
//   - Include path: <repo_root>/include
//   - Link with:    <repo_root>/lib/win32/x64/MocapApi.lib
//   - Put          <repo_root>/bin/win32/x64/MocapApi.dll
//     next to your executable (or in PATH).

#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <optional>
#include <functional>
#include <thread>
#include <unordered_map>

#include <GLFW/glfw3.h>

#include "MocapApi.h"

using namespace MocapApi;

// -------------------------------------------------------------------------------------------------
// Minimal math helpers (no external dependencies)
// -------------------------------------------------------------------------------------------------
struct Vec3
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct Quat
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float w = 1.0f;
};

// Quaternion multiplication (combines rotations: parent * child)
Quat Multiply(const Quat& a, const Quat& b)
{
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

// Rotate a vector by a quaternion (q * v * q^-1)
Vec3 Rotate(const Quat& q, const Vec3& v)
{
    Quat qConj{-q.x, -q.y, -q.z, q.w};
    Quat vQuat{v.x, v.y, v.z, 0.0f};
    Quat tmp = Multiply(q, vQuat);
    Quat res = Multiply(tmp, qConj);
    return {res.x, res.y, res.z};
}

struct JointFullData
{
    std::string jointName;
    EMCPJointTag tag = JointTag_Invalid;
    float localPx    = 0.0f;
    float localPy    = 0.0f;
    float localPz    = 0.0f;
    float localRx    = 0.0f;
    float localRy    = 0.0f;
    float localRz    = 0.0f;
    float localQx    = 0.0f;
    float localQy    = 0.0f;
    float localQz    = 0.0f;
    float localQw    = 1.0f;
    float worldPx    = 0.0f;
    float worldPy    = 0.0f;
    float worldPz    = 0.0f;
    float worldQx    = 0.0f;
    float worldQy    = 0.0f;
    float worldQz    = 0.0f;
    float worldQw    = 1.0f;
};

// Simple holder for forward-kinematics data per joint
struct JointSample
{
    std::string jointName;
    EMCPJointTag tag       = JointTag_Invalid;
    EMCPJointTag parentTag = JointTag_Invalid; // Provided by API; JointTag_Invalid means "root"
    Vec3        localPos{};                    // Local to parent
    Quat        localRot{};                    // Local to parent
    Vec3        worldPos{};                    // Computed via forward kinematics
    Quat        worldRot{0.0f, 0.0f, 0.0f, 0.0f};
};

// Global buffer that always stores the most recent joint world positions for the last received frame.
// Other modules can read this directly.
std::vector<JointSample> g_latestJointPositions;

std::optional<JointFullData> GetJointFullData(IMCPJoint*      jointIf,
                                              IMCPBodyPart*   bodyPartIf,
                                              MCPJointHandle_t jointHandle)
{
    if (!jointIf || !bodyPartIf || !jointHandle) return std::nullopt;

    JointFullData data;

    jointIf->GetJointTag(&data.tag, jointHandle);

    const char* jointName = nullptr;
    if (jointIf->GetJointName(&jointName, jointHandle) == Error_None && jointName) {
        data.jointName = jointName;
    } else {
        data.jointName = "<unknown_joint>";
    }

    jointIf->GetJointLocalPosition(&data.localPx, &data.localPy, &data.localPz, jointHandle);
    jointIf->GetJointLocalRotationByEuler(&data.localRx, &data.localRy, &data.localRz, jointHandle);
    jointIf->GetJointLocalRotation(&data.localQx, &data.localQy, &data.localQz, &data.localQw,
                                   jointHandle);

    MCPBodyPartHandle_t bodyPartHandle = 0;
    if (jointIf->GetJointBodyPart(&bodyPartHandle, jointHandle) == Error_None &&
        bodyPartHandle != 0) {
        bodyPartIf->GetJointPosition(&data.worldPx, &data.worldPy, &data.worldPz, bodyPartHandle);
        bodyPartIf->GetBodyPartPosture(&data.worldQx, &data.worldQy, &data.worldQz, &data.worldQw,
                                       bodyPartHandle);
    }

    return data;
}

// Pull local transform data and parent tag for a joint so we can reconstruct world transforms.
std::optional<JointSample> GetJointSample(IMCPJoint*       jointIf,
                                          IMCPBodyPart*    bodyPartIf,
                                          MCPJointHandle_t jointHandle)
{
    if (!jointIf || !bodyPartIf || !jointHandle) return std::nullopt;

    JointSample sample;

    const char* name = nullptr;
    if (jointIf->GetJointName(&name, jointHandle) == Error_None && name) {
        sample.jointName = name;
    } else {
        sample.jointName = "<unknown_joint>";
    }

    jointIf->GetJointTag(&sample.tag, jointHandle);
    jointIf->GetJointLocalPosition(&sample.localPos.x, &sample.localPos.y, &sample.localPos.z, jointHandle);
    jointIf->GetJointLocalRotation(&sample.localRot.x, &sample.localRot.y, &sample.localRot.z, &sample.localRot.w,
                                   jointHandle);

    // Parent tag is needed for forward kinematics. If the API cannot provide it, we treat it as a root.
    EMCPJointTag parentTag = JointTag_Invalid;
    if (jointIf->GetJointParentJointTag(&parentTag, sample.tag) == Error_None) {
        sample.parentTag = parentTag;
    }

    return sample;
}

// Compute world-space transforms in-place using a simple depth-first traversal over the tag hierarchy.
void ComputeWorldTransforms(std::vector<JointSample>& joints)
{
    std::unordered_map<int, size_t> tagToIndex;
    tagToIndex.reserve(joints.size());
    for (size_t i = 0; i < joints.size(); ++i) {
        tagToIndex[static_cast<int>(joints[i].tag)] = i;
    }

    std::function<void(size_t)> resolve = [&](size_t idx) {
        JointSample& j = joints[idx];
        // Already computed
        if (j.worldRot.w != 0.0f || j.worldRot.x != 0.0f || j.worldRot.y != 0.0f || j.worldRot.z != 0.0f) {
            return;
        }

        // If we have a parent, make sure it is computed first.
        if (j.parentTag != JointTag_Invalid) {
            auto it = tagToIndex.find(static_cast<int>(j.parentTag));
            if (it != tagToIndex.end()) {
                resolve(it->second);
                const JointSample& parent = joints[it->second];
                j.worldRot              = Multiply(parent.worldRot, j.localRot);
                j.worldPos              = parent.worldPos;
                Vec3 rotatedLocalOffset = Rotate(parent.worldRot, j.localPos);
                j.worldPos.x += rotatedLocalOffset.x;
                j.worldPos.y += rotatedLocalOffset.y;
                j.worldPos.z += rotatedLocalOffset.z;
                return;
            }
        }

        // No valid parent (treat as root)
        j.worldRot = j.localRot;
        j.worldPos = j.localPos;
    };

    for (size_t i = 0; i < joints.size(); ++i) {
        resolve(i);
    }
}

// -------------------------------------------------------------------------------------------------
// Minimal OpenGL viewer helpers (GLFW + fixed-function pipeline)
// -------------------------------------------------------------------------------------------------
GLFWwindow* CreateViewer(int width, int height)
{
    if (!glfwInit()) {
        std::cerr << "Failed to init GLFW (viewer disabled).\n";
        return nullptr;
    }

    // Old-school fixed-function pipeline is enough for lines.
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    GLFWwindow* window = glfwCreateWindow(width, height, "Mocap Skeleton Viewer", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window (viewer disabled).\n";
        glfwTerminate();
        return nullptr;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    glEnable(GL_DEPTH_TEST);
    glLineWidth(2.0f);

    return window;
}

void DrawSkeletonLines(const std::vector<JointSample>& joints)
{
    // Build a quick lookup from joint tag to index so we can connect child->parent.
    std::unordered_map<int, size_t> tagToIndex;
    tagToIndex.reserve(joints.size());
    for (size_t i = 0; i < joints.size(); ++i) {
        tagToIndex[static_cast<int>(joints[i].tag)] = i;
    }

    glBegin(GL_LINES);
    glColor3f(0.1f, 0.9f, 0.6f);

    for (const auto& j : joints) {
        if (j.parentTag == JointTag_Invalid) continue;
        auto it = tagToIndex.find(static_cast<int>(j.parentTag));
        if (it == tagToIndex.end()) continue;

        const Vec3& a = j.worldPos;
        const Vec3& b = joints[it->second].worldPos;

        // Simple scale to keep units visible on screen (BVH is typically in cm)
        const float scale = 0.01f;
        glVertex3f(a.x * scale, a.y * scale, a.z * scale);
        glVertex3f(b.x * scale, b.y * scale, b.z * scale);
    }

    glEnd();
}

int main()
{
    const char* SERVER_IP = "127.0.0.1";
    const uint16_t PORT   = 7012;     // BVH broadcast port

    std::cout << "MocapApi BVH subscriber starting...\n";

    // Surface how many data categories the API exposes through its event system.
    const std::vector<std::string> apiDataTypes = {
        "Avatar motion frames (MCPEvent_AvatarUpdated)",
        "Rigid body transforms (MCPEvent_RigidBodyUpdated)",
        "Sensor module telemetry (MCPEvent_SensorModulesUpdated)",
        "Tracker transforms (MCPEvent_TrackerUpdated)",
        "Marker positions (MCPEvent_MarkerData)",
        "PWR data (MCPEvent_PWRData)",
        "Command replies (MCPEvent_CommandReply)",
        "Notifications (MCPEvent_Notify)"};

    std::cout << "API exposes " << apiDataTypes.size()
              << " data categories via events:" << std::endl;
    for (const auto& type : apiDataTypes) {
        std::cout << "  - " << type << std::endl;
    }

    EMCPError err = Error_None;

    // === 1. Get core interfaces ===
    IMCPApplication* app         = nullptr;
    IMCPSettings*    settingsIf  = nullptr;
    IMCPAvatar*      avatarIf    = nullptr;
    IMCPJoint*       jointIf     = nullptr;
    IMCPBodyPart*    bodyPartIf  = nullptr;

    err = MCPGetGenericInterface(IMCPApplication_Version,
                                 reinterpret_cast<void**>(&app));
    if (err != Error_None || !app) {
        std::cerr << "Failed to get IMCPApplication, err="
                  << static_cast<int>(err) << "\n";
        return 1;
    }

    err = MCPGetGenericInterface(IMCPSettings_Version,
                                 reinterpret_cast<void**>(&settingsIf));
    if (err != Error_None || !settingsIf) {
        std::cerr << "Failed to get IMCPSettings, err="
                  << static_cast<int>(err) << "\n";
        return 1;
    }

    err = MCPGetGenericInterface(IMCPAvatar_Version,
                                 reinterpret_cast<void**>(&avatarIf));
    if (err != Error_None || !avatarIf) {
        std::cerr << "Failed to get IMCPAvatar, err="
                  << static_cast<int>(err) << "\n";
        return 1;
    }

    err = MCPGetGenericInterface(IMCPJoint_Version,
                                 reinterpret_cast<void**>(&jointIf));
    if (err != Error_None || !jointIf) {
        std::cerr << "Failed to get IMCPJoint, err="
                  << static_cast<int>(err) << "\n";
        return 1;
    }

    err = MCPGetGenericInterface(IMCPBodyPart_Version,
                                 reinterpret_cast<void**>(&bodyPartIf));
    if (err != Error_None || !bodyPartIf) {
        std::cerr << "Failed to get IMCPBodyPart, err="
                  << static_cast<int>(err) << "\n";
        return 1;
    }

    // === 2. Create Application + Settings ===
    MCPApplicationHandle_t appHandle = 0;
    err = app->CreateApplication(&appHandle);
    if (err != Error_None) {
        std::cerr << "CreateApplication failed, err="
                  << static_cast<int>(err) << "\n";
        return 1;
    }

    MCPSettingsHandle_t settingsHandle = 0;
    err = settingsIf->CreateSettings(&settingsHandle);
    if (err != Error_None) {
        std::cerr << "CreateSettings failed, err="
                  << static_cast<int>(err) << "\n";
        return 1;
    }

    // === 3. Configure settings for BVH over UDP ===
    //
    // NOTE: The exact signatures may differ slightly;
    // check MocapApi.h if you get compile errors.

    // Listen on PORT and connect to Axis Studio at SERVER_IP:PORT
    err = settingsIf->SetSettingsUDP(PORT, settingsHandle);
    if (err != Error_None) {
        std::cerr << "SetSettingsUDP failed, err="
                  << static_cast<int>(err) << "\n";
        return 1;
    }

    err = settingsIf->SetSettingsUDPServer(SERVER_IP, PORT, settingsHandle);
    if (err != Error_None) {
        std::cerr << "SetSettingsUDPServer failed, err="
                  << static_cast<int>(err) << "\n";
        return 1;
    }

    // BVH rotation order (match your Axis Studio setting)
    err = settingsIf->SetSettingsBvhRotation(BvhRotation_XYZ, settingsHandle);
    if (err != Error_None) {
        std::cerr << "SetSettingsBvhRotation failed, err="
                  << static_cast<int>(err) << "\n";
        return 1;
    }

    // BVH data format (Binary is typical; change to BvhDataType_String if needed)
    err = settingsIf->SetSettingsBvhData(BvhDataType_Binary, settingsHandle);
    if (err != Error_None) {
        std::cerr << "SetSettingsBvhData failed, err="
                  << static_cast<int>(err) << "\n";
        return 1;
    }

    // BVH transformation (enable world-space transform if desired)
    err = settingsIf->SetSettingsBvhTransformation(BvhTransformation_Enable,
                                                   settingsHandle);
    if (err != Error_None) {
        std::cerr << "SetSettingsBvhTransformation failed, err="
                  << static_cast<int>(err) << "\n";
        return 1;
    }

    // Bind settings to the application
    err = app->SetApplicationSettings(settingsHandle, appHandle);
    if (err != Error_None) {
        std::cerr << "SetApplicationSettings failed, err="
                  << static_cast<int>(err) << "\n";
        return 1;
    }

    // === 4. Open application (connect to Axis Studio) ===
    err = app->OpenApplication(appHandle);
    if (err != Error_None) {
        std::cerr << "OpenApplication failed, err="
                  << static_cast<int>(err) << "\n";
        return 1;
    }

    std::cout << "Connected to Axis Studio at "
              << SERVER_IP << ":" << PORT << " (BVH).\n";
    std::cout << "Press Ctrl+C to stop.\n";

    GLFWwindow* viewerWindow = CreateViewer(960, 720);
    if (!viewerWindow) {
        std::cout << "Continuing without viewer (GLFW unavailable).\n";
    }

    // Timer for 0.5 Hz printing (every 2 seconds)
    auto lastPrintTime = std::chrono::steady_clock::now();

    // === 5. Main loop ===
    while (!viewerWindow || !glfwWindowShouldClose(viewerWindow)) {
        // --- Pump events so MocapApi processes incoming data ---
        // We don't care about event types here; we just want data to update.
        MCPEvent_t events[16];
        uint32_t eventCount = 16;
        for (uint32_t i = 0; i < eventCount; ++i) {
            events[i].size = sizeof(MCPEvent_t);
        }

        err = app->PollApplicationNextEvent(events, &eventCount, appHandle);
        // Ignore most errors here; if something is seriously wrong, you'll see no data.
        (void)err;

        // Grab the latest avatar state every loop so we always have fresh joint positions.
        uint32_t avatarCount = 0;
        err = app->GetApplicationAvatars(nullptr, &avatarCount, appHandle);
        if (err == Error_None && avatarCount > 0) {
            std::vector<MCPAvatarHandle_t> avatars(avatarCount);
            if (app->GetApplicationAvatars(avatars.data(), &avatarCount, appHandle) == Error_None) {
                g_latestJointPositions.clear();

                for (uint32_t i = 0; i < avatarCount; ++i) {
                    MCPAvatarHandle_t avatarHandle = avatars[i];

                    uint32_t jointCount = 0;
                    if (avatarIf->GetAvatarJoints(nullptr, &jointCount, avatarHandle) != Error_None ||
                        jointCount == 0) {
                        continue;
                    }

                    std::vector<MCPJointHandle_t> joints(jointCount);
                    if (avatarIf->GetAvatarJoints(joints.data(), &jointCount, avatarHandle) != Error_None) {
                        continue;
                    }

                    // Build FK inputs (local transform + hierarchy)
                    std::vector<JointSample> samples;
                    samples.reserve(jointCount);
                    for (uint32_t j = 0; j < jointCount; ++j) {
                        auto sample = GetJointSample(jointIf, bodyPartIf, joints[j]);
                        if (sample) samples.push_back(*sample);
                    }

                    // Compute world poses per joint and stash globally for other systems.
                    ComputeWorldTransforms(samples);
                    g_latestJointPositions.insert(g_latestJointPositions.end(), samples.begin(), samples.end());
                }
            }
        }

        // --- Render live skeleton ---
        if (viewerWindow) {
            int fbWidth = 0, fbHeight = 0;
            glfwGetFramebufferSize(viewerWindow, &fbWidth, &fbHeight);
            glViewport(0, 0, fbWidth, fbHeight);

            glClearColor(0.05f, 0.05f, 0.08f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            float aspect = fbHeight > 0 ? static_cast<float>(fbWidth) / fbHeight : 1.0f;
            const float viewSize = 2.5f;
            glOrtho(-viewSize * aspect, viewSize * aspect, -viewSize, viewSize, -20.0f, 20.0f);

            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            glTranslatef(0.0f, -0.5f, -6.0f);
            glRotatef(-20.0f, 1.0f, 0.0f, 0.0f);
            glRotatef(static_cast<float>(glfwGetTime()) * 10.0f, 0.0f, 1.0f, 0.0f);

            if (!g_latestJointPositions.empty()) {
                DrawSkeletonLines(g_latestJointPositions);
            }

            glfwSwapBuffers(viewerWindow);
            glfwPollEvents();
        }

        // --- Decide if it's time to print (every 2 seconds) ---
        auto now = std::chrono::steady_clock::now();
        auto dt  = std::chrono::duration_cast<std::chrono::seconds>(now - lastPrintTime).count();

        if (dt >= 2) {
            lastPrintTime = now;

            if (g_latestJointPositions.empty()) {
                std::cout << "[Info] No avatars yet (check Axis Studio is streaming BVH)...\n";
            } else {
                std::cout << "\n-- Latest joint world positions (computed via FK) --\n";
                for (const auto& joint : g_latestJointPositions) {
                    std::cout << "  " << joint.jointName
                              << " [tag " << static_cast<int>(joint.tag) << "]" << std::endl;
                    std::cout << "    World Pos : (" << joint.worldPos.x << ", "
                              << joint.worldPos.y << ", " << joint.worldPos.z << ")\n";
                }
            }
        }

        // Small sleep to avoid busy-waiting (network thread still runs)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // === 6. Cleanup (normally unreachable unless you add a break) ===
    if (viewerWindow) {
        glfwDestroyWindow(viewerWindow);
        glfwTerminate();
    }

    settingsIf->DestroySettings(settingsHandle);
    app->CloseApplication(appHandle);
    app->DestroyApplication(appHandle);

    return 0;
}
