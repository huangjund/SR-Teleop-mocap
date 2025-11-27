// main.cpp
//
// Simple MocapApi BVH subscriber.
// - Connects to Axis Studio BVH broadcast at 127.0.0.1:7012
// - Every 2 seconds (0.5 Hz), prints hand joint Euler angles and wrist 6D pose
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
#include <thread>

#include "MocapApi.h"

using namespace MocapApi;

struct WristPose
{
    std::string jointName;
    float px = 0.0f;
    float py = 0.0f;
    float pz = 0.0f;
    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;
    float qw = 1.0f;
};

std::optional<WristPose> GetWristPose(IMCPJoint*     jointIf,
                                      IMCPBodyPart* bodyPartIf,
                                      MCPJointHandle_t jointHandle)
{
    if (!jointIf || !bodyPartIf || !jointHandle) return std::nullopt;

    WristPose pose;

    const char* jointName = nullptr;
    if (jointIf->GetJointName(&jointName, jointHandle) == Error_None && jointName) {
        pose.jointName = jointName;
    } else {
        pose.jointName = "<unknown_wrist>";
    }

    MCPBodyPartHandle_t bodyPartHandle = 0;
    if (jointIf->GetJointBodyPart(&bodyPartHandle, jointHandle) != Error_None ||
        bodyPartHandle == 0) {
        return std::nullopt;
    }

    if (bodyPartIf->GetJointPosition(&pose.px, &pose.py, &pose.pz, bodyPartHandle) !=
        Error_None) {
        return std::nullopt;
    }

    if (bodyPartIf->GetBodyPartPosture(&pose.qx, &pose.qy, &pose.qz, &pose.qw,
                                       bodyPartHandle) != Error_None) {
        return std::nullopt;
    }

    return pose;
}

bool IsRightHandTag(EMCPJointTag tag)
{
    return tag >= JointTag_RightHand && tag <= JointTag_RightHandPinky3;
}

bool IsLeftHandTag(EMCPJointTag tag)
{
    return tag >= JointTag_LeftHand && tag <= JointTag_LeftHandPinky3;
}

int main()
{
    const char* SERVER_IP = "127.0.0.1";
    const uint16_t PORT   = 7012;     // BVH broadcast port

    std::cout << "MocapApi BVH subscriber starting...\n";

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

    // Timer for 0.5 Hz printing (every 2 seconds)
    auto lastPrintTime = std::chrono::steady_clock::now();

    // === 5. Main loop ===
    while (true) {
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

        // --- Decide if it's time to print (every 2 seconds) ---
        auto now = std::chrono::steady_clock::now();
        auto dt  = std::chrono::duration_cast<std::chrono::seconds>(now - lastPrintTime).count();

        if (dt >= 2) {
            lastPrintTime = now;

            // === Query avatars ===
            uint32_t avatarCount = 0;
            err = app->GetApplicationAvatars(nullptr, &avatarCount, appHandle);
            if (err != Error_None || avatarCount == 0) {
                std::cout << "[Info] No avatars yet (check Axis Studio is streaming BVH)...\n";
            } else {
                std::vector<MCPAvatarHandle_t> avatars(avatarCount);
                err = app->GetApplicationAvatars(avatars.data(), &avatarCount, appHandle);
                if (err != Error_None) {
                    std::cerr << "GetApplicationAvatars failed, err="
                              << static_cast<int>(err) << "\n";
                } else {
                    for (uint32_t i = 0; i < avatarCount; ++i) {
                        MCPAvatarHandle_t avatarHandle = avatars[i];
                        const char* avatarName = nullptr;
                        avatarIf->GetAvatarName(&avatarName, avatarHandle);

                        std::cout << "\n=== Avatar "
                                  << (avatarName ? avatarName : "<unnamed>")
                                  << " ===\n";

                        uint32_t jointCount = 0;
                        err = avatarIf->GetAvatarJoints(nullptr, &jointCount, avatarHandle);
                        if (err != Error_None || jointCount == 0) {
                            std::cout << "[Info] No joints yet for this avatar.\n";
                            continue;
                        }

                        std::vector<MCPJointHandle_t> joints(jointCount);
                        err = avatarIf->GetAvatarJoints(joints.data(), &jointCount, avatarHandle);
                        if (err != Error_None) {
                            std::cerr << "GetAvatarJoints failed, err="
                                      << static_cast<int>(err) << "\n";
                            continue;
                        }

                        MCPJointHandle_t leftWrist  = 0;
                        MCPJointHandle_t rightWrist = 0;

                        std::cout << "-- Hand joint Euler angles --\n";
                        for (uint32_t j = 0; j < jointCount; ++j) {
                            MCPJointHandle_t jointHandle = joints[j];
                            EMCPJointTag tag = JointTag_Invalid;
                            if (jointIf->GetJointTag(&tag, jointHandle) != Error_None) {
                                continue;
                            }

                            if (tag == JointTag_LeftHand) {
                                leftWrist = jointHandle;
                            } else if (tag == JointTag_RightHand) {
                                rightWrist = jointHandle;
                            }

                            if (!IsLeftHandTag(tag) && !IsRightHandTag(tag)) {
                                continue;
                            }

                            const char* name = nullptr;
                            jointIf->GetJointName(&name, jointHandle);

                            float rx = 0.0f, ry = 0.0f, rz = 0.0f;
                            jointIf->GetJointLocalRotationByEuler(&rx, &ry, &rz, jointHandle);

                            std::cout << "  "
                                      << (name ? name : "<joint>")
                                      << " : Euler(" << rx << ", " << ry
                                      << ", " << rz << ")\n";
                        }

                        std::cout << "-- Wrist 6D pose (position + quaternion) --\n";

                        auto rightPose = GetWristPose(jointIf, bodyPartIf, rightWrist);
                        if (rightPose) {
                            std::cout << "  Right wrist [" << rightPose->jointName
                                      << "]: Pos(" << rightPose->px << ", "
                                      << rightPose->py << ", " << rightPose->pz
                                      << ")  Rot(" << rightPose->qx << ", "
                                      << rightPose->qy << ", " << rightPose->qz
                                      << ", " << rightPose->qw << ")\n";
                        } else {
                            std::cout << "  Right wrist data unavailable.\n";
                        }

                        auto leftPose = GetWristPose(jointIf, bodyPartIf, leftWrist);
                        if (leftPose) {
                            std::cout << "  Left wrist [" << leftPose->jointName
                                      << "]: Pos(" << leftPose->px << ", "
                                      << leftPose->py << ", " << leftPose->pz
                                      << ")  Rot(" << leftPose->qx << ", "
                                      << leftPose->qy << ", " << leftPose->qz
                                      << ", " << leftPose->qw << ")\n";
                        } else {
                            std::cout << "  Left wrist data unavailable.\n";
                        }
                    }
                }
            }
        }

        // Small sleep to avoid busy-waiting (network thread still runs)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // === 6. Cleanup (normally unreachable unless you add a break) ===
    settingsIf->DestroySettings(settingsHandle);
    app->CloseApplication(appHandle);
    app->DestroyApplication(appHandle);

    return 0;
}
