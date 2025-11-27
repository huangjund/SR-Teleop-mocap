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

// MocapApi BVH subscriber starting...
// API exposes 8 data categories via events:
//   - Avatar motion frames (MCPEvent_AvatarUpdated)
//   - Rigid body transforms (MCPEvent_RigidBodyUpdated)
//   - Sensor module telemetry (MCPEvent_SensorModulesUpdated)
//   - Tracker transforms (MCPEvent_TrackerUpdated)
//   - Marker positions (MCPEvent_MarkerData)
//   - PWR data (MCPEvent_PWRData)
//   - Command replies (MCPEvent_CommandReply)
//   - Notifications (MCPEvent_Notify)
// Connected to Axis Studio at 127.0.0.1:7012 (BVH).


#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <optional>
#include <thread>

#include "MocapApi.h"

using namespace MocapApi;

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

                        std::cout << "-- Joint data (BVH broadcast) --\n";
                        for (uint32_t j = 0; j < jointCount; ++j) {
                            MCPJointHandle_t jointHandle = joints[j];
                            auto data = GetJointFullData(jointIf, bodyPartIf, jointHandle);
                            if (!data) continue;

                            std::cout << "  " << data->jointName
                                      << " [tag " << static_cast<int>(data->tag) << "]"
                                      << "\n";
                            std::cout << "    Local Pos : (" << data->localPx << ", "
                                      << data->localPy << ", " << data->localPz << ")\n";
                            std::cout << "    Local Rot : Euler(" << data->localRx << ", "
                                      << data->localRy << ", " << data->localRz
                                      << ")  Quaternion(" << data->localQx << ", "
                                      << data->localQy << ", " << data->localQz
                                      << ", " << data->localQw << ")\n";
                            std::cout << "    World Pos : (" << data->worldPx << ", "
                                      << data->worldPy << ", " << data->worldPz << ")\n";
                            std::cout << "    World Rot : Quaternion(" << data->worldQx
                                      << ", " << data->worldQy << ", " << data->worldQz
                                      << ", " << data->worldQw << ")\n";


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
