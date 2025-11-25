// main.cpp
//
// Simple MocapApi BVH subscriber.
// - Connects to Axis Studio BVH broadcast at 127.0.0.1:7012
// - Every 1 second, prints all joints' local Euler angles
//
// Build:
//   - Include path: <repo_root>/include
//   - Link with:    <repo_root>/lib/win32/x64/MocapApi.lib
//   - Put          <repo_root>/bin/win32/x64/MocapApi.dll
//     next to your executable (or in PATH).

#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>

#include "MocapApi.h"

using namespace MocapApi;

// Helper: print one joint and recurse on children
void DumpJointTree(IMCPJoint* jointIf,
                   MCPJointHandle_t jointHandle,
                   int indentSpaces)
{
    if (!jointIf || !jointHandle) return;

    EMCPError err = Error_None;

    // Joint name
    const char* jointName = nullptr;
    err = jointIf->GetJointName(&jointName, jointHandle);
    if (err != Error_None) {
        jointName = "<unknown_joint>";
    }

    // Local Euler rotation
    float rx = 0.0f, ry = 0.0f, rz = 0.0f;
    err = jointIf->GetJointLocalRotationByEuler(&rx, &ry, &rz, jointHandle);
    // If it fails, we still print name but skip angles

    std::string indent(indentSpaces, ' ');
    std::cout << indent << (jointName ? jointName : "<null>")
              << " : Euler(" << rx << ", " << ry << ", " << rz << ")\n";

    // Children
    MCPJointHandle_t childJoints[64];
    uint32_t childCount = 64;
    err = jointIf->GetJointChild(childJoints, &childCount, jointHandle);
    if (err != Error_None || childCount == 0) {
        return; // no children or error
    }

    for (uint32_t i = 0; i < childCount; ++i) {
        if (childJoints[i]) {
            DumpJointTree(jointIf, childJoints[i], indentSpaces + 2);
        }
    }
}

int main()
{
    const char* SERVER_IP = "127.0.0.1";
    const uint16_t PORT   = 7012;     // BVH broadcast port

    std::cout << "MocapApi BVH subscriber starting...\n";

    EMCPError err = Error_None;

    // === 1. Get core interfaces ===
    IMCPApplication* app        = nullptr;
    IMCPSettings*    settingsIf = nullptr;
    IMCPAvatar*      avatarIf   = nullptr;
    IMCPJoint*       jointIf    = nullptr;

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

    // Timer for 1 Hz printing
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

        // --- Decide if it's time to print (once per second) ---
        auto now = std::chrono::steady_clock::now();
        auto dt  = std::chrono::duration_cast<std::chrono::seconds>(now - lastPrintTime).count();

        if (dt >= 10) {
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

                        // Get root joint
                        MCPJointHandle_t rootJoint = 0;
                        err = avatarIf->GetAvatarRootJoint(&rootJoint, avatarHandle);
                        if (err != Error_None || !rootJoint) {
                            std::cerr << "GetAvatarRootJoint failed, err="
                                      << static_cast<int>(err) << "\n";
                        } else {
                            // Print entire joint tree from root
                            DumpJointTree(jointIf, rootJoint, 0);
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
