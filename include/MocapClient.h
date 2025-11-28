#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "JointData.h"

class MocapClient {
public:
    MocapClient(std::string serverIp, uint16_t port);
    ~MocapClient();

    bool Initialize();
    void Shutdown();
    bool Poll();

    const std::vector<JointSample>& LatestJoints() const { return latestJoints_; }

private:
    bool AcquireInterfaces();
    bool CreateApplication();
    bool ConfigureSettings();
    bool FetchAvatarData();

    std::string serverIp_;
    uint16_t    port_;

    MocapApi::IMCPApplication* appIf_    = nullptr;
    MocapApi::IMCPSettings*    settings_ = nullptr;
    MocapApi::IMCPAvatar*      avatarIf_ = nullptr;
    MocapApi::IMCPJoint*       jointIf_  = nullptr;
    MocapApi::IMCPBodyPart*    bodyPart_ = nullptr;
    MocapApi::IMCPRenderSettings* renderSettingsIf_ = nullptr;

    MocapApi::MCPApplicationHandle_t appHandle_      = 0;
    MocapApi::MCPSettingsHandle_t    settingsHandle_ = 0;
    MocapApi::MCPRenderSettingsHandle_t renderHandle_ = 0;

    std::vector<JointSample> latestJoints_;
};
