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

    IMCPApplication* appIf_   = nullptr;
    IMCPSettings*    settings_ = nullptr;
    IMCPAvatar*      avatarIf_ = nullptr;
    IMCPJoint*       jointIf_  = nullptr;
    IMCPBodyPart*    bodyPart_ = nullptr;

    MCPApplicationHandle_t appHandle_      = 0;
    MCPSettingsHandle_t    settingsHandle_ = 0;

    std::vector<JointSample> latestJoints_;
};
