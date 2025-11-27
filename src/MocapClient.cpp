#include "MocapClient.h"

#include <chrono>
#include <iostream>
#include <thread>

MocapClient::MocapClient(std::string serverIp, uint16_t port) : serverIp_(std::move(serverIp)), port_(port) {}

MocapClient::~MocapClient() { Shutdown(); }

bool MocapClient::Initialize() {
    std::cout << "MocapApi BVH subscriber starting...\n";

    if (!AcquireInterfaces()) return false;
    if (!CreateApplication()) return false;
    if (!ConfigureSettings()) return false;

    MocapApi::EMCPError err = appIf_->OpenApplication(appHandle_);
    if (err != MocapApi::Error_None) {
        std::cerr << "OpenApplication failed, err=" << static_cast<int>(err) << "\n";
        return false;
    }

    std::cout << "Connected to Axis Studio at " << serverIp_ << ":" << port_ << " (BVH).\n";
    std::cout << "Press Ctrl+C to stop.\n";
    return true;
}

void MocapClient::Shutdown() {
    if (settings_) settings_->DestroySettings(settingsHandle_);
    if (appIf_) {
        appIf_->CloseApplication(appHandle_);
        appIf_->DestroyApplication(appHandle_);
    }
}

bool MocapClient::AcquireInterfaces() {
    MocapApi::EMCPError err = MocapApi::Error_None;

    err = MocapApi::MCPGetGenericInterface(MocapApi::IMCPApplication_Version, reinterpret_cast<void**>(&appIf_));
    if (err != MocapApi::Error_None || !appIf_) {
        std::cerr << "Failed to get IMCPApplication, err=" << static_cast<int>(err) << "\n";
        return false;
    }

    err = MocapApi::MCPGetGenericInterface(MocapApi::IMCPSettings_Version, reinterpret_cast<void**>(&settings_));
    if (err != MocapApi::Error_None || !settings_) {
        std::cerr << "Failed to get IMCPSettings, err=" << static_cast<int>(err) << "\n";
        return false;
    }

    err = MocapApi::MCPGetGenericInterface(MocapApi::IMCPAvatar_Version, reinterpret_cast<void**>(&avatarIf_));
    if (err != MocapApi::Error_None || !avatarIf_) {
        std::cerr << "Failed to get IMCPAvatar, err=" << static_cast<int>(err) << "\n";
        return false;
    }

    err = MocapApi::MCPGetGenericInterface(MocapApi::IMCPJoint_Version, reinterpret_cast<void**>(&jointIf_));
    if (err != MocapApi::Error_None || !jointIf_) {
        std::cerr << "Failed to get IMCPJoint, err=" << static_cast<int>(err) << "\n";
        return false;
    }

    err = MocapApi::MCPGetGenericInterface(MocapApi::IMCPBodyPart_Version, reinterpret_cast<void**>(&bodyPart_));
    if (err != MocapApi::Error_None || !bodyPart_) {
        std::cerr << "Failed to get IMCPBodyPart, err=" << static_cast<int>(err) << "\n";
        return false;
    }

    return true;
}

bool MocapClient::CreateApplication() {
    MocapApi::EMCPError err = appIf_->CreateApplication(&appHandle_);
    if (err != MocapApi::Error_None) {
        std::cerr << "CreateApplication failed, err=" << static_cast<int>(err) << "\n";
        return false;
    }

    err = settings_->CreateSettings(&settingsHandle_);
    if (err != MocapApi::Error_None) {
        std::cerr << "CreateSettings failed, err=" << static_cast<int>(err) << "\n";
        return false;
    }

    return true;
}

bool MocapClient::ConfigureSettings() {
    MocapApi::EMCPError err = settings_->SetSettingsUDP(port_, settingsHandle_);
    if (err != MocapApi::Error_None) {
        std::cerr << "SetSettingsUDP failed, err=" << static_cast<int>(err) << "\n";
        return false;
    }

    err = settings_->SetSettingsUDPServer(serverIp_.c_str(), port_, settingsHandle_);
    if (err != MocapApi::Error_None) {
        std::cerr << "SetSettingsUDPServer failed, err=" << static_cast<int>(err) << "\n";
        return false;
    }

    err = settings_->SetSettingsBvhRotation(MocapApi::BvhRotation_XYZ, settingsHandle_);
    if (err != MocapApi::Error_None) {
        std::cerr << "SetSettingsBvhRotation failed, err=" << static_cast<int>(err) << "\n";
        return false;
    }

    err = settings_->SetSettingsBvhData(MocapApi::BvhDataType_Binary, settingsHandle_);
    if (err != MocapApi::Error_None) {
        std::cerr << "SetSettingsBvhData failed, err=" << static_cast<int>(err) << "\n";
        return false;
    }

    err = settings_->SetSettingsBvhTransformation(MocapApi::BvhTransformation_Enable, settingsHandle_);
    if (err != MocapApi::Error_None) {
        std::cerr << "SetSettingsBvhTransformation failed, err=" << static_cast<int>(err) << "\n";
        return false;
    }

    err = appIf_->SetApplicationSettings(settingsHandle_, appHandle_);
    if (err != MocapApi::Error_None) {
        std::cerr << "SetApplicationSettings failed, err=" << static_cast<int>(err) << "\n";
        return false;
    }

    return true;
}

bool MocapClient::FetchAvatarData() {
    uint32_t avatarCount = 0;
    MocapApi::EMCPError err        = appIf_->GetApplicationAvatars(nullptr, &avatarCount, appHandle_);
    if (err != MocapApi::Error_None || avatarCount == 0) {
        latestJoints_.clear();
        return false;
    }

    std::vector<MocapApi::MCPAvatarHandle_t> avatars(avatarCount);
    if (appIf_->GetApplicationAvatars(avatars.data(), &avatarCount, appHandle_) != MocapApi::Error_None) {
        latestJoints_.clear();
        return false;
    }

    latestJoints_.clear();
    for (uint32_t i = 0; i < avatarCount; ++i) {
        MocapApi::MCPAvatarHandle_t avatarHandle = avatars[i];

        uint32_t jointCount = 0;
        if (avatarIf_->GetAvatarJoints(nullptr, &jointCount, avatarHandle) != MocapApi::Error_None || jointCount == 0) {
            continue;
        }

        std::vector<MocapApi::MCPJointHandle_t> joints(jointCount);
        if (avatarIf_->GetAvatarJoints(joints.data(), &jointCount, avatarHandle) != MocapApi::Error_None) {
            continue;
        }

        std::vector<JointSample> samples;
        samples.reserve(jointCount);
        for (uint32_t j = 0; j < jointCount; ++j) {
            auto sample = GetJointSample(jointIf_, bodyPart_, joints[j]);
            if (sample) samples.push_back(*sample);
        }

        ComputeWorldTransforms(samples);
        latestJoints_.insert(latestJoints_.end(), samples.begin(), samples.end());
    }

    return !latestJoints_.empty();
}

bool MocapClient::Poll() {
    MocapApi::MCPEvent_t events[16];
    uint32_t   eventCount = 16;
    for (uint32_t i = 0; i < eventCount; ++i) {
        events[i].size = sizeof(MocapApi::MCPEvent_t);
    }

    MocapApi::EMCPError err = appIf_->PollApplicationNextEvent(events, &eventCount, appHandle_);
    (void)err;

    return FetchAvatarData();
}
