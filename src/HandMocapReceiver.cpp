#include "HandMocapReceiver.h"

#include <algorithm>
#include <iostream>

using namespace MocapApi;

HandMocapReceiver::HandMocapReceiver(const char* ip, uint16_t port)
    : ip_(ip ? ip : ""), port_(port) {}

HandMocapReceiver::~HandMocapReceiver()
{
    shutdown();
}

bool HandMocapReceiver::init()
{
    EMCPError err = Error_None;

    err = MCPGetGenericInterface(IMCPApplication_Version,
                                 reinterpret_cast<void**>(&app_));
    if (err != Error_None || !app_) {
        std::cerr << "Failed to get IMCPApplication, err=" << static_cast<int>(err)
                  << "\n";
        return false;
    }

    err = MCPGetGenericInterface(IMCPSettings_Version,
                                 reinterpret_cast<void**>(&settings_));
    if (err != Error_None || !settings_) {
        std::cerr << "Failed to get IMCPSettings, err=" << static_cast<int>(err)
                  << "\n";
        return false;
    }

    err = MCPGetGenericInterface(IMCPAvatar_Version,
                                 reinterpret_cast<void**>(&avatarIf_));
    if (err != Error_None || !avatarIf_) {
        std::cerr << "Failed to get IMCPAvatar, err=" << static_cast<int>(err)
                  << "\n";
        return false;
    }

    err = MCPGetGenericInterface(IMCPJoint_Version,
                                 reinterpret_cast<void**>(&jointIf_));
    if (err != Error_None || !jointIf_) {
        std::cerr << "Failed to get IMCPJoint, err=" << static_cast<int>(err)
                  << "\n";
        return false;
    }

    err = app_->CreateApplication(&appHandle_);
    if (err != Error_None) {
        std::cerr << "CreateApplication failed, err=" << static_cast<int>(err)
                  << "\n";
        return false;
    }

    err = settings_->CreateSettings(&settingsHandle_);
    if (err != Error_None) {
        std::cerr << "CreateSettings failed, err=" << static_cast<int>(err)
                  << "\n";
        return false;
    }

    err = settings_->SetSettingsUDP(port_, settingsHandle_);
    if (err != Error_None) {
        std::cerr << "SetSettingsUDP failed, err=" << static_cast<int>(err) << "\n";
        return false;
    }

    err = settings_->SetSettingsUDPServer(ip_.c_str(), port_, settingsHandle_);
    if (err != Error_None) {
        std::cerr << "SetSettingsUDPServer failed, err=" << static_cast<int>(err)
                  << "\n";
        return false;
    }

    err = settings_->SetSettingsBvhRotation(BvhRotation_XYZ, settingsHandle_);
    if (err != Error_None) {
        std::cerr << "SetSettingsBvhRotation failed, err=" << static_cast<int>(err)
                  << "\n";
        return false;
    }

    err = settings_->SetSettingsBvhData(BvhDataType_Binary, settingsHandle_);
    if (err != Error_None) {
        std::cerr << "SetSettingsBvhData failed, err=" << static_cast<int>(err)
                  << "\n";
        return false;
    }

    err = settings_->SetSettingsBvhTransformation(BvhTransformation_Enable,
                                                  settingsHandle_);
    if (err != Error_None) {
        std::cerr << "SetSettingsBvhTransformation failed, err="
                  << static_cast<int>(err) << "\n";
        return false;
    }

    if (settings_->SetSettingsCalcData(settingsHandle_) == Error_None) {
        std::cout << "Enabled calc data for BVH stream.\n";
    }

    err = app_->SetApplicationSettings(settingsHandle_, appHandle_);
    if (err != Error_None) {
        std::cerr << "SetApplicationSettings failed, err=" << static_cast<int>(err)
                  << "\n";
        return false;
    }

    err = app_->OpenApplication(appHandle_);
    if (err != Error_None) {
        std::cerr << "OpenApplication failed, err=" << static_cast<int>(err)
                  << "\n";
        return false;
    }

    return true;
}

void HandMocapReceiver::shutdown()
{
    if (settings_ && settingsHandle_ != 0) {
        settings_->DestroySettings(settingsHandle_);
        settingsHandle_ = 0;
    }

    if (app_) {
        if (appHandle_ != 0) {
            app_->CloseApplication(appHandle_);
            app_->DestroyApplication(appHandle_);
            appHandle_ = 0;
        }
    }
}

void HandMocapReceiver::pollEvents()
{
    if (!app_ || appHandle_ == 0) {
        return;
    }

    MCPEvent_t events[32];
    uint32_t count = 32;
    for (uint32_t i = 0; i < count; ++i) {
        events[i].size = sizeof(MCPEvent_t);
    }
    app_->PollApplicationNextEvent(events, &count, appHandle_);
}

namespace {
bool isRightFingerJoint(const std::string& name)
{
    static const char* patterns[] = {
        "RightHandThumb",
        "RightHandIndex",
        "RightHandMiddle",
        "RightHandRing",
        "RightHandPinky",
        "RightInHandIndex",
        "RightInHandMiddle",
        "RightInHandRing",
        "RightInHandPinky"};

    for (const char* p : patterns) {
        if (name.find(p) != std::string::npos) {
            return true;
        }
    }
    return false;
}

bool isLeftFingerJoint(const std::string& name)
{
    static const char* patterns[] = {
        "LeftHandThumb",
        "LeftHandIndex",
        "LeftHandMiddle",
        "LeftHandRing",
        "LeftHandPinky",
        "LeftInHandIndex",
        "LeftInHandMiddle",
        "LeftInHandRing",
        "LeftInHandPinky"};

    for (const char* p : patterns) {
        if (name.find(p) != std::string::npos) {
            return true;
        }
    }
    return false;
}
}

bool HandMocapReceiver::updateHands(HandState& leftHand, HandState& rightHand)
{
    leftHand = HandState{};
    rightHand = HandState{};
    leftHand.side = "left";
    rightHand.side = "right";

    if (!app_ || !avatarIf_ || !jointIf_ || appHandle_ == 0) {
        return false;
    }

    uint32_t avatarCount = 0;
    app_->GetApplicationAvatars(nullptr, &avatarCount, appHandle_);
    if (avatarCount == 0) {
        return false;
    }

    std::vector<MCPAvatarHandle_t> avatars(avatarCount);
    app_->GetApplicationAvatars(avatars.data(), &avatarCount, appHandle_);
    if (avatarCount == 0 || avatars.empty()) {
        return false;
    }

    MCPAvatarHandle_t avatarHandle = avatars[0];

    uint32_t jointCount = 0;
    avatarIf_->GetAvatarJoints(nullptr, &jointCount, avatarHandle);
    if (jointCount == 0) {
        return false;
    }

    std::vector<MCPJointHandle_t> joints(jointCount);
    avatarIf_->GetAvatarJoints(joints.data(), &jointCount, avatarHandle);

    for (uint32_t i = 0; i < jointCount; ++i) {
        MCPJointHandle_t jointHandle = joints[i];
        if (!jointHandle) {
            continue;
        }

        const char* nameC = nullptr;
        jointIf_->GetJointName(&nameC, jointHandle);
        if (!nameC) {
            continue;
        }
        std::string name(nameC);

        if (name == "RightHand") {
            float px = 0.0f, py = 0.0f, pz = 0.0f;
            float qx = 0.0f, qy = 0.0f, qz = 0.0f, qw = 1.0f;
            jointIf_->GetJointGlobalTranslation(&px, &py, &pz, jointHandle);
            jointIf_->GetJointGlobalRotation(&qx, &qy, &qz, &qw, jointHandle);

            rightHand.wristPos[0] = px;
            rightHand.wristPos[1] = py;
            rightHand.wristPos[2] = pz;
            rightHand.wristQuat[0] = qx;
            rightHand.wristQuat[1] = qy;
            rightHand.wristQuat[2] = qz;
            rightHand.wristQuat[3] = qw;
            rightHand.valid = true;
            rightHand.side = "right";
        } else if (name == "LeftHand") {
            float px = 0.0f, py = 0.0f, pz = 0.0f;
            float qx = 0.0f, qy = 0.0f, qz = 0.0f, qw = 1.0f;
            jointIf_->GetJointGlobalTranslation(&px, &py, &pz, jointHandle);
            jointIf_->GetJointGlobalRotation(&qx, &qy, &qz, &qw, jointHandle);

            leftHand.wristPos[0] = px;
            leftHand.wristPos[1] = py;
            leftHand.wristPos[2] = pz;
            leftHand.wristQuat[0] = qx;
            leftHand.wristQuat[1] = qy;
            leftHand.wristQuat[2] = qz;
            leftHand.wristQuat[3] = qw;
            leftHand.valid = true;
            leftHand.side = "left";
        }

        if (isRightFingerJoint(name)) {
            float ex = 0.0f, ey = 0.0f, ez = 0.0f;
            float px = 0.0f, py = 0.0f, pz = 0.0f;
            jointIf_->GetJointLocalRotationByEuler(&ex, &ey, &ez, jointHandle);
            jointIf_->GetJointGlobalTranslation(&px, &py, &pz, jointHandle);
            JointAngle ja;
            ja.name = name;
            ja.ex = ex;
            ja.ey = ey;
            ja.ez = ez;
            ja.pos[0] = px;
            ja.pos[1] = py;
            ja.pos[2] = pz;
            rightHand.fingers.push_back(ja);
        } else if (isLeftFingerJoint(name)) {
            float ex = 0.0f, ey = 0.0f, ez = 0.0f;
            float px = 0.0f, py = 0.0f, pz = 0.0f;
            jointIf_->GetJointLocalRotationByEuler(&ex, &ey, &ez, jointHandle);
            jointIf_->GetJointGlobalTranslation(&px, &py, &pz, jointHandle);
            JointAngle ja;
            ja.name = name;
            ja.ex = ex;
            ja.ey = ey;
            ja.ez = ez;
            ja.pos[0] = px;
            ja.pos[1] = py;
            ja.pos[2] = pz;
            leftHand.fingers.push_back(ja);
        }
    }

    return leftHand.valid || rightHand.valid;
}

