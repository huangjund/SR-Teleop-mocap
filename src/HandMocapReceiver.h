#ifndef HAND_MOCAP_RECEIVER_H
#define HAND_MOCAP_RECEIVER_H

#include <string>
#include <vector>

#include "MocapApi.h"

struct JointAngle {
    std::string name;  // e.g., "RightHandIndex2"
    float ex{0.0f};
    float ey{0.0f};
    float ez{0.0f};
    float pos[3]{0.0f, 0.0f, 0.0f};
};

struct HandState {
    bool valid = false;
    std::string side;       // "left" or "right"
    float wristPos[3] = {0.0f, 0.0f, 0.0f};      // global position (x,y,z)
    float wristQuat[4] = {0.0f, 0.0f, 0.0f, 1.0f};  // global orientation (qx,qy,qz,qw)
    std::vector<JointAngle> fingers;
};

class HandMocapReceiver {
public:
    HandMocapReceiver(const char* ip, uint16_t port);
    ~HandMocapReceiver();

    bool init();                 // setup MocapApi and connect
    void pollEvents();           // pump MocapApi events
    bool updateHands(HandState& leftHand, HandState& rightHand);
    void shutdown();

private:
    std::string ip_;
    uint16_t port_;

    MocapApi::IMCPApplication* app_      = nullptr;
    MocapApi::IMCPSettings*    settings_ = nullptr;
    MocapApi::IMCPAvatar*      avatarIf_ = nullptr;
    MocapApi::IMCPJoint*       jointIf_  = nullptr;

    MocapApi::MCPApplicationHandle_t appHandle_      = 0;
    MocapApi::MCPSettingsHandle_t    settingsHandle_ = 0;
};

#endif  // HAND_MOCAP_RECEIVER_H
