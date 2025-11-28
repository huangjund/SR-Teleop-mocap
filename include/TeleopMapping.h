#pragma once

#include <string>
#include <vector>

#include "JointData.h"

struct WristPose {
    std::string side;
    Vec3        position{};
    Quat        orientation{};
};

struct FingerPose {
    std::string            name;
    MocapApi::EMCPJointTag tag = MocapApi::JointTag_Invalid;
    Vec3                   position{};
    Quat                   orientation{};
};

struct ErgonomicJointAngles {
    std::string name;
    float       flexionDeg   = 0.0f;  // rotation about the local X axis
    float       abductionDeg = 0.0f;  // rotation about the local Y axis (splay)
    float       twistDeg     = 0.0f;  // rotation about the local Z axis
};

class TeleopMapping {
public:
    void Update(const std::vector<JointSample>& joints);

    const std::vector<WristPose>& WristPoses() const { return wrists_; }
    const std::vector<ErgonomicJointAngles>& ErgonomicAngles() const { return ergonomicAngles_; }
    const std::vector<FingerPose>& RawFingerPoses() const { return fingerWorldPoses_; }

private:
    static bool IsWrist(MocapApi::EMCPJointTag tag);
    static bool IsFinger(MocapApi::EMCPJointTag tag);

    std::vector<WristPose>            wrists_;
    std::vector<ErgonomicJointAngles> ergonomicAngles_;
    std::vector<FingerPose>           fingerWorldPoses_;
};
