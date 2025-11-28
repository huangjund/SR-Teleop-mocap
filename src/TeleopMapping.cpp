#include "TeleopMapping.h"

#include <algorithm>
#include <cmath>

namespace {
struct EulerAngles {
    float yawY   = 0.0f;  // rotation about Y
    float pitchX = 0.0f;  // rotation about X
    float rollZ  = 0.0f;  // rotation about Z
};

EulerAngles ToEulerYXZ(const Quat& q) {
    const float xx = q.x * q.x;
    const float yy = q.y * q.y;
    const float zz = q.z * q.z;
    const float xy = q.x * q.y;
    const float xz = q.x * q.z;
    const float yz = q.y * q.z;
    const float wx = q.w * q.x;
    const float wy = q.w * q.y;
    const float wz = q.w * q.z;

    const float m02 = 2.0f * (xz + wy);
    const float m12 = 2.0f * (yz - wx);
    const float m10 = 2.0f * (xy + wz);
    const float m11 = 1.0f - 2.0f * (xx + zz);
    const float m22 = 1.0f - 2.0f * (xx + yy);

    float pitch = -std::asin(std::max(-1.0f, std::min(1.0f, m12)));
    float cp    = std::cos(pitch);

    float yaw  = (std::fabs(cp) > 1e-4f) ? std::atan2(m02, m22) : 0.0f;
    float roll = (std::fabs(cp) > 1e-4f) ? std::atan2(m10, m11) : 0.0f;

    return {yaw, pitch, roll};
}

float RadToDeg(float radians) { return radians * 57.2957795f; }

std::string SideFromTag(MocapApi::EMCPJointTag tag) {
    return (tag == MocapApi::JointTag_RightHand) ? "right" : "left";
}
}  // namespace

void TeleopMapping::Update(const std::vector<JointSample>& joints) {
    wrists_.clear();
    ergonomicAngles_.clear();
    fingerWorldPoses_.clear();

    for (const JointSample& joint : joints) {
        if (IsWrist(joint.tag)) {
            wrists_.push_back({SideFromTag(joint.tag), joint.worldPos, joint.worldRot});
            continue;
        }

        if (IsFinger(joint.tag)) {
            fingerWorldPoses_.push_back({joint.jointName, joint.tag, joint.worldPos, joint.worldRot});

            const EulerAngles euler = ToEulerYXZ(joint.localRot);
            ergonomicAngles_.push_back({joint.jointName, RadToDeg(euler.pitchX), RadToDeg(euler.yawY), RadToDeg(euler.rollZ)});
        }
    }
}

bool TeleopMapping::IsWrist(MocapApi::EMCPJointTag tag) {
    return tag == MocapApi::JointTag_LeftHand || tag == MocapApi::JointTag_RightHand;
}

bool TeleopMapping::IsFinger(MocapApi::EMCPJointTag tag) {
    switch (tag) {
        case MocapApi::JointTag_RightHandThumb1:
        case MocapApi::JointTag_RightHandThumb2:
        case MocapApi::JointTag_RightHandThumb3:
        case MocapApi::JointTag_RightHandIndex1:
        case MocapApi::JointTag_RightHandIndex2:
        case MocapApi::JointTag_RightHandIndex3:
        case MocapApi::JointTag_RightHandMiddle1:
        case MocapApi::JointTag_RightHandMiddle2:
        case MocapApi::JointTag_RightHandMiddle3:
        case MocapApi::JointTag_RightHandRing1:
        case MocapApi::JointTag_RightHandRing2:
        case MocapApi::JointTag_RightHandRing3:
        case MocapApi::JointTag_RightHandPinky1:
        case MocapApi::JointTag_RightHandPinky2:
        case MocapApi::JointTag_RightHandPinky3:
        case MocapApi::JointTag_LeftHandThumb1:
        case MocapApi::JointTag_LeftHandThumb2:
        case MocapApi::JointTag_LeftHandThumb3:
        case MocapApi::JointTag_LeftHandIndex1:
        case MocapApi::JointTag_LeftHandIndex2:
        case MocapApi::JointTag_LeftHandIndex3:
        case MocapApi::JointTag_LeftHandMiddle1:
        case MocapApi::JointTag_LeftHandMiddle2:
        case MocapApi::JointTag_LeftHandMiddle3:
        case MocapApi::JointTag_LeftHandRing1:
        case MocapApi::JointTag_LeftHandRing2:
        case MocapApi::JointTag_LeftHandRing3:
        case MocapApi::JointTag_LeftHandPinky1:
        case MocapApi::JointTag_LeftHandPinky2:
        case MocapApi::JointTag_LeftHandPinky3:
            return true;
        default:
            return false;
    }
}
