#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "MathTypes.h"
#include "MocapApi.h"

struct JointFullData {
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

struct JointSample {
    std::string jointName;
    EMCPJointTag tag       = JointTag_Invalid;
    EMCPJointTag parentTag = JointTag_Invalid;
    Vec3        localPos{};
    Quat        localRot{};
    Vec3        worldPos{};
    Quat        worldRot{0.0f, 0.0f, 0.0f, 0.0f};
};

std::optional<JointFullData> GetJointFullData(IMCPJoint*       jointIf,
                                              IMCPBodyPart*    bodyPartIf,
                                              MCPJointHandle_t jointHandle);

std::optional<JointSample> GetJointSample(IMCPJoint*       jointIf,
                                          IMCPBodyPart*    bodyPartIf,
                                          MCPJointHandle_t jointHandle);

void ComputeWorldTransforms(std::vector<JointSample>& joints);
