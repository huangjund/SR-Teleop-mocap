#include "JointData.h"

#include <functional>

std::optional<JointFullData> GetJointFullData(MocapApi::IMCPJoint*       jointIf,
                                              MocapApi::IMCPBodyPart*    bodyPartIf,
                                              MocapApi::MCPJointHandle_t jointHandle) {
    if (!jointIf || !bodyPartIf || !jointHandle) return std::nullopt;

    JointFullData data;

    jointIf->GetJointTag(&data.tag, jointHandle);

    const char* jointName = nullptr;
    if (jointIf->GetJointName(&jointName, jointHandle) == MocapApi::Error_None && jointName) {
        data.jointName = jointName;
    } else {
        data.jointName = "<unknown_joint>";
    }

    jointIf->GetJointLocalPosition(&data.localPx, &data.localPy, &data.localPz, jointHandle);
    jointIf->GetJointLocalRotationByEuler(&data.localRx, &data.localRy, &data.localRz, jointHandle);
    jointIf->GetJointLocalRotation(&data.localQx, &data.localQy, &data.localQz, &data.localQw,
                                   jointHandle);

    MocapApi::MCPBodyPartHandle_t bodyPartHandle = 0;
    if (jointIf->GetJointBodyPart(&bodyPartHandle, jointHandle) == MocapApi::Error_None && bodyPartHandle != 0) {
        bodyPartIf->GetJointPosition(&data.worldPx, &data.worldPy, &data.worldPz, bodyPartHandle);
        bodyPartIf->GetBodyPartPosture(&data.worldQx, &data.worldQy, &data.worldQz, &data.worldQw,
                                       bodyPartHandle);
    }

    return data;
}

std::optional<JointSample> GetJointSample(MocapApi::IMCPJoint*       jointIf,
                                          MocapApi::IMCPBodyPart*    bodyPartIf,
                                          MocapApi::MCPJointHandle_t jointHandle) {
    if (!jointIf || !bodyPartIf || !jointHandle) return std::nullopt;

    JointSample sample;

    const char* name = nullptr;
    if (jointIf->GetJointName(&name, jointHandle) == MocapApi::Error_None && name) {
        sample.jointName = name;
    } else {
        sample.jointName = "<unknown_joint>";
    }

    jointIf->GetJointTag(&sample.tag, jointHandle);
    jointIf->GetJointLocalPosition(&sample.localPos.x, &sample.localPos.y, &sample.localPos.z, jointHandle);
    jointIf->GetJointLocalRotation(&sample.localRot.x, &sample.localRot.y, &sample.localRot.z, &sample.localRot.w,
                                   jointHandle);

    MocapApi::EMCPJointTag parentTag = MocapApi::JointTag_Invalid;
    if (jointIf->GetJointParentJointTag(&parentTag, sample.tag) == MocapApi::Error_None) {
        sample.parentTag = parentTag;
    }

    return sample;
}

void ComputeWorldTransforms(std::vector<JointSample>& joints) {
    std::unordered_map<int, size_t> tagToIndex;
    tagToIndex.reserve(joints.size());
    for (size_t i = 0; i < joints.size(); ++i) {
        tagToIndex[static_cast<int>(joints[i].tag)] = i;
    }

    std::function<void(size_t)> resolve = [&](size_t idx) {
        JointSample& j = joints[idx];
        if (j.worldRot.w != 0.0f || j.worldRot.x != 0.0f || j.worldRot.y != 0.0f || j.worldRot.z != 0.0f) {
            return;
        }

        if (j.parentTag != MocapApi::JointTag_Invalid) {
            auto it = tagToIndex.find(static_cast<int>(j.parentTag));
            if (it != tagToIndex.end()) {
                resolve(it->second);
                const JointSample& parent = joints[it->second];
                j.worldRot              = Multiply(parent.worldRot, j.localRot);
                j.worldPos              = parent.worldPos;
                Vec3 rotatedLocalOffset = Rotate(parent.worldRot, j.localPos);
                j.worldPos.x += rotatedLocalOffset.x;
                j.worldPos.y += rotatedLocalOffset.y;
                j.worldPos.z += rotatedLocalOffset.z;
                return;
            }
        }

        j.worldRot = j.localRot;
        j.worldPos = j.localPos;
    };

    for (size_t i = 0; i < joints.size(); ++i) {
        resolve(i);
    }
}
