// Minimal BVH -> hand-position UDP sender
// Usage: mocap_hands_udp_sender --server 127.0.0.1 --port 7012 --out-ip 127.0.0.1 --out-port 16000 --hz 60
// Packet format (binary, little-endian):
//  - magic: 4 bytes ASCII "HND1"
//  - version: uint16 = 1
//  - flags: uint16 bitmask (bit0=left valid, bit1=right valid)
//  - timestamp_us: uint64
//  - dof: uint16 (number of float32 values following)
//  - reserved: uint16 (0)
//  - payload: per side (left then right): wrist(7 floats px,py,pz,qx,qy,qz,qw) + 15 finger joint world positions(45 floats)
// Joint order: same as main.cpp visualization - direct world positions from MocapApi.
// Finger structure per hand: Thumb(1,2,3), Index(1,2,3), Middle(1,2,3), Ring(1,2,3), Pinky(1,2,3)

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#ifdef _WIN32
#    include <WinSock2.h>
#    include <WS2tcpip.h>
#else
#    include <arpa/inet.h>
#    include <netinet/in.h>
#    include <sys/socket.h>
#    include <unistd.h>
#endif

#include "MocapClient.h"
#include "TeleopMapping.h"

namespace {
struct Options {
    std::string serverIp = "127.0.0.1";
    uint16_t    port     = 7012;
    std::string outIp   = "10.42.0.100";
    uint16_t    outPort  = 16000;
    int         hz       = 60; // 0 = every frame
};

void PrintUsage(const char* exeName) {
    std::cout << "Usage: " << exeName << " [--server <ip>] [--port <udp_port>] [--out-ip <ip>] [--out-port <port>] [--hz <rate>]\n";
}

Options ParseArgs(int argc, char** argv) {
    Options opts{};
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--server" && i + 1 < argc) opts.serverIp = argv[++i];
        else if (arg == "--port" && i + 1 < argc) opts.port = static_cast<uint16_t>(std::stoi(argv[++i]));
        else if (arg == "--out-ip" && i + 1 < argc) opts.outIp = argv[++i];
        else if (arg == "--out-port" && i + 1 < argc) opts.outPort = static_cast<uint16_t>(std::stoi(argv[++i]));
        else if (arg == "--hz" && i + 1 < argc) opts.hz = std::stoi(argv[++i]);
        else if (arg == "--help" || arg == "-h") { PrintUsage(argv[0]); std::exit(0); }
    }
    return opts;
}

class UdpSender {
public:
    UdpSender(std::string destIp, uint16_t destPort) : destIp_(std::move(destIp)), destPort_(destPort) {}

    bool Initialize() {
#ifdef _WIN32
        WSADATA wsaData{};
        const int startupResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
        if (startupResult != 0) return false;
        wsaStarted_ = true;
#endif
        sock_ = static_cast<int>(::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP));
        if (sock_ < 0) return false;

        std::memset(&addr_, 0, sizeof(addr_));
        addr_.sin_family = AF_INET;
        addr_.sin_port   = htons(destPort_);
        addr_.sin_addr.s_addr = inet_addr(destIp_.c_str());
        return true;
    }

    ~UdpSender() { Shutdown(); }

    void Shutdown() {
        if (sock_ >= 0) {
#ifdef _WIN32
            closesocket(sock_);
#else
            close(sock_);
#endif
            sock_ = -1;
        }
#ifdef _WIN32
        if (wsaStarted_) { WSACleanup(); wsaStarted_ = false; }
#endif
    }

    bool Send(const std::vector<uint8_t>& buf) {
        if (sock_ < 0) return false;
        const int sent = static_cast<int>(::sendto(sock_, reinterpret_cast<const char*>(buf.data()), static_cast<int>(buf.size()), 0,
                                                   reinterpret_cast<sockaddr*>(&addr_), sizeof(addr_)));
        return sent >= 0;
    }

private:
    std::string destIp_;
    uint16_t    destPort_;
    int         sock_{-1};
    sockaddr_in addr_{};
    bool        wsaStarted_{false};
};

// Finger joint tags in order (1 per finger MCP/PIP/DIP, 5 fingers)
static const std::vector<MocapApi::EMCPJointTag> kFingerJointTags = {
    // Thumb
    MocapApi::JointTag_LeftHandThumb1, MocapApi::JointTag_LeftHandThumb2, MocapApi::JointTag_LeftHandThumb3,
    // Index
    MocapApi::JointTag_LeftHandIndex1, MocapApi::JointTag_LeftHandIndex2, MocapApi::JointTag_LeftHandIndex3,
    // Middle
    MocapApi::JointTag_LeftHandMiddle1, MocapApi::JointTag_LeftHandMiddle2, MocapApi::JointTag_LeftHandMiddle3,
    // Ring
    MocapApi::JointTag_LeftHandRing1, MocapApi::JointTag_LeftHandRing2, MocapApi::JointTag_LeftHandRing3,
    // Pinky
    MocapApi::JointTag_LeftHandPinky1, MocapApi::JointTag_LeftHandPinky2, MocapApi::JointTag_LeftHandPinky3
};

std::vector<Vec3> ExtractFingerJointPositions(const std::vector<JointSample>& joints, bool isRight) {
    std::vector<Vec3> positions;
    positions.reserve(15);
    const float nanf = std::numeric_limits<float>::quiet_NaN();

    // Adjust tags for right hand
    auto getTag = [isRight](MocapApi::EMCPJointTag leftTag) -> MocapApi::EMCPJointTag {
        if (!isRight) return leftTag;
        // Convert left to right by offset (Left=200-214, Right=216-230)
        int offset = static_cast<int>(MocapApi::JointTag_RightHandThumb1) - static_cast<int>(MocapApi::JointTag_LeftHandThumb1);
        return static_cast<MocapApi::EMCPJointTag>(static_cast<int>(leftTag) + offset);
    };

    // Build tag-to-index map
    std::unordered_map<int, size_t> tagToIndex;
    for (size_t i = 0; i < joints.size(); ++i) {
        tagToIndex[static_cast<int>(joints[i].tag)] = i;
    }

    // Extract world positions in finger order
    for (const auto& leftTag : kFingerJointTags) {
        auto tag = getTag(leftTag);
        auto it = tagToIndex.find(static_cast<int>(tag));
        if (it != tagToIndex.end()) {
            positions.push_back(joints[it->second].worldPos);
        } else {
            positions.push_back({nanf, nanf, nanf});
        }
    }

    return positions;
}

// Helpers to append little-endian binary
template <typename T>
void AppendLE(std::vector<uint8_t>& buf, T value) {
    uint8_t const* p = reinterpret_cast<uint8_t const*>(&value);
    for (size_t i = 0; i < sizeof(T); ++i) buf.push_back(p[i]);
}

} // anonymous namespace

int main(int argc, char** argv) {
    const Options opts = ParseArgs(argc, argv);

    std::cout << "\n=== BVH -> Hand-Angle UDP Sender ===\n";
    std::cout << "Server : " << opts.serverIp << ":" << opts.port << "\n";
    std::cout << "Streaming hand angles to " << opts.outIp << ":" << opts.outPort << "\n";

    UdpSender sender(opts.outIp, opts.outPort);
    if (!sender.Initialize()) {
        std::cerr << "Failed to initialize UDP sender\n";
        return 1;
    }

    MocapClient client(opts.serverIp, opts.port);
    if (!client.Initialize()) return 1;

    TeleopMapping mapper;

    bool printedWaiting = false;
    bool printedStreaming = false;

    const double minIntervalSec = (opts.hz > 0) ? (1.0 / static_cast<double>(opts.hz)) : 0.0;
    auto lastSendTime = std::chrono::steady_clock::now() - std::chrono::seconds(1);

    while (true) {
        client.Poll();
        mapper.Update(client.LatestJoints());

        if (mapper.ErgonomicAngles().empty()) {
            if (!printedWaiting) {
                printedWaiting = true;
                std::cout << "Waiting for MocapApi joints..." << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        printedWaiting = false;

        // Build transformed wrist poses (match teleop_main.cpp): rotationZhalf * rotationX * pose
        std::unordered_map<std::string, std::array<float,7>> wristPoseMap; // side -> {x,y,z,qx,qy,qz,qw}
        const float halfPi = 3.14159265358979323846f * 0.5f;
        const float onePi = 3.14159265358979323846f * 1.0f;
        const Eigen::Quaternionf rotationZ(Eigen::AngleAxisf(onePi, Eigen::Vector3f::UnitZ()));
        const Eigen::Quaternionf rotationZhalf(Eigen::AngleAxisf(halfPi, Eigen::Vector3f::UnitZ()));
        const Eigen::Quaternionf rotationX(Eigen::AngleAxisf(halfPi, Eigen::Vector3f::UnitX()));

        for (const WristPose& wrist : mapper.WristPoses()) {
            WristPose transformed = wrist;
            const Eigen::Vector3f positionEigen(wrist.position.x, wrist.position.y, wrist.position.z);
            const Eigen::Vector3f rotatedPosition = rotationZhalf * rotationX * positionEigen;

            const Eigen::Quaternionf currentOrientation(wrist.orientation.w,
                                                        wrist.orientation.x,
                                                        wrist.orientation.y,
                                                        wrist.orientation.z);
            const Eigen::Quaternionf rotatedOrientation = rotationZhalf * rotationX * currentOrientation;

            transformed.position.x     = rotatedPosition.x();
            transformed.position.y     = rotatedPosition.y();
            transformed.position.z     = rotatedPosition.z();
            transformed.orientation.x  = rotatedOrientation.x();
            transformed.orientation.y  = rotatedOrientation.y();
            transformed.orientation.z  = rotatedOrientation.z();
            transformed.orientation.w  = rotatedOrientation.w();
            wristPoseMap[wrist.side] = {transformed.position.x, transformed.position.y, transformed.position.z,
                                       transformed.orientation.x, transformed.orientation.y, transformed.orientation.z, transformed.orientation.w};
        }

        // Extract per-side finger joint world positions (matching main.cpp SkeletonViewer approach)
        std::vector<Vec3> leftPositions = ExtractFingerJointPositions(client.LatestJoints(), false);
        std::vector<Vec3> rightPositions = ExtractFingerJointPositions(client.LatestJoints(), true);

        const bool leftValid = std::any_of(leftPositions.begin(), leftPositions.end(), [](const Vec3& v) { return !std::isnan(v.x); });
        const bool rightValid = std::any_of(rightPositions.begin(), rightPositions.end(), [](const Vec3& v) { return !std::isnan(v.x); });

        // Rate limiting
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - lastSendTime).count();
        if (minIntervalSec > 0.0 && elapsed < minIntervalSec) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // Build packet
        std::vector<uint8_t> packet;
        // magic
        packet.push_back('H'); packet.push_back('N'); packet.push_back('D'); packet.push_back('1');
        uint16_t version = 1;
        AppendLE<uint16_t>(packet, version);
        uint16_t flags = 0;
        if (leftValid) flags |= 0x1;
        if (rightValid) flags |= 0x2;
        AppendLE<uint16_t>(packet, flags);

        // timestamp_us
        const uint64_t tsUs = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                std::chrono::system_clock::now().time_since_epoch()).count());
        AppendLE<uint64_t>(packet, tsUs);

        // payload: for each side (left then right): wrist(7 floats: px,py,pz,qx,qy,qz,qw) then 15 joint positions (3 floats each)
        const size_t perSidePositions = 15;  // 15 finger joints
        const size_t perSideWrist = 7;
        const size_t perSideTotal = perSideWrist + (perSidePositions * 3);  // 7 + 45 = 52 per side
        const size_t totalDof = perSideTotal * 2;
        AppendLE<uint16_t>(packet, static_cast<uint16_t>(totalDof));
        uint16_t reserved = 0;
        AppendLE<uint16_t>(packet, reserved);

        const float nanf = std::numeric_limits<float>::quiet_NaN();

        auto append_side = [&](const std::string& side, const std::vector<Vec3>& positions) {
            // wrist
            auto it = wristPoseMap.find(side);
            if (it != wristPoseMap.end()) {
                for (int i = 0; i < 7; ++i) AppendLE<float>(packet, static_cast<float>(it->second[i]));
            } else {
                for (int i = 0; i < 7; ++i) AppendLE<float>(packet, nanf);
            }
            // 15 joint world positions (3 floats each)
            for (const Vec3& pos : positions) {
                AppendLE<float>(packet, pos.x);
                AppendLE<float>(packet, pos.y);
                AppendLE<float>(packet, pos.z);
            }
        };

        append_side("left", leftPositions);
        append_side("right", rightPositions);

        if (sender.Send(packet)) {
            lastSendTime = now;
            if (!printedStreaming) {
                printedStreaming = true;
                std::cout << "Streaming hand joint positions to " << opts.outIp << ":" << opts.outPort << std::endl;
            }
        } else {
            std::cerr << "Failed to send UDP packet\n";
        }

        // if hz==0, send every received frame; otherwise throttle via minIntervalSec
        if (opts.hz == 0) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return 0;
}
