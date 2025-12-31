#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
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
    std::string              serverIp = "127.0.0.1";
    uint16_t                 port     = 7012;
    std::string              outIp    = "127.0.0.1";
    uint16_t                 outPort  = 16000;
    bool                     enableUdp{true};
    std::vector<std::string> sides{"left", "right"};
};

void PrintUsage(const char* exeName) {
    std::cout << "Usage: " << exeName << " [--server <ip>] [--port <udp_port>]\\n";
    std::cout << "  --server    Axis Studio host that streams BVH (default 127.0.0.1)\\n";
    std::cout << "  --port      UDP port for BVH stream (default 7012)\\n";
    std::cout << "  --out-ip    Destination IP for wrist/hand UDP stream (default 127.0.0.1)\\n";
    std::cout << "  --out-port  Destination port for wrist/hand UDP stream (default 16000)\\n";
    std::cout << "  --sides     Comma-separated list of arms to stream: left,right (default both)\\n";
    std::cout << "  --no-udp    Disable wrist/hand UDP streaming\\n";
}

Options ParseArgs(int argc, char** argv) {
    Options opts{};
    auto parseSides = [](const std::string& csv) {
        std::vector<std::string> parsed{};
        std::stringstream        ss(csv);
        std::string              token;
        while (std::getline(ss, token, ',')) {
            std::string lowered = token;
            std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char c) { return std::tolower(c); });
            if (lowered == "left" || lowered == "right") {
                if (std::find(parsed.begin(), parsed.end(), lowered) == parsed.end()) parsed.push_back(lowered);
            }
        }
        return parsed;
    };

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--server" && i + 1 < argc) {
            opts.serverIp = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            opts.port = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if (arg == "--out-ip" && i + 1 < argc) {
            opts.outIp = argv[++i];
        } else if (arg == "--out-port" && i + 1 < argc) {
            opts.outPort = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if (arg == "--sides" && i + 1 < argc) {
            const std::vector<std::string> parsedSides = parseSides(argv[++i]);
            if (!parsedSides.empty()) opts.sides = parsedSides;
        } else if (arg == "--no-udp") {
            opts.enableUdp = false;
        } else if (arg == "--help" || arg == "-h") {
            PrintUsage(argv[0]);
            std::exit(0);
        }
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
        if (startupResult != 0) {
            std::cerr << "WSAStartup failed: " << startupResult << "\\n";
            return false;
        }
        wsaStarted_ = true;
#endif

        socketHandle_ = static_cast<int>(::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP));
        if (socketHandle_ < 0) {
            std::cerr << "Unable to create UDP socket\\n";
            return false;
        }

        std::memset(&destAddr_, 0, sizeof(destAddr_));
        destAddr_.sin_family = AF_INET;
        destAddr_.sin_port   = htons(destPort_);
        destAddr_.sin_addr.s_addr = inet_addr(destIp_.c_str());
        return true;
    }

    ~UdpSender() { Shutdown(); }

    void Shutdown() {
        if (socketHandle_ >= 0) {
#ifdef _WIN32
            closesocket(socketHandle_);
#else
            close(socketHandle_);
#endif
            socketHandle_ = -1;
        }

#ifdef _WIN32
        if (wsaStarted_) {
            WSACleanup();
            wsaStarted_ = false;
        }
#endif
    }

    bool Send(const std::string& payload) {
        if (socketHandle_ < 0) return false;

        const int sent = static_cast<int>(::sendto(socketHandle_, payload.data(), static_cast<int>(payload.size()), 0,
                                                   reinterpret_cast<sockaddr*>(&destAddr_), sizeof(destAddr_)));
        return sent >= 0;
    }

private:
    std::string destIp_;
    uint16_t    destPort_;
    int         socketHandle_{-1};
    sockaddr_in destAddr_{};
    bool        wsaStarted_{false};
};

std::vector<float> RetargetHand(const std::vector<ErgonomicJointAngles>& ergonomicAngles, const std::string& side) {
    static constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;
    static const std::vector<std::string> kFingerOrder = {"Thumb1", "Thumb2", "Thumb3", "Index1", "Index2", "Index3", "Middle1",
                                                          "Middle2", "Middle3", "Ring1", "Ring2", "Ring3", "Pinky1", "Pinky2",
                                                          "Pinky3"};

    std::vector<float> joints(kFingerOrder.size(), 0.0f);

    auto matchesSide = [&side](const std::string& name) {
        const std::string prefix = (side == "right") ? "Right" : "Left";
        return name.find(prefix) == 0;
    };

    for (size_t i = 0; i < kFingerOrder.size(); ++i) {
        const std::string targetName = kFingerOrder[i];
        for (const ErgonomicJointAngles& angles : ergonomicAngles) {
            if (!matchesSide(angles.name)) continue;

            if (angles.name.find(targetName) != std::string::npos) {
                joints[i] = angles.flexionDeg * kDegToRad;
                break;
            }
        }
    }

    return joints;
}

std::string BuildPayload(size_t frame,
                         const std::unordered_map<std::string, WristPose>& wrists,
                         const std::unordered_map<std::string, std::vector<float>>& handJoints,
                         const std::vector<std::string>& sides) {
    std::ostringstream oss;
    oss << "frame," << frame << "\n";

    for (const std::string& side : sides) {
        auto wristIt = wrists.find(side);
        if (wristIt != wrists.end()) {
            const WristPose& wrist = wristIt->second;
            oss << "wrist," << wrist.side << ',' << wrist.position.x << ',' << wrist.position.y << ',' << wrist.position.z << ','
                << wrist.orientation.x << ',' << wrist.orientation.y << ',' << wrist.orientation.z << ',' << wrist.orientation.w
                << "\n";
        }

        auto handIt = handJoints.find(side);
        if (handIt != handJoints.end()) {
            oss << "hand," << side;
            for (float value : handIt->second) {
                oss << ',' << value;
            }
            oss << "\n";
        }
    }

    return oss.str();
}
}  // namespace

int main(int argc, char** argv) {
    const Options opts = ParseArgs(argc, argv);

    std::cout << "\n=== BVH Wrist/Hand Stream -> UDP ===\n";
    std::cout << "Server : " << opts.serverIp << ":" << opts.port << "\n";
    std::cout << "BVH -> MocapApi -> UDP wrist/hand payloads for Python IK.\n\n";

    UdpSender sender(opts.outIp, opts.outPort);
    if (opts.enableUdp) {
        if (sender.Initialize()) {
            std::cout << "Wrist/hand UDP stream -> " << opts.outIp << ":" << opts.outPort << " (sides=";
            for (size_t i = 0; i < opts.sides.size(); ++i) {
                std::cout << opts.sides[i];
                if (i + 1 < opts.sides.size()) std::cout << ',';
            }
            std::cout << ")\n";
        } else {
            std::cerr << "Failed to set up UDP stream; continuing without it.\n";
        }
    } else {
        std::cout << "UDP streaming disabled via --no-udp\n";
    }

    MocapClient client(opts.serverIp, opts.port);
    if (!client.Initialize()) return 1;

    TeleopMapping mapper;
    size_t        frame           = 0;
    bool          printedWaiting  = false;
    auto          sideEnabled     = [&opts](const std::string& side) {
        return std::find(opts.sides.begin(), opts.sides.end(), side) != opts.sides.end();
    };

    const float              halfPi     = 3.14159265358979323846f * 0.5f;
    const float              onePi     = 3.14159265358979323846f * 1.0f;
    const Eigen::Quaternionf rotationZ(Eigen::AngleAxisf(onePi, Eigen::Vector3f::UnitZ()));
    const Eigen::Quaternionf rotationZhalf(Eigen::AngleAxisf(halfPi, Eigen::Vector3f::UnitZ()));
    const Eigen::Quaternionf rotationX(Eigen::AngleAxisf(halfPi, Eigen::Vector3f::UnitX()));

    while (true) {
        client.Poll();
        mapper.Update(client.LatestJoints());

        if (mapper.WristPoses().empty()) {
            if (!printedWaiting) {
                printedWaiting = true;
                std::cout << "Waiting for MocapApi joints..." << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        printedWaiting = false;

        std::unordered_map<std::string, WristPose> wristsBySide;
        for (const WristPose& wrist : mapper.WristPoses()) {
            if (!sideEnabled(wrist.side)) continue;
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
            wristsBySide[wrist.side] = transformed;
        }

        std::unordered_map<std::string, std::vector<float>> handJointsBySide;
        for (const std::string& side : opts.sides) {
            const std::vector<float> retargeted = RetargetHand(mapper.ErgonomicAngles(), side);
            if (!retargeted.empty()) handJointsBySide[side] = retargeted;
        }

        if (opts.enableUdp && (!wristsBySide.empty() || !handJointsBySide.empty())) {
            const std::string payload = BuildPayload(frame, wristsBySide, handJointsBySide, opts.sides);
            sender.Send(payload);
        }

        ++frame;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return 0;
}
