#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

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
    std::string outIp    = "127.0.0.1";
    uint16_t    outPort  = 15000;
    uint16_t    handPort = 15001;
    bool        enableUdp{true};
    bool        enableHandUdp{false};
    std::vector<std::string> sides{"left", "right"};
};

void PrintUsage(const char* exeName) {
    std::cout << "Usage: " << exeName << " [--server <ip>] [--port <udp_port>]\\n";
    std::cout << "  --server  Axis Studio host that streams BVH (default 127.0.0.1)\\n";
    std::cout << "  --port    UDP port for BVH stream (default 7012)\\n";
    std::cout << "  --out-ip  Destination IP for joint-angle stream (default 127.0.0.1)\\n";
    std::cout << "  --out-port Destination port for joint-angle stream (default 15000)\\n";
    std::cout << "  --hand-port Destination port for dexterous-hand joint stream (default 15001)\\n";
    std::cout << "  --sides   Comma-separated list of arms to stream: left,right (default both)\\n";
    std::cout << "  --no-udp  Disable joint-angle UDP streaming\\n";
    std::cout << "  --no-hand-udp Disable dexterous-hand UDP streaming\\n";
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
        } else if (arg == "--hand-port" && i + 1 < argc) {
            opts.handPort = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if (arg == "--sides" && i + 1 < argc) {
            const std::vector<std::string> parsedSides = parseSides(argv[++i]);
            if (!parsedSides.empty()) opts.sides = parsedSides;
        } else if (arg == "--no-udp") {
            opts.enableUdp = false;
        } else if (arg == "--no-hand-udp") {
            opts.enableHandUdp = false;
        } else if (arg == "--help" || arg == "-h") {
            PrintUsage(argv[0]);
            std::exit(0);
        }
    }
    return opts;
}

struct JointCommand {
    std::string       side;
    std::vector<float> joints;
};

// Placeholder for the FastIK integration; right now we use a simple geometric
// decomposition so the pipeline from BVH -> IK -> UDP -> visualizer is wired.
class FastIkSolver {
public:
    explicit FastIkSolver(size_t dof) : dof_(dof) {}

    std::vector<float> Solve(const WristPose& wrist) const {
        // Extract a basic yaw-pitch-roll from the wrist quaternion to populate the
        // first three joints, then use the wrist position to drive the remaining
        // translational DOFs. Replace this with the actual FastIK solution for
        // your Fanuc when available.
        const float xx = wrist.orientation.x * wrist.orientation.x;
        const float yy = wrist.orientation.y * wrist.orientation.y;
        const float zz = wrist.orientation.z * wrist.orientation.z;
        const float xy = wrist.orientation.x * wrist.orientation.y;
        const float xz = wrist.orientation.x * wrist.orientation.z;
        const float yz = wrist.orientation.y * wrist.orientation.z;
        const float wx = wrist.orientation.w * wrist.orientation.x;
        const float wy = wrist.orientation.w * wrist.orientation.y;
        const float wz = wrist.orientation.w * wrist.orientation.z;

        const float m02 = 2.0f * (xz + wy);
        const float m12 = 2.0f * (yz - wx);
        const float m10 = 2.0f * (xy + wz);
        const float m11 = 1.0f - 2.0f * (xx + zz);
        const float m22 = 1.0f - 2.0f * (xx + yy);

        const float pitch = -std::asin(std::clamp(m12, -1.0f, 1.0f));
        const float cp    = std::cos(pitch);

        const float yaw  = (std::fabs(cp) > 1e-4f) ? std::atan2(m02, m22) : 0.0f;
        const float roll = (std::fabs(cp) > 1e-4f) ? std::atan2(m10, m11) : 0.0f;

        std::vector<float> joints(dof_, 0.0f);
        if (dof_ > 0) joints[0] = roll;
        if (dof_ > 1) joints[1] = pitch;
        if (dof_ > 2) joints[2] = yaw;
        if (dof_ > 3) joints[3] = wrist.position.x;
        if (dof_ > 4) joints[4] = wrist.position.y;
        if (dof_ > 5) joints[5] = wrist.position.z;
        return joints;
    }

private:
    size_t dof_{0};
};

class JointCommandStreamer {
public:
    JointCommandStreamer(std::string destIp, uint16_t destPort, std::string label)
        : destIp_(std::move(destIp)), destPort_(destPort), label_(std::move(label)) {}

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
            Shutdown();
            return false;
        }

        std::memset(&destAddr_, 0, sizeof(destAddr_));
        destAddr_.sin_family = AF_INET;
        destAddr_.sin_port   = htons(destPort_);
        destAddr_.sin_addr.s_addr = inet_addr(destIp_.c_str());
        return true;
    }

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

    bool Send(const std::vector<JointCommand>& commands, size_t frame) {
        if (socketHandle_ < 0) return false;

        std::ostringstream oss;
        oss << "frame," << frame << "\n";
        for (const JointCommand& cmd : commands) {
            oss << label_ << ',' << cmd.side;
            for (float angle : cmd.joints) {
                oss << ',' << angle;
            }
            oss << "\n";
        }

        const std::string payload = oss.str();
        const int         sent    = static_cast<int>(::sendto(socketHandle_, payload.data(), static_cast<int>(payload.size()), 0,
                                                     reinterpret_cast<sockaddr*>(&destAddr_), sizeof(destAddr_)));
        if (sent < 0) {
            std::cerr << "Failed to send joint-angle datagram\\n";
            return false;
        }
        return true;
    }

    ~JointCommandStreamer() { Shutdown(); }

private:
    std::string       destIp_;
    uint16_t          destPort_;
    int               socketHandle_{-1};
    sockaddr_in       destAddr_{};
    bool              wsaStarted_{false};
    std::string       label_;
};

class HandRetargeter {
public:
    // Map ergonomic angles (flexion/abduction/twist) to the dexterous hand joint vector.
    // Replace the mapping inside this function when implementing a new retargeting algorithm.
    std::vector<float> Retarget(const std::vector<ErgonomicJointAngles>& ergonomicAngles, const std::string& side) const {
        // The current implementation outputs a 15-DOF vector (3 joints per finger) using the
        // flexion value only. Update the per-finger order or computations here to change the
        // retargeting behavior.
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
                    // Convert degrees to radians and use flexion as a placeholder command.
                    joints[i] = angles.flexionDeg * kDegToRad;
                    break;
                }
            }
        }

        return joints;
    }
};

void PrintWrist(const WristPose& wrist) {
    std::cout << "  " << wrist.side << " wrist pos(" << wrist.position.x << ", " << wrist.position.y << ", " << wrist.position.z
              << ") rot(quat) [" << wrist.orientation.x << ", " << wrist.orientation.y << ", " << wrist.orientation.z << ", "
              << wrist.orientation.w << "]\n";
}

void PrintErgonomic(const ErgonomicJointAngles& angles) {
    std::cout << "    " << angles.name << " flex=" << angles.flexionDeg << "deg abduction=" << angles.abductionDeg
              << "deg twist=" << angles.twistDeg << "deg\n";
}

void PrintJointCommand(const JointCommand& cmd) {
    std::cout << "  " << cmd.side << " joints";
    for (size_t i = 0; i < cmd.joints.size(); ++i) {
        std::cout << " j" << i << '=' << cmd.joints[i];
    }
    std::cout << "\n";
}
}  // namespace

int main(int argc, char** argv) {
    const Options opts = ParseArgs(argc, argv);

    std::cout << "\n=== Bimanual Teleop Stream (FastIK -> joint UDP) ===\n";
    std::cout << "Server : " << opts.serverIp << ":" << opts.port << "\n";
    std::cout << "Expecting BVH in OPT coords with YXZ rotation order; output is MocapApi joint poses.\n\n";

    JointCommandStreamer wristStreamer(opts.outIp, opts.outPort, "joint");
    JointCommandStreamer handStreamer(opts.outIp, opts.handPort, "hand");

    if (opts.enableUdp) {
        if (wristStreamer.Initialize()) {
            std::cout << "Joint-angle UDP stream -> " << opts.outIp << ":" << opts.outPort << " (sides=";
            for (size_t i = 0; i < opts.sides.size(); ++i) {
                std::cout << opts.sides[i];
                if (i + 1 < opts.sides.size()) std::cout << ',';
            }
            std::cout << ")\n";
        } else {
            std::cerr << "Failed to set up joint-angle UDP stream; continuing without it.\n";
        }
    } else {
        std::cout << "UDP streaming disabled via --no-udp\n";
    }

    if (opts.enableHandUdp) {
        if (handStreamer.Initialize()) {
            std::cout << "Dexterous-hand UDP stream -> " << opts.outIp << ":" << opts.handPort << " (sides=";
            for (size_t i = 0; i < opts.sides.size(); ++i) {
                std::cout << opts.sides[i];
                if (i + 1 < opts.sides.size()) std::cout << ',';
            }
            std::cout << ")\n";
        } else {
            std::cerr << "Failed to set up dexterous-hand UDP stream; continuing without it.\n";
        }
    } else {
        std::cout << "Dexterous-hand UDP streaming disabled via --no-hand-udp\n";
    }

    MocapClient client(opts.serverIp, opts.port);
    if (!client.Initialize()) return 1;

    TeleopMapping mapper;
    FastIkSolver ikSolver(6);  // Fanuc arm has 6 joints
    HandRetargeter handRetargeter;
    size_t       frame          = 0;
    bool         printedWaiting = false;

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
        auto sideEnabled = [&opts](const std::string& side) {
            return std::find(opts.sides.begin(), opts.sides.end(), side) != opts.sides.end();
        };

        std::vector<JointCommand> commands;
        std::vector<JointCommand> handCommands;
        commands.reserve(mapper.WristPoses().size());
        handCommands.reserve(mapper.WristPoses().size());
        for (const WristPose& wrist : mapper.WristPoses()) {
            if (!sideEnabled(wrist.side)) continue;
            commands.push_back({wrist.side, ikSolver.Solve(wrist)});
            handCommands.push_back({wrist.side, handRetargeter.Retarget(mapper.ErgonomicAngles(), wrist.side)});
        }

        // if (frame % 30 == 0) {
        //     std::cout << "\nFrame " << frame << "\n";
        //     for (const WristPose& wrist : mapper.WristPoses()) {
        //         PrintWrist(wrist);
        //     }
        //     std::cout << "  IK solution (radians/meters)\n";
        //     for (const JointCommand& cmd : commands) {
        //         PrintJointCommand(cmd);
        //     }
        //     std::cout << "  Ergonomic hand joints (local YXZ -> flex/abduction/twist in deg):\n";
        //     for (const ErgonomicJointAngles& angles : mapper.ErgonomicAngles()) {
        //         PrintErgonomic(angles);
        //     }
        //     std::cout << std::flush;
        // }

        if (opts.enableUdp && !commands.empty()) {
            wristStreamer.Send(commands, frame);
        }

        if (opts.enableHandUdp && !handCommands.empty()) {
            handStreamer.Send(handCommands, frame);
        }

        ++frame;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return 0;
}
