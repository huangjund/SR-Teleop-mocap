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
#include <unordered_map>
#include <vector>

#ifdef _WIN32
#    include <WinSock2.h>
#    include <WS2tcpip.h>
#    include <conio.h>
#else
#    include <arpa/inet.h>
#    include <netinet/in.h>
#    include <sys/socket.h>
#    include <sys/select.h>
#    include <termios.h>
#    include <unistd.h>
#endif

#include <Eigen/Dense>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/se3.hpp>

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
    bool        enableHandUdp{true};
    std::vector<std::string> sides{"left", "right"};
    std::string            urdfPath{"urdf/lrmate_without_hand.urdf"};
    std::string            endEffectorFrame{"wrist"};
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
    std::cout << "  --urdf    URDF path for the Fanuc arm (default urdf/lrmate_without_hand.urdf)\\n";
    std::cout << "  --ee-frame End-effector frame name in the URDF (default wrist)\\n";
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
        } else if (arg == "--urdf" && i + 1 < argc) {
            opts.urdfPath = argv[++i];
        } else if (arg == "--ee-frame" && i + 1 < argc) {
            opts.endEffectorFrame = argv[++i];
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

class TerminalModeGuard {
public:
    TerminalModeGuard() {
#ifndef _WIN32
        if (tcgetattr(STDIN_FILENO, &original_) == 0) {
            termios raw = original_;
            raw.c_lflag &= static_cast<unsigned>(~(ICANON | ECHO));
            raw.c_cc[VMIN]  = 0;
            raw.c_cc[VTIME] = 0;
            tcsetattr(STDIN_FILENO, TCSANOW, &raw);
            engaged_ = true;
        }
#endif
    }

    ~TerminalModeGuard() {
#ifndef _WIN32
        if (engaged_) { tcsetattr(STDIN_FILENO, TCSANOW, &original_); }
#endif
    }

private:
#ifndef _WIN32
    termios original_{};
    bool    engaged_{false};
#endif
};

struct KeyPollResult {
    bool holdK{false};
    bool triggerL{false};
};

KeyPollResult PollKeys() {
    KeyPollResult result{};
#ifdef _WIN32
    while (_kbhit()) {
        const int ch = _getch();
        if (ch == 'k' || ch == 'K') result.holdK = true;
        if (ch == 'l' || ch == 'L') result.triggerL = true;
    }
#else
    fd_set set;
    timeval tv{};
    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);

    while (select(STDIN_FILENO + 1, &set, nullptr, nullptr, &tv) > 0) {
        char buffer = 0;
        const ssize_t bytes = ::read(STDIN_FILENO, &buffer, 1);
        if (bytes <= 0) break;
        if (buffer == 'k' || buffer == 'K') result.holdK = true;
        if (buffer == 'l' || buffer == 'L') result.triggerL = true;

        FD_ZERO(&set);
        FD_SET(STDIN_FILENO, &set);
        tv = timeval{};
    }
#endif
    return result;
}

pinocchio::SE3 WristToSE3(const WristPose& wrist) {
    Eigen::Quaterniond quat(wrist.orientation.w, wrist.orientation.x, wrist.orientation.y, wrist.orientation.z);
    quat.normalize();
    return {quat.toRotationMatrix(), Eigen::Vector3d(wrist.position.x, wrist.position.y, wrist.position.z)};
}

class PinocchioIkSolver {
public:
    PinocchioIkSolver(const std::string& urdfPath, std::string endEffectorFrame, size_t dof)
        : dof_(dof), endEffectorFrame_(std::move(endEffectorFrame)) {
        try {
            pinocchio::urdf::buildModel(urdfPath, model_);
            data_           = pinocchio::Data(model_);
            frameId_        = model_.getFrameId(endEffectorFrame_);
            neutralQ_       = Eigen::VectorXd::Zero(model_.nq);
            lastSolutionQ_  = neutralQ_;
            pinocchio::forwardKinematics(model_, data_, neutralQ_);
            pinocchio::updateFramePlacements(model_, data_);
            homePose_ = data_.oMf[frameId_];
        } catch (const std::exception& e) {
            std::cerr << "Failed to construct Pinocchio model: " << e.what() << "\\n";
        }
    }

    size_t Dof() const { return dof_; }

    bool IsReady() const { return model_.nq > 0 && frameId_ < model_.frames.size(); }

    pinocchio::SE3 HomePose() const { return homePose_; }

    std::vector<float> Solve(const pinocchio::SE3& target) {
        if (!IsReady()) return std::vector<float>(dof_, 0.0f);

        Eigen::VectorXd q = lastSolutionQ_.size() == model_.nq ? lastSolutionQ_ : neutralQ_;

        static constexpr double damping    = 1e-4;
        static constexpr int    maxIters   = 50;
        static constexpr double tolerance  = 1e-4;

        for (int iter = 0; iter < maxIters; ++iter) {
            pinocchio::forwardKinematics(model_, data_, q);
            pinocchio::updateFramePlacements(model_, data_);
            const pinocchio::SE3& current = data_.oMf[frameId_];

            pinocchio::SE3 errorSE3 = current.actInv(target);
            Eigen::Matrix<double, 6, 1> error = pinocchio::log6(errorSE3).toVector();
            if (error.norm() < tolerance) break;

            Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, model_.nv);
            pinocchio::computeFrameJacobian(model_, data_, q, frameId_, pinocchio::LOCAL, J);

            Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
            JJt.diagonal().array() += damping;
            Eigen::Matrix<double, Eigen::Dynamic, 1> dq = -J.transpose() * JJt.ldlt().solve(error);

            q = pinocchio::integrate(model_, q, dq);
        }

        lastSolutionQ_ = q;

        std::vector<float> result(dof_, 0.0f);
        for (size_t i = 0; i < std::min(dof_, static_cast<size_t>(q.size())); ++i) {
            result[i] = static_cast<float>(q[static_cast<long>(i)]);
        }
        return result;
    }

private:
    size_t                dof_{0};
    pinocchio::Model      model_{};
    pinocchio::Data       data_{};
    pinocchio::SE3        homePose_ = pinocchio::SE3::Identity();
    pinocchio::FrameIndex frameId_{0};
    std::string           endEffectorFrame_{};
    Eigen::VectorXd       neutralQ_{};
    Eigen::VectorXd       lastSolutionQ_{};
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

struct CalibrationState {
    pinocchio::SE3 offset = pinocchio::SE3::Identity();
    bool           hasOffset{false};
};

bool RobotAtHome(const std::unordered_map<std::string, std::vector<float>>& lastCommands) {
    static constexpr float kHomeTolerance = 1e-3f;
    if (lastCommands.empty()) return false;
    for (const auto& [_, joints] : lastCommands) {
        for (float j : joints) {
            if (std::fabs(j) > kHomeTolerance) return false;
        }
    }
    return true;
}

std::vector<JointCommand> MakeHomeCommands(const std::vector<std::string>& sides, size_t dof) {
    std::vector<JointCommand> commands;
    for (const std::string& side : sides) {
        commands.push_back({side, std::vector<float>(dof, 0.0f)});
    }
    return commands;
}
}  // namespace

int main(int argc, char** argv) {
    const Options opts = ParseArgs(argc, argv);

    std::cout << "\n=== Bimanual Teleop Stream (Pinocchio IK -> joint UDP) ===\n";
    std::cout << "Server : " << opts.serverIp << ":" << opts.port << "\n";
    std::cout << "Expecting BVH in OPT coords with YXZ rotation order; output is MocapApi joint poses.\n\n";
    std::cout << "Hold 'k' to stream IK commands. Press 'l' to home, then press 'l' again at home to capture a teleop offset.\n\n";

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
    PinocchioIkSolver ikSolver(opts.urdfPath, opts.endEffectorFrame, 6);  // Fanuc arm has 6 joints
    HandRetargeter    handRetargeter;
    size_t            frame          = 0;
    bool              printedWaiting = false;
    std::unordered_map<std::string, CalibrationState> calibrationBySide;
    std::unordered_map<std::string, std::vector<float>> lastJointCommands;
    bool              homingMode  = false;
    TerminalModeGuard _terminalGuard;
    const pinocchio::SE3 robotHomePose = ikSolver.HomePose();

    if (!ikSolver.IsReady()) {
        std::cerr << "[teleop] Pinocchio model not ready; IK commands will remain zero." << std::endl;
    }

    auto sideEnabled = [&opts](const std::string& side) {
        return std::find(opts.sides.begin(), opts.sides.end(), side) != opts.sides.end();
    };

    while (true) {
        client.Poll();
        mapper.Update(client.LatestJoints());

        const KeyPollResult keyState = PollKeys();
        const bool          robotAtHome = RobotAtHome(lastJointCommands);

        if (keyState.triggerL) {
            if (!robotAtHome) {
                homingMode = true;
                std::cout << "[teleop] Homing robot to zero joints before calibration..." << std::endl;
            } else {
                for (const WristPose& wrist : mapper.WristPoses()) {
                    if (!sideEnabled(wrist.side)) continue;
                    const pinocchio::SE3 deviceHome   = WristToSE3(wrist);
                    CalibrationState&    calibration  = calibrationBySide[wrist.side];
                    calibration.offset                 = robotHomePose * deviceHome.inverse();
                    calibration.hasOffset              = true;
                    std::cout << "[teleop] Captured calibration offset for " << wrist.side << " wrist." << std::endl;
                }
            }
        }

        if (mapper.WristPoses().empty()) {
            if (!printedWaiting) {
                printedWaiting = true;
                std::cout << "Waiting for MocapApi joints..." << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        printedWaiting = false;

        std::vector<JointCommand> commands;
        std::vector<JointCommand> handCommands;

        if (homingMode) {
            commands = MakeHomeCommands(opts.sides, ikSolver.Dof());
        } else if (keyState.holdK) {
            commands.reserve(mapper.WristPoses().size());
            handCommands.reserve(mapper.WristPoses().size());
            for (const WristPose& wrist : mapper.WristPoses()) {
                if (!sideEnabled(wrist.side)) continue;

                const CalibrationState& calibration = calibrationBySide[wrist.side];
                const pinocchio::SE3    target      = calibration.hasOffset
                                                     ? calibration.offset * WristToSE3(wrist)
                                                     : WristToSE3(wrist);
                commands.push_back({wrist.side, ikSolver.Solve(target)});
                handCommands.push_back({wrist.side, handRetargeter.Retarget(mapper.ErgonomicAngles(), wrist.side)});
            }
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
            for (const JointCommand& cmd : commands) {
                lastJointCommands[cmd.side] = cmd.joints;
            }
        }

        if (opts.enableHandUdp && !handCommands.empty()) {
            handStreamer.Send(handCommands, frame);
        }

        if (homingMode && RobotAtHome(lastJointCommands)) {
            homingMode = false;
            std::cout << "[teleop] Homing complete; press 'l' again to capture calibration." << std::endl;
        }

        ++frame;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return 0;
}
