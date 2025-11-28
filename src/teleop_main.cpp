#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include "MocapClient.h"
#include "TeleopMapping.h"

namespace {
struct Options {
    std::string serverIp = "127.0.0.1";
    uint16_t    port     = 7012;
};

void PrintUsage(const char* exeName) {
    std::cout << "Usage: " << exeName << " [--server <ip>] [--port <udp_port>]\\n";
    std::cout << "  --server  Axis Studio host that streams BVH (default 127.0.0.1)\\n";
    std::cout << "  --port    UDP port for BVH stream (default 7012)\\n";
}

Options ParseArgs(int argc, char** argv) {
    Options opts{};
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--server" && i + 1 < argc) {
            opts.serverIp = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            opts.port = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if (arg == "--help" || arg == "-h") {
            PrintUsage(argv[0]);
            std::exit(0);
        }
    }
    return opts;
}

void PrintWrist(const WristPose& wrist) {
    std::cout << "  " << wrist.side << " wrist pos(" << wrist.position.x << ", " << wrist.position.y << ", " << wrist.position.z
              << ") rot(quat) [" << wrist.orientation.x << ", " << wrist.orientation.y << ", " << wrist.orientation.z << ", "
              << wrist.orientation.w << "]\n";
}

void PrintErgonomic(const ErgonomicJointAngles& angles) {
    std::cout << "    " << angles.name << " flex=" << angles.flexionDeg << "deg abduction=" << angles.abductionDeg
              << "deg twist=" << angles.twistDeg << "deg\n";
}
}  // namespace

int main(int argc, char** argv) {
    const Options opts = ParseArgs(argc, argv);

    std::cout << "\n=== Bimanual Teleop Stream (wrists + ergonomic hands) ===\n";
    std::cout << "Server : " << opts.serverIp << ":" << opts.port << "\n";
    std::cout << "Expecting BVH in OPT coords with YXZ rotation order; output is MocapApi joint poses.\n\n";

    MocapClient client(opts.serverIp, opts.port);
    if (!client.Initialize()) return 1;

    TeleopMapping mapper;
    size_t       frame = 0;
    bool         printedWaiting = false;

    while (true) {
        client.Poll();
        mapper.Update(client.LatestJoints());

        if (mapper.WristPoses().empty() || mapper.ErgonomicAngles().empty()) {
            if (!printedWaiting) {
                printedWaiting = true;
                std::cout << "Waiting for MocapApi joints..." << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        printedWaiting = false;
        if (frame % 30 == 0) {
            std::cout << "\nFrame " << frame << "\n";
            for (const WristPose& wrist : mapper.WristPoses()) {
                PrintWrist(wrist);
            }
            std::cout << "  Ergonomic hand joints (local YXZ -> flex/abduction/twist in deg):\n";
            for (const ErgonomicJointAngles& angles : mapper.ErgonomicAngles()) {
                PrintErgonomic(angles);
            }
            std::cout << std::flush;
        }

        ++frame;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return 0;
}
