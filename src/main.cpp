#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include "MocapClient.h"
#include "SkeletonViewer.h"

namespace {
struct Options {
    std::string serverIp = "127.0.0.1";
    uint16_t    port     = 7012;
    SkeletonViewer::SkeletonFilter filter = SkeletonViewer::SkeletonFilter::FullSkeleton;
};

SkeletonViewer::SkeletonFilter ParseFilter(const std::string& filter) {
    if (filter == "hands") return SkeletonViewer::SkeletonFilter::HandsOnly;
    if (filter == "body") return SkeletonViewer::SkeletonFilter::BodyOnly;
    return SkeletonViewer::SkeletonFilter::FullSkeleton;
}

void PrintUsage(const char* exeName) {
    std::cout << "Usage: " << exeName
              << " [--server <ip>] [--port <udp_port>] [--filter full|hands|body]\n";
    std::cout << "  --server  Axis Studio host that streams BVH (default 127.0.0.1)\n";
    std::cout << "  --port    UDP port for BVH stream (default 7012)\n";
    std::cout << "  --filter  Which part of the skeleton to draw (default full)\n";
}

Options ParseArgs(int argc, char** argv) {
    Options opts{};
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--server" && i + 1 < argc) {
            opts.serverIp = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            opts.port = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if (arg == "--filter" && i + 1 < argc) {
            opts.filter = ParseFilter(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            PrintUsage(argv[0]);
            std::exit(0);
        }
    }
    return opts;
}
}  // namespace

int main(int argc, char** argv) {
    const Options opts = ParseArgs(argc, argv);

    std::cout << "\n=== Axis BVH -> MocapApi Skeleton Viewer ===\n";
    std::cout << "Server : " << opts.serverIp << ":" << opts.port << "\n";
    std::cout << "Filter : "
              << (opts.filter == SkeletonViewer::SkeletonFilter::FullSkeleton
                      ? "full"
                      : (opts.filter == SkeletonViewer::SkeletonFilter::BodyOnly ? "body" : "hands"))
              << "\n";
    std::cout << "Connecting to Axis Studio BVH stream (OPT coord system, YXZ rotation order); all "
                 "visualization will use MocapApi joints reported via IMCPAvatar/IMCPJoint.\n\n";

    MocapClient client(opts.serverIp, opts.port);
    if (!client.Initialize()) return 1;

    SkeletonViewer viewer;
    viewer.Create(1280, 720);
    viewer.SetViewAngles(-25.0f, -35.0f);
    viewer.SetViewDistance(6.5f);
    viewer.SetSkeletonFilter(opts.filter);

    bool printedWaiting = false;
    bool printedReady   = false;

    while (viewer.Alive()) {
        client.Poll();
        const auto& joints = client.LatestJoints();

        if (!joints.empty()) {
            if (!printedReady) {
                printedReady = true;
                std::cout << "Skeleton ready. Drawing fitted MocapApi hierarchy from BVH frames (no raw BVH joints)"
                          << std::endl;
            }
            viewer.Draw(joints);
        } else if (!printedWaiting) {
            printedWaiting = true;
            std::cout << "Waiting for BVH frames from Axis Studio..." << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return 0;
}
