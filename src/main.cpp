#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "HandMocapReceiver.h"

int main() {
    HandMocapReceiver receiver("127.0.0.1", 7012);
    if (!receiver.init()) {
        std::cerr << "Failed to init HandMocapReceiver\n";
        return 1;
    }

    auto lastPrint = std::chrono::steady_clock::now();

    while (true) {
        receiver.pollEvents();

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - lastPrint).count() >= 1) {
            lastPrint = now;

            HandState left, right;
            if (receiver.updateHands(left, right)) {
                std::cout << "==== Frame ====\n";
                if (left.valid) {
                    std::cout << "[Left wrist] pos=("
                              << left.wristPos[0] << ", "
                              << left.wristPos[1] << ", "
                              << left.wristPos[2] << ")  quat=("
                              << left.wristQuat[0] << ", "
                              << left.wristQuat[1] << ", "
                              << left.wristQuat[2] << ", "
                              << left.wristQuat[3] << ")\n";
                    for (const auto& ja : left.fingers) {
                        if (ja.name.find("Index") != std::string::npos ||
                            ja.name.find("Thumb") != std::string::npos) {
                            std::cout << "  " << ja.name << " : ex="
                                      << ja.ex << " ey=" << ja.ey << " ez=" << ja.ez << "\n";
                        }
                    }
                }
                if (right.valid) {
                    std::cout << "[Right wrist] pos=("
                              << right.wristPos[0] << ", "
                              << right.wristPos[1] << ", "
                              << right.wristPos[2] << ")  quat=("
                              << right.wristQuat[0] << ", "
                              << right.wristQuat[1] << ", "
                              << right.wristQuat[2] << ", "
                              << right.wristQuat[3] << ")\n";
                    for (const auto& ja : right.fingers) {
                        if (ja.name.find("Index") != std::string::npos ||
                            ja.name.find("Thumb") != std::string::npos) {
                            std::cout << "  " << ja.name << " : ex="
                                      << ja.ex << " ey=" << ja.ey << " ez=" << ja.ez << "\n";
                        }
                    }
                }
            } else {
                std::cout << "[No avatar yet...]\n";
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    receiver.shutdown();
    return 0;
}
