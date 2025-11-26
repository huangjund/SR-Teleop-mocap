#include <iostream>

#include "HandMocapReceiver.h"
#include "HandVisualizer.h"

int main() {
    HandMocapReceiver receiver("127.0.0.1", 7012);
    if (!receiver.init()) {
        std::cerr << "Failed to init HandMocapReceiver\n";
        return 1;
    }

    HandVisualizer viz(receiver);
    if (!viz.initWindow(1280, 720, "SR-Teleop 2-Hand Viewer")) {
        std::cerr << "Failed to init window\n";
        return 1;
    }

    viz.renderLoop();
    receiver.shutdown();
    return 0;
}
