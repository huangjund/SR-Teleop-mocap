#ifndef HAND_VISUALIZER_H
#define HAND_VISUALIZER_H

#include "HandMocapReceiver.h"

struct GLFWwindow;

class HandVisualizer {
public:
    explicit HandVisualizer(HandMocapReceiver& receiver);
    bool initWindow(int width, int height, const char* title);
    void renderLoop();

private:
    HandMocapReceiver& receiver_;
    GLFWwindow* window_ = nullptr;

    void drawHand(const HandState& hand);
};

#endif  // HAND_VISUALIZER_H
