#include "HandVisualizer.h"

#include <GLFW/glfw3.h>
#include <cmath>
#include <iostream>

HandVisualizer::HandVisualizer(HandMocapReceiver& receiver) : receiver_(receiver) {}

bool HandVisualizer::initWindow(int width, int height, const char* title)
{
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return false;
    }

    window_ = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (!window_) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    glViewport(0, 0, width, height);
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-1.0, 1.0, -0.6, 0.6, 1.0, 100.0);
    glMatrixMode(GL_MODELVIEW);

    return true;
}

void HandVisualizer::renderLoop()
{
    while (window_ && !glfwWindowShouldClose(window_)) {
        receiver_.pollEvents();

        HandState left;
        HandState right;
        receiver_.updateHands(left, right);

        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glTranslatef(0.0f, -0.5f, -5.0f);
        glRotatef(-20.0f, 1.0f, 0.0f, 0.0f);

        drawHand(left);
        drawHand(right);

        glfwSwapBuffers(window_);
        glfwPollEvents();
    }

    if (window_) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
    }
    glfwTerminate();
}

void HandVisualizer::drawHand(const HandState& hand)
{
    if (!hand.valid) {
        return;
    }

    if (hand.side == "left") {
        glColor3f(0.2f, 0.6f, 1.0f);
    } else {
        glColor3f(1.0f, 0.3f, 0.3f);
    }

    glPointSize(8.0f);
    glBegin(GL_POINTS);
    glVertex3f(hand.wristPos[0], hand.wristPos[1], hand.wristPos[2]);
    for (const auto& ja : hand.fingers) {
        glVertex3f(ja.pos[0], ja.pos[1], ja.pos[2]);
    }
    glEnd();
}
