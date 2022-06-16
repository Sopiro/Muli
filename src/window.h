#pragma once

#include "common.h"

class Window
{
public:
    Window(int width, int height, std::string title);

    bool errorOccurred();
    bool shouldClose();
    void beginFrame();
    void endFrame();

    ~Window();
private:
    GLFWwindow* window;
    static void onFramebufferSizeChange(GLFWwindow* window, int width, int height);
    static void onKeyEvent(GLFWwindow* window, int key, int scancode, int action, int mods);
};