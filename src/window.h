#pragma once

#include "common.h"

class Window
{
public:
    Window(int width, int height, std::string title);

    bool ShouldClose();
    void BeginFrame();
    void EndFrame();

    ~Window();
private:
    GLFWwindow* window;
    
    static void OnFramebufferSizeChange(GLFWwindow* window, int width, int height);
    static void OnKeyEvent(GLFWwindow* window, int key, int scancode, int action, int mods);
};