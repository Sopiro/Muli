#ifndef __WINDOW_H__
#define __WINDOW_H__

#include "common.h"

#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

class Window
{
public:
    Window(int width, int height, std::string title);
    ~Window();

    bool ShouldClose();
    void BeginFrame();
    void EndFrame();

    static int Width;
    static int Height;
private:
    GLFWwindow* window;

    static void OnFramebufferSizeChange(GLFWwindow* window, int width, int height);
    static void OnKeyEvent(GLFWwindow* window, int key, int scancode, int action, int mods);
    static void OnMouseButton(GLFWwindow* window, int button, int action, int modifier);
    static void OnCharEvent(GLFWwindow* window, unsigned int ch);
    static void OnCursorPos(GLFWwindow* window, double xpos, double ypos);
    static void OnScroll(GLFWwindow* window, double xoffset, double yoffset);
};

#endif