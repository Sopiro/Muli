#pragma once

#include "common.h"
#include "input.h"
#include "util.h"

#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

namespace muli
{

class Window final : NonCopyable
{
public:
    ~Window() noexcept;

    void SetFramebufferSizeChangeCallback(const std::function<void(int32 width, int32 height)>& callback);

    Vec2 GetWindowSize() const;
    int32 GetRefreshRate() const;

    static Window& Get();

private:
    friend class Application;
    friend void OnFramebufferSizeChange(GLFWwindow* glfwWindow, int32 width, int32 height);

    Window(int32 width, int32 height, const std::string& title);

    bool ShouldClose() const;
    void BeginFrame(const Vec4& clearColor) const;
    void EndFrame() const;

    int32 width, height;
    int32 refreshRate;

    GLFWwindow* glfwWindow;
    std::function<void(int32 width, int32 height)> framebufferSizeChangeCallback = nullptr;

    static void OnFramebufferSizeChange(GLFWwindow* glfwWindow, int32 width, int32 height);
    static void OnKeyEvent(GLFWwindow* glfwWindow, int32 key, int32 scancode, int32 action, int32 mods);
    static void OnMouseButton(GLFWwindow* glfwWindow, int32 button, int32 action, int32 modifier);
    static void OnCharEvent(GLFWwindow* glfwWindow, uint32 ch);
    static void OnCursorPos(GLFWwindow* glfwWindow, double xpos, double ypos);
    static void OnScroll(GLFWwindow* glfwWindow, double xoffset, double yoffset);

    static inline Window* window = nullptr;
};

inline Window& Window::Get()
{
    return *window;
}

inline void Window::OnFramebufferSizeChange(GLFWwindow* glfwWindow, int32 width, int32 height)
{
    window->width = width;
    window->height = height;

    if (window->framebufferSizeChangeCallback != nullptr)
    {
        window->framebufferSizeChangeCallback(width, height);
    }
}

inline void Window::OnKeyEvent(GLFWwindow* glfwWindow, int32 key, int32 scancode, int32 action, int32 mods)
{
    ImGui_ImplGlfw_KeyCallback(glfwWindow, key, scancode, action, mods);

    if (key < 0) return;

    switch (action)
    {
    case GLFW_PRESS:
        Input::curr_keys[key] = true;
        break;
    case GLFW_RELEASE:
        Input::curr_keys[key] = false;
        break;
    }
}

inline void Window::OnMouseButton(GLFWwindow* glfwWindow, int32 button, int32 action, int32 modifier)
{
    ImGui_ImplGlfw_MouseButtonCallback(glfwWindow, button, action, modifier);

    if (button < 0) return;

    switch (action)
    {
    case GLFW_PRESS:
        Input::curr_btns[button] = true;
        break;
    case GLFW_RELEASE:
        Input::curr_btns[button] = false;
        break;
    }
}

inline void Window::OnCharEvent(GLFWwindow* glfwWindow, uint32 ch)
{
    ImGui_ImplGlfw_CharCallback(glfwWindow, ch);
}

inline void Window::OnCursorPos(GLFWwindow* glfwWindow, double xpos, double ypos)
{
    ImGui_ImplGlfw_CursorPosCallback(glfwWindow, xpos, ypos);

    Input::curr_mouse_pos.x = float(xpos);
    Input::curr_mouse_pos.y = float(ypos);
}

inline void Window::OnScroll(GLFWwindow* glfwWindow, double xoffset, double yoffset)
{
    ImGui_ImplGlfw_ScrollCallback(glfwWindow, xoffset, yoffset);

    Input::mouse_scroll.x = float(xoffset);
    Input::mouse_scroll.y = float(yoffset);
}

inline void Window::SetFramebufferSizeChangeCallback(const std::function<void(int32 width, int32 height)>& callback)
{
    framebufferSizeChangeCallback = callback;
}

inline bool Window::ShouldClose() const
{
    return glfwWindowShouldClose(glfwWindow);
}

inline Vec2 Window::GetWindowSize() const
{
    return Vec2{ float(width), float(height) };
}

inline int32 Window::GetRefreshRate() const
{
    return refreshRate;
}

} // namespace muli