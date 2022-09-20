#pragma once

#include "common.h"

#include "input.h"
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

namespace muli
{

class Window final
{
    friend class Application;
    friend void OnFramebufferSizeChange(GLFWwindow* glfwWindow, int width, int height);

public:
    static Window& Get();

    ~Window() noexcept;

    Window(const Window&) noexcept = delete;
    Window& operator=(const Window&) noexcept = delete;

    Window(Window&&) noexcept = delete;
    Window& operator=(const Window&&) noexcept = delete;

    void SetFramebufferSizeChangeCallback(std::function<void(int width, int height)> callback);

    Vec2 GetWindowSize() const;
    int32 GetRefreshRate() const;

private:
    static Window* window;
    Window(int width, int height, std::string title);

    int width;
    int height;
    int32 refreshRate;

    GLFWwindow* glfwWindow;
    std::function<void(int width, int height)> framebufferSizeChangeCallback = nullptr;

    bool ShouldClose() const;
    void BeginFrame() const;
    void EndFrame() const;

    static void OnFramebufferSizeChange(GLFWwindow* glfwWindow, int width, int height);
    static void OnKeyEvent(GLFWwindow* glfwWindow, int key, int scancode, int action, int mods);
    static void OnMouseButton(GLFWwindow* glfwWindow, int button, int action, int modifier);
    static void OnCharEvent(GLFWwindow* glfwWindow, unsigned int ch);
    static void OnCursorPos(GLFWwindow* glfwWindow, double xpos, double ypos);
    static void OnScroll(GLFWwindow* glfwWindow, double xoffset, double yoffset);
};

inline Window& Window::Get()
{
    return *window;
}

inline void Window::OnFramebufferSizeChange(GLFWwindow* glfwWindow, int width, int height)
{
    // SPDLOG_INFO("framebuffer size changed: ({} x {})", width, height);

    window->width = width;
    window->height = height;

    if (window->framebufferSizeChangeCallback != nullptr)
    {
        window->framebufferSizeChangeCallback(width, height);
    }
}

inline void Window::OnKeyEvent(GLFWwindow* glfwWindow, int key, int scancode, int action, int mods)
{
    ImGui_ImplGlfw_KeyCallback(glfwWindow, key, scancode, action, mods);

    if (key < 0) return;

    switch (action)
    {
    case GLFW_PRESS:
        Input::currKeys[key] = true;
        break;
    case GLFW_RELEASE:
        Input::currKeys[key] = false;
    }
}

inline void Window::OnMouseButton(GLFWwindow* glfwWindow, int button, int action, int modifier)
{
    ImGui_ImplGlfw_MouseButtonCallback(glfwWindow, button, action, modifier);

    if (button < 0) return;

    switch (action)
    {
    case GLFW_PRESS:
        Input::currBtns[button] = true;
        break;
    case GLFW_RELEASE:
        Input::currBtns[button] = false;
    }
}

inline void Window::OnCharEvent(GLFWwindow* glfwWindow, unsigned int ch)
{
    ImGui_ImplGlfw_CharCallback(glfwWindow, ch);
}

inline void Window::OnCursorPos(GLFWwindow* glfwWindow, double xpos, double ypos)
{
    ImGui_ImplGlfw_CursorPosCallback(glfwWindow, xpos, ypos);

    Input::currMousePos.x = static_cast<float>(xpos);
    Input::currMousePos.y = static_cast<float>(ypos);
}

inline void Window::OnScroll(GLFWwindow* glfwWindow, double xoffset, double yoffset)
{
    ImGui_ImplGlfw_ScrollCallback(glfwWindow, xoffset, yoffset);

    Input::mouseScroll.x = static_cast<float>(xoffset);
    Input::mouseScroll.y = static_cast<float>(yoffset);
}

inline void Window::SetFramebufferSizeChangeCallback(std::function<void(int width, int height)> callback)
{
    framebufferSizeChangeCallback = callback;
}

inline bool Window::ShouldClose() const
{
    return glfwWindowShouldClose(glfwWindow);
}

inline void Window::BeginFrame() const
{
    Input::Update();

    glfwPollEvents();

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

inline void Window::EndFrame() const
{
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(glfwWindow);
}

inline Vec2 Window::GetWindowSize() const
{
    return Vec2{ (float)width, (float)height };
}

inline int32 Window::GetRefreshRate() const
{
    return refreshRate;
}

} // namespace muli