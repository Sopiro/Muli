#pragma once

#include "common.h"

#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include "input.h"

namespace spe
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

        glm::ivec2 GetWindowSize() const;

    private:
        static Window* window;
        Window(int width, int height, std::string title);

        int width;
        int height;

        bool ShouldClose() const;
        void BeginFrame() const;
        void EndFrame() const;
        GLFWwindow* glfwWindow;

        std::function<void(int width, int height)> framebufferSizeChangeCallback = nullptr;
        static void OnFramebufferSizeChange(GLFWwindow* glfwWindow, int width, int height);
        static void OnKeyEvent(GLFWwindow* glfwWindow, int key, int scancode, int action, int mods);
        static void OnMouseButton(GLFWwindow* glfwWindow, int button, int action, int modifier);
        static void OnCharEvent(GLFWwindow* glfwWindow, unsigned int ch);
        static void OnCursorPos(GLFWwindow* glfwWindow, double xpos, double ypos);
        static void OnScroll(GLFWwindow* glfwWindow, double xoffset, double yoffset);
    };
}