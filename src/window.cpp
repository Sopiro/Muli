#include "window.h"

Window::Window(int width, int height, std::string title)
{
    SPDLOG_INFO("Initialize glfw");

    error = false;

    if (!glfwInit())
    {
        const char* description = nullptr;
        glfwGetError(&description);
        SPDLOG_ERROR("failed to initialize glfw: {}", description);
        error = true;
        return;
    }

    window = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
    if (!window)
    {
        SPDLOG_ERROR("failed to create glfw window");
        glfwTerminate();
        error = true;
        return;
    }
}

bool Window::errorOccurred()
{
    return error;
}

bool Window::shouldClose()
{
    return glfwWindowShouldClose(window);
}

void Window::pollEvents()
{
    glfwPollEvents();
}

Window::~Window()
{
    glfwTerminate();
}