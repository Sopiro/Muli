#include "window.h"

Window::Window(int width, int height, std::string title)
{
    SPDLOG_INFO("Initialize glfw");

    if (!glfwInit())
    {
        const char* description = nullptr;
        glfwGetError(&description);
        SPDLOG_ERROR("failed to initialize glfw: {}", description);
        exit(1);
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
    if (!window)
    {
        SPDLOG_ERROR("failed to create glfw window");
        glfwTerminate();
        exit(1);
    }

    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        SPDLOG_ERROR("failed to initialize glad");
        glfwTerminate();
        exit(1);
    }

    auto version = glGetString(GL_VERSION);
    SPDLOG_INFO("OpenGL context version: {}", version);

    // Register some window callbacks
    glfwSetFramebufferSizeCallback(window, onFramebufferSizeChange);
    glfwSetKeyCallback(window, onKeyEvent);
}

void Window::onFramebufferSizeChange(GLFWwindow* window, int width, int height)
{
    // SPDLOG_INFO("framebuffer size changed: ({} x {})", width, height);
    glViewport(0, 0, width, height);
}

void Window::onKeyEvent(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    SPDLOG_INFO("key: {}, scancode: {}, action: {}, mods: {}{}{}", key, scancode,
        action == GLFW_PRESS ? "Pressed" :
        action == GLFW_RELEASE ? "Released" :
        action == GLFW_REPEAT ? "Repeat" : "Unknown",
        mods & GLFW_MOD_CONTROL ? "C" : "-",
        mods & GLFW_MOD_SHIFT ? "S" : "-",
        mods & GLFW_MOD_ALT ? "A" : "-"
    );

    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    {
        glfwSetWindowShouldClose(window, true);
    }
}

bool Window::shouldClose()
{
    return glfwWindowShouldClose(window);
}

void Window::beginFrame()
{
    glfwPollEvents();
}

void Window::endFrame()
{
    glfwSwapBuffers(window);
}

Window::~Window()
{
    glfwTerminate();
}