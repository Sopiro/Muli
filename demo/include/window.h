#pragma once

#include "common.h"
#include "input.h"
#include "util.h"

#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

namespace muli
{

class Window : NonCopyable
{
public:
    ~Window() noexcept;

    void SetFramebufferSizeChangeCallback(std::function<void(int32, int32)> callback);
    Vec2 GetWindowSize() const;
    int32 GetRefreshRate() const;

    bool GetCursorHidden();
    void SetCursorHidden(bool hidden);

    bool ShouldClose() const;
    void BeginFrame(const Vec3& clear_color) const;
    void EndFrame() const;

    static Window* Get();
    static Window* Init(int32 width, int32 height, const char* title);

private:
    Window(int32 width, int32 height, const char* title);

    inline static std::unique_ptr<Window> window;

    GLFWwindow* glfw_window;
    int32 width, height;
    int32 refresh_rate;
    std::function<void(int32, int32)> framebuffer_size_change_callback = nullptr;

    static void ErrorCallback(int32 error, const char* description);
    static void OnFramebufferSizeChange(GLFWwindow* glfw_window, int32 width, int32 height);
    static void OnKeyEvent(GLFWwindow* glfw_window, int32 key, int32 scancode, int32 action, int32 mods);
    static void OnMouseButton(GLFWwindow* glfw_window, int32 button, int32 action, int32 modifier);
    static void OnCharEvent(GLFWwindow* glfw_window, uint32 ch);
    static void OnCursorPos(GLFWwindow* glfw_window, double xpos, double ypos);
    static void OnScroll(GLFWwindow* glfw_window, double xoffset, double yoffset);
};

inline Window* Window::Get()
{
    return window.get();
}

inline Window* Window::Init(int32 width, int32 height, const char* title)
{
    if (window)
    {
        return window.get();
    }

    window = std::unique_ptr<Window>(new Window(width, height, title));
    return window.get();
}

inline void Window::OnFramebufferSizeChange(GLFWwindow* glfw_window, int32 width, int32 height)
{
    MuliNotUsed(glfw_window);

    window->width = width;
    window->height = height;

    if (window->framebuffer_size_change_callback)
    {
        window->framebuffer_size_change_callback(width, height);
    }
}

inline void Window::OnKeyEvent(GLFWwindow* glfw_window, int32 key, int32 scancode, int32 action, int32 mods)
{
    ImGui_ImplGlfw_KeyCallback(glfw_window, key, scancode, action, mods);

    if (key < 0)
    {
        return;
    }

    switch (action)
    {
    case GLFW_PRESS:
        Input::curr_keys[key] = true;
        break;
    case GLFW_RELEASE:
        Input::curr_keys[key] = false;
    }
}

inline void Window::OnMouseButton(GLFWwindow* glfw_window, int32 button, int32 action, int32 modifier)
{
    ImGui_ImplGlfw_MouseButtonCallback(glfw_window, button, action, modifier);

    if (button < 0)
    {
        return;
    }

    switch (action)
    {
    case GLFW_PRESS:
        Input::curr_btns[button] = true;
        break;
    case GLFW_RELEASE:
        Input::curr_btns[button] = false;
    }
}

inline void Window::OnCharEvent(GLFWwindow* glfw_window, uint32 ch)
{
    ImGui_ImplGlfw_CharCallback(glfw_window, ch);
}

inline void Window::OnCursorPos(GLFWwindow* glfw_window, double xpos, double ypos)
{
    ImGui_ImplGlfw_CursorPosCallback(glfw_window, xpos, ypos);

    Input::curr_mouse_pos.x = float(xpos);
    Input::curr_mouse_pos.y = float(ypos);
}

inline void Window::OnScroll(GLFWwindow* glfw_window, double xoffset, double yoffset)
{
    ImGui_ImplGlfw_ScrollCallback(glfw_window, xoffset, yoffset);

    Input::mouse_scroll.x = float(xoffset);
    Input::mouse_scroll.y = float(yoffset);
}

inline void Window::ErrorCallback(int32 error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

inline Window::Window(int32 width, int32 height, const char* title)
    : width{ width }
    , height{ height }
{
    fprintf(stdout, "Initialize glfw\n");
    glfwSetErrorCallback(ErrorCallback);

    if (!glfwInit())
    {
#ifndef __EMSCRIPTEN__
        const char* description;
        glfwGetError(&description);
        fprintf(stdout, "Failed to initialize glfw: %s\n", description);
#else
        fprintf(stdout, "Failed to initialize glfw");
#endif
        exit(1);
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    glfwWindowHint(GLFW_SAMPLES, 4); // 4xMSAA

    glfw_window = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (!glfw_window)
    {
        fprintf(stdout, "%s\n", "Failed to create glfw window");
        glfwTerminate();
        exit(1);
    }

#ifndef __EMSCRIPTEN__
    const GLFWvidmode* vidmode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    glfwSetWindowMonitor(
        glfw_window, NULL, (vidmode->width / 2) - (width / 2), (vidmode->height / 2) - (height / 2), width, height, GLFW_DONT_CARE
    );
#else
    glfwSetWindowSize(glfw_window, width, height);
#endif

    glfwMakeContextCurrent(glfw_window);
    glfwSwapInterval(0); // Disable vsync

#ifndef __EMSCRIPTEN__
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        puts("Failed to initialize glad");
        glfwTerminate();
        exit(1);
    }
#endif

    const char* version = (const char*)glGetString(GL_VERSION);
    fprintf(stdout, "OpenGL context version: %s\n", version);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    // ImGuiIO& io = ImGui::GetIO();

    // ImGui::StyleColorsClassic();
    // ImGui::StyleColorsLight();
    ImGui::StyleColorsDark();

    // Rounded corner style
    float rounding = 5.0f;
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = rounding;
    style.ChildRounding = rounding;
    style.FrameRounding = rounding;
    style.GrabRounding = rounding;
    style.PopupRounding = rounding;
    style.ScrollbarRounding = rounding;

    ImGui_ImplGlfw_InitForOpenGL(glfw_window, false);
#ifndef __EMSCRIPTEN__
    ImGui_ImplOpenGL3_Init("#version 330");
#else
    ImGui_ImplOpenGL3_Init("#version 100");
#endif

    // Setup font
    // io.Fonts->AddFontFromFileTTF("misc/Roboto-Medium.ttf", 13.0f);
    // io.Fonts->AddFontFromFileTTF("misc/NotoSans-Regular.ttf", 13.0f);

    // Register some window callbacks
    glfwSetFramebufferSizeCallback(glfw_window, OnFramebufferSizeChange);
    glfwSetKeyCallback(glfw_window, OnKeyEvent);
    glfwSetCharCallback(glfw_window, OnCharEvent);
    glfwSetCursorPosCallback(glfw_window, OnCursorPos);
    glfwSetMouseButtonCallback(glfw_window, OnMouseButton);
    glfwSetScrollCallback(glfw_window, OnScroll);

#ifndef __EMSCRIPTEN__
    refresh_rate = glfwGetVideoMode(glfwGetPrimaryMonitor())->refreshRate;
#else
    refresh_rate = 60;
#endif

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);
}

inline Window::~Window() noexcept
{
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(glfw_window);
    glfwTerminate();
}

inline void Window::SetFramebufferSizeChangeCallback(std::function<void(int32 width, int32 height)> callback)
{
    framebuffer_size_change_callback = std::move(callback);
}

inline bool Window::GetCursorHidden()
{
    return glfwGetInputMode(glfw_window, GLFW_CURSOR) == GLFW_CURSOR_DISABLED;
}

inline void Window::SetCursorHidden(bool hidden)
{
    if (hidden)
    {
        glfwSetInputMode(glfw_window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    }
    else
    {
        glfwSetInputMode(glfw_window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        glfwSetCursorPos(glfw_window, width / 2.0f, height / 2.0f);
    }
}

inline bool Window::ShouldClose() const
{
    return glfwWindowShouldClose(glfw_window);
}

inline void Window::BeginFrame(const Vec3& clearColor) const
{
    glfwPollEvents();

    // Begin ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    glClearColor(clearColor.x, clearColor.y, clearColor.z, 1);
    glClear(GL_COLOR_BUFFER_BIT);
}

inline void Window::EndFrame() const
{
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(glfw_window);

    Input::Update();
}

inline Vec2 Window::GetWindowSize() const
{
    return Vec2{ float(width), float(height) };
}

inline int32 Window::GetRefreshRate() const
{
    return refresh_rate;
}

} // namespace muli