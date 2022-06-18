#include "window.h"

int Window::Width = 0;
int Window::Height = 0;

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

void Window::OnFramebufferSizeChange(GLFWwindow* window, int width, int height)
{
    // SPDLOG_INFO("framebuffer size changed: ({} x {})", width, height);

    Window::Width = width;
    Window::Height = height;

    glViewport(0, 0, width, height);
}

void Window::OnKeyEvent(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    ImGui_ImplGlfw_KeyCallback(window, key, scancode, action, mods);

    // SPDLOG_INFO("key: {}, scancode: {}, action: {}, mods: {}{}{}", key, scancode,
    //     action == GLFW_PRESS ? "Pressed" :
    //     action == GLFW_RELEASE ? "Released" :
    //     action == GLFW_REPEAT ? "Repeat" : "Unknown",
    //     mods & GLFW_MOD_CONTROL ? "C" : "-",
    //     mods & GLFW_MOD_SHIFT ? "S" : "-",
    //     mods & GLFW_MOD_ALT ? "A" : "-"
    // );

    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    {
        glfwSetWindowShouldClose(window, true);
    }
}

void Window::OnMouseButton(GLFWwindow* window, int button, int action, int modifier)
{
    ImGui_ImplGlfw_MouseButtonCallback(window, button, action, modifier);
}

void Window::OnCharEvent(GLFWwindow* window, unsigned int ch)
{
    ImGui_ImplGlfw_CharCallback(window, ch);
}

void Window::OnCursorPos(GLFWwindow* window, double xpos, double ypos)
{
    ImGui_ImplGlfw_CursorPosCallback(window, xpos, ypos);
}

void Window::OnScroll(GLFWwindow* window, double xoffset, double yoffset)
{
    ImGui_ImplGlfw_ScrollCallback(window, xoffset, yoffset);
}

Window::Window(int width, int height, std::string title)
{
    SPDLOG_INFO("Initialize glfw");

    glfwSetErrorCallback(glfw_error_callback);

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

    Window::Width = width;
    Window::Height = height;

    window = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
    if (!window)
    {
        SPDLOG_ERROR("failed to create glfw window");
        glfwTerminate();
        exit(1);
    }

    glfwMakeContextCurrent(window);
    // glfwSwapInterval(1); // Enable vsync

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        SPDLOG_ERROR("failed to initialize glad");
        glfwTerminate();
        exit(1);
    }

    auto version = glGetString(GL_VERSION);
    SPDLOG_INFO("OpenGL context version: {}", version);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    ImGui::StyleColorsDark();
    //ImGui::StyleColorsClassic();

    // Rounded corner style
    float rounding = 5.0f;
    auto& style = ImGui::GetStyle();
    style.WindowRounding = rounding;
    style.ChildRounding = rounding;
    style.FrameRounding = rounding;
    style.GrabRounding = rounding;
    style.PopupRounding = rounding;
    style.ScrollbarRounding = rounding;

    ImGui_ImplGlfw_InitForOpenGL(window, false);
    ImGui_ImplOpenGL3_Init("#version 330");

    // Setup font
    ImFontConfig config;
    config.OversampleH = 1;
    config.OversampleV = 1;
    config.PixelSnapH = true;
    io.Fonts->AddFontFromFileTTF("../../res/fonts/Roboto-Medium.ttf", 18.0f, &config);
    // io.Fonts->AddFontFromFileTTF("../../res/fonts/NotoSans-Regular.ttf", 16.0f, &config);

    // Register some window callbacks
    glfwSetFramebufferSizeCallback(window, OnFramebufferSizeChange);
    glfwSetKeyCallback(window, OnKeyEvent);
    glfwSetCharCallback(window, OnCharEvent);
    glfwSetCursorPosCallback(window, OnCursorPos);
    glfwSetMouseButtonCallback(window, OnMouseButton);
    glfwSetScrollCallback(window, OnScroll);
}

bool Window::ShouldClose()
{
    return glfwWindowShouldClose(window);
}

void Window::BeginFrame()
{
    glfwPollEvents();

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void Window::EndFrame()
{
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
}

Window::~Window()
{
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
}