#include "window.h"

namespace muli
{

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

Window::Window(int width, int height, std::string title)
{
    muliAssert(window == nullptr);
    window = this;

    std::puts("Initialize glfw");

    glfwSetErrorCallback(glfw_error_callback);

    if (!glfwInit())
    {
        const char* description;
        glfwGetError(&description);
        std::printf("failed to initialize glfw: %s", description);
        exit(1);
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    Window::width = width;
    Window::height = height;

    glfwWindowHint(GLFW_SAMPLES, 4);

    glfwWindow = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
    if (!glfwWindow)
    {
        std::printf("%s\n", "failed to create glfw window");
        glfwTerminate();
        exit(1);
    }

    const GLFWvidmode* vidmode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    glfwSetWindowMonitor(glfwWindow, NULL, (vidmode->width / 2) - (width / 2), (vidmode->height / 2) - (height / 2), width,
                         height, GLFW_DONT_CARE);

    glfwMakeContextCurrent(glfwWindow);
    // glfwSwapInterval(1); // Enable vsync

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::puts("failed to initialize glad");
        glfwTerminate();
        exit(1);
    }

    const char* version = (const char*)glGetString(GL_VERSION);
    printf("OpenGL context version: %s\n", version);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;

    ImGui::StyleColorsDark();
    // ImGui::StyleColorsClassic();

    // Rounded corner style
    float rounding = 5.0f;
    auto& style = ImGui::GetStyle();
    style.WindowRounding = rounding;
    style.ChildRounding = rounding;
    style.FrameRounding = rounding;
    style.GrabRounding = rounding;
    style.PopupRounding = rounding;
    style.ScrollbarRounding = rounding;

    ImGui_ImplGlfw_InitForOpenGL(glfwWindow, false);
    ImGui_ImplOpenGL3_Init("#version 330");

    // Setup font
    ImFontConfig config;
    config.OversampleH = 1;
    config.OversampleV = 1;
    config.PixelSnapH = true;
    // io.Fonts->AddFontFromFileTTF("../../res/fonts/Roboto-Medium.ttf", 18.0f, &config);
    //  io.Fonts->AddFontFromFileTTF("../../res/fonts/NotoSans-Regular.ttf", 16.0f, &config);

    // Register some window callbacks
    glfwSetFramebufferSizeCallback(glfwWindow, OnFramebufferSizeChange);
    glfwSetKeyCallback(glfwWindow, OnKeyEvent);
    glfwSetCharCallback(glfwWindow, OnCharEvent);
    glfwSetCursorPosCallback(glfwWindow, OnCursorPos);
    glfwSetMouseButtonCallback(glfwWindow, OnMouseButton);
    glfwSetScrollCallback(glfwWindow, OnScroll);

    refreshRate = glfwGetVideoMode(glfwGetPrimaryMonitor())->refreshRate;
}

Window::~Window() noexcept
{
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(glfwWindow);
    glfwTerminate();
}

} // namespace muli