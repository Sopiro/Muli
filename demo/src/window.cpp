#include "window.h"

namespace muli
{

static void glfw_error_callback(int32 error, const char* description)
{
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

Window::Window(int32 width, int32 height, const std::string& title)
    : width{ width }
    , height{ height }
{
    MuliAssert(window == nullptr);
    window = this;

    puts("Initialize glfw");

    glfwSetErrorCallback(glfw_error_callback);

    if (!glfwInit())
    {
        const char* description;
        glfwGetError(&description);
        printf("failed to initialize glfw: %s\n", description);
        exit(1);
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    glfwWindowHint(GLFW_SAMPLES, 4); // 4xMSAA

    glfwWindow = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
    if (!glfwWindow)
    {
        printf("%s\n", "failed to create glfw window");
        glfwTerminate();
        exit(1);
    }

    const GLFWvidmode* vidmode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    glfwSetWindowMonitor(
        glfwWindow, NULL, (vidmode->width / 2) - (width / 2), (vidmode->height / 2) - (height / 2), width, height, GLFW_DONT_CARE
    );

    glfwMakeContextCurrent(glfwWindow);
    glfwSwapInterval(0); // Disable vsync

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        puts("failed to initialize glad");
        glfwTerminate();
        exit(1);
    }

    const char* version = (const char*)glGetString(GL_VERSION);
    printf("OpenGL context version: %s\n", version);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    // io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    // io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
    // io.ConfigFlags |= ImGuiConfigFlags_DpiEnableScaleFonts;

    // ImGui::StyleColorsClassic();
    // ImGui::StyleColorsLight();
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();

    style.Colors[ImGuiCol_WindowBg] = ImColor(33, 34, 32, 240);

    // Rounded corner style
    float rounding = 5.0f;
    style.WindowRounding = rounding;
    style.ChildRounding = rounding;
    style.FrameRounding = rounding;
    style.GrabRounding = rounding;
    style.PopupRounding = rounding;
    style.ScrollbarRounding = rounding;

    ImGui_ImplGlfw_InitForOpenGL(glfwWindow, false);
    ImGui_ImplOpenGL3_Init("#version 330");

    // Setup font
    // io.Fonts->AddFontFromFileTTF("misc/Roboto-Medium.ttf", 13.0f);
    // io.Fonts->AddFontFromFileTTF("misc/NotoSans-Regular.ttf", 13.0f);

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

void Window::BeginFrame(const Vec4& clearColor) const
{
    Input::Update();

    glfwPollEvents();

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    glClearColor(clearColor.x, clearColor.y, clearColor.z, clearColor.w);
    glClear(GL_COLOR_BUFFER_BIT);
    // glViewport(0, 0, Window::Width, Window::Height);
}

void Window::EndFrame() const
{
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    // ImGuiIO& io = ImGui::GetIO();
    // if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
    // {
    //     GLFWwindow* backup_current_context = glfwGetCurrentContext();
    //     ImGui::UpdatePlatformWindows();
    //     ImGui::RenderPlatformWindowsDefault();
    //     glfwMakeContextCurrent(backup_current_context);
    // }

    glfwSwapBuffers(glfwWindow);
}

} // namespace muli