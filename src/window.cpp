#include "spe/window.h"

using namespace spe;

Window* Window::window = nullptr;

Window& Window::Get()
{
	return *window;
}

static void glfw_error_callback(int error, const char* description)
{
	fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

void Window::SetFramebufferSizeChangeCallback(std::function<void(int width, int height)> callback)
{
	framebufferSizeChangeCallback = callback;
}

void Window::OnFramebufferSizeChange(GLFWwindow* glfwWindow, int width, int height)
{
	// SPDLOG_INFO("framebuffer size changed: ({} x {})", width, height);

	window->width = width;
	window->height = height;

	if (window->framebufferSizeChangeCallback != nullptr)
	{
		window->framebufferSizeChangeCallback(width, height);
	}
}

void Window::OnKeyEvent(GLFWwindow* glfwWindow, int key, int scancode, int action, int mods)
{
	ImGui_ImplGlfw_KeyCallback(glfwWindow, key, scancode, action, mods);

	// SPDLOG_INFO("key: {}, scancode: {}, action: {}, mods: {}{}{}", key, scancode,
	//     action == GLFW_PRESS ? "Pressed" :
	//     action == GLFW_RELEASE ? "Released" :
	//     action == GLFW_REPEAT ? "Repeat" : "Unknown",
	//     mods & GLFW_MOD_CONTROL ? "C" : "-",
	//     mods & GLFW_MOD_SHIFT ? "S" : "-",
	//     mods & GLFW_MOD_ALT ? "A" : "-"
	// );

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

void Window::OnMouseButton(GLFWwindow* glfwWindow, int button, int action, int modifier)
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

void Window::OnCharEvent(GLFWwindow* glfwWindow, unsigned int ch)
{
	ImGui_ImplGlfw_CharCallback(glfwWindow, ch);
}

void Window::OnCursorPos(GLFWwindow* glfwWindow, double xpos, double ypos)
{
	ImGui_ImplGlfw_CursorPosCallback(glfwWindow, xpos, ypos);

	Input::currMousePos.x = static_cast<float>(xpos);
	Input::currMousePos.y = static_cast<float>(ypos);
}

void Window::OnScroll(GLFWwindow* glfwWindow, double xoffset, double yoffset)
{
	ImGui_ImplGlfw_ScrollCallback(glfwWindow, xoffset, yoffset);

	Input::mouseScroll.x = static_cast<float>(xoffset);
	Input::mouseScroll.y = static_cast<float>(yoffset);
}

Window::Window(int width, int height, std::string title)
{
	assert(window == nullptr);
	window = this;

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

	Window::width = width;
	Window::height = height;

	glfwWindow = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
	if (!glfwWindow)
	{
		SPDLOG_ERROR("failed to create glfw window");
		glfwTerminate();
		exit(1);
	}

	glfwMakeContextCurrent(glfwWindow);
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

	ImGui_ImplGlfw_InitForOpenGL(glfwWindow, false);
	ImGui_ImplOpenGL3_Init("#version 330");

	// Setup font
	ImFontConfig config;
	config.OversampleH = 1;
	config.OversampleV = 1;
	config.PixelSnapH = true;
	//io.Fonts->AddFontFromFileTTF("../../res/fonts/Roboto-Medium.ttf", 18.0f, &config);
	// io.Fonts->AddFontFromFileTTF("../../res/fonts/NotoSans-Regular.ttf", 16.0f, &config);

	// Register some window callbacks
	glfwSetFramebufferSizeCallback(glfwWindow, OnFramebufferSizeChange);
	glfwSetKeyCallback(glfwWindow, OnKeyEvent);
	glfwSetCharCallback(glfwWindow, OnCharEvent);
	glfwSetCursorPosCallback(glfwWindow, OnCursorPos);
	glfwSetMouseButtonCallback(glfwWindow, OnMouseButton);
	glfwSetScrollCallback(glfwWindow, OnScroll);

	refreshRate = glfwGetVideoMode(glfwGetPrimaryMonitor())->refreshRate;
}

Window::~Window()
{
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glfwDestroyWindow(glfwWindow);
	glfwTerminate();
}

bool Window::ShouldClose() const
{
	return glfwWindowShouldClose(glfwWindow);
}

void Window::BeginFrame() const
{
	Input::Update();

	glfwPollEvents();

	// Start the Dear ImGui frame
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
}

void Window::EndFrame() const
{
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

	glfwSwapBuffers(glfwWindow);
}

glm::ivec2 Window::GetWindowSize() const
{
	return glm::ivec2{ width, height };
}

int32_t Window::GetRefreshRate() const
{
	return refreshRate;
}