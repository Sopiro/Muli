#include "application.h"

namespace spe
{

Application* Application::app = nullptr;

Application::Application(int width, int height, std::string title) :
    window(std::move(width), std::move(height), std::move(title))
{
    Input::Init();
    game = std::make_unique<Game>(*this);
    frameTime = 1.0 / window.GetRefreshRate();
}

Application::~Application()
{
    SPDLOG_INFO("Terminate program");
}

void Application::Run()
{
    SPDLOG_INFO("Start main loop");

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);

    auto lastTime = std::chrono::steady_clock::now();
    double deltaTime = 0.0f;

    while (!window.ShouldClose())
    {
        auto currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<double> duration = currentTime - lastTime;
        double passedTime = duration.count();
        lastTime = currentTime;

        deltaTime += passedTime;

        if (deltaTime > frameTime)
        {
            window.BeginFrame();

            Update((float)deltaTime);
            Render();

            deltaTime = 0.0f;
            window.EndFrame();
        }
        else
        {
            // std::this_thread::yield();
        }
    }
}

void Application::Update(float dt)
{
    game->Update(dt);
}

void Application::Render()
{
    glClearColor(clearColor.r, clearColor.g, clearColor.b, clearColor.a);
    glClear(GL_COLOR_BUFFER_BIT);
    // glViewport(0, 0, Window::Width, Window::Height);

    game->Render();
}

}