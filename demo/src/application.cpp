#include "application.h"

namespace muli
{

Application* Application::app = nullptr;

Application::Application(int32 width, int32 height, std::string title)
    : window(width, height, std::move(title))
{
    Input::Init();
    game = std::make_unique<Game>(*this);
    frameTime = 1.0f / window.GetRefreshRate();
}

Application::~Application() noexcept
{
    std::cout << "Terminate program" << '\n';
}

void Application::Run()
{
    std::cout << "Start main loop" << '\n';

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
            window.BeginFrame(clearColor);

            Update(static_cast<float>(deltaTime));
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
    game->Render();
}

} // namespace muli