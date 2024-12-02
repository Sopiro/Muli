#include "application.h"

namespace muli
{

std::unique_ptr<Application> Application::Create(int32 width, int32 height, std::string title)
{
    MuliAssert(app == nullptr);

    app = new Application(width, height, std::move(title));
    return std::unique_ptr<Application>(app);
}

Application::Application(int32 width, int32 height, std::string title)
    : window(width, height, std::move(title))
{
    Input::Init();

    game = new Game(*this);
    frameTime = 1.0f / window.GetRefreshRate();
}

Application::~Application() noexcept
{
    delete game;
    std::cout << "Terminate program" << '\n';
}

void Application::Run()
{
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);

    std::cout << "Start main loop" << '\n';
    auto lastTime = std::chrono::steady_clock::now();
    float deltaTime = 0.0f;

    while (!window.ShouldClose())
    {
        auto currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<float> elapsed = currentTime - lastTime;
        lastTime = currentTime;

        deltaTime += elapsed.count();
        if (deltaTime > frameTime)
        {
            window.BeginFrame(clearColor);

            Update(deltaTime);
            Render();

            deltaTime = 0.0f;
            window.EndFrame();
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

void Application::SetFrameRate(int32 frameRate)
{
    frameRate = std::clamp<int32>(frameRate, 30, 300);
    frameTime = 1.0 / frameRate;
}

} // namespace muli
