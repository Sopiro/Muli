#pragma once

#include "game.h"
#include "util.h"
#include "window.h"

#include <chrono>

namespace muli
{

class Application final : NonCopyable
{
public:
    static std::unique_ptr<Application> Create(int32 width, int32 height, std::string title);

    ~Application() noexcept;

    void SetFrameRate(int32 frameRate);
    void Run();

    Vec4 clearColor{ 190.0f / 255.0f, 220.0f / 255.0f, 230.0f / 255.0f, 255.0f / 255.0f };

private:
    static inline Application* app = nullptr;

    Application(int32 width, int32 height, std::string title);

    void Update(float dt);
    void Render();

    Window window;
    std::unique_ptr<Game> game;
    float frameTime;
};

inline std::unique_ptr<Application> Application::Create(int32 width, int32 height, std::string title)
{
    MuliAssert(app == nullptr);

    app = new Application(width, height, std::move(title));
    return std::unique_ptr<Application>(app);
}

inline Application::Application(int32 width, int32 height, std::string title)
    : window(width, height, std::move(title))
{
    Input::Init();

    game = std::make_unique<Game>(*this);
    frameTime = 1.0f / window.GetRefreshRate();
}

inline Application::~Application() noexcept
{
    std::cout << "Terminate program" << '\n';
}

inline void Application::Run()
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

inline void Application::Update(float dt)
{
    game->Update(dt);
}

inline void Application::Render()
{
    game->Render();
}

inline void Application::SetFrameRate(int32 frameRate)
{
    frameRate = std::clamp<int32>(frameRate, 30, 300);
    frameTime = 1.0 / frameRate;
}

} // namespace muli