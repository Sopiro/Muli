#pragma once

#include "game.h"
#include "window.h"

namespace muli
{

class Application final
{
public:
    static Application* Create(int32 width, int32 height, std::string title);

    ~Application() noexcept;

    Application(const Application&) noexcept = delete;
    Application& operator=(const Application&) noexcept = delete;

    Application(Application&&) noexcept = delete;
    Application& operator=(Application&&) noexcept = delete;

    void SetFrameRate(uint32 frameRate);

    void Run();

    Vec4 clearColor{ 190.0f / 255.0f, 220.0f / 255.0f, 230.0f / 255.0f, 1.0f };

private:
    static Application* app;

    Application(int32 width, int32 height, std::string title);
    void Update(float dt);
    void Render();

    Window window;
    std::unique_ptr<Game> game;
    double frameTime;
};

inline Application* Application::Create(int32 width, int32 height, std::string title)
{
    muliAssert(app == nullptr);

    Application::app = new Application(width, height, title);
    return Application::app;
}

inline void Application::SetFrameRate(uint32 frameRate)
{
    frameRate = std::clamp<int32>(frameRate, 30, 300);
    frameTime = 1.0 / frameRate;
}

} // namespace muli