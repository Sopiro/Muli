#pragma once

#include "window.h"
#include "game.h"

namespace spe
{
class Application final
{
public:
    static Application* Create(int width, int height, std::string title);

    ~Application() noexcept;

    Application(const Application&) noexcept = delete;
    Application& operator=(const Application&) noexcept = delete;

    Application(Application&&) noexcept = delete;
    Application& operator=(Application&&) noexcept = delete;

    void SetFrameRate(uint32_t frameRate);

    void Run();

    glm::vec4 clearColor{ 190.0f / 255.0f, 220.0f / 255.0f, 230.0f / 255.0f, 1.0f };
private:
    static Application* app;

    Application(int width, int height, std::string title);
    void Update(float dt);
    void Render();

    Window window;
    std::unique_ptr<Game> game;
    double frameTime;
};

inline Application* Application::Create(int width, int height, std::string title)
{
    assert(app == nullptr);

    Application::app = new Application(width, height, title);
    return Application::app;
}

inline void Application::SetFrameRate(uint32_t frameRate)
{
    frameRate = std::clamp<int>(frameRate, 30, 300);
    frameTime = 1.0 / frameRate;
}

}