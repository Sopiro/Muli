#pragma once

#include "game.h"
#include "util.h"
#include "window.h"

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
    double frameTime;
};

inline std::unique_ptr<Application> Application::Create(int32 width, int32 height, std::string title)
{
    muliAssert(app == nullptr);

    app = new Application(width, height, std::move(title));
    return std::unique_ptr<Application>(app);
}

inline void Application::SetFrameRate(int32 frameRate)
{
    frameRate = std::clamp<int32>(frameRate, 30, 300);
    frameTime = 1.0 / frameRate;
}

} // namespace muli