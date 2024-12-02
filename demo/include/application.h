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
    Game* game;
    float frameTime;
};

} // namespace muli