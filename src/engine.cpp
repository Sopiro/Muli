#include "engine.h"

Engine::Engine(int width, int height, std::string title)
    :window(std::move(width), std::move(height), std::move(title))
{
    game = std::make_unique<Game>();
}

Engine::~Engine()
{
    SPDLOG_INFO("Terminate program");
}

void Engine::Run()
{
    SPDLOG_INFO("Start main loop");

    auto lastTime = std::chrono::steady_clock::now();

    constexpr float frameTime = 1 / 60.0f;

    float unprocessedTime = 0.0f;

    while (!window.ShouldClose())
    {
        auto currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<float> duration = currentTime - lastTime;
        float passedTime = duration.count();
        lastTime = currentTime;

        unprocessedTime += passedTime;

        if (unprocessedTime > frameTime)
        {
            window.BeginFrame();

            Update(unprocessedTime);
            Render();

            unprocessedTime = 0.0f;
            window.EndFrame();
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::duration<float>(frameTime - unprocessedTime));
        }
    }
}

void Engine::Update(float dt)
{
    game->Update(dt);
}

void Engine::Render()
{
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    game->Render();
}