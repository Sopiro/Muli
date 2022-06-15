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

void Engine::run()
{
    SPDLOG_INFO("Start main loop");

    auto lastTime = std::chrono::steady_clock::now();

    constexpr float frameTime = 1 / 60.0f;

    float unprocessedTime = 0.0f;

    while (!window.shouldClose())
    {
        window.beginFrame();

        auto currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<float> duration = currentTime - lastTime;
        float passedTime = duration.count();
        lastTime = currentTime;

        unprocessedTime += passedTime;

        if (unprocessedTime > frameTime)
        {
            update(unprocessedTime);
            render();

            window.endFrame();

            unprocessedTime = 0.0f;
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::duration<float>(frameTime - unprocessedTime));
        }
    }
}

void Engine::update(float dt)
{
    game->update(dt);
}

void Engine::render()
{
    glClearColor(1.0f, 0.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    game->render();
}