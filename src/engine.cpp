#include "engine.h"

Engine::Engine(int width, int height, std::string title)
    :window(std::move(width), std::move(height), std::move(title))
{
    if (window.errorOccurred())
    {
        exit(1);
    }
}

Engine::~Engine()
{
    SPDLOG_INFO("Terminate program");
}

void Engine::run()
{
    SPDLOG_INFO("Start main loop");

    while (!window.shouldClose())
    {
        window.pollEvents();
    }
}