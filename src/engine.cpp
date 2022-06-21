#include "engine.h"

using namespace spe;

glm::ivec2 Engine::GetWindowSize()
{
    return glm::ivec2{ Window::Width, Window::Height };
}

Engine::Engine(int width, int height, std::string title)
    :window(std::move(width), std::move(height), std::move(title))
{
    game = std::make_unique<Game>(*this);
}

Engine::~Engine()
{
    SPDLOG_INFO("Terminate program");
}

void Engine::Run()
{
    SPDLOG_INFO("Start main loop");

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CW);

    auto lastTime = std::chrono::steady_clock::now();
    double deltaTime = 0.0f;
    double sleepAdjust = 1.0;

    while (!window.ShouldClose())
    {
        auto currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<double> duration = currentTime - lastTime;
        double passedTime = duration.count();
        lastTime = currentTime;

        deltaTime += passedTime;

        if (deltaTime > frameTime)
        {
            window.BeginFrame();

            Update((float)deltaTime);
            Render();

            deltaTime = 0.0f;
            window.EndFrame();
        }
        else
        {
            // Dynamic sleep-time adjustment
            double targetSleepTime = (frameTime - deltaTime) * sleepAdjust;

            std::this_thread::sleep_for(std::chrono::duration<double>(targetSleepTime));

            auto awakeTime = std::chrono::steady_clock::now();
            double error = targetSleepTime / (awakeTime - currentTime).count();
            sleepAdjust = 0.9 * sleepAdjust + 0.1 * error;
        }
    }
}

void Engine::Update(float dt)
{
    game->Update(dt);
}

void Engine::Render()
{
    glClearColor(clearColor.r, clearColor.g, clearColor.b, clearColor.a);
    glClear(GL_COLOR_BUFFER_BIT);
    // glViewport(0, 0, Window::Width, Window::Height);

    game->Render();
}

void Engine::SetFrameRate(uint32_t frameRate)
{
    frameRate = std::clamp<int>(frameRate, 30, 300);
    frameTime = 1.0 / frameRate;
}