#pragma once

#include "common.h"
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

        glm::vec4 clearColor{ 0.45f, 0.55f, 0.60f, 1.0f };
    private:
        static Application* app;

        Application(int width, int height, std::string title);
        void Update(float dt);
        void Render();

        Window window;
        std::unique_ptr<Game> game;
        double frameTime{ 1.0f / 60.0f };
    };
}