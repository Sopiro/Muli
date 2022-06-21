#ifndef __ENGINE_H__
#define __ENGINE_H__

#include "common.h"
#include "window.h"
#include "game.h"

namespace spe
{
    class Engine
    {
    public:
        Engine(int width, int height, std::string title);
        ~Engine();

        Engine(const Engine&) = delete;
        Engine(Engine&&) = delete;
        Engine& operator=(const Engine&) = delete;
        Engine& operator=(const Engine&&) = delete;

        void SetFrameRate(uint32_t frameRate);
        glm::ivec2 GetWindowSize();

        void Run();
        void Update(float dt);
        void Render();

        glm::vec4 clearColor{ 0.45f, 0.55f, 0.60f, 1.0f };
    private:
        Window window;
        std::unique_ptr<Game> game;
        double frameTime{ 1.0f / 60.0f };
    };
}

#endif