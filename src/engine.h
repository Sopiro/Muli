#pragma once

#include "common.h"
#include "window.h"
#include "game.h"

class Engine
{
public:
    Engine(int width, int height, std::string title);
    ~Engine();

    Engine(const Engine&) = delete;
    Engine(Engine&&) = delete;
    Engine& operator=(const Engine&) = delete;

    void SetClearColor(glm::vec4 _clearColor);
    void SetFrameRate(uint32_t frameRate);

    void Run();
    void Update(float dt);
    void Render();
private:
    Window window;
    std::unique_ptr<Game> game;
    double frameTime{ 1.0f / 60.0f };
    glm::vec4 clearColor{ 0.0f, 0.0f, 0.0f, 1.0f };
};