#pragma once

#include "common.h"
#include "window.h"
#include "game.h"

class Engine
{
public:
    Engine(int width, int height, std::string title);
    ~Engine();

    Engine() = delete;
    Engine(const Engine&) = delete;
    Engine(Engine&&) = delete;
    Engine& operator=(const Engine&) = delete;

    void Run();
    void Update(float dt);
    void Render();
private:
    Window window;
    std::unique_ptr<Game> game;
};