#pragma once

#include "common.h"
#include "shader.h"
#include "poly.h"

class Game
{
public:
    Game();
    ~Game();

    void update(float dt);
    void render();

private:
    std::unique_ptr<Shader> s;
    std::unique_ptr<Poly> p;
    float time{ 0.0f };
};
