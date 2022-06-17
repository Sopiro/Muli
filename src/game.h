#pragma once

#include "common.h"
#include "shader.h"
#include "myshader.h"
#include "poly.h"

class Game
{
public:
    Game();
    ~Game();

    void Update(float dt);
    void Render();

private:
    std::unique_ptr<MyShader> s;
    std::unique_ptr<Poly> p;
    float time{ 0.0f };
};
