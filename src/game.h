#ifndef __GAME_H__
#define __GAME_H__

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

    std::unique_ptr<Shader> t;
    std::unique_ptr<Poly> p;
private:
    float time{ 0.0f };
};

#endif