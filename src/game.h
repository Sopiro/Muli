#ifndef __GAME_H__
#define __GAME_H__

#include "common.h"

class Game
{
public:
    Game();
    ~Game();

    void update(float dt);
    void render();
};

#endif