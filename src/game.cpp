#include "game.h"

Game::Game()
{}

Game::~Game()
{}

void Game::update(float dt)
{
    
}
void Game::render()
{
    glm::vec3 v(1, 2, 3);

    SPDLOG_INFO("{} {} {}", v.x, v.y, v.z);
}