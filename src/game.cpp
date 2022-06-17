#include "game.h"

Game::Game()
{
    s = MyShader::Create();
    s->Use();
    s->SetProjectionMatrix(glm::ortho(-12.8f, 12.8f, -7.2f, 7.2f, 0.0f, 100.0f));
    s->SetViewMatrix(glm::translate(glm::mat4{ 1.0 }, glm::vec3(0, 0, -1)));

    p = std::unique_ptr<Poly>(new Poly(
        {
            glm::vec3(0.5f,  0.5f, 0.0f),
            glm::vec3(0.5f, -0.5f, 0.0f),
            glm::vec3(-0.5f, -0.5f, 0.0f),
            glm::vec3(-0.5f,  0.5f, 0.0f),
        }
        ,
        {
            0, 1, 3,
            1, 2, 3,
        }
        ));
}

Game::~Game()
{

}

void Game::Update(float dt)
{
    time += dt;
}

void Game::Render()
{
    s->Use();

    glm::mat4 t = glm::translate(glm::mat4(1.0f), glm::vec3(time, 0.0f, 0.0f));
    glm::mat4 r = glm::rotate(glm::mat4{ 1.0f }, glm::radians(time * 90.0f), glm::vec3(0.0f, 0.0f, 1.0f));

    s->SetModelMatrix(t * r);
    s->SetColor({ glm::sin(time * 2) * 0.5f + 1.0f, glm::cos(time * 3) * 0.5f + 1.0f, glm::sin(time * 1.5) * 0.5f + 1.0f });

    p->Draw();
}