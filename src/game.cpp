#include "game.h"

Game::Game()
{
    const char* vertexShaderSource = "#version 330 core\n"
        "layout (location = 0) in vec3 aPos;\n"
        "void main()\n"
        "{\n"
        "   gl_Position = vec4(aPos, 1.0);\n"
        "}\0";
    const char* fragmentShaderSource = "#version 330 core\n"
        "out vec4 fragColor;\n"

        "uniform vec3 color;\n"

        "void main()\n"
        "{\n"
        "   fragColor = vec4(color, 1.0f);\n"
        "}\n\0";

    t = Shader::create(vertexShaderSource, fragmentShaderSource);

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

void Game::update(float dt)
{
    time += dt;
}
void Game::render()
{
    t->use();
    t->setColor({ glm::sin(time * 2) * 0.5f + 1.0f, glm::cos(time * 3) * 0.5f + 1.0f, glm::sin(time * 1.5) * 0.5f + 1.0f });

    p->draw();
}