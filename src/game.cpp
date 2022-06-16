#include "game.h"

Game::Game()
{
    const char* vertexShaderSource =
        "#version 330 core\n"
        "layout (location = 0) in vec3 aPos;\n"

        "uniform mat4 view;\n"
        "uniform mat4 proj;\n"
        "uniform mat4 model;\n"

        "void main()\n"
        "{\n"
        "   mat4 mvp = proj * view * model;"
        "   gl_Position = mvp * vec4(aPos, 1.0);\n"
        "}\0";

    const char* fragmentShaderSource =
        "#version 330 core\n"
        "out vec4 fragColor;\n"

        "uniform vec3 color;\n"

        "void main()\n"
        "{\n"
        "   fragColor = vec4(color, 1.0f);\n"
        "}\n\0";

    s = Shader::create(vertexShaderSource, fragmentShaderSource);
    s->use();
    s->setProjectionMatrix(glm::ortho(-12.8f, 12.8f, -7.2f, 7.2f, 0.0f, 100.0f));
    s->setViewMatrix(glm::translate(glm::mat4{ 1.0 }, glm::vec3(0, 0, -1)));

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
    s->use();

    glm::mat4 t = glm::translate(glm::mat4(1.0f), glm::vec3(time, 0.0f, 0.0f));
    glm::mat4 r = glm::rotate(glm::mat4{ 1.0f }, glm::radians(time * 90.0f), glm::vec3(0.0f, 0.0f, 1.0f));

    s->setModelMatrix(t * r);
    s->setColor({ glm::sin(time * 2) * 0.5f + 1.0f, glm::cos(time * 3) * 0.5f + 1.0f, glm::sin(time * 1.5) * 0.5f + 1.0f });

    p->draw();
}