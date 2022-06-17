#include "game.h"

Game::Game()
{
    s = MyShader::Create();
    s->Use();
    s->SetProjectionMatrix(glm::ortho(-12.8f, 12.8f, -7.2f, 7.2f, 0.0f, 100.0f));
    s->SetViewMatrix(glm::translate(glm::mat4{ 1.0 }, glm::vec3(0, 0, -1)));

    m = std::unique_ptr<Mesh>(new Mesh(
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

void Game::Update(float dt)
{
    time += dt;

    {
        static float f = 0.0f;
        static float f2 = 0.0f;
        static int counter = 0;

        ImGui::Begin("Hello, world!");                          // Create a window called "Hello, world!" and append into it.

        ImGui::Text("This is some useful text.");               // Display some text (you can use a format strings too)

        // ImGui::SliderFloat("float", &f, 0.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
        ImGui::DragFloat("float", &f2, 1.0f, 0.0f, 100.0f);

        static glm::vec3 color{ 0.0f };

        ImGui::ColorEdit3("Color edit", &color.r);

        if (ImGui::Button("Button"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
            counter++;
        ImGui::SameLine();
        ImGui::Text("counter = %d", counter);

        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGui::End();
    }
}

void Game::Render()
{
    s->Use();
    {
        glm::mat4 t = glm::translate(glm::mat4(1.0f), glm::vec3(time, 0.0f, 0.0f));
        glm::mat4 r = glm::rotate(glm::mat4{ 1.0f }, glm::radians(time * 90.0f), glm::vec3(0.0f, 0.0f, 1.0f));

        s->SetModelMatrix(t * r);
        s->SetColor({ glm::sin(time * 2) * 0.5f + 1.0f, glm::cos(time * 3) * 0.5f + 1.0f, glm::sin(time * 1.5) * 0.5f + 1.0f });

        m->Draw();
    }
}

Game::~Game()
{

}