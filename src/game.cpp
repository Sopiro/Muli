#include "game.h"
#include "engine.h"

Game::Game(Engine& _engine) :
    engine{ _engine }
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

    static int f = 60;
    static int counter = 0;
    static glm::vec4 color{ 0.0f, 0.0f, 0.0f, 1.0f };

    if (ImGui::Begin("Control panel"))
    {
        ImGui::Text("This is some useful text.");               // Display some text (you can use a format strings too)

        ImGui::SliderInt("Frame rate", &f, 30, 300);
        engine.SetFrameRate(f);

        ImGui::ColorEdit4("Color edit", &color.r);
        engine.SetClearColor(color);

        if (ImGui::Button("Button"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
            counter++;
        ImGui::SameLine();
        ImGui::Text("counter = %d", counter);

        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    }
    ImGui::End();
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