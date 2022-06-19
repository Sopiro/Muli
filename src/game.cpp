#include "game.h"
#include "engine.h"

Game::Game(Engine& _engine) :
    engine{ _engine }
{
    s = MyShader::Create();
    s->Use();

    viewportSize = engine.GetWindowSize();
    glm::vec2 windowSize = viewportSize;
    windowSize /= 100.0f;

    s->SetProjectionMatrix(glm::ortho(-windowSize.x, windowSize.x, -windowSize.y, windowSize.y, 0.0f, 100.0f));
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

    // Update projection matrix
    glm::vec2 windowSize = engine.GetWindowSize();
    if (viewportSize != windowSize)
    {
        UpdateProjectionMatrix();
    }

    if (ImGui::Begin("Control Panel"))
    {
        // ImGui::Text("This is some useful text.");

        static int f = 60;
        if (ImGui::SliderInt("Frame rate", &f, 30, 300))
        {
            engine.SetFrameRate(f);
        }
        ImGui::Separator();

        ImGui::Text("%.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

        ImGui::Separator();

        ImGui::ColorEdit4("Background color", glm::value_ptr(engine.clearColor));

        ImGui::Separator();

        if (ImGui::SliderFloat("Zoom", &zoom, 10, 300))
        {
            UpdateProjectionMatrix();
        }
    }
    ImGui::End();

    Entity e;
    e.Translate({ 123, 456 });
    e.Rotate(1);
    e.Scale({ 2.0f, 2.0f });

    SPDLOG_INFO(glm::to_string(e.LocalToGlobal() * glm::vec2(1, 2))); // -> 120.714722, 459.844147
    SPDLOG_INFO(glm::to_string(e.GlobalToLocal() * e.LocalToGlobal() * glm::vec2(1, 2))); // -> 1, 2
    SPDLOG_INFO("-");

}

void Game::Render()
{
    s->Use();
    {
        glm::mat4 t = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 0.0f));
        glm::mat4 r = glm::rotate(glm::mat4{ 1.0f }, glm::radians(time * 90.0f), glm::vec3(0.0f, 0.0f, 1.0f));

        s->SetModelMatrix(t * r);
        s->SetColor({ glm::sin(time * 2) * 0.5f + 1.0f, glm::cos(time * 3) * 0.5f + 1.0f, glm::sin(time * 1.5) * 0.5f + 1.0f });

        m->Draw();
    }
}

void Game::UpdateProjectionMatrix()
{
    glm::vec2 windowSize = engine.GetWindowSize();
    viewportSize = windowSize;
    windowSize /= zoom;

    s->SetProjectionMatrix(glm::ortho(-windowSize.x, windowSize.x, -windowSize.y, windowSize.y, 0.0f, 100.0f));
}

Game::~Game()
{

}