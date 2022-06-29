#include "game.h"
#include "application.h"
#include "physics/detection.h"

using namespace spe;

Game::Game(Application& _app) :
    app{ _app }
{
    // Set up shader parameters
    s = MyShader::Create();
    s->Use();

    UpdateProjectionMatrix();
    s->SetViewMatrix(glm::translate(glm::mat4{ 1.0 }, glm::vec3(0, 0, -1)));

    Window::Get().SetFramebufferSizeChangeCallback
    (
        [&](int width, int height) -> void
        {
            glViewport(0, 0, width, height);
            UpdateProjectionMatrix();
        }
    );

    // meshes.push_back(generate_mesh_from_rigidbody(create_regular_polygon(2, 7)));
    meshes.push_back(generate_mesh_from_rigidbody(create_random_convex_body(2)));

    Box b1{ 1, 1 };
    Box b2{ 1, 1 };
    b1.Rotate(0.1f);
    b1.Translate({ 0.4, 0.4 });

    auto res = detect_collision(b1, b2);

    SPDLOG_INFO("------------------");
    if (res)
    {
        SPDLOG_INFO(glm::to_string(res->contactNormal));
        SPDLOG_INFO(res->penetrationDepth);
        for (size_t i = 0; i < res->contantPoints.size(); i++)
        {
            SPDLOG_INFO("contact point {}: {}", i, glm::to_string(res->contantPoints[i].point));
        }
        SPDLOG_INFO(res->featureFlipped);
    }
}

void Game::Update(float dt)
{
    time += dt;

    if (Input::GetMouseScroll().y != 0)
    {
        zoom += Input::GetMouseScroll().y * 10;
        zoom = glm::clamp<float>(zoom, 10, 500);
        UpdateProjectionMatrix();
    }

    ImGui::SetNextWindowPos({ 10, 10 }, ImGuiCond_Once);
    ImGui::SetNextWindowSize({ 400, 200 }, ImGuiCond_Once);

    if (ImGui::Begin("Control Panel"))
    {
        // ImGui::Text("This is some useful text.");

        static int f = 60;
        if (ImGui::SliderInt("Frame rate", &f, 30, 300))
        {
            app.SetFrameRate(f);
        }
        ImGui::Separator();

        ImGui::Text("%.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

        ImGui::Separator();

        ImGui::ColorEdit4("Background color", glm::value_ptr(app.clearColor));

        ImGui::Separator();

        if (ImGui::SliderFloat("Zoom", &zoom, 10, 500))
        {
            UpdateProjectionMatrix();
        }

        ImGui::Separator();

        ImGui::Checkbox("Draw outline only", &drawOutlineOnly);
    }
    ImGui::End();
}

void Game::Render()
{
    s->Use();
    {
        glm::mat4 t = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 0.0f));
        glm::mat4 r = glm::rotate(glm::mat4{ 1.0f }, glm::radians(0.0f * 90.0f), glm::vec3(0.0f, 0.0f, 1.0f));

        s->SetModelMatrix(t * r);

        for (size_t i = 0; i < meshes.size(); i++)
        {
            if (!drawOutlineOnly)
            {
                s->SetColor({ 1, 1, 1 });
                meshes[i].Draw(GL_TRIANGLES);
            }
            glLineWidth(1.5f);
            s->SetColor({ 0, 0, 0 });
            meshes[i].Draw(GL_LINE_LOOP);
        }
    }
}

void Game::UpdateProjectionMatrix()
{
    glm::vec2 windowSize = Window::Get().GetWindowSize();
    windowSize /= zoom;

    s->SetProjectionMatrix(glm::ortho(-windowSize.x, windowSize.x, -windowSize.y, windowSize.y, 0.0f, 100.0f));
}