#include "game.h"
#include "application.h"

using namespace spe;

Game::Game(Application& _app) :
    app{ _app }
{
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

    m = std::unique_ptr<Mesh>(new Mesh(
        {
            glm::vec3(0.5f,  0.5f, 0.0f),
            glm::vec3(0.5f, -0.5f, 0.0f),
            glm::vec3(-0.5f, -0.5f, 0.0f),
            glm::vec3(-0.5f,  0.5f, 0.0f),
        }
        ,
        {
            glm::vec2{1.0f, 1.0f},
            glm::vec2{1.0f, 0.0f},
            glm::vec2{0.0f, 0.0f},
            glm::vec2{0.0f, 1.0f},
        }
        ,
        {
            0, 1, 3,
            1, 2, 3,
        }
        ));

    auto p = Polygon
    (
        {
            glm::vec2(0.5f,  0.5f),
            glm::vec2(0.5f, -0.5f),
            glm::vec2(-0.5f, -0.5f),
            glm::vec2(-0.5f,  0.5f),
        }
    );

    SPDLOG_INFO("{} {} {} {}", p.GetMass(), p.GetInverseMass(), p.GetInertia(), p.GetInverseInertia());

    auto c = Circle(1);

    SPDLOG_INFO("{} {} {} {}", c.GetMass(), c.GetInverseMass(), c.GetInertia(), c.GetInverseInertia());

    auto b = Box(1, 1);

    SPDLOG_INFO("{} {} {} {}", b.GetMass(), b.GetInverseMass(), b.GetInertia(), b.GetInverseInertia());

    auto at = AABBTree();

    auto k = at.Add(c);
    at.Add(b);

    for (int i = 0; i < 10; i++)
    {
        at.Add(Box(1, 1));
    }

    SPDLOG_INFO("manual remove");
    at.Remove(k);

    SPDLOG_INFO("collision pairs {}", at.GetCollisionPairs().size());
    SPDLOG_INFO("Tree cost {}", at.GetTreeCost());

    SPDLOG_INFO("--------");
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

        static bool wireFrameDraw = false;
        ImGui::Checkbox("Wire frame mode", &wireFrameDraw);
        if (wireFrameDraw)
        {
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        }
        else
        {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        }
    }
    ImGui::End();
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
    glm::vec2 windowSize = Window::Get().GetWindowSize();
    windowSize /= zoom;

    s->SetProjectionMatrix(glm::ortho(-windowSize.x, windowSize.x, -windowSize.y, windowSize.y, 0.0f, 100.0f));
}

Game::~Game()
{

}