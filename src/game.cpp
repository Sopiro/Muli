#include "game.h"
#include "application.h"
#include "physics/detection.h"

using namespace spe;

Game::Game(Application& _app) :
    app{ _app }
{
    glm::vec2 windowSize = Window::Get().GetWindowSize();

    UpdateProjectionMatrix();
    Window::Get().SetFramebufferSizeChangeCallback
    (
        [&](int width, int height) -> void
        {
            glViewport(0, 0, width, height);
            UpdateProjectionMatrix();
        }
    );

    camera.position = glm::vec2{ 0, 0 };
    camera.scale = glm::vec2{ 1, 1 };
    renderer.SetViewMatrix(camera.CameraTransform());

    bodies.push_back(new Box{ .5f, .5f });

    world.Register(bodies);
    renderer.Register(bodies);
}

Game::~Game() noexcept
{
    for (RigidBody* body : bodies)
    {
        delete body;
    }
}

void Game::Update(float dt)
{
    time += dt;

    HandleInput();
    world.Update(1.0f / dt);
}

void Game::HandleInput()
{
    glm::vec2 mpos = renderer.Pick();

    if (Input::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
    {
        RigidBody* b = create_random_convex_body(1.0f);
        b->position = mpos;

        bodies.push_back(b);

        world.Register(b);
        renderer.Register(b);
    }

    if (Input::GetMouseScroll().y != 0)
    {
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

        // ImGui::Separator();
        // if (ImGui::SliderFloat("Zoom", &zoom, 10, 500))
        // {
        //     UpdateProjectionMatrix();
        // }

        ImGui::Separator();

        ImGui::Checkbox("Draw outline only", &drawOutlineOnly);
    }
    ImGui::End();
}

void Game::Render()
{
    renderer.Render();
}

void Game::UpdateProjectionMatrix()
{
    glm::vec2 windowSize = Window::Get().GetWindowSize();
    windowSize /= 100.0f;

    renderer.SetProjectionMatrix(glm::ortho(-windowSize.x / 2.0f, windowSize.x / 2.0f, -windowSize.y / 2.0f, windowSize.y / 2.0f, 0.0f, 1.0f));
}