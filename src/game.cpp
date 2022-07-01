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

    RigidBody* b = new Box{ .5f, .5f };

    bodies.insert(b);

    world.Register(b);
    rRenderer.Register(b);
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
    if (!ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow))
    {
        mpos = rRenderer.Pick(Input::GetMousePosition());

        if (Input::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
        {
            RigidBody* b = new Box(1, 1);
            b->position = mpos;

            bodies.insert(b);
            world.Register(b);
            rRenderer.Register(b);
        }

        if (Input::IsMousePressed(GLFW_MOUSE_BUTTON_RIGHT))
        {
            auto q = world.QueryPoint(mpos);

            world.Unregister(q);
            rRenderer.Unregister(q);

            for (auto b : q)
            {
                auto it = std::find(bodies.begin(), bodies.end(), b);
                bodies.erase(it);

                delete b;
            }
        }

        if (Input::GetMouseScroll().y != 0)
        {
            camera.scale += -Input::GetMouseScroll().y * 0.1f;

            camera.scale = glm::clamp(camera.scale, glm::vec2{ 0.1f }, glm::vec2{ FLT_MAX });
        }

        // Camera moving
        {
            static bool cameraMove = false;
            static glm::vec2 cursorStart;
            static glm::vec2 cameraPosStart;

            if (!cameraMove && Input::IsMousePressed(GLFW_MOUSE_BUTTON_RIGHT))
            {
                cameraMove = true;
                cursorStart = Input::GetMousePosition();
                cameraPosStart = camera.position;
            }
            else if (Input::IsMouseReleased(GLFW_MOUSE_BUTTON_RIGHT))
            {
                cameraMove = false;
            }

            if (cameraMove)
            {
                glm::vec2 dist = Input::GetMousePosition() - cursorStart;
                dist.x *= 0.01f * -camera.scale.x;
                dist.y *= 0.01f * camera.scale.y;
                camera.position = cameraPosStart + dist;
            }
        }
    }

    // ImGui Window
    ImGui::SetNextWindowPos({ 10, 10 }, ImGuiCond_Once);
    ImGui::SetNextWindowSize({ 400, 250 }, ImGuiCond_Once);

    if (ImGui::Begin("Control Panel"))
    {
        static int f = 144;
        if (ImGui::SliderInt("Frame rate", &f, 10, 300))
        {
            app.SetFrameRate(f);
        }
        ImGui::Separator();

        ImGui::Text("%.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

        ImGui::Separator();

        ImGui::ColorEdit4("Background color", glm::value_ptr(app.clearColor));

        ImGui::Separator();

        ImGui::Checkbox("Draw outline only", &drawOutlineOnly);
        ImGui::Checkbox("Show BVH", &showBVH);

        ImGui::Separator();
        ImGui::Text("Body count: %d", bodies.size());
    }
    ImGui::End();
}

void Game::Render()
{
    rRenderer.SetViewMatrix(camera.CameraTransform());
    rRenderer.SetDrawOutlined(drawOutlineOnly);
    rRenderer.Render();

    if (showBVH)
    {
        const AABBTree& tree = world.GetBVH();
        std::vector<glm::vec2> v{};
        tree.Traverse([&](const Node* n)->void
            {
                v.push_back(n->aabb.min);
                v.push_back({ n->aabb.max.x, n->aabb.min.y });
                v.push_back({ n->aabb.max.x, n->aabb.min.y });
                v.push_back(n->aabb.max);
                v.push_back(n->aabb.max);
                v.push_back({ n->aabb.min.x, n->aabb.max.y });
                v.push_back({ n->aabb.min.x, n->aabb.max.y });
                v.push_back(n->aabb.min);
            });

        glLineWidth(1.0f);
        dRenderer.SetViewMatrix(camera.CameraTransform());
        dRenderer.Draw(v, GL_LINES);
    }
}

void Game::UpdateProjectionMatrix()
{
    glm::vec2 windowSize = Window::Get().GetWindowSize();
    windowSize /= 100.0f;

    glm::mat4 projMatrix = glm::ortho(-windowSize.x / 2.0f, windowSize.x / 2.0f, -windowSize.y / 2.0f, windowSize.y / 2.0f, 0.0f, 1.0f);
    rRenderer.SetProjectionMatrix(projMatrix);
    dRenderer.SetProjectionMatrix(projMatrix);
}