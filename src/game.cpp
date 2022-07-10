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

    world = std::unique_ptr<World>(new World(settings));

    camera.position = glm::vec2{ 0, 3.6 };
    camera.scale = glm::vec2{ 1, 1 };

    RigidBody* b = new Box{ 12.8f * 5.0f, 0.4f, Static };
    AddBody(b);
    // RigidBody* b = new Circle(1, Static);
    // AddBody(b);

    for (size_t i = 0; i < 10; i++)
    {
        auto c = new Circle(0.5);
        c->position.y = 3 + i + 0.1f * i;

        AddBody(c);
    }
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

    if (pause)
    {
        if (step)
        {
            step = false;
            world->Update(dt);
        }
    }
    else
    {
        world->Update(dt);
    }
}

void Game::HandleInput()
{
    if (!ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow))
    {
        mpos = rRenderer.Pick(Input::GetMousePosition());

        if (Input::IsKeyPressed(GLFW_KEY_C))
        {
            for (RigidBody* body : bodies)
            {
                delete body;
            }
            bodies.clear();
            world->Reset();
            rRenderer.Clear();

            RigidBody* b = new Box{ 12.8f * 5.0f, 0.4f, Static };
            AddBody(b);
        }

        if (Input::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
        {
            RigidBody* b = new Box(0.5f, 0.5f);
            b->position = mpos;

            AddBody(b);
        }

        if (Input::IsMousePressed(GLFW_MOUSE_BUTTON_RIGHT))
        {
            auto q = world->QueryPoint(mpos);

            world->Unregister(q);
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
            camera.scale *= Input::GetMouseScroll().y < 0 ? 1.1 : 1.0f / 1.1f;
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
    ImGui::SetNextWindowSize({ 400, 260 }, ImGuiCond_Once);

    if (ImGui::Begin("Control Panel"))
    {
        // Simulation buttons
        {
            ImGui::BeginDisabled(pause);
            if (ImGui::Button("Pause")) pause = true;
            ImGui::EndDisabled();

            ImGui::SameLine();

            ImGui::BeginDisabled(!pause);
            ImGui::PushButtonRepeat(true);
            if (ImGui::Button("Step")) step = true;
            ImGui::PopButtonRepeat();
            ImGui::EndDisabled();

            ImGui::SameLine();

            ImGui::BeginDisabled(!pause);
            if (ImGui::Button("Start")) pause = false;
            ImGui::EndDisabled();
        }

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
        ImGui::Checkbox("Show Contact point", &showCP);
        ImGui::SliderInt("Solve iteration", &settings.SOLVE_ITERATION, 1, 50);

        ImGui::Separator();
        ImGui::Text("Bodies: %d", bodies.size());
        ImGui::Text("Sleeping dynamic bodies: %d", world->GetSleepingBodyCount());
    }
    ImGui::End();
}

void Game::Render()
{
    rRenderer.SetViewMatrix(camera.CameraTransform());
    rRenderer.SetDrawOutlined(drawOutlineOnly);
    rRenderer.Render();

    dRenderer.SetViewMatrix(camera.CameraTransform());
    if (showBVH)
    {
        const AABBTree& tree = world->GetBVH();
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
        dRenderer.Draw(v, GL_LINES);
    }

    if (showCP)
    {
        auto& cc = world->GetContactConstraints();
        std::vector<glm::vec2> v{};
        v.reserve(cc.size());

        for (size_t i = 0; i < cc.size(); i++)
        {
            auto ci = cc[i]->GetContactInfo();
            for (size_t j = 0; j < ci.numContacts; j++)
            {
                v.push_back(ci.contactPoints[j].point);
            }
        }
        glPointSize(5.0f);
        dRenderer.Draw(v, GL_POINTS);
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

void Game::AddBody(RigidBody* b)
{
    bodies.insert(b);
    world->Register(b);
    rRenderer.Register(b);
}
