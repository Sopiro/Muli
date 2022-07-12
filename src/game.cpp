#include "game.h"
#include "application.h"
#include "physics/detection.h"
#include "demo.h"

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

    settings.INV_DT = static_cast<float>(Window::Get().GetRefreshRate());
    settings.DT = 1.0f / settings.INV_DT;

    world = std::unique_ptr<World>(new World(settings));

    // Init demos
    {
        demos = get_demos();
    }

    InitSimulation(0);
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

        if (Input::IsKeyPressed(GLFW_KEY_R))
        {
            InitSimulation(currentDemo);
        }

        if (Input::IsKeyPressed(GLFW_KEY_SPACE))
        {
            pause = !pause;
        }

        if (Input::IsKeyDown(GLFW_KEY_RIGHT))
        {
            step = true;
        }

        if (Input::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
        {
            RigidBody* b = new Box(0.5f, 0.5f);
            b->position = mpos;

            AddBody(b);
        }

        if (Input::IsMousePressed(GLFW_MOUSE_BUTTON_RIGHT))
        {
            std::vector<RigidBody*> q = world->QueryPoint(mpos);

            RemoveBody(q);
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

    // ImGui::ShowDemoWindow();

    // ImGui Windows
    ImGui::SetNextWindowPos({ 10, 10 }, ImGuiCond_Once);
    ImGui::SetNextWindowSize({ 400, 320 }, ImGuiCond_Once);

    if (ImGui::Begin("Control Panel"))
    {
        ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_AutoSelectNewTabs;
        if (ImGui::BeginTabBar("TabBar", tab_bar_flags))
        {
            if (ImGui::BeginTabItem("Control"))
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

                static int f = Window::Get().GetRefreshRate();
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
                ImGui::Text(demos[currentDemo].first.data());
                ImGui::Text("Bodies: %d", bodies.size());
                ImGui::Text("Sleeping dynamic bodies: %d", world->GetSleepingBodyCount());
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Demos"))
            {
                if (ImGui::BeginListBox("##listbox 2", ImVec2(-FLT_MIN, 15 * ImGui::GetTextLineHeightWithSpacing())))
                {
                    for (size_t i = 0; i < demos.size(); i++)
                    {
                        const bool selected = (currentDemo == i);

                        if (ImGui::Selectable(demos[i].first.data(), selected))
                        {
                            InitSimulation(i);
                        }

                        if (selected)
                        {
                            ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndListBox();
                }
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
        ImGui::End();
    }

    // ImGuiWindowFlags window_flags = 0;
    // // window_flags |= ImGuiWindowFlags_NoBackground;
    // window_flags |= ImGuiWindowFlags_NoTitleBar;
    // window_flags |= ImGuiWindowFlags_NoResize;
    // // etc.
    // bool open_ptr = true;
    // if (ImGui::Begin("", &open_ptr, window_flags))
    // {
    //     ImGui::Text("Bodies: %d", bodies.size());

    //     ImGui::End();
    // }
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

void Game::AddBody(std::vector<RigidBody*> bodies)
{
    for (size_t i = 0; i < bodies.size(); i++)
        AddBody(bodies[i]);
}

void Game::AddBody(RigidBody* body)
{
    bodies.push_back(body);
    world->Register(body);
    rRenderer.Register(body);
}

void Game::RemoveBody(std::vector<RigidBody*> bodies)
{
    for (size_t i = 0; i < bodies.size(); i++)
        RemoveBody(bodies[i]);
}

void Game::RemoveBody(RigidBody* body)
{
    world->Unregister(body);
    rRenderer.Unregister(body);

    auto it = std::find(bodies.begin(), bodies.end(), body);
    bodies.erase(it);

    delete body;
}

void Game::InitSimulation(size_t demo)
{
    time = 0;
    camera.position = glm::vec2{ 0, 3.6 };
    camera.scale = glm::vec2{ 1, 1 };

    Reset();

    currentDemo = demo;
    demoTitle = demos[currentDemo].first;
    demos[currentDemo].second(*this, settings);
}

void Game::Reset()
{
    for (RigidBody* body : bodies) delete body;
    bodies.clear();
    world->Reset();
    rRenderer.Reset();
}