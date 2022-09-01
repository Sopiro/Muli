#include "game.h"
#include "application.h"
#include "demo.h"
#include "spe/detection.h"

namespace spe
{

Game::Game(Application& _app)
    : app{ _app }
{
    glm::vec2 windowSize = Window::Get().GetWindowSize();

    UpdateProjectionMatrix();
    Window::Get().SetFramebufferSizeChangeCallback([&](int width, int height) -> void {
        glViewport(0, 0, width, height);
        UpdateProjectionMatrix();
    });

    // simulationDeltaTime = 1.0f / Window::Get().GetRefreshRate();
    simulationDeltaTime = 1.0f / 144.0f;
    settings.VALID_REGION.min.y = -20.0f;

    world = new World(settings);

    demos = get_demos();
    InitSimulation(0);
}

Game::~Game() noexcept
{
    delete world;
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
            world->Step(simulationDeltaTime);
        }
    }
    else
    {
        world->Step(simulationDeltaTime);
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

        if (Input::IsKeyDown(GLFW_KEY_RIGHT) || Input::IsKeyPressed(GLFW_KEY_S))
        {
            step = true;
        }

        if (Input::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
        {
            auto q = world->Query(mpos);

            if (q.size() != 0)
            {
                gj = world->CreateGrabJoint(q[0], mpos, mpos, 2.0f, 0.5f, q[0]->GetMass());
                gj->OnDestroy = [&](Joint* me) -> void { gj = nullptr; };
            }
            else
            {
                RigidBody* b = world->CreateBox(0.5f);
                b->position = mpos;
                rRenderer.Register(b);

                b->OnDestroy = [&](RigidBody* me) -> void { rRenderer.Unregister(me); };
            }
        }

        if (gj != nullptr)
        {
            gj->SetTarget(mpos);
        }

        if (Input::IsMouseReleased(GLFW_MOUSE_BUTTON_LEFT))
        {
            if (gj != nullptr)
            {
                world->Destroy(gj);
                gj = nullptr;
            }
        }

        if (Input::IsMousePressed(GLFW_MOUSE_BUTTON_RIGHT))
        {
            std::vector<RigidBody*> q = world->Query(mpos);

            world->Destroy(q);
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
    ImGui::SetNextWindowPos({ 10, 10 }, ImGuiCond_Once, { 0.0f, 0.0f });
    ImGui::SetNextWindowSize({ 240, 360 }, ImGuiCond_Once);

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

                    ImGui::SameLine();

                    if (ImGui::Button("Restart")) InitSimulation(currentDemo);
                }

                static int f = Window::Get().GetRefreshRate();
                ImGui::SetNextItemWidth(120);
                if (ImGui::SliderInt("Frame rate", &f, 10, 300))
                {
                    app.SetFrameRate(f);
                }
                ImGui::Separator();

                ImGui::Text("%.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

                ImGui::Separator();

                // ImGui::ColorEdit4("Background color", glm::value_ptr(app.clearColor));
                // ImGui::Separator();

                ImGui::Checkbox("Camera reset", &resetCamera);
                ImGui::Checkbox("Draw outline only", &drawOutlineOnly);
                ImGui::Checkbox("Show BVH", &showBVH);
                ImGui::Checkbox("Show Contact point", &showCP);
                ImGui::Separator();
                if (ImGui::Checkbox("Apply gravity", &settings.APPLY_GRAVITY)) world->Awake();
                ImGui::SetNextItemWidth(120);
                static int iteration = settings.SOLVE_ITERATION;
                ImGui::SliderInt("Solve iteration", &iteration, 1, 50);
                settings.SOLVE_ITERATION = static_cast<uint32_t>(iteration);

                ImGui::Separator();
                ImGui::Text(demos[currentDemo].first.data());
                ImGui::Text("Bodies: %d", world->GetBodyCount());
                ImGui::Text("Sleeping dynamic bodies: %d", world->GetSleepingBodyCount());
                ImGui::Text("Broad phase contacts: %d", world->GetContactCount());
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Demos"))
            {
                if (ImGui::BeginListBox("##listbox 2", ImVec2(-FLT_MIN, 17 * ImGui::GetTextLineHeightWithSpacing())))
                {
                    for (uint32_t i = 0; i < demos.size(); i++)
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

    points.clear();
    lines.clear();

    for (Joint* j = world->GetJoints(); j; j = j->GetNext())
    {
        Joint::Type type = j->GetType();

        switch (type)
        {
        case Joint::Type::JointGrab:
        {
            RigidBody* b = j->GetBodyA();
            GrabJoint* gj = static_cast<GrabJoint*>(j);

            const glm::vec2& anchor = b->LocalToGlobal() * gj->GetLocalAnchor();
            points.push_back(anchor);
            points.push_back(gj->GetTarget());

            lines.push_back(anchor);
            lines.push_back(gj->GetTarget());
        }
        break;
        case Joint::Type::JointRevolute:
        {

            RigidBody* ba = j->GetBodyA();
            RigidBody* bb = j->GetBodyB();
            RevoluteJoint* rj = static_cast<RevoluteJoint*>(j);

            const glm::vec2& anchorA = ba->LocalToGlobal() * rj->GetLocalAnchorA();
            const glm::vec2& anchorB = bb->LocalToGlobal() * rj->GetLocalAnchorB();

            points.push_back(anchorA);
            points.push_back(anchorB);

            lines.push_back(anchorA);
            lines.push_back(ba->position);
            lines.push_back(anchorB);
            lines.push_back(bb->position);
        }
        break;
        case Joint::Type::JointDistance:
        {
            RigidBody* ba = j->GetBodyA();
            RigidBody* bb = j->GetBodyB();
            DistanceJoint* dj = static_cast<DistanceJoint*>(j);

            const glm::vec2& anchorA = ba->LocalToGlobal() * dj->GetLocalAnchorA();
            const glm::vec2& anchorB = bb->LocalToGlobal() * dj->GetLocalAnchorB();

            points.push_back(anchorA);
            points.push_back(anchorB);

            lines.push_back(anchorA);
            lines.push_back(anchorB);
        }
        break;
        default:
            break;
        }
    }

    if (showBVH)
    {
        const AABBTree& tree = world->GetBVH();
        tree.Traverse([&](const Node* n) -> void {
            lines.push_back(n->aabb.min);
            lines.push_back({ n->aabb.max.x, n->aabb.min.y });
            lines.push_back({ n->aabb.max.x, n->aabb.min.y });
            lines.push_back(n->aabb.max);
            lines.push_back(n->aabb.max);
            lines.push_back({ n->aabb.min.x, n->aabb.max.y });
            lines.push_back({ n->aabb.min.x, n->aabb.max.y });
            lines.push_back(n->aabb.min);
        });
    }

    if (showCP)
    {
        const Contact* c = world->GetContacts();

        while (c)
        {
            for (uint32_t j = 0; j < c->GetContactManifold().numContacts; j++)
            {
                points.push_back(c->GetContactManifold().contactPoints[j].point);
            }

            c = c->GetNext();
        }
    }

    dRenderer.SetViewMatrix(camera.CameraTransform());
    glPointSize(5.0f);
    dRenderer.Draw(points, GL_POINTS);
    glLineWidth(1.0f);
    dRenderer.Draw(lines, GL_LINES);
}

void Game::UpdateProjectionMatrix()
{
    glm::vec2 windowSize = Window::Get().GetWindowSize();
    windowSize /= 100.0f;

    glm::mat4 projMatrix =
        glm::ortho(-windowSize.x / 2.0f, windowSize.x / 2.0f, -windowSize.y / 2.0f, windowSize.y / 2.0f, 0.0f, 1.0f);
    rRenderer.SetProjectionMatrix(projMatrix);
    dRenderer.SetProjectionMatrix(projMatrix);
}

void Game::InitSimulation(uint32_t demo)
{
    time = 0;
    if (resetCamera)
    {
        camera.position = glm::vec2{ 0, 3.6 };
        camera.scale = glm::vec2{ 1, 1 };
    }

    Reset();

    currentDemo = demo;
    demoTitle = demos[currentDemo].first;
    demos[currentDemo].second(*this, *world, settings);

    for (RigidBody* b = world->GetBodyList(); b; b = b->GetNext())
    {
        rRenderer.Register(b);

        b->OnDestroy = [&](RigidBody* me) -> void { rRenderer.Unregister(me); };
    }
}

void Game::Reset()
{
    world->Reset();
    rRenderer.Reset();
}

} // namespace spe