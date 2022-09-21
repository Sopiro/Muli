#include "game.h"
#include "application.h"

namespace muli
{

Game::Game(Application& _app)
    : app{ _app }
{
    UpdateProjectionMatrix();
    Window::Get().SetFramebufferSizeChangeCallback([&](int width, int height) -> void {
        glViewport(0, 0, width, height);
        UpdateProjectionMatrix();
    });

    InitDemo(0);
}

Game::~Game() noexcept
{
    delete demo;
}

void Game::Update(float dt)
{
    time += dt;

    mpos = rRenderer.Pick(Input::GetMousePosition());

    HandleInput();
    UpdateUI();

    if (pause)
    {
        if (step)
        {
            step = false;
            demo->Step();
        }
    }
    else
    {
        demo->Step();
    }
}

void Game::HandleInput()
{
    if (Input::IsKeyPressed(GLFW_KEY_R)) InitDemo(demoIndex);
    if (Input::IsKeyPressed(GLFW_KEY_V)) showBVH = !showBVH;
    if (Input::IsKeyPressed(GLFW_KEY_B)) showAABB = !showAABB;
    if (Input::IsKeyPressed(GLFW_KEY_P)) showContactPoint = !showContactPoint;
    if (Input::IsKeyPressed(GLFW_KEY_N)) showContactNormal = !showContactNormal;
    if (Input::IsKeyPressed(GLFW_KEY_C)) resetCamera = !resetCamera;
    if (Input::IsKeyPressed(GLFW_KEY_SPACE)) pause = !pause;
    if (Input::IsKeyDown(GLFW_KEY_RIGHT) || Input::IsKeyPressed(GLFW_KEY_S)) step = true;
    if (Input::IsKeyPressed(GLFW_KEY_G))
    {
        demo->GetWorldSettings().APPLY_GRAVITY = !demo->GetWorldSettings().APPLY_GRAVITY;
        demo->GetWorld().Awake();
    }

    if (!ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow))
    {
        qr = demo->GetWorld().Query(mpos);

        if (Input::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
        {
            if (qr.size() != 0)
            {
                RigidBody* target = qr[0];
                if (target->GetType() != RigidBody::Type::Static)
                {
                    gj = demo->GetWorld().CreateGrabJoint(target, mpos, mpos, 4.0f, 0.5f, target->GetMass());
                    gj->OnDestroy = [&](Joint* me) -> void { gj = nullptr; };
                }
            }
            else
            {
                RigidBody* b = demo->GetWorld().CreateBox(0.5f);
                b->SetPosition(mpos);
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
                demo->GetWorld().Destroy(gj);
                gj = nullptr;
            }
        }

        if (Input::IsMousePressed(GLFW_MOUSE_BUTTON_RIGHT))
        {
            if (gj != nullptr)
            {
                gj = nullptr; // Stick to the air!
            }
            else
            {
                std::vector<RigidBody*> q = demo->GetWorld().Query(mpos);

                demo->GetWorld().Destroy(q);
            }
        }

        if (Input::GetMouseScroll().y != 0)
        {
            camera.scale *= Input::GetMouseScroll().y < 0 ? 1.1f : 1.0f / 1.1f;
            camera.scale = Clamp(camera.scale, Vec2{ 0.1f }, Vec2{ FLT_MAX });
        }

        // Camera moving
        {
            static bool cameraMove = false;
            static Vec2 cursorStart;
            static Vec2 cameraPosStart;

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
                Vec2 dist = Input::GetMousePosition() - cursorStart;
                dist.x *= 0.01f * -camera.scale.x;
                dist.y *= 0.01f * camera.scale.y;
                camera.position = cameraPosStart + dist;
            }
        }
    }
}

void Game::UpdateUI()
{
    // ImGui::ShowDemoWindow();

    // ImGui Windows
    ImGui::SetNextWindowPos({ 10, 10 }, ImGuiCond_Once, { 0.0f, 0.0f });
    ImGui::SetNextWindowSize({ 240, 500 }, ImGuiCond_Once);

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

                    if (ImGui::Button("Restart")) InitDemo(demoIndex);
                }

                static int f = Window::Get().GetRefreshRate();
                ImGui::SetNextItemWidth(120);
                if (ImGui::SliderInt("Frame rate", &f, 30, 300))
                {
                    app.SetFrameRate(f);
                }
                ImGui::Separator();

                ImGui::Text("%.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

                ImGui::Separator();

                // ImGui::ColorEdit4("Background color", &app.clearColor.x);
                // ImGui::Separator();
                ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                if (ImGui::CollapsingHeader("Debug options"))
                {
                    ImGui::Checkbox("Camera reset", &resetCamera);
                    ImGui::Checkbox("Draw outline only", &drawOutlineOnly);
                    ImGui::Checkbox("Show BVH", &showBVH);
                    ImGui::Checkbox("Show AABB", &showAABB);
                    ImGui::Checkbox("Show contact point", &showContactPoint);
                    ImGui::Checkbox("Show contact normal", &showContactNormal);
                }

                if (ImGui::CollapsingHeader("Simulation settings"))
                {
                    WorldSettings& settings = demo->GetWorldSettings();

                    if (ImGui::Checkbox("Apply gravity", &settings.APPLY_GRAVITY)) demo->GetWorld().Awake();
                    ImGui::Text("Constraint solve iterations");
                    {
                        ImGui::SetNextItemWidth(120);
                        static int velIterations = settings.VELOCITY_SOLVE_ITERATIONS;
                        ImGui::SliderInt("Velocity", &velIterations, 0, 50);
                        settings.VELOCITY_SOLVE_ITERATIONS = static_cast<uint32>(velIterations);

                        ImGui::SetNextItemWidth(120);
                        static int posIterations = settings.POSITION_SOLVE_ITERATIONS;
                        ImGui::SliderInt("Position", &posIterations, 0, 50);
                        settings.POSITION_SOLVE_ITERATIONS = static_cast<uint32>(posIterations);
                    }
                    ImGui::Checkbox("Contact block solve", &settings.BLOCK_SOLVE);
                    ImGui::Checkbox("Warm starting", &settings.WARM_STARTING);
                    ImGui::Checkbox("Sleeping", &settings.SLEEPING);
                }

                ImGui::Separator();
                ImGui::Text(demos[demoIndex].name);
                ImGui::Text("Bodies: %d", demo->GetWorld().GetBodyCount());
                ImGui::Text("Sleeping dynamic bodies: %d", demo->GetWorld().GetSleepingBodyCount());
                ImGui::Text("Broad phase contacts: %d", demo->GetWorld().GetContactCount());

                ImGui::Separator();
                if (qr.size() > 0)
                {
                    RigidBody* t = qr[0];
                    ImGui::Text("ID: %d", t->GetID());
                    ImGui::Text("Mass: %.4f", t->GetMass());
                    ImGui::Text("Inertia: %.4f", t->GetInertia());
                    ImGui::Text("Pos: %.4f, %.4f", t->GetPosition().x, t->GetPosition().y);
                    ImGui::Text("Rot: %.4f", t->GetRotation().angle);
                }

                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Demos"))
            {
                if (ImGui::BeginListBox("##listbox 2", ImVec2{ -FLT_MIN, 26 * ImGui::GetTextLineHeightWithSpacing() }))
                {
                    for (uint32 i = 0; i < demo_count; i++)
                    {
                        const bool selected = (demoIndex == i);

                        if (ImGui::Selectable(demos[i].name, selected))
                        {
                            InitDemo(i);
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
    }
    ImGui::End();
}

void Game::Render()
{
    rRenderer.SetViewMatrix(camera.GetCameraMatrix());
    rRenderer.SetDrawOutlined(drawOutlineOnly);
    rRenderer.Render();

    points.clear();
    lines.clear();

    for (Joint* j = demo->GetWorld().GetJoints(); j; j = j->GetNext())
    {
        Joint::Type type = j->GetType();

        switch (type)
        {
        case Joint::Type::JointGrab:
        {
            RigidBody* b = j->GetBodyA();
            GrabJoint* gj = static_cast<GrabJoint*>(j);

            const Vec2& anchor = b->GetTransform() * gj->GetLocalAnchor();
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

            const Vec2& anchorA = ba->GetTransform() * rj->GetLocalAnchorA();
            const Vec2& anchorB = bb->GetTransform() * rj->GetLocalAnchorB();

            points.push_back(anchorA);
            points.push_back(anchorB);

            lines.push_back(anchorA);
            lines.push_back(ba->GetPosition());
            lines.push_back(anchorB);
            lines.push_back(bb->GetPosition());
        }
        break;
        case Joint::Type::JointDistance:
        {
            RigidBody* ba = j->GetBodyA();
            RigidBody* bb = j->GetBodyB();
            DistanceJoint* dj = static_cast<DistanceJoint*>(j);

            const Vec2& anchorA = ba->GetTransform() * dj->GetLocalAnchorA();
            const Vec2& anchorB = bb->GetTransform() * dj->GetLocalAnchorB();

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

    if (showBVH || showAABB)
    {
        const AABBTree& tree = demo->GetWorld().GetBVH();
        tree.Traverse([&](const Node* n) -> void {
            if (!showBVH && !n->isLeaf) return;
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

    if (showContactPoint || showContactNormal)
    {
        const Contact* c = demo->GetWorld().GetContacts();

        while (c)
        {
            const ContactManifold& m = c->GetContactManifold();

            for (uint32 j = 0; j < m.numContacts; j++)
            {
                const Vec2& cp = m.contactPoints[j].position;

                if (showContactPoint)
                {
                    points.push_back(cp);
                }
                if (showContactNormal)
                {
                    lines.push_back(cp);
                    lines.push_back(cp + m.contactNormal * 0.15f);

                    lines.push_back(cp + m.contactNormal * 0.15f);
                    lines.push_back(cp + m.contactNormal * 0.13f + m.contactTangent * 0.02f);

                    lines.push_back(cp + m.contactNormal * 0.15f);
                    lines.push_back(cp + m.contactNormal * 0.13f - m.contactTangent * 0.02f);
                }
            }

            c = c->GetNext();
        }
    }

    dRenderer.SetViewMatrix(camera.GetCameraMatrix());
    glPointSize(5.0f);
    dRenderer.Draw(points, GL_POINTS);
    glLineWidth(1.0f);
    dRenderer.Draw(lines, GL_LINES);

    demo->Render(dRenderer);
}

void Game::UpdateProjectionMatrix()
{
    Vec2 windowSize = Window::Get().GetWindowSize();
    windowSize /= 100.0f;

    Mat4 projMatrix = Orth(-windowSize.x / 2.0f, windowSize.x / 2.0f, -windowSize.y / 2.0f, windowSize.y / 2.0f, 0.0f, 1.0f);
    rRenderer.SetProjectionMatrix(projMatrix);
    dRenderer.SetProjectionMatrix(projMatrix);
}

void Game::InitDemo(uint32 index)
{
    if (index >= demo_count) return;
    if (demo) delete demo;

    Reset();

    demoIndex = index;
    demo = demos[demoIndex].createFunction();

    for (RigidBody* b = demo->GetWorld().GetBodyList(); b; b = b->GetNext())
    {
        rRenderer.Register(b);

        b->OnDestroy = [&](RigidBody* me) -> void { rRenderer.Unregister(me); };
    }
}

void Game::Reset()
{
    time = 0;
    rRenderer.Reset();

    if (resetCamera)
    {
        camera.position = Vec2{ 0.0f, 3.6f };
        camera.scale = Vec2{ 1, 1 };
    }
}

} // namespace muli