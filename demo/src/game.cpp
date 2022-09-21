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

    UpdateInput();
    demo->Step(*this);
    UpdateUI();
}

void Game::UpdateInput()
{
    if (Input::IsKeyPressed(GLFW_KEY_R))
    {
        InitDemo(demoIndex);
    }

    demo->UpdateInput(*this);
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
                    ImGui::BeginDisabled(options.pause);
                    if (ImGui::Button("Pause")) options.pause = true;
                    ImGui::EndDisabled();

                    ImGui::SameLine();

                    ImGui::BeginDisabled(!options.pause);
                    ImGui::PushButtonRepeat(true);
                    if (ImGui::Button("Step")) options.step = true;
                    ImGui::PopButtonRepeat();
                    ImGui::EndDisabled();

                    ImGui::SameLine();

                    ImGui::BeginDisabled(!options.pause);
                    if (ImGui::Button("Start")) options.pause = false;
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
                    ImGui::Checkbox("Camera reset", &options.resetCamera);
                    ImGui::Checkbox("Draw outline only", &options.drawOutlineOnly);
                    ImGui::Checkbox("Show BVH", &options.showBVH);
                    ImGui::Checkbox("Show AABB", &options.showAABB);
                    ImGui::Checkbox("Show contact point", &options.showContactPoint);
                    ImGui::Checkbox("Show contact normal", &options.showContactNormal);
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

                RigidBody* t = demo->GetTarget();

                if (t)
                {
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
    Camera& camera = demo->GetCamera();
    rRenderer.SetViewMatrix(camera.GetCameraMatrix());
    rRenderer.SetDrawOutlined(options.drawOutlineOnly);
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

    if (options.showBVH || options.showAABB)
    {
        const AABBTree& tree = demo->GetWorld().GetBVH();
        tree.Traverse([&](const Node* n) -> void {
            if (!options.showBVH && !n->isLeaf) return;
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

    if (options.showContactPoint || options.showContactNormal)
    {
        const Contact* c = demo->GetWorld().GetContacts();

        while (c)
        {
            const ContactManifold& m = c->GetContactManifold();

            for (uint32 j = 0; j < m.numContacts; j++)
            {
                const Vec2& cp = m.contactPoints[j].position;

                if (options.showContactPoint)
                {
                    points.push_back(cp);
                }
                if (options.showContactNormal)
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

    bool restoreCameraPosition = false;
    Camera prevCamera;

    if (demo)
    {
        restoreCameraPosition = !options.resetCamera;
        prevCamera = demo->GetCamera();
        delete demo;
    }

    time = 0;
    rRenderer.Reset();

    demoIndex = index;
    demo = demos[demoIndex].createFunction();

    if (restoreCameraPosition)
    {
        demo->GetCamera() = prevCamera;
    }

    for (RigidBody* b = demo->GetWorld().GetBodyList(); b; b = b->GetNext())
    {
        rRenderer.Register(b);

        b->OnDestroy = [&](RigidBody* me) -> void { rRenderer.Unregister(me); };
    }
}

} // namespace muli