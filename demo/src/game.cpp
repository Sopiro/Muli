#include "game.h"
#include "application.h"

namespace muli
{

Game::Game(Application& _app)
    : app{ _app }
    , rRenderer{ *this }
{
    UpdateProjectionMatrix();
    Window::Get().SetFramebufferSizeChangeCallback([&](int32 width, int32 height) -> void {
        glViewport(0, 0, width, height);
        UpdateProjectionMatrix();
    });

    InitDemo(demo_count - 1);
    // InitDemo(0);
    // InitDemo(42); // Logo
}

Game::~Game() noexcept
{
    delete demo;
}

void Game::Update(float dt)
{
    if (restart)
    {
        InitDemo(newIndex);
        restart = false;
    }

    time += dt;

    UpdateInput();
    demo->Step();
    UpdateUI();
}

void Game::UpdateInput()
{
    if (Input::IsKeyPressed(GLFW_KEY_R)) RestartDemo();
    if (Input::IsKeyPressed(GLFW_KEY_PAGE_DOWN)) PrevDemo();
    if (Input::IsKeyPressed(GLFW_KEY_PAGE_UP)) NextDemo();

    demo->UpdateInput();
}

void Game::UpdateUI()
{
    // ImGui::ShowDemoWindow();

    // ImGui Windows
    ImGui::SetNextWindowPos({ 5, 5 }, ImGuiCond_Once, { 0.0f, 0.0f });
    ImGui::SetNextWindowSize({ 240, 535 }, ImGuiCond_Once);

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

                static int32 f = Window::Get().GetRefreshRate();
                ImGui::SetNextItemWidth(150);
                if (ImGui::SliderInt("Frame rate", &f, 30, 300))
                {
                    app.SetFrameRate(f);
                }
                ImGui::Separator();

                ImGui::Text("%.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

                ImGui::Separator();

                // ImGui::ColorEdit4("Background color", &app.clearColor.x);
                // ImGui::Separator();
                ImGui::SetNextItemOpen(false, ImGuiCond_Once);
                if (ImGui::CollapsingHeader("Debug options"))
                {
                    ImGui::Checkbox("Camera reset", &options.reset_camera);
                    ImGui::Checkbox("Colorize island", &options.colorize_island);
                    ImGui::Checkbox("Draw outline only", &options.draw_outline_only);
                    ImGui::Checkbox("Show BVH", &options.show_bvh);
                    ImGui::Checkbox("Show AABB", &options.show_aabb);
                    ImGui::Checkbox("Show contact point", &options.show_contact_point);
                    ImGui::Checkbox("Show contact normal", &options.show_contact_normal);
                }

                ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                if (ImGui::CollapsingHeader("Simulation settings"))
                {
                    WorldSettings& settings = demo->GetWorldSettings();

                    if (ImGui::Checkbox("Apply gravity", &settings.apply_gravity)) demo->GetWorld().Awake();
                    ImGui::Text("Constraint solve iterations");
                    {
                        ImGui::SetNextItemWidth(120);
                        int32 velIterations = settings.velocity_iterations;
                        ImGui::SliderInt("Velocity", &velIterations, 0, 50);
                        settings.velocity_iterations = velIterations;

                        ImGui::SetNextItemWidth(120);
                        int32 posIterations = settings.position_iterations;
                        ImGui::SliderInt("Position", &posIterations, 0, 50);
                        settings.position_iterations = posIterations;
                    }
                    ImGui::Checkbox("Contact block solve", &settings.block_solve);
                    ImGui::Checkbox("Warm starting", &settings.warm_starting);
                    ImGui::Checkbox("Sleeping", &settings.sleeping);
                    ImGui::Checkbox("Continuous", &settings.continuous);
                    ImGui::Checkbox("Sub-stepping", &settings.sub_stepping);
                }

                World& world = demo->GetWorld();

                ImGui::Separator();
                ImGui::Text("%s", demos[demoIndex].name);
                ImGui::Text("Bodies: %d", world.GetBodyCount());
                ImGui::Text("Sleeping dynamic bodies: %d", world.GetSleepingBodyCount());
                // ImGui::Text("Awake island count: %d", world.GetIslandCount());
                ImGui::Text("Broad phase contacts: %d", world.GetContactCount());

                ImGui::Separator();

                RigidBody* t = demo->GetTargetBody();

                if (t)
                {
                    ImGui::Text("Continuous: %s", t->IsContinuous() ? "true" : "false");
                    ImGui::Text("Mass: %.4f", t->GetMass());
                    ImGui::Text("Inertia: %.4f", t->GetInertia());
                    ImGui::Text("Pos: %.4f, %.4f", t->GetPosition().x, t->GetPosition().y);
                    ImGui::Text("Rot: %.4f", t->GetAngle());
                }

                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Demos"))
            {
                if (ImGui::BeginListBox("##listbox 2", ImVec2{ -FLT_MIN, 28 * ImGui::GetTextLineHeightWithSpacing() }))
                {
                    for (int32 i = 0; i < demo_count; ++i)
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

    demo->UpdateUI();
}

void Game::Render()
{
    Camera& camera = demo->GetCamera();
    rRenderer.SetViewMatrix(camera.GetCameraMatrix());
    rRenderer.Render();

    for (Joint* j = demo->GetWorld().GetJoints(); j; j = j->GetNext())
    {
        Joint::Type type = j->GetType();

        switch (type)
        {
        case Joint::Type::grab_joint:
        {
            RigidBody* b = j->GetBodyA();
            GrabJoint* gj = static_cast<GrabJoint*>(j);

            const Vec2& anchor = b->GetTransform() * gj->GetLocalAnchor();
            points.emplace_back(anchor);
            points.emplace_back(gj->GetTarget());

            lines.emplace_back(anchor);
            lines.emplace_back(gj->GetTarget());
        }
        break;
        case Joint::Type::revolute_joint:
        {

            RigidBody* ba = j->GetBodyA();
            RigidBody* bb = j->GetBodyB();
            RevoluteJoint* rj = static_cast<RevoluteJoint*>(j);

            const Vec2& anchorA = ba->GetTransform() * rj->GetLocalAnchorA();
            const Vec2& anchorB = bb->GetTransform() * rj->GetLocalAnchorB();

            points.emplace_back(anchorA);
            points.emplace_back(anchorB);

            lines.emplace_back(anchorA);
            lines.emplace_back(ba->GetPosition());
            lines.emplace_back(anchorB);
            lines.emplace_back(bb->GetPosition());
        }
        break;
        case Joint::Type::distance_joint:
        {
            RigidBody* ba = j->GetBodyA();
            RigidBody* bb = j->GetBodyB();
            DistanceJoint* dj = static_cast<DistanceJoint*>(j);

            const Vec2& anchorA = ba->GetTransform() * dj->GetLocalAnchorA();
            const Vec2& anchorB = bb->GetTransform() * dj->GetLocalAnchorB();

            points.emplace_back(anchorA);
            points.emplace_back(anchorB);

            lines.emplace_back(anchorA);
            lines.emplace_back(anchorB);
        }
        break;
        case Joint::Type::line_joint:
        {
            RigidBody* ba = j->GetBodyA();
            RigidBody* bb = j->GetBodyB();
            LineJoint* lj = static_cast<LineJoint*>(j);

            const Vec2& anchorA = ba->GetTransform() * lj->GetLocalAnchorA();
            const Vec2& anchorB = bb->GetTransform() * lj->GetLocalAnchorB();

            points.emplace_back(anchorA);
            points.emplace_back(anchorB);

            lines.emplace_back(anchorA);
            lines.emplace_back(anchorB);
        }
        case Joint::Type::prismatic_joint:
        {
            RigidBody* ba = j->GetBodyA();
            RigidBody* bb = j->GetBodyB();
            PrismaticJoint* pj = static_cast<PrismaticJoint*>(j);

            const Vec2& anchorA = ba->GetTransform() * pj->GetLocalAnchorA();
            const Vec2& anchorB = bb->GetTransform() * pj->GetLocalAnchorB();

            points.emplace_back(anchorA);
            points.emplace_back(anchorB);

            lines.emplace_back(anchorA);
            lines.emplace_back(anchorB);
        }
        break;
        case Joint::Type::pulley_joint:
        {
            RigidBody* ba = j->GetBodyA();
            RigidBody* bb = j->GetBodyB();
            PulleyJoint* pj = static_cast<PulleyJoint*>(j);

            const Vec2& anchorA = ba->GetTransform() * pj->GetLocalAnchorA();
            const Vec2& anchorB = bb->GetTransform() * pj->GetLocalAnchorB();
            const Vec2& groundAnchorA = pj->GetGroundAnchorA();
            const Vec2& groundAnchorB = pj->GetGroundAnchorB();

            points.emplace_back(anchorA);
            points.emplace_back(groundAnchorA);
            points.emplace_back(anchorB);
            points.emplace_back(groundAnchorB);

            lines.emplace_back(anchorA);
            lines.emplace_back(groundAnchorA);
            lines.emplace_back(anchorB);
            lines.emplace_back(groundAnchorB);
            lines.emplace_back(groundAnchorA);
            lines.emplace_back(groundAnchorB);
        }
        break;
        case Joint::Type::motor_joint:
        {
            RigidBody* ba = j->GetBodyA();
            RigidBody* bb = j->GetBodyB();
            MotorJoint* pj = static_cast<MotorJoint*>(j);

            const Vec2& anchorA = ba->GetTransform() * pj->GetLocalAnchorA();
            const Vec2& anchorB = bb->GetTransform() * pj->GetLocalAnchorB();

            points.emplace_back(anchorA);
            points.emplace_back(anchorB);
        }
        break;
        default:
            break;
        }
    }

    if (options.show_bvh || options.show_aabb)
    {
        const AABBTree& tree = demo->GetWorld().GetDynamicTree();
        tree.Traverse([&](const Node* n) -> void {
            if (options.show_bvh == false && n->IsLeaf() == false)
            {
                return;
            }

            lines.emplace_back(n->aabb.min);
            lines.emplace_back(n->aabb.max.x, n->aabb.min.y);
            lines.emplace_back(n->aabb.max.x, n->aabb.min.y);
            lines.emplace_back(n->aabb.max);
            lines.emplace_back(n->aabb.max);
            lines.emplace_back(n->aabb.min.x, n->aabb.max.y);
            lines.emplace_back(n->aabb.min.x, n->aabb.max.y);
            lines.emplace_back(n->aabb.min);
        });
    }

    if (options.show_contact_point || options.show_contact_normal)
    {
        const Contact* c = demo->GetWorld().GetContacts();

        while (c)
        {
            if (c->IsTouching() == false)
            {
                c = c->GetNext();
                continue;
            }

            const ContactManifold& m = c->GetContactManifold();

            for (int32 j = 0; j < m.numContacts; ++j)
            {
                const Vec2& cp = m.contactPoints[j].position;

                if (options.show_contact_point)
                {
                    points.emplace_back(cp);
                }
                if (options.show_contact_normal)
                {
                    lines.emplace_back(cp);
                    lines.emplace_back(cp + m.contactNormal * 0.15f);

                    lines.emplace_back(cp + m.contactNormal * 0.15f);
                    lines.emplace_back(cp + m.contactNormal * 0.13f + m.contactTangent * 0.02f);

                    lines.emplace_back(cp + m.contactNormal * 0.15f);
                    lines.emplace_back(cp + m.contactNormal * 0.13f - m.contactTangent * 0.02f);
                }
            }

            c = c->GetNext();
        }
    }

    dRenderer.SetViewMatrix(camera.GetCameraMatrix());

    demo->Render();

    glPointSize(5.0f);
    dRenderer.Draw(points, GL_POINTS);
    glLineWidth(1.0f);
    dRenderer.Draw(lines, GL_LINES);

    points.clear();
    lines.clear();
}

void Game::UpdateProjectionMatrix()
{
    Vec2 windowSize = Window::Get().GetWindowSize();
    windowSize /= 100.0f;

    Mat4 projMatrix = Orth(-windowSize.x / 2.0f, windowSize.x / 2.0f, -windowSize.y / 2.0f, windowSize.y / 2.0f, 0.0f, 1.0f);
    rRenderer.SetProjectionMatrix(projMatrix);
    dRenderer.SetProjectionMatrix(projMatrix);
}

void Game::InitDemo(int32 index)
{
    if (index >= demo_count)
    {
        return;
    }

    bool restoreCameraPosition = false;
    bool restoreSettings = (demoIndex == index);
    Camera prevCamera;
    WorldSettings prevSettings;

    if (restoreSettings)
    {
        prevSettings = demo->GetWorldSettings();
    }

    if (demo)
    {
        restoreCameraPosition = !options.reset_camera;
        prevCamera = demo->GetCamera();
        delete demo;
    }

    time = 0;
    rRenderer.Reset();

    demoIndex = index;
    demo = demos[demoIndex].createFunction(*this);

    if (restoreSettings)
    {
        demo->GetWorldSettings() = prevSettings;
    }

    if (restoreCameraPosition)
    {
        demo->GetCamera() = prevCamera;
    }

    for (RigidBody* b = demo->GetWorld().GetBodyList(); b; b = b->GetNext())
    {
        RegisterRenderBody(b);
    }
}

} // namespace muli