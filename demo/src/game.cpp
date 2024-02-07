#include "game.h"
#include "application.h"

namespace muli
{

Game::Game(Application& _app)
    : app{ _app }
    , demoCount{ demos.size() }
    , demoIndex{ demos.size() }
{
    UpdateProjectionMatrix();
    Window::Get().SetFramebufferSizeChangeCallback([&](int32 width, int32 height) -> void {
        glViewport(0, 0, width, height);
        UpdateProjectionMatrix();
    });

    Srand(uint32(std::time(nullptr)));

    sort_demos(demos);

    InitDemo(0);
    // InitDemo(demoCount - 1);
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

extern bool block_solve;

void Game::UpdateUI()
{
    // ImGui::ShowDemoWindow();

    // ImGui Windows
    ImGui::SetNextWindowPos({ 5, 5 }, ImGuiCond_Once, { 0.0f, 0.0f });
    ImGui::SetNextWindowSize({ 240, 535 }, ImGuiCond_Once);

    static bool collapsed = false;
    if (Input::IsKeyPressed(GLFW_KEY_GRAVE_ACCENT)) collapsed = !collapsed;
    ImGui::SetNextWindowCollapsed(collapsed, ImGuiCond_None);

    if (ImGui::Begin("Muli Engine", NULL))
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
                    ImGui::Checkbox("Draw body", &options.draw_body);
                    ImGui::Checkbox("Draw outlined", &options.draw_outlined);
                    ImGui::Checkbox("Show BVH", &options.show_bvh);
                    ImGui::Checkbox("Show AABB", &options.show_aabb);
                    ImGui::Checkbox("Show contact point", &options.show_contact_point);
                    ImGui::Checkbox("Show contact normal", &options.show_contact_normal);
                }

                World& world = demo->GetWorld();
                WorldSettings& settings = demo->GetWorldSettings();

                ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                if (ImGui::CollapsingHeader("Simulation settings"))
                {
                    if (ImGui::Checkbox("Apply gravity", &settings.apply_gravity))
                    {
                        world.Awake();
                    }

                    ImGui::Text("Constraint solve iterations");
                    {
                        ImGui::SetNextItemWidth(120);
                        ImGui::SliderInt("Velocity", &settings.step.velocity_iterations, 0, 50);

                        ImGui::SetNextItemWidth(120);
                        ImGui::SliderInt("Position", &settings.step.position_iterations, 0, 50);
                    }
                    ImGui::Checkbox("Contact block solve", &block_solve);
                    ImGui::Checkbox("Warm starting", &settings.step.warm_starting);
                    ImGui::Checkbox("Sleeping", &settings.sleeping);
                    ImGui::Checkbox("Continuous", &settings.continuous);
                    ImGui::Checkbox("Sub-stepping", &settings.sub_stepping);
                }

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
                    for (int32 i = 0; i < demoCount; ++i)
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

    if (!collapsed && ImGui::IsWindowCollapsed()) collapsed = true;
    if (collapsed && !ImGui::IsWindowCollapsed()) collapsed = false;

    ImGui::End();

    demo->UpdateUI();
}

void Game::Render()
{
    World& world = demo->GetWorld();
    Camera& camera = demo->GetCamera();
    Mat4 cameraMatrix = camera.GetCameraMatrix();

    renderer.SetViewMatrix(cameraMatrix);
    renderer.SetPointSize(4.0f);
    renderer.SetLineWidth(1.0f);

    // Draw bodies
    if (options.draw_body)
    {
        Renderer::DrawMode drawMode;

        if (options.draw_outlined)
        {
            drawMode.fill = false;

            for (RigidBody* b = world.GetBodyList(); b; b = b->GetNext())
            {
                const Transform& tf = b->GetTransform();
                drawMode.rounded = UserFlag::IsEnabled(b, UserFlag::render_polygon_radius);

                for (Collider* c = b->GetColliderList(); c; c = c->GetNext())
                {
                    renderer.DrawShape(c->GetShape(), tf, drawMode);
                }
            }
        }
        else
        {
            for (RigidBody* b = world.GetBodyList(); b; b = b->GetNext())
            {
                const Transform& tf = b->GetTransform();
                drawMode.rounded = UserFlag::IsEnabled(b, UserFlag::render_polygon_radius);

                if (b->IsSleeping())
                {
                    drawMode.fill = false;
                    drawMode.outline = true;

                    for (Collider* c = b->GetColliderList(); c; c = c->GetNext())
                    {
                        renderer.DrawShape(c->GetShape(), tf, drawMode);
                    }
                }
                else
                {
                    drawMode.colorIndex = options.colorize_island ? (b->GetIslandID() - 1) : (b->GetType() - 2);
                    drawMode.fill = true;
                    drawMode.outline = !UserFlag::IsEnabled(b, UserFlag::remove_outline);

                    for (Collider* c = b->GetColliderList(); c; c = c->GetNext())
                    {
                        renderer.DrawShape(c->GetShape(), tf, drawMode);
                    }
                }
            }
        }
    }

    // Draw joints
    for (Joint* j = world.GetJoints(); j; j = j->GetNext())
    {
        Joint::Type type = j->GetType();

        switch (type)
        {
        case Joint::Type::grab_joint:
        {
            RigidBody* b = j->GetBodyA();
            const GrabJoint* gj = (const GrabJoint*)j;

            const Vec2& anchor = Mul(b->GetTransform(), gj->GetLocalAnchor());
            renderer.DrawPoint(anchor);
            renderer.DrawPoint(gj->GetTarget());

            renderer.DrawLine(anchor, gj->GetTarget());
        }
        break;
        case Joint::Type::revolute_joint:
        {
            RigidBody* ba = j->GetBodyA();
            RigidBody* bb = j->GetBodyB();
            const RevoluteJoint* rj = (const RevoluteJoint*)j;

            const Vec2& anchorA = Mul(ba->GetTransform(), rj->GetLocalAnchorA());
            const Vec2& anchorB = Mul(bb->GetTransform(), rj->GetLocalAnchorB());

            renderer.DrawPoint(anchorA);
            renderer.DrawPoint(anchorB);

            renderer.DrawLine(anchorA, ba->GetPosition());
            renderer.DrawLine(anchorB, bb->GetPosition());
        }
        break;
        case Joint::Type::distance_joint:
        {
            RigidBody* ba = j->GetBodyA();
            RigidBody* bb = j->GetBodyB();
            const DistanceJoint* dj = (const DistanceJoint*)j;

            const Vec2& anchorA = Mul(ba->GetTransform(), dj->GetLocalAnchorA());
            const Vec2& anchorB = Mul(bb->GetTransform(), dj->GetLocalAnchorB());

            renderer.DrawPoint(anchorA);
            renderer.DrawPoint(anchorB);

            renderer.DrawLine(anchorA, anchorB);
        }
        break;
        case Joint::Type::line_joint:
        {
            RigidBody* ba = j->GetBodyA();
            RigidBody* bb = j->GetBodyB();
            const LineJoint* lj = (const LineJoint*)j;

            const Vec2& anchorA = Mul(ba->GetTransform(), lj->GetLocalAnchorA());
            const Vec2& anchorB = Mul(bb->GetTransform(), lj->GetLocalAnchorB());

            renderer.DrawPoint(anchorA);
            renderer.DrawPoint(anchorB);

            renderer.DrawLine(anchorA, anchorB);
        }
        case Joint::Type::prismatic_joint:
        {
            RigidBody* ba = j->GetBodyA();
            RigidBody* bb = j->GetBodyB();
            const PrismaticJoint* pj = (const PrismaticJoint*)j;

            const Vec2& anchorA = Mul(ba->GetTransform(), pj->GetLocalAnchorA());
            const Vec2& anchorB = Mul(bb->GetTransform(), pj->GetLocalAnchorB());

            renderer.DrawPoint(anchorA);
            renderer.DrawPoint(anchorB);

            renderer.DrawLine(anchorA, anchorB);
        }
        break;
        case Joint::Type::pulley_joint:
        {
            RigidBody* ba = j->GetBodyA();
            RigidBody* bb = j->GetBodyB();
            const PulleyJoint* pj = (const PulleyJoint*)j;

            const Vec2& anchorA = Mul(ba->GetTransform(), pj->GetLocalAnchorA());
            const Vec2& anchorB = Mul(bb->GetTransform(), pj->GetLocalAnchorB());
            const Vec2& groundAnchorA = pj->GetGroundAnchorA();
            const Vec2& groundAnchorB = pj->GetGroundAnchorB();

            renderer.DrawPoint(anchorA);
            renderer.DrawPoint(groundAnchorA);
            renderer.DrawPoint(anchorB);
            renderer.DrawPoint(groundAnchorB);

            renderer.DrawLine(anchorA, groundAnchorA);
            renderer.DrawLine(anchorB, groundAnchorB);
            renderer.DrawLine(groundAnchorA, groundAnchorB);
        }
        break;
        case Joint::Type::motor_joint:
        {
            RigidBody* ba = j->GetBodyA();
            RigidBody* bb = j->GetBodyB();
            const MotorJoint* pj = (const MotorJoint*)j;

            const Vec2& anchorA = Mul(ba->GetTransform(), pj->GetLocalAnchorA());
            const Vec2& anchorB = Mul(bb->GetTransform(), pj->GetLocalAnchorB());

            renderer.DrawPoint(anchorA);
            renderer.DrawPoint(anchorB);
        }
        break;
        default:
            break;
        }
    }

    if (options.show_bvh || options.show_aabb)
    {
        const AABBTree& tree = world.GetDynamicTree();
        tree.Traverse([&](const AABBTree::Node* n) -> void {
            if (options.show_bvh == false && n->IsLeaf() == false)
            {
                return;
            }

            Vec2 br{ n->aabb.max.x, n->aabb.min.y };
            Vec2 tl{ n->aabb.min.x, n->aabb.max.y };
            renderer.DrawLine(n->aabb.min, br);
            renderer.DrawLine(br, n->aabb.max);
            renderer.DrawLine(n->aabb.max, tl);
            renderer.DrawLine(tl, n->aabb.min);
        });
    }

    if (options.show_contact_point || options.show_contact_normal)
    {
        const Contact* c = world.GetContacts();

        while (c)
        {
            if (c->IsTouching() == false)
            {
                c = c->GetNext();
                continue;
            }

            const ContactManifold& m = c->GetContactManifold();

            for (int32 j = 0; j < m.contactCount; ++j)
            {
                Vec2 p1 = m.contactPoints[j].p;

                if (options.show_contact_point)
                {
                    renderer.DrawPoint(p1);
                }
                if (options.show_contact_normal)
                {
                    Vec2 p2 = p1 + m.contactNormal * 0.15f;
                    Vec2 t0 = p2 - m.contactNormal * 0.035f;
                    Vec2 t1 = t0 + m.contactTangent * 0.02f;
                    Vec2 t2 = t0 - m.contactTangent * 0.02f;

                    renderer.DrawLine(p1, p2);
                    renderer.DrawLine(p2, t1);
                    renderer.DrawLine(p2, t2);
                }
            }

            c = c->GetNext();
        }
    }

    demo->Render();

    renderer.FlushAll();
}

void Game::UpdateProjectionMatrix()
{
    Vec2 windowSize = Window::Get().GetWindowSize();
    windowSize /= 100.0f;

    Mat4 projMatrix = Orth(-windowSize.x / 2.0f, windowSize.x / 2.0f, -windowSize.y / 2.0f, windowSize.y / 2.0f, 0.0f, 1.0f);
    renderer.SetProjectionMatrix(projMatrix);
}

void Game::InitDemo(int32 index)
{
    if (index >= demos.size())
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
}

} // namespace muli