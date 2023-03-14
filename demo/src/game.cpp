#include "game.h"
#include "application.h"

namespace muli
{

Game::Game(Application& _app)
    : app{ _app }
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

extern bool block_solve;

void Game::UpdateUI()
{
    // ImGui::ShowDemoWindow();

    // ImGui Windows
    ImGui::SetNextWindowPos({ 5, 5 }, ImGuiCond_Once, { 0.0f, 0.0f });
    ImGui::SetNextWindowSize({ 240, 535 }, ImGuiCond_Once);

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
                    ImGui::Checkbox("Draw outline", &options.draw_outline);
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
                        int32 velIterations = settings.velocity_iterations;
                        ImGui::SliderInt("Velocity", &velIterations, 0, 50);
                        settings.velocity_iterations = velIterations;

                        ImGui::SetNextItemWidth(120);
                        int32 posIterations = settings.position_iterations;
                        ImGui::SliderInt("Position", &posIterations, 0, 50);
                        settings.position_iterations = posIterations;
                    }
                    ImGui::Checkbox("Contact block solve", &block_solve);
                    ImGui::Checkbox("Warm starting", &settings.warm_starting);
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
    Mat4 cameraMatrix = camera.GetCameraMatrix();

    renderer.SetViewMatrix(cameraMatrix);

    renderer.SetPointSize(5.0f);
    renderer.SetLineWidth(1.0f);

    if (options.draw_body || options.draw_outline)
    {
    }

    World& world = demo->GetWorld();
    for (RigidBody* b = world.GetBodyList(); b; b = b->GetNext())
    {
        const Transform& tf = b->GetTransform();

        for (Collider* c = b->GetColliderList(); c; c = c->GetNext())
        {
            renderer.DrawShape(c->GetShape(), tf, b->GetIslandID() - 1);
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
            const GrabJoint* gj = static_cast<const GrabJoint*>(j);

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
            const RevoluteJoint* rj = static_cast<const RevoluteJoint*>(j);

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
            const DistanceJoint* dj = static_cast<const DistanceJoint*>(j);

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
            const LineJoint* lj = static_cast<const LineJoint*>(j);

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
            const PrismaticJoint* pj = static_cast<const PrismaticJoint*>(j);

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
            const PulleyJoint* pj = static_cast<const PulleyJoint*>(j);

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
            const MotorJoint* pj = static_cast<const MotorJoint*>(j);

            const Vec2& anchorA = Mul(ba->GetTransform(), pj->GetLocalAnchorA());
            const Vec2& anchorB = Mul(bb->GetTransform(), pj->GetLocalAnchorB());

            renderer.DrawPoint(anchorA);
            renderer.DrawPoint(anchorB);
        }
        break;
        default:
            muliAssert(false);
            break;
        }
    }

    if (options.show_bvh || options.show_aabb)
    {
        const AABBTree& tree = world.GetDynamicTree();
        tree.Traverse([&](const Node* n) -> void {
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
                const Vec2& cp = m.contactPoints[j].position;

                if (options.show_contact_point)
                {
                    renderer.DrawPoint(cp);
                }
                if (options.show_contact_normal)
                {
                    renderer.DrawLine(cp, cp + m.contactNormal * 0.15f);
                    renderer.DrawLine(cp + m.contactNormal * 0.15f, cp + m.contactNormal * 0.13f + m.contactTangent * 0.02f);
                    renderer.DrawLine(cp + m.contactNormal * 0.15f, cp + m.contactNormal * 0.13f - m.contactTangent * 0.02f);
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