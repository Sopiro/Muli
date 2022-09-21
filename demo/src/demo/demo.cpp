#include "demo.h"
#include "game.h"

namespace muli
{

void Demo::UpdateInput(Game& game)
{
    DebugOptions& options = game.GetDebugOptions();

    if (Input::IsKeyPressed(GLFW_KEY_V)) options.showBVH = !options.showBVH;
    if (Input::IsKeyPressed(GLFW_KEY_B)) options.showAABB = !options.showAABB;
    if (Input::IsKeyPressed(GLFW_KEY_P)) options.showContactPoint = !options.showContactPoint;
    if (Input::IsKeyPressed(GLFW_KEY_N)) options.showContactNormal = !options.showContactNormal;
    if (Input::IsKeyPressed(GLFW_KEY_C)) options.resetCamera = !options.resetCamera;
    if (Input::IsKeyPressed(GLFW_KEY_SPACE)) options.pause = !options.pause;
    if (Input::IsKeyDown(GLFW_KEY_RIGHT) || Input::IsKeyPressed(GLFW_KEY_S)) options.step = true;
    if (Input::IsKeyPressed(GLFW_KEY_G))
    {
        settings.APPLY_GRAVITY = !settings.APPLY_GRAVITY;
        world->Awake();
    }

    RigidBodyRenderer& rRenderer = game.GetRigidBodyRenderer();
    Vec2 mpos = rRenderer.Pick(Input::GetMousePosition());
    std::vector<RigidBody*> qr = world->Query(mpos);

    target = qr.size() > 0 ? qr[0] : nullptr;

    if (!ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow))
    {
        if (Input::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
        {
            if (target)
            {
                if (target->GetType() != RigidBody::Type::Static)
                {
                    gj = world->CreateGrabJoint(target, mpos, mpos, 4.0f, 0.5f, target->GetMass());
                    gj->OnDestroy = [&](Joint* me) -> void { gj = nullptr; };
                }
            }
            else
            {
                RigidBody* b = world->CreateBox(0.5f);
                b->SetPosition(mpos);

                b->OnDestroy = [&](RigidBody* me) -> void { rRenderer.Unregister(me); };
                rRenderer.Register(b);
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
            if (gj != nullptr)
            {
                gj = nullptr; // Stick to the air!
            }
            else
            {
                std::vector<RigidBody*> q = world->Query(mpos);

                world->Destroy(q);
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

void Demo::Step(Game& game)
{
    DebugOptions& options = game.GetDebugOptions();

    if (options.pause)
    {
        if (options.step)
        {
            options.step = false;
            world->Step(simulationDeltaTime);
        }
    }
    else
    {
        world->Step(simulationDeltaTime);
    }
}

uint32 demo_count = 0;
DemoFrame demos[MAX_DEMOS];

extern DemoFrame single_box;
extern DemoFrame box_stacking;
extern DemoFrame pyramid;
extern DemoFrame single_pendulum;
extern DemoFrame springs;
extern DemoFrame random_convex_polygons;
extern DemoFrame seesaw;
extern DemoFrame frictions;
extern DemoFrame restitutions;
extern DemoFrame multi_pendulum;
extern DemoFrame suspension_bridge;
extern DemoFrame circle_stacking;
extern DemoFrame circles_1000;
extern DemoFrame boxes_1000;
extern DemoFrame capsules_1000;
extern DemoFrame convex_polygons_1000;
extern DemoFrame mix_1000;
extern DemoFrame dense_collision;

static int32 init_demos()
{
    demos[demo_count++] = single_box;
    demos[demo_count++] = box_stacking;
    demos[demo_count++] = pyramid;
    demos[demo_count++] = single_pendulum;
    demos[demo_count++] = springs;
    demos[demo_count++] = random_convex_polygons;
    demos[demo_count++] = seesaw;
    demos[demo_count++] = frictions;
    demos[demo_count++] = restitutions;
    demos[demo_count++] = multi_pendulum;
    demos[demo_count++] = suspension_bridge;
    demos[demo_count++] = circle_stacking;
    demos[demo_count++] = circles_1000;
    demos[demo_count++] = boxes_1000;
    demos[demo_count++] = capsules_1000;
    demos[demo_count++] = convex_polygons_1000;
    demos[demo_count++] = mix_1000;
    demos[demo_count++] = dense_collision;

    return demo_count;
}

static int32 _ = init_demos();

} // namespace muli
