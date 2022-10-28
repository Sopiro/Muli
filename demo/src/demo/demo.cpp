#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli
{

Demo::Demo(Game& _game)
    : game{ _game }
    , options{ game.GetDebugOptions() }
{
    dt = 1.0f / Window::Get().GetRefreshRate();
    // dt = 1.0f / 60.0f;
    settings.VALID_REGION.min.y = -20.0f;

    world = new World(settings);

    camera.scale.Set(1.0f, 1.0f);
    camera.rotation = 0.0f;
    camera.position.Set(0.0f, 3.6f);
}

void Demo::UpdateInput()
{
    ComputeProperty();
    EnableKeyboardShortcut();

    if (!ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow))
    {
        EnableCameraControl();

        if (!EnablePolygonCreate())
        {
            EnableBodyCreate();
            EnableBodyRemove();

            if (!EnableAddForce())
            {
                EnableBodyGrab();
            }
        }
    }
}

void Demo::ComputeProperty()
{
    mpos = game.GetWorldMousePosition();      // Moust position
    qr = world->Query(mpos);                  // Query result
    target = qr.size() > 0 ? qr[0] : nullptr; // Mouseovered body
}

void Demo::EnableBodyCreate()
{
    static bool create;
    static Vec2 mStart;

    if (!target && Input::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
    {
        if (Input::IsKeyDown(GLFW_KEY_LEFT_SHIFT))
        {
            mStart = mpos;
            create = true;
        }
        else
        {
            RigidBody* b = world->CreateBox(0.5f);
            b->SetPosition(mpos);
            game.RegisterRenderBody(b);
        }
    }

    if (target && !gj && Input::IsMousePressed(GLFW_MOUSE_BUTTON_RIGHT))
    {
        world->Destroy(qr);
    }

    if (create)
    {
        auto& pl = game.GetPointList();
        auto& ll = game.GetLineList();

        pl.push_back(mStart);
        pl.push_back(mpos);
        ll.push_back(mStart);
        ll.push_back(mpos);

        if (Input::IsMouseReleased(GLFW_MOUSE_BUTTON_LEFT))
        {
            RigidBody* b = world->CreateBox(0.3f);
            b->SetPosition(mStart);

            Vec2 f = mStart - mpos;
            f *= settings.INV_DT * b->GetMass() * 3.0f;
            b->SetForce(f);
            create = false;
            game.RegisterRenderBody(b);
        }
    }
}

bool Demo::EnablePolygonCreate()
{
    static bool creating = false;
    static bool staticBody;
    static std::vector<Vec2> points;
    static std::vector<Vec2> hull;

    if (Input::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
    {
        if (Input::IsKeyDown(GLFW_KEY_LEFT_CONTROL))
        {
            creating = true;
            staticBody = false;
        }
        else if (Input::IsKeyDown(GLFW_KEY_LEFT_ALT))
        {
            creating = true;
            staticBody = true;
        }
    }

    if (creating)
    {
        std::vector<Vec2>& pl = game.GetPointList();
        std::vector<Vec2>& ll = game.GetLineList();

        pl = points;

        if (Input::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
        {
            points.push_back(mpos);
            hull = ComputeConvexHull(points);
        }

        for (uint32 i = 0; i < hull.size(); i++)
        {
            Vec2& v0 = hull[i];
            Vec2& v1 = hull[(i + 1) % hull.size()];
            ll.push_back(v0);
            ll.push_back(v1);
        }

        auto create_body = [&](RigidBody::Type type) {
            RigidBody* b;

            switch (hull.size())
            {
            case 1:
                b = world->CreateCircle(0.1f, type);
                b->SetPosition(hull[0]);
                break;
            case 2:
                b = world->CreateCapsule(hull[0], hull[1], 0.05f, type);
                break;
            default:
                b = world->CreatePolygon(hull, type, false);
                break;
            }

            game.RegisterRenderBody(b);
            creating = false;
            points.clear();
            hull.clear();
        };

        if (!staticBody && Input::IsKeyReleased(GLFW_KEY_LEFT_CONTROL))
        {
            create_body(RigidBody::Type::Dynamic);
        }
        else if (staticBody && Input::IsKeyReleased(GLFW_KEY_LEFT_ALT))
        {
            create_body(RigidBody::Type::Static);
        }
    }

    return creating;
}

void Demo::EnableBodyRemove()
{
    static bool draging = false;
    static Vec2 mStart;

    if (!draging && Input::IsMousePressed(GLFW_MOUSE_BUTTON_MIDDLE))
    {
        mStart = mpos;
        draging = true;
    }

    if (draging)
    {
        auto& ll = game.GetLineList();

        AABB aabb{ Min(mStart, mpos), Max(mStart, mpos) };

        ll.push_back(aabb.min);
        ll.push_back(Vec2{ aabb.min.x, aabb.max.y });
        ll.push_back(Vec2{ aabb.min.x, aabb.max.y });
        ll.push_back(aabb.max);
        ll.push_back(aabb.max);
        ll.push_back(Vec2{ aabb.max.x, aabb.min.y });
        ll.push_back(Vec2{ aabb.max.x, aabb.min.y });
        ll.push_back(aabb.min);

        if (Input::IsMouseReleased(GLFW_MOUSE_BUTTON_MIDDLE))
        {
            auto qr = (aabb.max == aabb.min) ? world->Query(aabb.min) : world->Query(aabb);
            world->Destroy(qr);

            draging = false;
        }
    }
}

bool Demo::EnableBodyGrab()
{
    if (target && Input::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
    {
        if (target->GetType() == RigidBody::Type::Dynamic)
        {
            target->Awake();
            gj = world->CreateGrabJoint(target, mpos, mpos, 4.0f, 0.5f, target->GetMass());
            gj->OnDestroy = [&](Joint* me) -> void { gj = nullptr; };
        }
    }

    if (gj)
    {
        gj->SetTarget(mpos);
        if (Input::IsMouseReleased(GLFW_MOUSE_BUTTON_LEFT))
        {
            world->Destroy(gj);
            gj = nullptr;
        }
        else if (Input::IsMousePressed(GLFW_MOUSE_BUTTON_RIGHT))
        {
            gj = nullptr; // Stick to the air!
        }
    }

    return gj != nullptr;
}

bool Demo::EnableAddForce()
{
    static RigidBody* ft;
    static Vec2 mStartLocal;

    if (target && target->GetType() != RigidBody::Type::Static)
    {
        if (Input::IsKeyDown(GLFW_KEY_LEFT_SHIFT) && Input::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
        {
            ft = target;
            mStartLocal = MulT(target->GetTransform(), mpos);
        }
    }

    if (ft)
    {
        auto& pl = game.GetPointList();
        auto& ll = game.GetLineList();

        pl.push_back(ft->GetTransform() * mStartLocal);
        pl.push_back(mpos);
        ll.push_back(ft->GetTransform() * mStartLocal);
        ll.push_back(mpos);

        if (Input::IsMouseReleased(GLFW_MOUSE_BUTTON_LEFT))
        {
            if (ft->GetWorld())
            {
                Vec2 mStartGlobal = ft->GetTransform() * mStartLocal;
                Vec2 f = mStartGlobal - mpos;
                f *= settings.INV_DT * ft->GetMass() * 3.0f;

                ft->AddForce(mStartLocal, f);
            }

            ft = nullptr;
        }
    }

    return ft != nullptr;
}

void Demo::EnableKeyboardShortcut()
{
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
}

void Demo::EnableCameraControl()
{
    if (Input::GetMouseScroll().y != 0)
    {
        camera.scale *= Input::GetMouseScroll().y < 0 ? 1.1f : 1.0f / 1.1f;
        camera.scale = Clamp(camera.scale, Vec2{ 0.1f }, Vec2{ FLT_MAX });
    }

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

void Demo::Step()
{
    DebugOptions& options = game.GetDebugOptions();

    if (options.pause)
    {
        if (options.step)
        {
            options.step = false;
            world->Step(dt);
        }
    }
    else
    {
        world->Step(dt);
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
extern DemoFrame kinematic_body;
extern DemoFrame convex_hull;
extern DemoFrame dynamic_aabb_tree;
extern DemoFrame compute_distance_point;
extern DemoFrame collision_detection;
extern DemoFrame cloth;
extern DemoFrame collision_filtering;
extern DemoFrame angle_joint_test;
extern DemoFrame weld_joint_test;
extern DemoFrame prismatic_joint_test;
extern DemoFrame conveyor_belt;
extern DemoFrame pulley;
extern DemoFrame windmill;
extern DemoFrame crank;
extern DemoFrame terrain;
extern DemoFrame rounded_polygon;
extern DemoFrame ray_casting;
extern DemoFrame line_of_sight;

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
    demos[demo_count++] = kinematic_body;
    demos[demo_count++] = convex_hull;
    demos[demo_count++] = dynamic_aabb_tree;
    demos[demo_count++] = compute_distance_point;
    demos[demo_count++] = collision_detection;
    demos[demo_count++] = cloth;
    demos[demo_count++] = collision_filtering;
    demos[demo_count++] = angle_joint_test;
    demos[demo_count++] = weld_joint_test;
    demos[demo_count++] = prismatic_joint_test;
    demos[demo_count++] = conveyor_belt;
    demos[demo_count++] = pulley;
    demos[demo_count++] = windmill;
    demos[demo_count++] = crank;
    demos[demo_count++] = terrain;
    demos[demo_count++] = rounded_polygon;
    demos[demo_count++] = ray_casting;
    demos[demo_count++] = line_of_sight;

    return demo_count;
}

static int32 _ = init_demos();

} // namespace muli
