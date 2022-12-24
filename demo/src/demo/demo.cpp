#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli
{

Demo::Demo(Game& _game)
    : game{ _game }
    , options{ _game.GetDebugOptions() }
    , targetBody{ nullptr }
    , targetCollider{ nullptr }
    , gj{ nullptr }
{
    dt = 1.0f / Window::Get().GetRefreshRate();
    // dt = 1.0f / 60.0f;
    settings.world_bounds.min.y = -20.0f;

    world = new World(settings);

    camera.scale.Set(1.0f, 1.0f);
    camera.rotation = 0.0f;
    camera.position.Set(0.0f, 3.6f);
}

Demo::~Demo()
{
    delete world;
    world = nullptr;
}

void Demo::UpdateInput()
{
    FindTargetBody();
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

void Demo::FindTargetBody()
{
    cursorPos = game.GetWorldCursorPosition();
    qr = world->Query(cursorPos); // Query result
    if (qr.size() > 0)
    {
        targetCollider = qr[0]; // Mouseovered body
        targetBody = targetCollider->GetBody();
    }
    else
    {
        targetCollider = nullptr;
        targetBody = nullptr;
    }
}

void Demo::EnableBodyCreate()
{
    static bool create;
    static bool create_circle;
    static Vec2 mStart;

    if (!targetCollider && Input::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
    {
        if (Input::IsKeyDown(GLFW_KEY_LEFT_SHIFT))
        {
            mStart = cursorPos;
            create = true;
            create_circle = Input::IsKeyDown(GLFW_KEY_Z);
        }
        else
        {
            RigidBody* b = world->CreateBox(0.5f);
            b->SetPosition(cursorPos);
            game.RegisterRenderBody(b);
        }
    }

    if (targetCollider && !gj && Input::IsMousePressed(GLFW_MOUSE_BUTTON_RIGHT))
    {
        targetBody->DestoryCollider(targetCollider);
        if (targetBody->GetColliderCount() == 0)
        {
            world->Destroy(targetBody);
        }
    }

    if (create)
    {
        auto& pl = game.GetPointList();
        auto& ll = game.GetLineList();

        pl.push_back(mStart);
        pl.push_back(cursorPos);
        ll.push_back(mStart);
        ll.push_back(cursorPos);

        if (Input::IsMouseReleased(GLFW_MOUSE_BUTTON_LEFT))
        {
            RigidBody* b = create_circle ? world->CreateCircle(0.15f * 1.414f) : world->CreateBox(0.3f);
            b->SetPosition(mStart);
            b->SetContinuous(true);

            Vec2 f = mStart - cursorPos;
            f *= settings.inv_dt * b->GetMass() * 7.0f;
            b->SetForce(f);
            create = false;
            game.RegisterRenderBody(b);
        }
    }
}

bool Demo::EnablePolygonCreate()
{
    static Vec2 mStart;
    static bool creating = false;
    static bool staticBody;
    static std::vector<Vec2> points;
    static std::vector<Vec2> hull;

    if (Input::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
    {
        if (Input::IsKeyDown(GLFW_KEY_LEFT_CONTROL))
        {
            mStart = cursorPos;
            creating = true;
            staticBody = false;
        }
        else if (Input::IsKeyDown(GLFW_KEY_LEFT_ALT))
        {
            mStart = cursorPos;
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
            points.push_back(cursorPos);
            hull = ComputeConvexHull(points);
        }

        if (hull.size() < 3)
        {
            ll.push_back(hull.back());
            ll.push_back(cursorPos);
        }

        for (size_t i = 0; i < hull.size(); ++i)
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
            {
                float r = Max(0.1f, Dist(hull.back(), cursorPos));
                b = world->CreateCircle(r, type);
                b->SetPosition(hull[0]);
                break;
            }
            case 2:
            {
                float r = Max(0.05f, Dist(hull.back(), cursorPos));
                b = world->CreateCapsule(hull[0], hull[1], r, type, false);
                break;
            }
            default:
            {
                b = world->CreatePolygon(hull, type, false);
                break;
            }
            }

            b->SetContinuous(settings.continuous);

            game.RegisterRenderBody(b);
            creating = false;
            points.clear();
            hull.clear();
        };

        if (!staticBody && Input::IsKeyReleased(GLFW_KEY_LEFT_CONTROL))
        {
            create_body(RigidBody::Type::dynamic_body);
        }
        else if (staticBody && Input::IsKeyReleased(GLFW_KEY_LEFT_ALT))
        {
            create_body(RigidBody::Type::static_body);
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
        mStart = cursorPos;
        draging = true;
    }

    if (draging)
    {
        auto& ll = game.GetLineList();

        AABB aabb{ Min(mStart, cursorPos), Max(mStart, cursorPos) };

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

            for (int32 i = 0; i < qr.size(); ++i)
            {
                Collider* c = qr[i];
                RigidBody* b = c->GetBody();
                b->DestoryCollider(c);

                if (b->GetColliderCount() == 0)
                {
                    world->Destroy(b);
                }
            }

            draging = false;
        }
    }
}

bool Demo::EnableBodyGrab()
{
    if (targetBody && Input::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
    {
        if (targetBody->GetType() == RigidBody::Type::dynamic_body)
        {
            targetBody->Awake();
            gj = world->CreateGrabJoint(targetBody, cursorPos, cursorPos, 4.0f, 0.5f, targetBody->GetMass());
            gj->OnDestroy = this;
        }
    }

    if (gj)
    {
        gj->SetTarget(cursorPos);
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

    if (targetBody && targetBody->GetType() != RigidBody::Type::static_body)
    {
        if (Input::IsKeyDown(GLFW_KEY_LEFT_SHIFT) && Input::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
        {
            ft = targetBody;
            mStartLocal = MulT(targetBody->GetTransform(), cursorPos);
        }
    }

    if (ft)
    {
        auto& pl = game.GetPointList();
        auto& ll = game.GetLineList();

        pl.push_back(ft->GetTransform() * mStartLocal);
        pl.push_back(cursorPos);
        ll.push_back(ft->GetTransform() * mStartLocal);
        ll.push_back(cursorPos);

        if (Input::IsMouseReleased(GLFW_MOUSE_BUTTON_LEFT))
        {
            if (ft->GetWorld())
            {
                Vec2 mStartGlobal = ft->GetTransform() * mStartLocal;
                Vec2 f = mStartGlobal - cursorPos;
                f *= settings.inv_dt * ft->GetMass() * 3.0f;

                ft->AddForce(mStartLocal, f);
            }

            ft = nullptr;
        }
    }

    return ft != nullptr;
}

void Demo::EnableKeyboardShortcut()
{
    if (Input::IsKeyPressed(GLFW_KEY_O)) options.draw_outline_only = !options.draw_outline_only;
    if (Input::IsKeyPressed(GLFW_KEY_L)) options.colorize_island = !options.colorize_island;
    if (Input::IsKeyPressed(GLFW_KEY_V)) options.show_bvh = !options.show_bvh;
    if (Input::IsKeyPressed(GLFW_KEY_B)) options.show_aabb = !options.show_aabb;
    if (Input::IsKeyPressed(GLFW_KEY_P)) options.show_contact_point = !options.show_contact_point;
    if (Input::IsKeyPressed(GLFW_KEY_N)) options.show_contact_normal = !options.show_contact_normal;
    if (Input::IsKeyPressed(GLFW_KEY_C)) options.reset_camera = !options.reset_camera;
    if (Input::IsKeyPressed(GLFW_KEY_SPACE)) options.pause = !options.pause;
    if (Input::IsKeyDown(GLFW_KEY_RIGHT) || Input::IsKeyPressed(GLFW_KEY_S)) options.step = true;

    if (Input::IsKeyPressed(GLFW_KEY_U)) settings.continuous = !settings.continuous;
    if (Input::IsKeyPressed(GLFW_KEY_I)) settings.sub_stepping = !settings.sub_stepping;
    if (Input::IsKeyPressed(GLFW_KEY_G))
    {
        settings.apply_gravity = !settings.apply_gravity;
        world->Awake();
    }
    if (Input::IsKeyPressed(GLFW_KEY_T))
    {
        if (targetBody)
        {
            targetBody->SetContinuous(!targetBody->IsContinuous());
        }
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

int32 demo_count = 0;
DemoFrame demos[MAX_DEMOS];

extern DemoFrame single_box;
extern DemoFrame box_stacking;
extern DemoFrame pyramid;
extern DemoFrame single_pendulum;
extern DemoFrame springs;
extern DemoFrame random_convex_polygons;
extern DemoFrame seesaw;
extern DemoFrame friction_test;
extern DemoFrame restitution_test;
extern DemoFrame multi_pendulum;
extern DemoFrame suspension_bridge;
extern DemoFrame circle_stacking;
extern DemoFrame capsule_stacking;
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
extern DemoFrame fixed_rotation;
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
extern DemoFrame ragdoll;
extern DemoFrame ragdoll_100;
extern DemoFrame car;
extern DemoFrame compound_shape;
extern DemoFrame contact_callbacks;
extern DemoFrame logo;
extern DemoFrame breakable;
extern DemoFrame distance_test;
extern DemoFrame continuous_test;
extern DemoFrame continuous_test2;
extern DemoFrame sub_stepping;
extern DemoFrame shape_cast;

static int32 init_demos()
{
    demos[demo_count++] = single_box;
    demos[demo_count++] = box_stacking;
    demos[demo_count++] = pyramid;
    demos[demo_count++] = single_pendulum;
    demos[demo_count++] = springs;
    demos[demo_count++] = random_convex_polygons;
    demos[demo_count++] = seesaw;
    demos[demo_count++] = friction_test;
    demos[demo_count++] = restitution_test;
    demos[demo_count++] = multi_pendulum;
    demos[demo_count++] = suspension_bridge;
    demos[demo_count++] = circle_stacking;
    demos[demo_count++] = capsule_stacking;
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
    demos[demo_count++] = fixed_rotation;
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
    demos[demo_count++] = ragdoll;
    demos[demo_count++] = ragdoll_100;
    demos[demo_count++] = car;
    demos[demo_count++] = compound_shape;
    demos[demo_count++] = contact_callbacks;
    demos[demo_count++] = logo;
    demos[demo_count++] = breakable;
    demos[demo_count++] = distance_test;
    demos[demo_count++] = continuous_test;
    demos[demo_count++] = continuous_test2;
    demos[demo_count++] = sub_stepping;
    demos[demo_count++] = shape_cast;

    return demo_count;
}

static int32 _ = init_demos();

} // namespace muli
