#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli
{

Demo::Demo(Game& _game)
    : game{ _game }
    , renderer{ _game.GetRenderer() }
    , options{ _game.GetDebugOptions() }
    , targetBody{ nullptr }
    , targetCollider{ nullptr }
    , cursorJoint{ nullptr }
{
    dt = 1.0f / Window::Get().GetRefreshRate();
    screenBounds = Window::Get().GetWindowSize() * 0.01f;

    // dt = 1.0f / 60.0f;
    settings.world_bounds.min.y = -30.0f;

    world = new World(settings);

    camera.scale.Set(1.0f);
    camera.rotation = 0.0f;
    camera.position.Set(0.0f, screenBounds.y / 2.0f);
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

    targetCollider = nullptr;
    targetBody = nullptr;

    world->Query(cursorPos, [&](Collider* collider) -> bool {
        targetCollider = collider;
        targetBody = collider->GetBody();

        return false;
    });
}

void Demo::EnableBodyCreate()
{
    static bool create;
    static bool create_circle;
    static Vec2 mStart;

    if (Input::IsKeyDown(GLFW_KEY_1))
    {
        RigidBody* b = world->CreateBox(0.5f);
        b->SetPosition(cursorPos);
    }
    if (Input::IsKeyDown(GLFW_KEY_2))
    {
        RigidBody* b = world->CreateCircle(0.25f);
        b->SetPosition(cursorPos);
    }
    if (Input::IsKeyDown(GLFW_KEY_3))
    {
        RigidBody* b = world->CreateCapsule(0.5f, 0.25f);
        b->SetPosition(cursorPos);
    }
    if (Input::IsKeyDown(GLFW_KEY_4) && targetBody)
    {
        RigidBody* b = world->DuplicateBody(targetBody);
    }
    if (Input::IsKeyDown(GLFW_KEY_5) && targetBody)
    {
        targetBody->Sleep();
    }

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
        }
    }

    if (targetCollider && !cursorJoint && Input::IsMousePressed(GLFW_MOUSE_BUTTON_RIGHT))
    {
        targetBody->DestroyCollider(targetCollider);
        if (targetBody->GetColliderCount() == 0)
        {
            world->Destroy(targetBody);
        }
    }

    if (create)
    {
        renderer.DrawPoint(mStart);
        renderer.DrawPoint(cursorPos);
        renderer.DrawLine(mStart, cursorPos);

        if (Input::IsMouseReleased(GLFW_MOUSE_BUTTON_LEFT))
        {
            RigidBody* b = create_circle ? world->CreateCircle(0.15f * 1.414f) : world->CreateBox(0.3f);
            b->SetPosition(mStart);
            b->SetContinuous(true);

            Vec2 v = mStart - cursorPos;
            b->SetLinearVelocity(v * 7.0f);
            create = false;
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
        for (Vec2& point : points)
        {
            renderer.DrawPoint(point);
        }

        if (Input::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
        {
            points.push_back(cursorPos);
            hull = ComputeConvexHull(points);
        }

        if (hull.size() < 3)
        {
            renderer.DrawLine(hull.back(), cursorPos);
        }

        for (size_t i = 0; i < hull.size(); ++i)
        {
            Vec2& v0 = hull[i];
            Vec2& v1 = hull[(i + 1) % hull.size()];
            renderer.DrawLine(v0, v1);
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
        AABB aabb{ Min(mStart, cursorPos), Max(mStart, cursorPos) };

        Vec2 br{ aabb.min.x, aabb.max.y };
        Vec2 tl{ aabb.max.x, aabb.min.y };

        renderer.DrawLine(aabb.min, br);
        renderer.DrawLine(br, aabb.max);
        renderer.DrawLine(aabb.max, tl);
        renderer.DrawLine(tl, aabb.min);

        if (Input::IsMouseReleased(GLFW_MOUSE_BUTTON_MIDDLE))
        {
            std::vector<Collider*> colliders;
            const auto callback = [&](Collider* c) -> bool {
                colliders.push_back(c);
                return true;
            };

            if (aabb.min == aabb.max)
            {
                world->Query(aabb.min, callback);
            }
            else
            {
                world->Query(aabb, callback);
            }

            for (int32 i = 0; i < colliders.size(); ++i)
            {
                Collider* c = colliders[i];
                RigidBody* b = c->GetBody();
                b->DestroyCollider(c);

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
            cursorJoint = world->CreateGrabJoint(targetBody, cursorPos, cursorPos, 4.0f, 0.5f, targetBody->GetMass());
            cursorJoint->OnDestroy = this;
        }
    }

    if (cursorJoint)
    {
        cursorJoint->GetBodyA()->Awake();
        cursorJoint->SetTarget(cursorPos);

        if (Input::IsMouseReleased(GLFW_MOUSE_BUTTON_LEFT))
        {
            world->Destroy(cursorJoint);
            cursorJoint = nullptr;
        }
        else if (Input::IsMousePressed(GLFW_MOUSE_BUTTON_RIGHT))
        {
            cursorJoint = nullptr; // Stick to the air!
        }
    }

    return cursorJoint != nullptr;
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
        Vec2 tl = Mul(ft->GetTransform(), mStartLocal);

        renderer.DrawPoint(tl);
        renderer.DrawPoint(cursorPos);
        renderer.DrawLine(tl, cursorPos);

        if (Input::IsMouseReleased(GLFW_MOUSE_BUTTON_LEFT))
        {
            if (ft->GetWorld())
            {
                Vec2 i = tl - cursorPos;
                i *= ft->GetMass() * 3.0f;

                ft->ApplyLinearImpulseLocal(mStartLocal, i, true);
            }

            ft = nullptr;
        }
    }

    return ft != nullptr;
}

void Demo::EnableKeyboardShortcut()
{
    if (Input::IsKeyPressed(GLFW_KEY_Y)) options.draw_body = !options.draw_body;
    if (Input::IsKeyPressed(GLFW_KEY_O)) options.draw_outlined = !options.draw_outlined;
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
    if (Input::IsKeyPressed(GLFW_KEY_F))
    {
        if (targetBody)
        {
            if (targetBody->IsRotationFixed())
            {
                targetBody->SetFixedRotation(false);
            }
            else
            {
                targetBody->SetFixedRotation(true);
                targetBody->SetAngularVelocity(0.0f);
            }
        }
    }
}

void Demo::EnableCameraControl()
{
    if (Input::GetMouseScroll().y != 0)
    {
        camera.scale *= Input::GetMouseScroll().y < 0 ? 1.1f : 1.0f / 1.1f;
        camera.scale = Clamp(camera.scale, Vec2{ 0.1f }, Vec2{ max_value });
    }

    static bool cameraMove = false;
    static Vec2 cursorStart;
    static Vec2 cameraPosStart;

    if (!cameraMove && Input::IsMousePressed(GLFW_MOUSE_BUTTON_MIDDLE))
    {
        cameraMove = true;
        cursorStart = Input::GetMousePosition();
        cameraPosStart = camera.position;
    }
    else if (Input::IsMouseReleased(GLFW_MOUSE_BUTTON_MIDDLE))
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

} // namespace muli
