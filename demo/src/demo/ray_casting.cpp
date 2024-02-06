#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli
{

class RayCasting : public Demo
{
public:
    int32 count;
    Vec2 from{ -3.0f, 2.7f };
    Vec2 to{ 3.0f, 3.3f };

    bool closest = true;

    RayCasting(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, RigidBody::Type::static_body);

        RigidBody* b = world->CreateCircle(0.3f);
        b->SetPosition(1.5f, 3);

        b = world->CreateCapsule(0.5f, 0.2f);
        b->SetPosition(-0.5f, 3);

        b = world->CreateBox(0.3f, RigidBody::dynamic_body, 0.1f);
        b->SetPosition(0.5f, 3);
        b->UserData = (void*)((size_t)b->UserData | UserFlag::render_polygon_radius);

        b = world->CreateRegularPolygon(0.2f, 3);
        b->SetPosition(-1.5f, 3);

        settings.apply_gravity = false;
        settings.sleeping = false;
    }

    void Step() override
    {
        Demo::Step();

        bool hit = false;
        Vec2 closestPoint;
        Vec2 closestNormal;

        count = 0;
        world->RayCastAny(from, to, [&](Collider* collider, const Vec2& point, const Vec2& normal, float fraction) -> float {
            ++count;

            if (closest == false)
            {
                renderer.DrawPoint(point);
                renderer.DrawLine(point, point + normal * 0.2f);

                return 1.0f;
            }
            else
            {
                hit = true;
                closestPoint = point;
                closestNormal = normal;

                return fraction;
            }
        });

        if (closest && hit)
        {
            renderer.DrawPoint(closestPoint);
            renderer.DrawLine(closestPoint, closestPoint + closestNormal * 0.2f);
        }
    }

    void UpdateInput() override
    {
        FindTargetBody();
        EnableKeyboardShortcut();
        EnablePolygonCreate();

        static bool pressed = false;

        if (!ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow))
        {
            EnableCameraControl();

            if (!pressed && Input::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
            {
                from = cursorPos;
                pressed = true;
            }

            if (pressed && Input::IsMouseDown(GLFW_MOUSE_BUTTON_LEFT))
            {
                to = cursorPos;
            }

            if (pressed && Input::IsMouseReleased(GLFW_MOUSE_BUTTON_LEFT))
            {
                pressed = false;
            }
        }
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get().GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Ray casting", NULL,
                         ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar))
        {
            ImGui::Checkbox("Closest", &closest);
        }
        ImGui::End();

        renderer.DrawLine(from, to);
        renderer.DrawPoint(from, Vec4{ 1, 0, 0, 1 });
        renderer.DrawPoint(to, Vec4{ 0, 0, 1, 1 });
    }

    static Demo* Create(Game& game)
    {
        return new RayCasting(game);
    }
};

DemoFrame ray_casting{ "Ray casting", RayCasting::Create };

} // namespace muli
