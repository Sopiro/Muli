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
    float radius = 0.0f;

    RayCasting(Game& game)
        : Demo(game)
    {
        RigidBody* b;
        b = world->CreateCircle(0.3f);
        b->SetPosition(1.5f, 3);

        b = world->CreateCapsule(0.5f, 0.2f);
        b->SetPosition(-0.5f, 3);

        b = world->CreateBox(0.3f, RigidBody::dynamic_body, 0.1f);
        b->SetPosition(0.5f, 3);
        UserFlag::SetFlag(b, UserFlag::render_polygon_radius, true);

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

        Circle c(radius);
        Renderer::DrawMode dm{};

        world->RayCastAny(from, to, radius,
                          [&](Collider* collider, const Vec2& point, const Vec2& normal, float fraction) -> float {
                              hit = true;

                              if (closest == false)
                              {
                                  renderer.DrawPoint(point);
                                  renderer.DrawLine(point, point + normal * 0.2f);

                                  ++dm.colorIndex;
                                  renderer.DrawShape(&c, Transform(point, identity), dm);

                                  return 1.0f;
                              }
                              else
                              {
                                  closestPoint = point;
                                  closestNormal = normal;

                                  return fraction;
                              }
                          });

        renderer.DrawLine(from, to);
        renderer.DrawPoint(from, Vec4{ 1, 0, 0, 1 });
        renderer.DrawPoint(to, Vec4{ 0, 0, 1, 1 });

        if (closest && hit)
        {
            renderer.DrawPoint(closestPoint);
            renderer.DrawLine(closestPoint, closestPoint + closestNormal * 0.2f);
            renderer.DrawShape(&c, Transform(closestPoint, identity), dm);
        }
        else
        {
            // renderer.DrawShape(&c, Transform(to, identity), dm);
            // AABB aabb;
            // c.ComputeAABB(Transform(to, identity), &aabb);
            // renderer.DrawAABB(aabb);
        }

        // {
        //     Vec2 p1 = Vec2{ 3, 1 };
        //     Vec2 p2 = Vec2{ 3, 3 };
        //     renderer.DrawLine(p1, p2);
        //     renderer.DrawPoint(p1);
        //     renderer.DrawPoint(p2);

        //     RayCastOutput ro;
        //     bool h =
        //         RayCastLineSegment(p1, p2, RayCastInput{ .from = from, .to = to, .maxFraction = 1.0f, .radius = radius }, &ro);
        //     if (h)
        //     {
        //         // std::cout << "hit" << std::endl;
        //         Vec2 p = from + ro.fraction * (to - from);
        //         renderer.DrawPoint(p);
        //         renderer.DrawShape(&c, Transform{ p, identity }, dm);
        //     }
        // }
    }

    void UpdateInput() override
    {
        FindTargetBody();
        EnableKeyboardShortcut();
        EnablePolygonCreate();
        EnableBodyRemove();

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
            ImGui::DragFloat("Ray radius", &radius, 0.01f, 0.0f, 0.5f, "%.2f");
        }
        ImGui::End();
    }

    static Demo* Create(Game& game)
    {
        return new RayCasting(game);
    }
};

static int index = register_demo("Ray casting", RayCasting::Create, 34);

} // namespace muli
