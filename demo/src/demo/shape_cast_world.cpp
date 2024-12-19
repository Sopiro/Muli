#include "demo.h"
#include "window.h"

namespace muli
{

static const char* items[] = { "Circle", "Capsule", "Polygon", "Rounded" };

class ShapeCastWorld : public Demo
{
public:
    Transform tf = identity;
    std::unique_ptr<Shape> shape;

    int32 count;
    Vec2 from{ -3.0f, 2.7f };
    Vec2 to{ 3.0f, 3.3f };

    bool closest = true;

    int item = 1;

    ShapeCastWorld(Game& game)
        : Demo(game)
    {
        RigidBody* b;
        b = world->CreateCircle(0.3f);
        b->SetPosition(1.5f, 3.0f);

        b = world->CreateCapsule(0.5f, 0.2f);
        b->SetPosition(-0.5f, 3.0f);

        b = world->CreateBox(0.3f, RigidBody::dynamic_body, 0.1f);
        b->SetPosition(0.5f, 3.0f);
        UserFlag::SetFlag(b, UserFlag::render_polygon_radius, true);

        b = world->CreateRegularPolygon(0.2f, 3);
        b->SetPosition(-1.5f, 3.0f);

        settings.apply_gravity = false;
        settings.sleeping = false;

        UpdateShape();
    }

    void Render() override
    {
        bool hit = false;
        float closestFraction;
        Vec2 closestPoint;
        Vec2 closestNormal;

        Renderer::DrawMode dm;
        dm.rounded = true;

        tf.position = from;
        Vec2 translation = to - from;

        world->ShapeCastAny(
            shape.get(), tf, translation,
            [&](Collider* collider, const Vec2& point, const Vec2& normal, float fraction) -> float {
                hit = true;

                if (closest == false)
                {
                    renderer.DrawPoint(point);
                    renderer.DrawLine(point, point + normal * 0.2f);
                    ++dm.colorIndex;
                    renderer.DrawShape(shape.get(), Transform(from + translation * fraction, identity), dm);

                    return 1.0f;
                }
                else
                {
                    closestPoint = point;
                    closestNormal = normal;
                    closestFraction = fraction;

                    return fraction;
                }
            }
        );

        renderer.DrawLine(from, to);
        renderer.DrawPoint(from, Vec4{ 1, 0, 0, 1 });
        renderer.DrawPoint(to, Vec4{ 0, 0, 1, 1 });

        if (closest && hit)
        {
            renderer.DrawPoint(closestPoint);
            renderer.DrawLine(closestPoint, closestPoint + closestNormal * 0.2f);
            renderer.DrawShape(shape.get(), Transform(from + translation * closestFraction, identity), dm);
        }
        else
        {
            // renderer.DrawShape(shape.get(), Transform(to, identity), dm);
            // AABB aabb;
            // shape->ComputeAABB(Transform(to, identity), &aabb);
            // renderer.DrawAABB(aabb);
        }
    }

    void UpdateInput() override
    {
        FindTargetBody();
        EnableKeyboardShortcut();
        EnablePolygonCreateDecomposition();
        EnableBodyRemove();

        static bool pressed = false;
        bool hover = ImGui::GetIO().WantCaptureMouse;

        EnableCameraControl();

        if (!pressed && !hover && Input::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
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

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin(
                "Shape cast world", NULL,
                ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar
            ))
        {
            ImGui::Checkbox("Closest", &closest);

            ImGui::SetNextItemWidth(100);
            if (ImGui::Combo("shape", &item, items, IM_ARRAYSIZE(items)))
            {
                UpdateShape();
            }
        }
        ImGui::End();
    }

    void UpdateShape()
    {
        switch (item)
        {
        case 0:
        {
            shape.reset(new Circle(0.2f));
        }
        break;
        case 1:
        {
            shape.reset(new Capsule(0.3f, 0.14f));
        }
        break;
        case 2:
        {
            shape.reset(new Polygon(0.3f));
        }
        break;
        case 3:
        {
            float h = 0.15f;
            shape.reset(new Polygon({ { -h, 0.0f }, { h, 0.0f }, { 0.0f, 2.0f * h / 1.732f } }, true, 0.05f));
        }
        break;

        default:
            break;
        }
    }

    ~ShapeCastWorld()
    {
        options.show_contact_normal = false;
        options.show_contact_point = false;
    }

    static Demo* Create(Game& game)
    {
        return new ShapeCastWorld(game);
    }
};

static int index = register_demo("Shape cast world", ShapeCastWorld::Create, 52);

} // namespace muli