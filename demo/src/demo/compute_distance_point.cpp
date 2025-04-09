#include "demo.h"
#include "window.h"

namespace muli
{

class ComputeDistancePoint : public Demo
{
public:
    RigidBody* b;
    Vec2 closest = Vec2::zero;
    float distance = 0.0f;

    ComputeDistancePoint(Game& game)
        : Demo(game)
    {
        options.draw_outlined = true;
        options.show_contact_normal = true;
        options.show_contact_point = true;
        settings.apply_gravity = false;
        settings.step.velocity_iterations = 0;
        settings.step.position_iterations = 0;

        float size = 1.0f;
        float range = size * 0.7f;

        float r = Rand(0.0f, 4.0f);

        if (r < 1.0f)
        {
            b = world->CreateRandomConvexPolygon(size / 2.0f, 10, identity, RigidBody::Type::dynamic_body, minimum_radius);
        }
        else if (r < 2.0f)
        {
            b = world->CreateRandomConvexPolygon(
                size / 2.0f, 10, identity, RigidBody::Type::dynamic_body, r / 10.0f + minimum_radius
            );
            UserFlag::SetFlag(b, UserFlag::render_polygon_radius, true);
        }
        else if (r < 3.0f)
        {
            b = world->CreateCircle(size / 2.0f);
        }
        else
        {
            b = world->CreateCapsule(size, size / 2.0f);
        }

        b->SetRotation(Rand(0.0f, pi));

        camera.position.SetZero();
        camera.scale.Set(0.5f);
    }

    void UpdateInput() override
    {
        FindTargetBody();
        EnableCameraControl();

        closest = b->GetClosestPoint(cursorPos);
        distance = Dist(closest, cursorPos);
    }

    void Render() override
    {
        renderer.DrawPoint(closest);
        renderer.DrawPoint(cursorPos);
        renderer.DrawLine(closest, cursorPos);
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5, 5 }, ImGuiCond_Always, { 1.0f, 0.0f });
        ImGui::Begin(
            "Distance to a point", NULL,
            ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize |
                ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoBackground
        );
        ImGui::TextColored(ImColor{ 12, 11, 14 }, "Distance: %.4f", distance);
        ImGui::End();
    }

    ~ComputeDistancePoint()
    {
        options.draw_outlined = false;
        options.show_contact_normal = false;
        options.show_contact_point = false;
    }

    static Demo* Create(Game& game)
    {
        return new ComputeDistancePoint(game);
    }
};

static int index = register_demo("Distance to a point", ComputeDistancePoint::Create, 23);

} // namespace muli
