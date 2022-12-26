#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli
{

class ComputeDistancePoint : public Demo
{
public:
    RigidBody* b;
    Vec2 closest{ 0.0f };
    float distance = 0.0f;

    ComputeDistancePoint(Game& game)
        : Demo(game)
    {
        options.draw_outline_only = true;
        options.show_contact_normal = true;
        options.show_contact_point = true;
        settings.apply_gravity = false;
        settings.velocity_iterations = 0;
        settings.position_iterations = 0;

        float size = 1.0f;
        float range = size * 0.7f;

        float r = LinearRand(0.0f, 4.0f);

        if (r < 1.0f)
        {
            b = world->CreateRandomConvexPolygon(size / 2.0f, 10, RigidBody::Type::dynamic_body, minimum_radius);
        }
        else if (r < 2.0f)
        {
            b = world->CreateRandomConvexPolygon(size / 2.0f, 10, RigidBody::Type::dynamic_body, r / 10.0f + minimum_radius);
            b->UserFlag |= UserFlag::render_polygon_radius;
        }
        else if (r < 3.0f)
        {
            b = world->CreateCircle(size / 2.0f);
        }
        else
        {
            b = world->CreateCapsule(size, size / 2.0f);
        }

        b->SetRotation(LinearRand(0.0f, pi));

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
        std::vector<Vec2>& pl = game.GetPointList();
        std::vector<Vec2>& ll = game.GetLineList();

        // pl.push_back(b->GetPosition());
        // ll.push_back(b->GetPosition());
        // ll.push_back(mpos);

        pl.push_back(closest);
        pl.push_back(cursorPos);
        ll.push_back(closest);
        ll.push_back(cursorPos);
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get().GetWindowSize().x - 5, 5 }, ImGuiCond_Always, { 1.0f, 0.0f });
        ImGui::Begin("Distance to a point", NULL,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize |
                         ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoBackground);
        ImGui::TextColored(ImColor{ 12, 11, 14 }, "Distance: %.4f", distance);
        ImGui::End();
    }

    ~ComputeDistancePoint()
    {
        options.draw_outline_only = false;
        options.show_contact_normal = false;
        options.show_contact_point = false;
    }

    static Demo* Create(Game& game)
    {
        return new ComputeDistancePoint(game);
    }
};

DemoFrame compute_distance_point{ "Distance to a point", ComputeDistancePoint::Create };

} // namespace muli
