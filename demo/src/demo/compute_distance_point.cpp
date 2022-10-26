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
        options.drawOutlineOnly = true;
        options.showContactNormal = true;
        options.showContactPoint = true;
        settings.APPLY_GRAVITY = false;
        settings.VELOCITY_SOLVE_ITERATIONS = 0;
        settings.POSITION_SOLVE_ITERATIONS = 0;

        float size = 1.0f;
        float range = size * 0.7f;

        float r = LinearRand(0.0f, 4.0f);

        if (r < 1.0f)
        {
            b = world->CreateRandomConvexPolygon(size / 2.0f, 10, 0.0f);
        }
        else if (r < 2.0f)
        {
            b = world->CreateRandomConvexPolygon(size / 2.0f, 10, r / 10.0f);
            b->userFlag |= UserFlag::RENDER_POLYGON_RADIUS;
        }
        else if (r < 3.0f)
        {
            b = world->CreateCircle(size / 2.0f);
        }
        else
        {
            b = world->CreateCapsule(size, size / 2.0f);
        }

        b->SetRotation(LinearRand(0.0f, MULI_PI));

        camera.position = 0.0f;
        camera.scale = 0.5f;
    }

    void UpdateInput() override
    {
        ComputeProperty();
        EnableCameraControl();

        closest = b->GetClosestPoint(mpos);
        distance = Dist(closest, mpos);
    }

    void Render() override
    {
        std::vector<Vec2>& pl = game.GetPointList();
        std::vector<Vec2>& ll = game.GetLineList();

        // pl.push_back(b->GetPosition());
        // ll.push_back(b->GetPosition());
        // ll.push_back(mpos);

        pl.push_back(closest);
        pl.push_back(mpos);
        ll.push_back(closest);
        ll.push_back(mpos);
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get().GetWindowSize().x - 5, 5 }, ImGuiCond_Always, { 1.0f, 0.0f });
        ImGui::Begin("Overlay", NULL,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize |
                         ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoBackground);
        ImGui::TextColored(ImColor{ 12, 11, 14 }, "Distance: %.4f", distance);
        ImGui::End();
    }

    ~ComputeDistancePoint()
    {
        options.drawOutlineOnly = false;
        options.showContactNormal = false;
        options.showContactPoint = false;
    }

    static Demo* Create(Game& game)
    {
        return new ComputeDistancePoint(game);
    }
};

DemoFrame compute_distance_point{ "Distance to a point", ComputeDistancePoint::Create };

} // namespace muli
