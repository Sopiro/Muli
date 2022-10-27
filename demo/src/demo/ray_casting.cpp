#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli
{

class RayCasting : public Demo
{
public:
    int32 count;

    RayCasting(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateBox(100.0f, 0.4f, RigidBody::Type::Static);
        options.showAABB = true;

        RigidBody* b = world->CreateCircle(1.0f);
        b->SetPosition(3, 3);

        b = world->CreateCapsule(1.0f, 0.5f);
        b->SetPosition(-3, 3);

        b = world->CreateBox(1.0f, RigidBody::Dynamic, 0.1f);
        b->SetPosition(1, 3);
        b->userFlag = UserFlag::RENDER_POLYGON_RADIUS;
    }

    ~RayCasting()
    {
        options.showAABB = false;
    }

    void Step() override
    {
        Demo::Step();

        Vec2 from{ 0.0f, 3.0f };
        Vec2 to = mpos;

        std::vector<Vec2>& pl = game.GetPointList();
        std::vector<Vec2>& ll = game.GetLineList();
        ll.push_back(from);
        ll.push_back(to);

        count = 0;
        world->Raycast(from, to, [&](RigidBody* body, const Vec2& point, const Vec2& normal, float fraction) -> float {
            count++;

            pl.push_back(point);
            ll.push_back(point);
            ll.push_back(point + normal * 0.2f);

            return 1.0f;
        });
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get().GetWindowSize().x - 5, 5 }, ImGuiCond_Always, { 1.0f, 0.0f });
        ImGui::Begin("Overlay", NULL,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize |
                         ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoBackground);
        ImGui::TextColored(ImColor{ 12, 11, 14 }, "AABB Hit count: %d", count);
        ImGui::End();
    }

    static Demo* Create(Game& game)
    {
        return new RayCasting(game);
    }
};

DemoFrame ray_casting{ "Ray casting", RayCasting::Create };

} // namespace muli
