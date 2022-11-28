#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli
{

class RayCasting : public Demo
{
public:
    int32 count;
    Vec2 from{ 0.0f, 3.0f };
    bool closest = true;

    RayCasting(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateBox(100.0f, 0.4f, RigidBody::Type::static_body);

        RigidBody* b = world->CreateCircle(1.0f);
        b->SetPosition(3, 3);

        b = world->CreateCapsule(1.0f, 0.5f);
        b->SetPosition(-3, 3);

        b = world->CreateBox(1.0f, RigidBody::dynamic_body, 0.1f);
        b->SetPosition(1, 3);
        b->userFlag = UserFlag::RENDER_POLYGON_RADIUS;

        b = world->CreateRegularPolygon(0.5f, 3, MULI_PI / 6.0f);
        b->SetPosition(-1, 3);
    }

    void Step() override
    {
        Demo::Step();

        Vec2 to = mpos;

        std::vector<Vec2>& pl = game.GetPointList();
        std::vector<Vec2>& ll = game.GetLineList();
        ll.push_back(from);
        ll.push_back(to);

        bool hit = false;
        Vec2 closestPoint;
        Vec2 closestNormal;

        count = 0;
        world->RayCastAny(from, to, [&](Collider* collider, const Vec2& point, const Vec2& normal, float fraction) -> float {
            ++count;

            if (closest == false)
            {
                pl.push_back(point);
                ll.push_back(point);
                ll.push_back(point + normal * 0.2f);

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
            pl.push_back(closestPoint);
            ll.push_back(closestPoint);
            ll.push_back(closestPoint + closestNormal * 0.2f);
        }
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get().GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });
        ImGui::SetNextWindowSize({ 200, 100 }, ImGuiCond_Once);

        if (ImGui::Begin("Options"))
        {
            ImGui::Text("Ray start");
            ImGui::DragFloat2("", &from.x, 0.1f);
            ImGui::Checkbox("Closest", &closest);
        }
        ImGui::End();
    }

    static Demo* Create(Game& game)
    {
        return new RayCasting(game);
    }
};

DemoFrame ray_casting{ "Ray casting", RayCasting::Create };

} // namespace muli
