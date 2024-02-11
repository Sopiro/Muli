#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli
{

class LineOfSight : public Demo
{
public:
    int32 count = 100;

    LineOfSight(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, RigidBody::Type::static_body);

        int32 count = 10;
        float size = 7.0f;

        RigidBody* b;

        for (int32 i = 0; i < count; ++i)
        {
            for (int32 j = 0; j < count; ++j)
            {
                float r = Rand(0.0f, 3.0f);
                if (r < 0.5f)
                {
                    b = world->CreateCircle(0.2f);
                }
                else if (r < 1.0f)
                {
                    b = world->CreateCapsule(0.2f, 0.1f);
                    b->SetRotation(Rand(0.0f, pi));
                }
                else if (r < 2.0f)
                {

                    b = world->CreateRandomConvexPolygon(0.2f, 0);
                }
                else
                {
                    b = world->CreateRandomConvexPolygon(0.15f, 0, RigidBody::Type::dynamic_body, 0.05f);
                    UserFlag::SetFlag(b, UserFlag::render_polygon_radius, true);
                }

                float y = (i - count / 2.0f) / count * size + size / 2.0f + 0.5f;
                float x = (j - count / 2.0f) / count * size;

                b->SetPosition(x, y);
            }
        }

        settings.apply_gravity = false;
    }

    void Step() override
    {
        Demo::Step();

        float angleDelta = pi * 2.0f / count;
        for (float angle = 0.0f; angle < pi * 2.0f; angle += angleDelta)
        {
            Vec2 to = cursorPos + Vec2{ Cos(angle), Sin(angle) } * 3.0f;

            if (!world->RayCastClosest(cursorPos, to, 0.0f,
                                       [&](Collider* collider, const Vec2& point, const Vec2& normal, float fraction) -> void {
                                           renderer.DrawPoint(point);
                                           renderer.DrawLine(cursorPos, point);
                                       }))
            {
                renderer.DrawLine(cursorPos, to);
            }
        }
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get().GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Line of sight", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Text("Ray count");
            ImGui::DragInt(" ", &count, 1.0f, 1, 360);
        }
        ImGui::End();
    }

    static Demo* Create(Game& game)
    {
        return new LineOfSight(game);
    }
};

static int index = register_demo("Line of sight", LineOfSight::Create, 35);

} // namespace muli
