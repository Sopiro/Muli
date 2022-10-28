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
        RigidBody* ground = world->CreateBox(100.0f, 0.4f, RigidBody::Type::Static);

        int32 count = 10;
        float size = 7.0f;

        RigidBody* b;

        for (int32 i = 0; i < count; i++)
        {
            for (int32 j = 0; j < count; j++)
            {
                float r = LinearRand(0.0f, 3.0f);
                if (r < 0.5f)
                {
                    b = world->CreateCircle(0.2f);
                }
                else if (r < 1.0f)
                {
                    b = world->CreateCapsule(0.2f, 0.1f);
                    b->SetRotation(LinearRand(0.0f, MULI_PI));
                }
                else if (r < 2.0f)
                {

                    b = world->CreateRandomConvexPolygon(0.2f, 0);
                }
                else
                {
                    b = world->CreateRandomConvexPolygon(0.15f, 0, 0.05f);
                    b->userFlag = UserFlag::RENDER_POLYGON_RADIUS;
                }

                float y = (i - count / 2.0f) / count * size + size / 2.0f + 0.5f;
                float x = (j - count / 2.0f) / count * size;

                b->SetPosition(x, y);
            }
        }

        settings.APPLY_GRAVITY = false;
    }

    void Step() override
    {
        Demo::Step();

        std::vector<Vec2>& pl = game.GetPointList();
        std::vector<Vec2>& ll = game.GetLineList();

        float angleDelta = MULI_PI * 2.0f / count;
        for (float angle = 0.0f; angle < MULI_PI * 2.0f; angle += angleDelta)
        {
            Vec2 to = mpos + Vec2{ Cos(angle), Sin(angle) } * 3.0f;

            ll.push_back(mpos);

            if (!world->RayCastClosest(mpos, to,
                                       [&](RigidBody* body, const Vec2& point, const Vec2& normal, float fraction) -> void {
                                           pl.push_back(point);
                                           ll.push_back(point);
                                       }))
            {
                ll.push_back(to);
            }
        }
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get().GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });
        ImGui::SetNextWindowSize({ 200, 100 }, ImGuiCond_Once);

        if (ImGui::Begin("Options"))
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

DemoFrame line_of_sight{ "Line of sight", LineOfSight::Create };

} // namespace muli