#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli
{

static float f = 2.0f;
static float d = 0.7f;

class Cloth : public Demo
{
public:
    Cloth(Game& game)
        : Demo(game)
    {
        options.draw_outline_only = true;

        const int32 rows = 24;
        const int32 cols = static_cast<int32>(rows * 1.4f);
        float radius = 0.02f;

        float width = 4.5f;

        float gap = width / rows;

        RigidBody* circles[rows][cols];

        float yStart = 2.0f;

        for (int32 j = 0; j < rows; ++j)
        {
            for (int32 i = 0; i < cols; ++i)
            {
                RigidBody* c = world->CreateCircle(radius);

                float x = ((i - (cols - 1) / 2.0f) / (float)cols) * cols * gap;
                float y = (j / (float)rows) * rows * gap + yStart;

                c->SetPosition(x, y);
                c->SetCollisionFilter(CollisionFilter{ 1, (1 << 1), 0xffffffff ^ (1 << 1) });

                circles[j][i] = c;
            }
        }

        for (int32 j = 0; j < rows; ++j)
        {
            for (int32 i = 0; i < cols; ++i)
            {
                RigidBody* c00 = circles[j][i];

                if (j + 1 < rows)
                {
                    RigidBody* c10 = circles[j + 1][i];
                    world->CreateDistanceJoint(c00, c10, -1.0f, f, d, 1.0f);
                }
                if (i + 1 < cols)
                {
                    RigidBody* c01 = circles[j][i + 1];
                    world->CreateDistanceJoint(c00, c01, -1.0f, f, d, 1.0f);
                }
            }
        }

        RigidBody* tl = circles[rows - 1][0];
        RigidBody* ml = circles[rows - 1][(int32)(cols / 3.0f) - 1];
        RigidBody* mr = circles[rows - 1][(int32)(cols * 2.0f / 3.0f)];
        RigidBody* tr = circles[rows - 1][cols - 1];

        RigidBody* w1 = world->CreateCircle(0.1f, RigidBody::Type::static_body);
        RigidBody* w2 = world->CreateCircle(0.1f, RigidBody::Type::static_body);
        RigidBody* w3 = world->CreateCircle(0.1f, RigidBody::Type::static_body);
        RigidBody* w4 = world->CreateCircle(0.1f, RigidBody::Type::static_body);

        world->CreateGrabJoint(tl, tl->GetPosition(), tl->GetPosition() + Vec2{ -gap, gap }, 15.0f, 1.0f, tl->GetMass());
        world->CreateGrabJoint(ml, ml->GetPosition(), ml->GetPosition() + Vec2{ 0.0f, gap }, 15.0f, 1.0f, tl->GetMass());
        world->CreateGrabJoint(mr, mr->GetPosition(), mr->GetPosition() + Vec2{ 0.0f, gap }, 15.0f, 1.0f, tl->GetMass());
        world->CreateGrabJoint(tr, tr->GetPosition(), tr->GetPosition() + Vec2{ gap, gap }, 15.0f, 1.0f, tl->GetMass());
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get().GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Cloth", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Text("Frequency");
            ImGui::SliderFloat("##Frequency", &f, 1.0f, 5.0f, "%.2f");
            ImGui::Text("Damping ratio");
            ImGui::SliderFloat("##Damping ratio", &d, 0.0f, 1.0f, "%.2f");
        }
        ImGui::End();
    }

    ~Cloth()
    {
        options.draw_outline_only = false;
    }

    static Demo* Create(Game& game)
    {
        return new Cloth(game);
    }
};

DemoFrame cloth{ "Cloth", Cloth::Create };

} // namespace muli
