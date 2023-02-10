#include "demo.h"
#include "game.h"
#include "window.h"
namespace muli
{

static int32 selection = 0;
static const char* items[] = { "Revolute joint", "Distance joint" };
static float f = 12.0f;
static float d = 0.5f;
static float m = 1.0f;

class MultiPendulum : public Demo
{
public:
    MultiPendulum(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, RigidBody::Type::static_body);

        float xStart = 0.0f;
        float yStart = 5.0f;
        float sizeW = 0.3f;
        float sizeH = 0.15f;
        float gap = 0.1f;

        RigidBody* b1 = world->CreateBox(sizeW, sizeH);
        b1->SetPosition(xStart - (gap + sizeW), yStart);

        Joint* j = world->CreateRevoluteJoint(ground, b1, { xStart, yStart }, -1.0f);

        bool t = LinearRand(0.0f, 1.0f) > 0.5;

        int32 count = 12;
        for (int32 i = 1; i < count; ++i)
        {
            RigidBody* b2 = world->CreateBox(sizeW, sizeH);
            b2->SetPosition(xStart - (gap + sizeW) * (i + 1), yStart);

            if (selection == 0)
            {
                j = world->CreateRevoluteJoint(b1, b2, { xStart - (sizeW + gap) / 2 - (gap + sizeW) * i, yStart }, f, d, m);
            }
            else
            {
                j = world->CreateDistanceJoint(b1, b2, b1->GetPosition() - Vec2{ sizeW / 2, 0 },
                                               b2->GetPosition() + Vec2{ sizeW / 2, 0 }, -1.0f, f, d, m);
            }

            b1 = b2;
        }
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get().GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Multi pendulum", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Text("Joint type");
            ImGui::PushID(0);
            if (ImGui::ListBox("", &selection, items, IM_ARRAYSIZE(items)))
            {
                game.RestartDemo();
            }
            ImGui::Text("Frequency");
            ImGui::SliderFloat("##Frequency", &f, 0.0f, 20.0f, "%.2f");
            ImGui::Text("Damping ratio");
            ImGui::SliderFloat("##Damping ratio", &d, 0.0f, 1.0f, "%.2f");
            ImGui::Text("Joint mass");
            ImGui::SliderFloat("##Joint mass", &m, 0.0f, 10.0f, "%.2f");
            ImGui::PopID();
        }
        ImGui::End();
    }

    static Demo* Create(Game& game)
    {
        return new MultiPendulum(game);
    }
};

DemoFrame multi_pendulum{ "Multi pendulum", MultiPendulum::Create };

} // namespace muli
