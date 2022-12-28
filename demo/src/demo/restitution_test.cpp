#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli
{

static int32 selection = 0;
static float threshold = 2.0f;
static const char* items[] = { "Circle", "Box", "Capsule" };
static int32 selection2 = 0;
static const char* items2[] = { "Quadratic", "Linear" };

class RestitutionTest : public Demo
{
public:
    RestitutionTest(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, RigidBody::Type::static_body);
        ground->SetRestitutionThreshold(threshold);

        int32 count = 11;
        float gap = 0.5f;
        float size = 0.3f;

        float xStart = -(count - 1) / 2 * gap;
        float yStart = 6.0f;

        RigidBody* b;
        for (int32 i = 0; i < count; ++i)
        {
            switch (selection)
            {
            case 0:
                b = world->CreateCircle(size * 0.5f);
                break;
            case 1:
                b = world->CreateBox(size);
                break;
            case 2:
                b = world->CreateCapsule(size * 0.8f, size * 0.8f * 0.5f);
                break;
            default:
                muliAssert(false);
                break;
            }

            b->SetPosition(xStart + gap * i, yStart);
            float attenuation = (count - i) / (float)count;
            float restitution = 1.0f - (selection2 == 0 ? attenuation * attenuation : attenuation);
            b->SetRestitution(restitution);
            b->SetRestitutionThreshold(threshold);
        }
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get().GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Resitution test", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Text("Shape");
            ImGui::PushID(0);
            if (ImGui::ListBox("##Shape", &selection, items, IM_ARRAYSIZE(items)))
            {
                game.RestartDemo();
            }
            ImGui::PopID();

            ImGui::Text("Attenuation");
            if (ImGui::ListBox("##Attenuation", &selection2, items2, IM_ARRAYSIZE(items2)))
            {
                game.RestartDemo();
            }

            ImGui::Text("Restitution threshold");
            ImGui::SliderFloat("##Restitution threshold", &threshold, 2.0f, 10.0f, "%.2f m/s");
        }
        ImGui::End();
    }

    static Demo* Create(Game& game)
    {
        return new RestitutionTest(game);
    }
};

DemoFrame restitution_test{ "Restitution test", RestitutionTest::Create };

} // namespace muli
