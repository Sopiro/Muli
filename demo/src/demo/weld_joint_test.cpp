#include "demo.h"
#include "window.h"

namespace muli
{

class WeldJointTest : public Demo
{
    static inline int32 selection = 0;
    static inline const char* items[] = { "Box", "Capsule" };

    static inline float freq = 8.0f;
    static inline float damp = 0.8f;

public:
    WeldJointTest(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, RigidBody::Type::static_body);

        float wallX = -3.0f;
        float wallRadius = 0.2f;
        RigidBody* wall = world->CreateCapsule(Vec2{ wallX, 0.0f }, Vec2{ wallX, 5.0f }, wallRadius, RigidBody::static_body);

        int32 count = 8;
        float xStart = wallX + wallRadius;
        float yStart = 3.0f;
        float w = 0.6f;
        float h = 0.2f;

        CollisionFilter filter;
        filter.bit = 2;
        filter.mask = ~2;

        RigidBody* b0;
        if (selection == 0)
        {
            b0 = world->CreateBox(w, h);
        }
        else
        {
            b0 = world->CreateCapsule(w, h / 2.0f, true);
            xStart += h / 2.0f;
        }
        b0->SetPosition(xStart + w / 2.0f, yStart);
        b0->SetCollisionFilter(filter);

        world->CreateWeldJoint(wall, b0, b0->GetPosition(), freq, damp, 1.0f);

        for (int32 i = 1; i < count; ++i)
        {
            RigidBody* b1;
            if (selection == 0)
            {
                b1 = world->CreateBox(w, h);
            }
            else
            {
                b1 = world->CreateCapsule(w, h / 2.0f, true);
            }

            b1->SetPosition(xStart + w / 2.0f + w * i, yStart);

            world->CreateWeldJoint(b0, b1, b0->GetPosition() + Vec2{ w / 2.0f, 0.0f }, freq, damp, 1.0f);
            b1->SetCollisionFilter(filter);
            b0 = b1;
        }

        b0 = world->CreateCircle(0.6f);
        b0->SetPosition(-1.5f, 6.5f);
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Weld joint test", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Text("Shapes");
            ImGui::PushID(0);
            if (ImGui::ListBox("", &selection, items, IM_ARRAYSIZE(items)))
            {
                game.RestartDemo();
            }
            ImGui::PopID();

            ImGui::Text("Joint frequency");
            ImGui::SliderFloat("##Joint frequency", &freq, 1.0f, 15.0f, "%.2f");
            ImGui::Text("Joint damping ratio");
            ImGui::SliderFloat("##Joint damping ratio", &damp, 0.0f, 1.0f, "%.2f");
        }
        ImGui::End();
    }

    static Demo* Create(Game& game)
    {
        return new WeldJointTest(game);
    }
};

static int index = register_demo("Weld joint test", WeldJointTest::Create, 29);

} // namespace muli
