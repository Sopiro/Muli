#include "demo.h"

namespace muli
{

class FixedRotation : public Demo
{
public:
    FixedRotation(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, RigidBody::Type::static_body);

        float start = 1.0f;
        float size = 0.3f;
        float gap = 0.05f;

        float error = 0.15f;

        float px = 0.0;
        float d = 1;

        int32 count = 20;
        for (int32 i = 0; i < count; ++i)
        {
            if (i % (count / 4) == 0)
            {
                d = -d;
            }

            RigidBody* b = world->CreateBox(size);
            px += d * error;
            b->SetPosition(px, start + i * (size + gap));
            b->SetFixedRotation(true);
        }

        // RigidBody* b1 = world->CreateBox(0.5f);
        // b1->SetPosition(0, 3);
        // RigidBody* b2 = world->CreateBox(0.5f);
        // b2->SetPosition(2, 2);

        // world->CreateAngleJoint(b1, b2)
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos(
            { Window::Get()->GetWindowSize().x - 5, Window::Get()->GetWindowSize().y - 5 }, ImGuiCond_Always, { 1.0f, 1.0f }
        );
        ImGui::Begin(
            "FixedHelp", NULL,
            ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize |
                ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoBackground
        );
        ImGui::TextColored(ImColor{ 12, 11, 14 }, "Press f on body to toggle fixed");
        ImGui::End();
    }

    static Demo* Create(Game& game)
    {
        return new FixedRotation(game);
    }
};

static int index = register_demo("Fixed rotation", FixedRotation::Create, 27);

} // namespace muli
