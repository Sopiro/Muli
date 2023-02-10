#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli
{

static int32 selection = 0;
static const char* items[] = { "Revolute joint", "Distance joint" };

class SuspensionBridge : public Demo
{
public:
    SuspensionBridge(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, RigidBody::Type::static_body);

        float groundStart = 0.2f;

        float xStart = -5.0f;
        float yStart = 4.0f;
        float gap = 0.1f;

        float pillarWidth = 0.3f;
        float sizeX = 0.5f;
        float sizeY = sizeX * 0.25f;

        RigidBody* pillar = world->CreateBox(pillarWidth, yStart, RigidBody::Type::static_body);
        pillar->SetPosition(xStart, yStart / 2 + 0.2f);

        RigidBody* b1 = world->CreateBox(sizeX, sizeY);
        b1->SetPosition(xStart + sizeX / 2 + pillarWidth / 2 + gap, yStart + groundStart);

        Joint* j;

        float frequency = 15.0f;

        if (selection == 0)
        {
            j = world->CreateRevoluteJoint(pillar, b1, pillar->GetPosition() + Vec2{ pillarWidth, yStart } / 2.0f, frequency,
                                           1.0f);
        }
        else
        {
            j = world->CreateDistanceJoint(pillar, b1, pillar->GetPosition() + Vec2{ pillarWidth / 2.0f, yStart / 2.0f },
                                           b1->GetPosition() + Vec2{ -sizeX / 2.0f, 0.0f }, -1.0f, frequency, 1.0f);
        }

        for (int32 i = 1; i + 1 < xStart * -2 / (sizeX + gap); ++i)
        {
            RigidBody* b2 = world->CreateBox(sizeX, sizeY);
            b2->SetPosition(xStart + sizeX / 2.0f + pillarWidth / 2.0f + gap + (gap + sizeX) * i, yStart + groundStart);

            if (selection == 0)
            {
                j = world->CreateRevoluteJoint(b1, b2, (b1->GetPosition() + b2->GetPosition()) / 2.0f, frequency, 1.0f);
            }
            else
            {
                j = world->CreateDistanceJoint(b1, b2, b1->GetPosition() + Vec2{ sizeX / 2.0f, 0.0f },
                                               b2->GetPosition() + Vec2{ -sizeX / 2.0f, 0.0f }, -1.0f, frequency, 1.0f);
            }

            b1 = b2;
        }

        pillar = world->CreateBox(pillarWidth, yStart, RigidBody::Type::static_body);
        pillar->SetPosition(-xStart, yStart / 2.0f + 0.2f);

        if (selection == 0)
        {
            j = world->CreateRevoluteJoint(pillar, b1, pillar->GetPosition() + Vec2{ -pillarWidth, yStart } / 2.0f, frequency,
                                           1.0f);
        }
        else
        {
            j = world->CreateDistanceJoint(pillar, b1, pillar->GetPosition() + Vec2{ -pillarWidth / 2.0f, yStart / 2.0f },
                                           b1->GetPosition() + Vec2{ sizeX / 2.0f, 0.0f }, -1, frequency, 1.0f);
        }

        camera.position = Vec2{ 0, 3.6f + 1.8f };
        camera.scale = Vec2{ 1.5f, 1.5f };
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get().GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });

        // if (ImGui::Begin("Suspension bridge", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        // {
        //     ImGui::Text("Joint type");
        //     ImGui::PushID(0);
        //     if (ImGui::ListBox("", &selection, items, IM_ARRAYSIZE(items)))
        //     {
        //         game.RestartDemo();
        //     }
        //     ImGui::PopID();
        // }
        // ImGui::End();
    }

    static Demo* Create(Game& game)
    {
        return new SuspensionBridge(game);
    }
};

DemoFrame suspension_bridge{ "Suspension bridge", SuspensionBridge::Create };

} // namespace muli
