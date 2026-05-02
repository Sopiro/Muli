#include "demo.h"

namespace muli
{

static int32 rows = 15;

class DoubleDomino : public Demo
{
public:
    DoubleDomino(Game& game)
        : Demo(game)
    {
        float l = 0.2f;
        RigidBody* ground = world->CreateCapsule(1000.0f, 0.2f, true, identity, RigidBody::static_body);

        float boxWidth = 1.0f;
        float boxHeight = boxWidth * 4;
        float xGap = boxHeight - boxWidth * 0.97f;
        float xStart = -(rows - 1.0f) * (boxWidth + xGap) / 2.0f;
        float yStart = l + boxHeight / 2;

        for (int32 x = 0; x < rows; ++x)
        {
            RigidBody* b = world->CreateBox(boxWidth, boxHeight);
            b->SetPosition(xStart + x * (boxWidth + xGap), yStart);

            if (x == 0)
            {
                b->ApplyLinearImpulseLocal({ boxWidth / 2, boxHeight / 2 }, Vec2(5, 0), true);
            }
        }

        float w = Max(15, rows) * (boxWidth + xGap) - xGap;

        camera.position.Set(0.0f, w / 5.0f);
        camera.scale.Set(w * game.GetWindowScale() * 10);
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Double domino", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            if (ImGui::SliderInt("Rows", &rows, 2, 30))
            {
                // game.RestartDemo();
            }
        }
        ImGui::End();
    }

    static Demo* Create(Game& game)
    {
        return new DoubleDomino(game);
    }
};

static int index = register_demo("Double domino", DoubleDomino::Create, 56);

} // namespace muli
