#include "demo.h"
#include "window.h"

namespace muli
{

static int32 rows = 15;

class Pyramid : public Demo
{
public:
    Pyramid(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, RigidBody::Type::static_body);

        float boxSize = 0.4f;
        float xGap = 0.0625f * boxSize / 0.5f;
        float yGap = 0.125f * boxSize / 0.5f;
        float xStart = -(rows - 1.0f) * (boxSize + xGap) / 2.0f;
        float yStart = 0.2f + boxSize / 2.0f + yGap;

        for (int32 y = 0; y < rows; ++y)
        {
            for (int32 x = 0; x < rows - y; ++x)
            {
                RigidBody* b = world->CreateBox(boxSize);
                b->SetPosition(xStart + y * (boxSize + xGap) / 2.0f + x * (boxSize + xGap), yStart + y * (boxSize + yGap));
            }
        }
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get().GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Pyramid", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::SliderInt("Rows", &rows, 1, 100);
        }
        ImGui::End();
    }

    static Demo* Create(Game& game)
    {
        return new Pyramid(game);
    }
};

DemoFrame pyramid{ "Pyramid", Pyramid::Create };

} // namespace muli
