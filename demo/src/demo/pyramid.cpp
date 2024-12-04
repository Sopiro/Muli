#include "demo.h"
#include "window.h"

namespace muli
{

class Pyramid : public Demo
{
    static inline int32 rows = 15;

public:
    Pyramid(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(1000.0f, 0.4f, true, RigidBody::Type::static_body);

        float boxSize = 3.0f;
        float xGap = 0.03f * boxSize / 0.5f;
        float yGap = 0.03f * boxSize / 0.5f;
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

        float w = Max(15, rows) * (boxSize + xGap) - xGap;

        camera.position.Set(0.0f, w / 2.0f);
        camera.scale.Set(w * game.GetWindowScale() * 15);
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });

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

static int index = register_demo("Pyramid", Pyramid::Create, 3);

} // namespace muli
