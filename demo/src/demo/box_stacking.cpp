#include "demo.h"
#include "window.h"

namespace muli
{

static int32 count = 20;
static float error = 0.0f;

class BoxStacking : public Demo
{
public:
    BoxStacking(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, RigidBody::Type::static_body);

        float size = 0.3f;
        float gap = 0.1f;
        float start = 0.2f + size / 2.0f + gap;

        for (int32 i = 0; i < count; ++i)
        {
            RigidBody* b = world->CreateBox(size);
            b->SetPosition(LinearRand(-error, error), start + i * (size + gap));
        }
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get().GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Box stacking", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::SliderInt("Count", &count, 1, 50);
            ImGui::SliderFloat("Error", &error, 0.0f, 0.1f, "%.2f", 1.0f);
        }
        ImGui::End();
    }

    static Demo* Create(Game& game)
    {
        return new BoxStacking(game);
    }
};

DemoFrame box_stacking{ "Box stacking", BoxStacking::Create };

} // namespace muli
