#include "demo.h"
#include "window.h"

namespace muli
{

class BoxStacking : public Demo
{
    static inline int32 count = 20;
    static inline float error = 0.0f;

public:
    BoxStacking(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(1000.0f, 0.4f, true, RigidBody::Type::static_body);

        float size = 0.6f;
        float gap = 0.05f;
        float start = 0.2f + size / 2.0f + gap;

        for (int32 i = 0; i < count; ++i)
        {
            RigidBody* b = world->CreateBox(size);
            b->SetPosition(Rand(-error, error), start + i * (size + gap));
        }

        float h = Max(25, count) * (size + gap) - gap;

        camera.position.Set(0.0f, h / 2.0f);
        camera.scale.Set(h / screenBounds.y);
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Box stacking", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::SliderInt("Count", &count, 1, 100);
            ImGui::SliderFloat("Error", &error, 0.0f, 0.1f, "%.2f");
        }
        ImGui::End();
    }

    static Demo* Create(Game& game)
    {
        return new BoxStacking(game);
    }
};

static int index = register_demo("Box stacking", BoxStacking::Create, 2);

} // namespace muli
