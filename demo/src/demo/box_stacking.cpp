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
        float l = 0.5f;
        RigidBody* ground = world->CreateCapsule(1000.0f, l, true, RigidBody::Type::static_body);

        float size = 1.0f;
        float gap = 0.03f;
        float start = l + size / 2.0f + gap;

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
