#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli
{

static int32 count = 20;
static float error = 0.0f;

class CorrectAwake : public Demo
{
public:
    CorrectAwake(Game& game)
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

        t = false;
    }

    bool t;

    void UpdateUI() override
    {
        if (!t && world->GetSleepingBodyCount() >= count)
        {
            t = true;
            RigidBody* b = world->CreateBox(0.4f);
            b->SetPosition(6.0f, 3.25f);
            b->SetContinuous(true);

            b->SetLinearVelocity(-200.0f, 0.0f);
            game.RegisterRenderBody(b);
            options.pause = true;
        }

        ImGui::SetNextWindowPos({ Window::Get().GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Correct awake", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Text("Press 's' to step simulation");
        }
        ImGui::End();
    }

    ~CorrectAwake()
    {
        if (options.pause)
        {
            options.pause = false;
        }
    }

    static Demo* Create(Game& game)
    {
        return new CorrectAwake(game);
    }
};

DemoFrame correct_awake{ "Correct awake", CorrectAwake::Create };

} // namespace muli
