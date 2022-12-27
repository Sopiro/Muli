#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli
{

static float angle = 45.0f;

class SubStepping : public Demo
{
public:
    float progress = 0.0f;

    SubStepping(Game& game)
        : Demo(game)
    {
        settings.sub_stepping = true;

        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, RigidBody::Type::static_body);

        RigidBody* b = world->CreateCircle(1.5f);

        Vec2 p = PolarToCart(DegToRad(angle), 120.0f);
        b->SetPosition(p + Vec2{ 0.0f, 9.0f });
        b->SetLinearVelocity(-p * 0.4f + Vec2{ 0.0f, 9.0f });
        b->SetAngularVelocity(0.0f);
        b->SetContinuous(true);

        int32 rows = 15;
        float boxSize = 0.4f;
        float xGap = 0.0625f * boxSize / 0.5f;
        float yGap = 0.125f * boxSize / 0.5f;
        float xStart = -(rows - 1.0f) * (boxSize + xGap) / 2.0f;
        float yStart = 0.2f + boxSize / 2.0f + yGap;

        for (int32 y = 0; y < rows; ++y)
        {
            for (int32 x = 0; x < rows - y; ++x)
            {
                b = world->CreateBox(boxSize);
                b->SetPosition(xStart + y * (boxSize + xGap) / 2.0f + x * (boxSize + xGap), yStart + y * (boxSize + yGap));
                b->SetContinuous(true);
            }
        }
    }

    void Step() override
    {
        DebugOptions& options = game.GetDebugOptions();

        if (options.pause)
        {
            if (options.step)
            {
                options.step = false;
                progress = world->Step(dt);
            }
        }
        else
        {
            progress = world->Step(dt);
        }
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get().GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Sub-stepping", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Text("Step progress");
            ImGui::ProgressBar(progress, ImVec2{ 0.0f, 0.0f });
            // ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
            ImGui::Text("Throw angle");
            ImGui::SliderFloat("##Throw angle", &angle, 0.0f, 180.0f, "%.2f", 1.0f);
        }
        ImGui::End();
    }

    static Demo* Create(Game& game)
    {
        return new SubStepping(game);
    }
};

DemoFrame sub_stepping{ "Sub-stepping", SubStepping::Create };

} // namespace muli
