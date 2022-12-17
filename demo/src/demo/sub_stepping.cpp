#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli
{

class SubStepping : public Demo
{
public:
    float progress = 0.0f;

    SubStepping(Game& game)
        : Demo(game)
    {
        settings.sub_stepping = true;

        RigidBody* ground = world->CreateBox(100.0f, 0.4f, RigidBody::Type::static_body);

        float size = 15.0f;
        float r = 0.25f;
        float spread = 10.0f;

        RigidBody* b = world->CreateCircle(2.0f);
        b->SetPosition(90.0, 28.0f);
        b->SetLinearVelocity(-40.0f, 0.0f);
        b->SetAngularVelocity(0.0f);
        b->SetContinuous(true);

        int32 rows = 15;
        float boxSize = 0.4f;
        float xGap = 0.0625f * boxSize / 0.5f;
        float yGap = 0.125f * boxSize / 0.5f;
        float xStart = -(rows - 1.0f) * (boxSize + xGap) / 2.0f;
        float yStart = 0.2f + boxSize / 2.0f + yGap;

        for (int y = 0; y < rows; ++y)
        {
            for (int x = 0; x < rows - y; ++x)
            {
                RigidBody* b = world->CreateBox(boxSize);
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
        ImGui::SetNextWindowSize({ 200, 100 }, ImGuiCond_Once);

        if (ImGui::Begin("Options"))
        {
            ImGui::Text("Step progress");
            ImGui::ProgressBar(progress, ImVec2{ 0.0f, 0.0f });
            // ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
        }
        ImGui::End();
    }

    static Demo* Create(Game& game)
    {
        return new SubStepping(game);
    }
};

DemoFrame sub_stepping{ "Sub stepping", SubStepping::Create };

} // namespace muli
