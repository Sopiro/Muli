#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli
{

class DynamicAABBTree : public Demo
{
public:
    DynamicAABBTree(Game& game)
        : Demo(game)
    {
        settings.APPLY_GRAVITY = false;
        settings.SLEEPING = false;
        options.showBVH = true;

        float size = 0.2f;
        float range = 3.0f;

        for (int32 i = 0; i < 10; i++)
        {
            float r = LinearRand(0.0f, 3.0f);
            RigidBody* b;

            if (r < 1.0f)
            {
                b = world->CreateBox(size);
            }
            else if (r < 2.0f)
            {
                b = world->CreateCircle(size / 2.0f);
            }
            else
            {
                b = world->CreateCapsule(size, size / 2.0f);
            }

            b->SetPosition(LinearRand(Vec2{ -range, -range }, Vec2{ range, range }));
            b->SetRotation(LinearRand(0.0f, MULI_PI));
        }

        camera.position.SetZero();
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get().GetWindowSize().x - 5, 5 }, ImGuiCond_Always, { 1.0f, 0.0f });
        ImGui::Begin("Overlay", NULL,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize |
                         ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoBackground);
        ImGui::TextColored(ImColor{ 12, 11, 14 }, "Tree cost: %.4f", world->GetBVH().ComputeTreeCost());
        ImGui::End();
    }

    ~DynamicAABBTree()
    {
        options.showBVH = false;
    }

    static Demo* Create(Game& game)
    {
        return new DynamicAABBTree(game);
    }
};

DemoFrame dynamic_aabb_tree{ "Dynamic AABB tree", DynamicAABBTree::Create };

} // namespace muli
