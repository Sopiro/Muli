#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli
{

static char seed[20] = "wakgood";

class DynamicAABBTree : public Demo
{
public:
    DynamicAABBTree(Game& game)
        : Demo(game)
    {
        settings.apply_gravity = false;
        settings.sleeping = false;
        options.show_bvh = true;

        float size = 0.2f;
        float range = 3.0f;

        srand(static_cast<uint32>(std::hash<std::string>{}(seed)));

        for (int32 i = 0; i < 10; ++i)
        {
            float r = Random(0.0f, 3.0f);
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

            float x = Random(-range, range);
            float y = Random(-range, range);

            b->SetPosition(x, y);
            b->SetRotation(Random(0.0f, pi));
        }

        camera.position.SetZero();
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get().GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Dynamic AABB tree", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Text("Tree cost: %.4f", world->GetDynamicTree().ComputeTreeCost());
            ImGui::InputText("Seed", seed, 20);
            ImGui::Separator();
            if (ImGui::Button("Generate"))
            {
                game.RestartDemo();
            }
            if (ImGui::Button("Random generate"))
            {
                std::string newSeed = std::to_string((int32)LinearRand(0, INT32_MAX));
                strcpy(seed, newSeed.c_str());

                game.RestartDemo();
            }
            if (ImGui::Button("Rebuild bottom up"))
            {
                world->RebuildDynamicTree();
            }
        }
        ImGui::End();
    }

    float Random(float left, float right)
    {
        return (rand() / (float)RAND_MAX) * (right - left) + left;
    }

    ~DynamicAABBTree()
    {
        options.show_bvh = false;
    }

    static Demo* Create(Game& game)
    {
        return new DynamicAABBTree(game);
    }
};

DemoFrame dynamic_aabb_tree{ "Dynamic AABB tree", DynamicAABBTree::Create };

} // namespace muli
