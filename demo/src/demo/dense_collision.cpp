#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli
{

static float lv = 40.0f;
static float av = 1.0f;
static float ld = 0.1f;
static float ad = 0.1f;

class DenseCollision : public Demo
{
public:
    DenseCollision(Game& game)
        : Demo(game)
    {
        settings.world_bounds.min.y = -100.0f;
        settings.apply_gravity = false;

        float size = 15.0f;
        float r = 0.25f;
        float spread = 10.0f;

        RigidBody* b = world->CreateRandomConvexPolygon(spread / 2.0f, 7);
        b->SetPosition(-25.0, 0.0f);
        b->SetLinearVelocity(lv, 0.0f);
        b->SetAngularVelocity(av);
        b->SetLinearDamping(ld);
        b->SetAngularDamping(ad);
        b->SetContinuous(true);

        for (int32 i = 0; i < 500; ++i)
        {
            RigidBody* c = world->CreateCircle(r);
            c->SetPosition(LinearRand(0.0f, spread * 1.414f), LinearRand(0.0f, spread * 0.9f) - spread / 2.0f);
            c->SetLinearDamping(0.1f);
        }

        camera.position.Set(-5.0f, 0.0f);
        camera.scale.Set(6.0f);
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get().GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Dense collision", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::SliderFloat("Linear velocity", &lv, 0.0f, 100.0f, "%.2f m/s");
            ImGui::SliderFloat("Angular velocity", &av, 0.0f, 10.0f, "%.2f rad/s");
            ImGui::SliderFloat("Linear damping", &ld, 0.0f, 1.0f, "%.2f");
            ImGui::SliderFloat("Angular damping", &ad, 0.0f, 1.0f, "%.2f");
        }
        ImGui::End();
    }

    static Demo* Create(Game& game)
    {
        return new DenseCollision(game);
    }
};

DemoFrame dense_collision{ "Dense collision", DenseCollision::Create };

} // namespace muli
