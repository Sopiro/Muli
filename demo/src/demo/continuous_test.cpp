#include "demo.h"
#include "window.h"

namespace muli
{

static bool drawTrajectory = true;

class ContinuousTest : public Demo
{
public:
    RigidBody* target;
    Capsule* stick;

    ContinuousTest(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, identity, RigidBody::Type::static_body);

        world->CreateCapsule(Vec2{ 0.0f, 2.0f }, Vec2{ 0.0f, 1.5f }, 0.1f, identity, RigidBody::Type::static_body);

        target = world->CreateCapsule(2.8f, 0.05f, true);
        target->SetPosition(Rand(-0.5f, 0.5f), screenBounds.y);
        target->SetLinearVelocity(0.0f, -100.0f);
        target->SetAngularVelocity(Rand(-20.0f, 20.0f));
        target->SetContinuous(true);

        stick = (Capsule*)target->GetColliderList()->GetShape();

        settings.continuous = true;
    }

    void Render() override
    {
        if (drawTrajectory)
        {
            Transform t = target->GetTransform();
            t.rotation = t.rotation.GetAngle() + target->GetAngularVelocity() * settings.step.dt;
            t.position += target->GetLinearVelocity() * settings.step.dt;

            Renderer::DrawMode drawMode;
            drawMode.fill = false;
            drawMode.outline = true;

            if (target->GetWorld())
            {
                game.GetRenderer().DrawShape(target->GetColliderList()->GetShape(), t, drawMode);
            }
        }
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Continuous test 1", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Checkbox("Draw trajectory", &drawTrajectory);
        }
        ImGui::End();
    }

    ~ContinuousTest() {}

    static Demo* Create(Game& game)
    {
        return new ContinuousTest(game);
    }
};

static int index = register_demo("Continuous test", ContinuousTest::Create, 48);

} // namespace muli