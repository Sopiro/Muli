#include "demo.h"
#include "window.h"

namespace muli
{

static bool create_bodies = true;
static float speed = 270.0f;
static float force = 1000.0f;
static float torque = 100.0f;

class Windmill : public Demo
{

public:
    MotorJoint* motor;
    RigidBody* windmill;

    Windmill(Game& game)
        : Demo(game)
    {
        RigidBody* stick = world->CreateCapsule(Vec2{ 0.0f, 0.0f }, Vec2{ 0.0f, 3.0f }, 0.075f, identity, RigidBody::static_body);

        windmill = world->CreateCapsule(2.0f, 0.075f, true);
        windmill->SetPosition(0.0f, 3.0f);
        windmill->SetContinuous(true);

        CollisionFilter filter;
        filter.bit = 1 << 1;
        filter.mask = ~(1 << 1);

        stick->SetCollisionFilter(filter);
        windmill->SetCollisionFilter(filter);

        motor = world->CreateMotorJoint(stick, windmill, windmill->GetPosition(), force, torque);
    }

    float t = 0.0f;

    void Step() override
    {
        motor->SetAngularOffset(windmill->GetAngle() + DegToRad(speed) * dt);
        motor->SetMaxForce(force);
        motor->SetMaxTorque(torque);

        if (!options.pause && game.GetTime() > t + 0.2f)
        {
            if (create_bodies)
            {
                RigidBody* c = CreateRegularPolygon(world, 0.18f, (int32)Rand(3, 8));
                c->SetPosition(RandVec2(Vec2{ -2.0f, 6.0f }, Vec2{ 2.0f, 6.0f }));
            }

            t = game.GetTime();
        }

        Demo::Step();
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Windmill", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Checkbox("Create bodies", &create_bodies);
            ImGui::SliderFloat("Speed", &speed, -3600.0f, 3600.0f, "%.2f deg/s");
            ImGui::SliderFloat("Motor force", &force, 0.0f, 10000.0f, "%.2f N");
            ImGui::SliderFloat("Motor torque", &torque, 0.0f, 10000.0f, "%.2f Nm");
        }
        ImGui::End();
    }

    static Demo* Create(Game& game)
    {
        return new Windmill(game);
    }
};

DemoFrame windmill{ "Windmill", Windmill::Create };
static int index = register_demo("Windmill", Windmill::Create, 33);

} // namespace muli
