#include "demo.h"
#include "game.h"

namespace muli
{

class Windmill : public Demo
{
public:
    MotorJoint* motor;
    RigidBody* windmill;

    Windmill(Game& game)
        : Demo(game)
    {
        RigidBody* stick = world->CreateCapsule(Vec2{ 0.0f, 0.0f }, Vec2{ 0.0f, 3.0f }, 0.075f, RigidBody::Type::static_body);

        windmill = world->CreateCapsule(2.0f, 0.075f, true);
        windmill->SetPosition(0.0f, 3.0f);

        CollisionFilter filter;
        filter.bit = 1 << 1;
        filter.mask = ~(1 << 1);

        stick->SetCollisionFilter(filter);
        windmill->SetCollisionFilter(filter);

        motor = world->CreateMotorJoint(stick, windmill, windmill->GetPosition(), 1000.0f, 100.0f);
    }

    float t = 0.0f;

    void Step() override
    {
        Demo::Step();
        motor->SetAngularOffset(windmill->GetAngle() + 5.0f * dt);

        if (game.GetTime() > t + 0.2f)
        {
            RigidBody* c = world->CreateRegularPolygon(0.18f, LinearRand(3, 8));
            c->SetPosition(LinearRand(Vec2{ -2.0f, 6.0f }, Vec2{ 2.0f, 6.0f }));
            game.RegisterRenderBody(c);

            t = game.GetTime();
        }
    }

    static Demo* Create(Game& game)
    {
        return new Windmill(game);
    }
};

DemoFrame windmill{ "Windmill", Windmill::Create };

} // namespace muli
