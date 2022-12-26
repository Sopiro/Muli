#include "demo.h"
#include "game.h"

namespace muli
{

class Crank : public Demo
{
public:
    MotorJoint* motor;
    RigidBody* disk;

    Crank(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, RigidBody::Type::static_body);

        disk = world->CreateCircle(0.5f, RigidBody::Type::dynamic_body);
        disk->SetPosition(0, 1.0f);

        RigidBody* arm1 = world->CreateCapsule(Vec2{ 0.5f, 1.0f }, Vec2{ 0.0f, 2.0f }, 0.1f);
        world->CreateRevoluteJoint(disk, arm1, Vec2{ 0.5f, 1.0f });

        RigidBody* arm2 = world->CreateCapsule(Vec2{ 0.0f, 2.0f }, Vec2{ 0.0f, 3.0f }, 0.1f);
        arm2->CreateBoxCollider(1.0f, 0.2f, default_radius, Vec2{ 0.0f, 0.6f });

        world->CreateRevoluteJoint(arm1, arm2, Vec2{ 0.0f, 2.0f });
        world->CreatePrismaticJoint(ground, arm2, -1.0f);

        CollisionFilter filter;
        filter.filter = 1 << 1;
        filter.mask = ~(1 << 1);

        disk->SetCollisionFilter(filter);
        arm1->SetCollisionFilter(filter);
        arm2->SetCollisionFilter(filter);

        motor = world->CreateMotorJoint(ground, disk, disk->GetPosition(), -1.0f, 100.0f);

        RigidBody* b = world->CreateBox(0.6f);
        b->SetPosition(0.0f, 4.0f);
    }

    void Step() override
    {
        Demo::Step();
        motor->SetAngularOffset(disk->GetAngle() + 4.5f * dt);
    }

    static Demo* Create(Game& game)
    {
        return new Crank(game);
    }
};

DemoFrame crank{ "Crank", Crank::Create };

} // namespace muli
