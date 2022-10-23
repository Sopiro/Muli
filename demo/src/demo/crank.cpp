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
        RigidBody* ground = world->CreateBox(100.0f, 0.4f, RigidBody::Type::Static);

        disk = world->CreateCircle(0.5f, RigidBody::Type::Dynamic);
        disk->SetPosition(0, 1.0f);

        RigidBody* arm1 = world->CreateCapsule(Vec2{ 0.5f, 1.0f }, Vec2{ 0.0f, 2.0f }, 0.1f);
        world->CreateRevoluteJoint(disk, arm1, Vec2{ 0.5f, 1.0f });

        RigidBody* arm2 = world->CreateCapsule(Vec2{ 0.0f, 2.0f }, Vec2{ 0.0f, 3.0f }, 0.1f);
        world->CreateRevoluteJoint(arm1, arm2, Vec2{ 0.0f, 2.0f });
        world->CreatePrismaticJoint(ground, arm2, -1.0f);

        RigidBody* plate = world->CreateBox(1.0f, 0.2f);
        plate->SetPosition(0.0f, 3.05f);
        world->CreateWeldJoint(arm2, plate);

        CollisionFilter filter;
        filter.filter = 1 << 1;
        filter.mask = ~(1 << 1);

        disk->SetCollisionFilter(filter);
        arm1->SetCollisionFilter(filter);
        arm2->SetCollisionFilter(filter);
        plate->SetCollisionFilter(filter);

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
