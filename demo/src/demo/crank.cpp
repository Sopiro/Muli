#include "demo.h"

namespace muli
{

class Crank : public Demo
{
public:
    MotorJoint* motor;
    RigidBody* arm1;

    Crank(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, RigidBody::Type::static_body);

        float radius = 0.1f;

        arm1 = world->CreateCapsule(Vec2{ 0.0f, 1.0f }, Vec2{ 0.5f, 1.0f }, radius, RigidBody::Type::dynamic_body);

        RigidBody* arm2 = world->CreateCapsule(Vec2{ 0.5f, 1.0f }, Vec2{ 0.0f, 2.0f }, radius);
        world->CreateRevoluteJoint(arm1, arm2, Vec2{ 0.5f, 1.0f });

        RigidBody* arm3 = world->CreateCapsule(Vec2{ 0.0f, 2.0f }, Vec2{ 0.0f, 3.0f }, radius);
        arm3->CreateBoxCollider(1.0f, 0.2f, default_radius, Vec2{ 0.0f, 0.6f });

        world->CreateRevoluteJoint(arm2, arm3, Vec2{ 0.0f, 2.0f });
        world->CreatePrismaticJoint(ground, arm3, -1.0f);

        CollisionFilter filter;
        filter.bit = 1 << 1;
        filter.mask = ~(1 << 1);

        arm1->SetCollisionFilter(filter);
        arm2->SetCollisionFilter(filter);
        arm3->SetCollisionFilter(filter);

        motor = world->CreateMotorJoint(ground, arm1, Vec2{ 0.0f, 1.0f }, -1.0f, 100.0f);

        RigidBody* b = world->CreateBox(0.6f);
        b->SetPosition(0.0f, 4.0f);
    }

    void Step() override
    {
        Demo::Step();
        motor->SetAngularOffset(arm1->GetAngle() + 4.5f * dt);
    }

    static Demo* Create(Game& game)
    {
        return new Crank(game);
    }
};

static int index = register_demo("Crank", Crank::Create, 34);

} // namespace muli
