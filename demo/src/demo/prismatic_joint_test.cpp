#include "demo.h"

namespace muli
{

class PrismaticJointTest : public Demo
{
public:
    RigidBody* ground;

    PrismaticJointTest(Game& game)
        : Demo(game)
    {
        ground = world->CreateBox(100.0f, 0.4f, RigidBody::Type::Static);

        RigidBody* b = world->CreateBox(0.5f);
        b->SetPosition(0.0f, 2.0f);
        world->CreatePrismaticJoint(ground, b);

        b = world->CreateBox(0.5f);
        b->SetPosition(0.0f, 5.0f);
        world->CreatePrismaticJoint(ground, b, b->GetPosition(), Vec2{ 1.0f, 0.0f });

        world->CreateCapsule(Vec2{ 3.0f, 4.8f }, Vec2{ 3.0f, 5.2f }, 0.1f, false, RigidBody::Type::Static);
        world->CreateCapsule(Vec2{ -3.0f, 4.8f }, Vec2{ -3.0f, 5.2f }, 0.1f, false, RigidBody::Type::Static);
    }

    void Step() override
    {
        Demo::Step();
    }

    static Demo* Create(Game& game)
    {
        return new PrismaticJointTest(game);
    }
};

DemoFrame prismatic_joint_test{ "Prismatic joint test", PrismaticJointTest::Create };

} // namespace muli
