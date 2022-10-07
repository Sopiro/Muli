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

        Box* b = world->CreateBox(0.5f);
        b->SetPosition(0.0f, 4.0f);

        world->CreatePrismaticJoint(ground, b);
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
