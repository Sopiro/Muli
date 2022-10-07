#include "demo.h"

namespace muli
{

class WeldJointTest : public Demo
{
public:
    WeldJointTest(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateBox(100.0f, 0.4f, RigidBody::Type::Static);

        for (uint32 i = 0; i < 50; i++)
        {
            RigidBody* b1 = world->CreateRegularPolygon(0.12f);
            b1->SetPosition(-0.3f, 0);
            RigidBody* b2 = world->CreateRegularPolygon(0.12f);
            b2->SetPosition(0.3f, 0);
            Capsule* c = world->CreateCapsule(b1->GetPosition(), b2->GetPosition(), 0.02f, false);

            Vec2 t = LinearRand(Vec2{ -5.0f, 1.0f }, Vec2{ 5.0f, 15.0f });
            b1->Translate(t);
            b2->Translate(t);
            c->Translate(t);

            world->CreateWeldJoint(b1, b2);
            world->CreateWeldJoint(b1, c);
            world->CreateWeldJoint(b2, c);

            CollisionFilter cf{ i + 2, 1, 0xfffffffe };
            b1->SetCollisionFilter(cf);
            b2->SetCollisionFilter(cf);
            c->SetCollisionFilter(cf);
        }
    }

    static Demo* Create(Game& game)
    {
        return new WeldJointTest(game);
    }
};

DemoFrame weld_joint_test{ "Weld joint test", WeldJointTest::Create };

} // namespace muli
