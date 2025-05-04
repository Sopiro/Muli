#include "demo.h"

namespace muli
{

class LimitedJoints : public Demo
{
public:
    LimitedJoints(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, identity, RigidBody::static_body);

        RigidBody* s = world->CreateCapsule(0.3, 0.1f, true, identity, RigidBody::static_body);
        s->SetPosition(-1.5f, 5.0f);

        RigidBody* b = world->CreateBox(0.4f);
        b->SetPosition(-1.5f, 3.0f);

        world->CreateLimitedDistanceJoint(s, b, -1, 4);

        CollisionFilter filter;
        filter.bit = (1 << 1);
        filter.mask = ~(1 << 1);
        s = world->CreateCapsule(0.3, 0.1f, true, identity, RigidBody::static_body);
        s->SetPosition(1.5f, 5.0f);
        s->SetCollisionFilter(filter);

        b = world->CreateCapsule(2.0f, 0.1f);
        b->SetPosition(1.5f, 4.0f);
        b->SetCollisionFilter(filter);

        world->CreateRevoluteJoint(s, b, s->GetPosition(), -1);
        world->CreateAngleJoint(s, b, DegToRad(90), 10.0f, 0.7f, b->GetMass());
    }

    static Demo* Create(Game& game)
    {
        return new LimitedJoints(game);
    }
};

static int index = register_demo("Joint limits", LimitedJoints::Create, 58);

} // namespace muli
