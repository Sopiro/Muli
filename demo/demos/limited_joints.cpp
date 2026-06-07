#include "demo.h"

namespace muli
{

class LimitedJoints : public Demo
{
public:
    LimitedJoints(Game& game)
        : Demo(game)
    {
        Body* ground = world->CreateCapsule(100.0f, 0.2f, true, identity, Body::static_body);

        Body* s = world->CreateCapsule(0.3, 0.1f, true, identity, Body::static_body);
        s->SetPosition(-1.5f, 5.0f);

        Body* b = world->CreateBox(0.4f);
        b->SetPosition(-1.5f, 3.0f);

        float frequency = 10.0f;
        float dampingRatio = 1.0f;

        world->CreateLimitedDistanceJoint(s, b, -1, 4, frequency, dampingRatio);

        CollisionFilter filter;
        filter.bit = (1 << 1);
        filter.mask = ~(1 << 1);
        s = world->CreateCapsule(0.3, 0.1f, true, identity, Body::static_body);
        s->SetPosition(1.5f, 5.0f);
        s->SetCollisionFilter(filter);

        b = world->CreateCapsule(2.0f, 0.1f);
        b->SetPosition(1.5f, 4.0f);
        b->SetCollisionFilter(filter);

        world->CreateRevoluteJoint(s, b, s->GetPosition(), -1);
        world->CreateLimitedAngleJoint(s, b, DegToRad(-45), DegToRad(90), frequency, dampingRatio);
    }

    static Demo* Create(Game& game)
    {
        return new LimitedJoints(game);
    }
};

static int index = register_demo("Joint limits", LimitedJoints::Create, 58);

} // namespace muli
