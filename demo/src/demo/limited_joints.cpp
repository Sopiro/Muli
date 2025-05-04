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
        s->SetPosition(0.0f, 5.0f);

        RigidBody* b = world->CreateBox(0.4f);
        b->SetPosition(0.0f, 3.0f);

        world->CreateLimitedDistanceJoint(s, b, -1, 4);
    }

    static Demo* Create(Game& game)
    {
        return new LimitedJoints(game);
    }
};

static int index = register_demo("Joint limits", LimitedJoints::Create, 58);

} // namespace muli
