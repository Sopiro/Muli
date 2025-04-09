#include "demo.h"

namespace muli
{

class SingleBox : public Demo
{
public:
    SingleBox(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, identity, RigidBody::Type::static_body);

        RigidBody* box = world->CreateBox(0.4f);
        box->SetPosition(0.0f, 5.0f);
        box->SetAngularVelocity(Rand(-12.0f, 12.0f));
    }

    static Demo* Create(Game& game)
    {
        return new SingleBox(game);
    }
};

static int index = register_demo("Single box", SingleBox::Create, 1);

} // namespace muli
