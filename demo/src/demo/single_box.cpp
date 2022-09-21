#include "demo.h"

namespace muli
{

class SingleBox : public Demo
{
public:
    SingleBox()
    {
        settings.APPLY_GRAVITY = true;
        RigidBody* ground = world->CreateBox(100.0f, 0.4f, RigidBody::Type::Static);

        RigidBody* box = world->CreateBox(0.4f);
        box->SetPosition(0.0f, 5.0f);
        box->SetAngularVelocity(LinearRand(-12.0f, 12.0f));
    }

    static Demo* Create()
    {
        return new SingleBox;
    }
};

DemoFrame single_box{ "Single Box", SingleBox::Create };

} // namespace muli
