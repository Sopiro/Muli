#include "demo.h"

namespace muli
{

class FixedRotation : public Demo
{
public:
    FixedRotation(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateBox(100.0f, 0.4f, RigidBody::Type::Static);

        float start = 0.5f;
        float size = 0.3f;
        float gap = 0.05f;

        float error = 0.15f;

        float px = 0.0;
        float d = 1;

        uint32 count = 20;
        for (uint32 i = 0; i < count; i++)
        {
            if (i % (count / 4) == 0)
            {
                d *= -1;
            }

            RigidBody* b = world->CreateBox(size);
            px += d * error;
            b->SetPosition(px, start + i * (size + gap));
            b->SetFixedRotation(true);
        }

        // RigidBody* b1 = world->CreateBox(0.5f);
        // b1->SetPosition(0, 3);
        // RigidBody* b2 = world->CreateBox(0.5f);
        // b2->SetPosition(2, 2);

        // world->CreateAngleJoint(b1, b2)
    }

    static Demo* Create(Game& game)
    {
        return new FixedRotation(game);
    }
};

DemoFrame fixed_rotation{ "Fixed rotation", FixedRotation::Create };

} // namespace muli