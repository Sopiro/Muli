#include "demo.h"

namespace muli
{

class MultiPendulum : public Demo
{
public:
    MultiPendulum(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateBox(100.0f, 0.4f, RigidBody::Type::Static);

        float xStart = 0.0f;
        float yStart = 5.0f;
        float sizeW = 0.3f;
        float sizeH = 0.15f;
        float gap = 0.1f;

        RigidBody* b1 = world->CreateBox(sizeW, sizeH);
        b1->SetMass(1.0);
        b1->SetPosition(xStart - (gap + sizeW), yStart);

        Joint* j = world->CreateRevoluteJoint(ground, b1, { xStart, yStart }, -1.0f);

        bool t = LinearRand(0.0f, 1.0f) > 0.5;

        int count = 12;
        for (int i = 1; i < count; i++)
        {
            RigidBody* b2 = world->CreateBox(sizeW, sizeH);
            b2->SetMass(1.0f);
            b2->SetPosition(xStart - (gap + sizeW) * (i + 1), yStart);

            if (t)
            {
                j = world->CreateRevoluteJoint(b1, b2, { xStart - (sizeW + gap) / 2 - (gap + sizeW) * i, yStart }, 15.0f, 0.5f);
            }
            else
            {
                j = world->CreateDistanceJoint(b1, b2, b1->GetPosition() - Vec2{ sizeW / 2, 0 },
                                               b2->GetPosition() + Vec2{ sizeW / 2, 0 });
            }

            b1 = b2;
        }
    }

    static Demo* Create(Game& game)
    {
        return new MultiPendulum(game);
    }
};

DemoFrame multi_pendulum{ "Multi pendulum", MultiPendulum::Create };

} // namespace muli
