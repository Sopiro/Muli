#include "demo.h"

namespace muli
{

class SuspensionBridge : public Demo
{
public:
    SuspensionBridge(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateBox(100.0f, 0.4f, RigidBody::Type::static_body);

        float groundStart = 0.2f;

        float xStart = -5.0f;
        float yStart = 4.0f;
        float gap = 0.1f;

        float pillarWidth = 0.3f;
        float sizeX = 0.5f;
        float sizeY = sizeX * 0.25f;

        RigidBody* pillar = world->CreateBox(pillarWidth, yStart, RigidBody::Type::static_body);
        pillar->SetPosition(xStart, yStart / 2 + 0.2f);

        RigidBody* b1 = world->CreateBox(sizeX, sizeY);
        b1->SetMass(10.0f);
        b1->SetPosition(xStart + sizeX / 2 + pillarWidth / 2 + gap, yStart + groundStart);

        Joint* j;

        bool revoluteBridge = LinearRand(0.0f, 1.0f) > 0.5;
        float frequency = 30.0f;

        if (revoluteBridge)
        {
            j = world->CreateRevoluteJoint(pillar, b1, pillar->GetPosition() + Vec2{ pillarWidth, yStart } / 2.0f, frequency,
                                           1.0f);
        }
        else
        {
            j = world->CreateDistanceJoint(pillar, b1, pillar->GetPosition() + Vec2{ pillarWidth / 2.0f, yStart / 2.0f },
                                           b1->GetPosition() + Vec2{ -sizeX / 2.0f, 0.03f }, -1.0f, frequency, 1.0f);
            j = world->CreateDistanceJoint(pillar, b1, pillar->GetPosition() + Vec2{ pillarWidth / 2.0f, yStart / 2.0f },
                                           b1->GetPosition() + Vec2{ -sizeX / 2.0f, -0.03f }, -1.0f, frequency, 1.0f);
        }

        for (int i = 1; i + 1 < xStart * -2 / (sizeX + gap); ++i)
        {
            RigidBody* b2 = world->CreateBox(sizeX, sizeY);
            b2->SetMass(10.0f);
            b2->SetPosition(xStart + sizeX / 2.0f + pillarWidth / 2.0f + gap + (gap + sizeX) * i, yStart + groundStart);

            if (revoluteBridge)
            {
                j = world->CreateRevoluteJoint(b1, b2, (b1->GetPosition() + b2->GetPosition()) / 2.0f, frequency, 1.0f);
            }
            else
            {
                j = world->CreateDistanceJoint(b1, b2, b1->GetPosition() + Vec2{ sizeX / 2.0f, 0.03f },
                                               b2->GetPosition() + Vec2{ -sizeX / 2.0f, 0.03f }, -1.0f, frequency, 1.0f);
                j = world->CreateDistanceJoint(b1, b2, b1->GetPosition() + Vec2{ sizeX / 2.0f, -0.03f },
                                               b2->GetPosition() + Vec2{ -sizeX / 2.0f, -0.03f }, -1.0f, frequency, 1.0f);
            }

            b1 = b2;
        }

        pillar = world->CreateBox(pillarWidth, yStart, RigidBody::Type::static_body);
        pillar->SetPosition(-xStart, yStart / 2.0f + 0.2f);

        if (revoluteBridge)
        {
            j = world->CreateRevoluteJoint(pillar, b1, pillar->GetPosition() + Vec2{ -pillarWidth, yStart } / 2.0f, frequency,
                                           1.0f);
        }
        else
        {
            j = world->CreateDistanceJoint(pillar, b1, pillar->GetPosition() + Vec2{ -pillarWidth / 2.0f, yStart / 2.0f },
                                           b1->GetPosition() + Vec2{ sizeX / 2.0f, 0.03f }, -1, frequency, 1.0f);
            j = world->CreateDistanceJoint(pillar, b1, pillar->GetPosition() + Vec2{ -pillarWidth / 2.0f, yStart / 2.0f },
                                           b1->GetPosition() + Vec2{ sizeX / 2.0f, -0.03f }, -1, frequency, 1.0f);
        }

        camera.position = Vec2{ 0, 3.6f + 1.8f };
        camera.scale = Vec2{ 1.5f, 1.5f };
    }

    static Demo* Create(Game& game)
    {
        return new SuspensionBridge(game);
    }
};

DemoFrame suspension_bridge{ "Suspension bridge", SuspensionBridge::Create };

} // namespace muli
