#include "demo.h"

namespace muli
{

class RandomConvexPolygons : public Demo
{
public:
    RandomConvexPolygons(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, RigidBody::Type::static_body);

        int32 rows = 12;
        float size = 0.25f;
        float xGap = 0.2f;
        float yGap = 0.15f;
        float xStart = -(rows - 1) * (size + xGap) / 2.0f;
        float yStart = 1.0f;

        for (int32 y = 0; y < rows; ++y)
        {
            for (int32 x = 0; x < rows - y; ++x)
            {
                RigidBody* b = world->CreateRandomConvexPolygon(size, 8);
                b->SetPosition(xStart + y * (size + xGap) / 2 + x * (size + xGap), yStart + y * (size + yGap));
                b->SetLinearVelocity(b->GetPosition() * RandRange(0.5f, 0.7f));
            }
        }

        RigidBody* pillar = world->CreateCapsule(4.0f, 0.1f, false, RigidBody::Type::static_body);
        pillar->SetPosition(xStart - 0.2f, 3.0f);

        pillar = world->CreateCapsule(4.0f, 0.1f, false, RigidBody::Type::static_body);
        pillar->SetPosition(-(xStart - 0.2f), 3.0f);

        // Capsule* c = world->CreateCapsule(4.0f, 0.1f, true, RigidBody::Type::Dynamic);
        // c->SetPosition(0.0f, 2.0f);

        // world->CreateGrabJoint(c, c->GetPosition(), c->GetPosition(), -1.0f);
    }

    static Demo* Create(Game& game)
    {
        return new RandomConvexPolygons(game);
    }
};

DemoFrame random_convex_polygons{ "Random convex polygons", RandomConvexPolygons::Create };

} // namespace muli
