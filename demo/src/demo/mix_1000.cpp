#include "demo.h"

namespace muli
{

class Mix1000 : public Demo
{
public:
    Mix1000(Game& game)
        : Demo(game)
    {
        float size = 15.0f;
        float halfSize = size / 2.0f;
        float wallWidth = 0.4f;
        float wallRadius = wallWidth / 2.0f;

        world->CreateCapsule(Vec2{ -halfSize, -halfSize }, Vec2{ halfSize, -halfSize }, wallRadius, RigidBody::Type::static_body);
        world->CreateCapsule(Vec2{ halfSize, -halfSize }, Vec2{ halfSize, halfSize }, wallRadius, RigidBody::Type::static_body);
        world->CreateCapsule(Vec2{ halfSize, halfSize }, Vec2{ -halfSize, halfSize }, wallRadius, RigidBody::Type::static_body);
        world->CreateCapsule(Vec2{ -halfSize, halfSize }, Vec2{ -halfSize, -halfSize }, wallRadius, RigidBody::Type::static_body);

        float r = 0.24f;

        RigidBody* b;
        for (int32 i = 0; i < 1000; ++i)
        {
            float random = RandRange(0.0f, 3.0f);
            if (random < 1.0f)
            {
                b = world->CreateRandomConvexPolygon(r, 7);
            }
            else if (random < 2.0f)
            {
                b = world->CreateCircle(r);
            }
            else
            {
                b = world->CreateCapsule(r * 1.2f, r * 1.2f / 2.0f);
            }

            b->SetPosition(RandRange(0.0f, size - wallWidth) - (size - wallWidth) / 2.0f,
                           RandRange(0.0f, size - wallWidth) - (size - wallWidth) / 2.0f);
        }

        camera.position = { 0.0f, 0.0f };
        camera.scale = { 3.f, 3.f };
    }

    static Demo* Create(Game& game)
    {
        return new Mix1000(game);
    }
};

DemoFrame mix_1000{ "1000 Random shapes", Mix1000::Create };

} // namespace muli
