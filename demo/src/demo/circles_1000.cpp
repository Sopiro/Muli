#include "demo.h"

namespace muli
{

class Circles1000 : public Demo
{
public:
    Circles1000(Game& game)
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

        float r = 0.22f;

        for (int32 i = 0; i < 1000; ++i)
        {
            RigidBody* b = world->CreateCircle(r);
            b->SetPosition(RandRange(0.0f, size - wallWidth) - (size - wallWidth) / 2.0f,
                           RandRange(0.0f, size - wallWidth) - (size - wallWidth) / 2.0f);
            b->SetRotation(RandRange(0.0f, pi * 2.0f));
        }

        camera.position = { 0.0f, 0.0f };
        camera.scale = { 3.f, 3.f };
    }

    static Demo* Create(Game& game)
    {
        return new Circles1000(game);
    }
};

DemoFrame circles_1000{ "1000 Circles", Circles1000::Create };

} // namespace muli
