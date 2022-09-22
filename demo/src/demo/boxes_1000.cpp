#include "demo.h"

namespace muli
{

class Boxes1000 : public Demo
{
public:
    Boxes1000(Game& game)
        : Demo(game)
    {
        settings.APPLY_GRAVITY = true;

        float size = 15.0f;
        float halfSize = size / 2.0f;
        float wallWidth = 0.4f;
        float wallRadius = wallWidth / 2.0f;

        world->CreateCapsule(Vec2{ -halfSize, -halfSize }, Vec2{ halfSize, -halfSize }, wallRadius, false,
                             RigidBody::Type::Static);
        world->CreateCapsule(Vec2{ halfSize, -halfSize }, Vec2{ halfSize, halfSize }, wallRadius, false, RigidBody::Type::Static);
        world->CreateCapsule(Vec2{ halfSize, halfSize }, Vec2{ -halfSize, halfSize }, wallRadius, false, RigidBody::Type::Static);
        world->CreateCapsule(Vec2{ -halfSize, halfSize }, Vec2{ -halfSize, -halfSize }, wallRadius, false,
                             RigidBody::Type::Static);

        float r = 0.38f;

        for (int i = 0; i < 1000; i++)
        {
            RigidBody* b = world->CreateBox(r);
            b->SetPosition(LinearRand(0.0f, size - wallWidth) - (size - wallWidth) / 2.0f,
                           LinearRand(0.0f, size - wallWidth) - (size - wallWidth) / 2.0f);
        }

        camera.position = { 0.0f, 0.0f };
        camera.scale = { 3.f, 3.f };
    }

    static Demo* Create(Game& game)
    {
        return new Boxes1000(game);
    }
};

DemoFrame boxes_1000{ "1000 Boxes", Boxes1000::Create };

} // namespace muli
