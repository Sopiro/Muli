#include "demo.h"

namespace muli
{

class Circles1000 : public Demo
{
public:
    Circles1000()
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

        float r = 0.22f;

        for (int i = 0; i < 1000; i++)
        {
            RigidBody* b = world->CreateCircle(r);
            b->SetPosition(LinearRand(0.0f, size - wallWidth) - (size - wallWidth) / 2.0f,
                           LinearRand(0.0f, size - wallWidth) - (size - wallWidth) / 2.0f);
            b->SetRotation(LinearRand(0.0f, SPE_PI * 2.0f));
        }

        camera.position = { 0.0f, 0.0f };
        camera.scale = { 3.f, 3.f };
    }

    static Demo* Create()
    {
        return new Circles1000;
    }
};

DemoFrame circles_1000{ "1000 Circles", Circles1000::Create };

} // namespace muli
