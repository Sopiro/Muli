#include "demo.h"

namespace muli
{

class KinematicBody : public Demo
{
public:
    KinematicBody(Game& game)
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
        // world->CreateCapsule(Vec2{ halfSize, halfSize }, Vec2{ -halfSize, halfSize }, wallRadius, false,
        // RigidBody::Type::Static);
        world->CreateCapsule(Vec2{ -halfSize, halfSize }, Vec2{ -halfSize, -halfSize }, wallRadius, false,
                             RigidBody::Type::Static);

        float r = 0.22f;

        for (int i = 0; i < 500; i++)
        {
            RigidBody* b = world->CreateCircle(r);
            b->SetPosition(LinearRand(0.0f, size - wallWidth) - (size - wallWidth) / 2.0f,
                           LinearRand(0.0f, size - wallWidth) - (size - wallWidth) / 2.0f);
            b->SetRotation(LinearRand(0.0f, SPE_PI * 2.0f));
        }

        RigidBody* k = world->CreateCapsule(size * 0.9f, 0.15f, true, RigidBody::Type::Kinematic);
        // k->SetAngularVelocity(SPE_PI / 2.0f);
        k = world->CreateCapsule(size * 0.9f, 0.15f, false, RigidBody::Type::Kinematic);
        // k->SetAngularVelocity(SPE_PI / 2.0f);

        camera.position = { 0.0f, 0.0f };
        camera.scale = { 3.f, 3.f };
    }

    static Demo* Create(Game& game)
    {
        return new KinematicBody(game);
    }
};

DemoFrame kinematic_body{ "Kinematic body", KinematicBody::Create };

} // namespace muli
