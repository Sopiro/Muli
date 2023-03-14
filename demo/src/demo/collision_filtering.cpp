#include "demo.h"
#include "game.h"

namespace muli
{

class CollisionFiltering : public Demo
{
public:
    uint32 group = 1;
    uint32 filter0 = 1; // Default filter
    uint32 filter1 = 1 << 1;
    uint32 filter2 = 1 << 2;
    uint32 filter3 = 1 << 3;

    CollisionFiltering(Game& game)
        : Demo(game)
    {
        auto f = [](float x) { return -0.15f * x + 4.0f; };

        RigidBody* w1 =
            world->CreateCapsule(Vec2{ -6.0f, f(-6.0f) }, Vec2{ -2.5f, f(-2.5f) }, 0.05f, RigidBody::Type::static_body);
        RigidBody* w2 =
            world->CreateCapsule(Vec2{ -1.5f, f(-1.5f) }, Vec2{ -0.5f, f(-0.5f) }, 0.05f, RigidBody::Type::static_body);
        RigidBody* w3 = world->CreateCapsule(Vec2{ 0.5f, f(0.5f) }, Vec2{ 1.5f, f(1.5f) }, 0.05f, RigidBody::Type::static_body);
        RigidBody* w4 = world->CreateCapsule(Vec2{ 2.5f, f(2.5f) }, Vec2{ 6.0f, f(6.0f) }, 0.05f, RigidBody::Type::static_body);

        RigidBody* h1 = world->CreateCapsule(Vec2{ -2.5f, 1.0f }, Vec2{ -1.5f, 1.0f }, 0.05f, RigidBody::Type::static_body);
        RigidBody* v1 = world->CreateCapsule(Vec2{ -2.5f, f(-2.5f) }, Vec2{ -2.5f, 1.0f }, 0.05f, RigidBody::Type::static_body);
        RigidBody* v2 = world->CreateCapsule(Vec2{ -1.5f, f(-1.5f) }, Vec2{ -1.5f, 1.0f }, 0.05f, RigidBody::Type::static_body);

        RigidBody* h2 = world->CreateCapsule(Vec2{ -0.5f, 1.0f }, Vec2{ 0.5f, 1.0f }, 0.05f, RigidBody::Type::static_body);
        RigidBody* v3 = world->CreateCapsule(Vec2{ -0.5f, f(-0.5f) }, Vec2{ -0.5f, 1.0f }, 0.05f, RigidBody::Type::static_body);
        RigidBody* v4 = world->CreateCapsule(Vec2{ 0.5f, f(0.5f) }, Vec2{ 0.5f, 1.0f }, 0.05f, RigidBody::Type::static_body);

        RigidBody* h3 = world->CreateCapsule(Vec2{ 1.5f, 1.0f }, Vec2{ 2.5f, 1.0f }, 0.05f, RigidBody::Type::static_body);
        RigidBody* v5 = world->CreateCapsule(Vec2{ 1.5f, f(1.5f) }, Vec2{ 1.5f, 1.0f }, 0.05f, RigidBody::Type::static_body);
        RigidBody* v6 = world->CreateCapsule(Vec2{ 2.5f, f(2.5f) }, Vec2{ 2.5f, 1.0f }, 0.05f, RigidBody::Type::static_body);

        RigidBody* a =
            world->CreateCapsule(Vec2{ -2.5f, f(-2.5f) }, Vec2{ -1.5f, f(-1.5f) }, 0.05f, RigidBody::Type::static_body);
        CollisionFilter f1 = CollisionFilter{ group, filter1, filter0 | filter2 | filter3 };
        a->SetCollisionFilter(f1);

        RigidBody* b = world->CreateCapsule(Vec2{ -0.5f, f(-0.5f) }, Vec2{ 0.5f, f(0.5f) }, 0.05f, RigidBody::Type::static_body);
        CollisionFilter f2 = CollisionFilter{ group, filter2, filter0 | filter1 | filter3 };
        b->SetCollisionFilter(f2);

        RigidBody* c = world->CreateCapsule(Vec2{ 1.5f, f(1.5f) }, Vec2{ 2.5f, f(2.5f) }, 0.05f, RigidBody::Type::static_body);
        CollisionFilter f3 = CollisionFilter{ group, filter3, filter0 | filter1 | filter2 };
        c->SetCollisionFilter(f3);

        camera.position.x -= 1;
    }

    float t = 0.0f;

    void Step() override
    {
        Demo::Step();

        if (game.GetTime() > t + 0.8f)
        {
            RigidBody* c = world->CreateCircle(0.15f);
            c->SetPosition(-4.0f, 6.0f);

            float r = LinearRand(0.0f, 3.0f);

            if (r < 1.0f)
            {
                c->SetCollisionFilter(CollisionFilter{ group, filter1, filter0 | filter1 | filter2 | filter3 });
            }
            else if (r < 2.0f)
            {
                c->SetCollisionFilter(CollisionFilter{ group, filter2, filter0 | filter1 | filter2 | filter3 });
            }
            else
            {
                c->SetCollisionFilter(CollisionFilter{ group, filter3, filter0 | filter1 | filter2 | filter3 });
            }

            t = game.GetTime();
        }
    }

    static Demo* Create(Game& game)
    {
        return new CollisionFiltering(game);
    }
};

DemoFrame collision_filtering{ "Collision filtering", CollisionFiltering::Create };

} // namespace muli
