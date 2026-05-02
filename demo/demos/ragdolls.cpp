#include "demo.h"
#include "ragdoll.h"

namespace muli
{

class Ragdolls : public Demo
{
public:
    Ragdolls(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, identity, RigidBody::static_body);

        CreateRagdoll(world, { 0.0f, 5.0f }, 1.0f, 1);

        RigidBody* c = world->CreateCircle(0.6f);
        float r = Rand(0.0f, pi);
        Vec2 p{ Cos(r), Sin(r) };
        p *= 8.0f;

        c->SetLinearVelocity(-p * Rand(4.0f, 8.0f) + Vec2{ 0.0f, Rand(5.0f, 15.0f) });
        p.y += 0.5f;
        c->SetPosition(p);

        camera.scale.Set(1.5f);
        camera.position.y = (screenBounds.y * 1.5f) / 2;
    }

    void Step()
    {
        Demo::Step();
    }

    static Demo* Create(Game& game)
    {
        return new Ragdolls(game);
    }
};

static int index = register_demo("Ragdolls", Ragdolls::Create, 59);

} // namespace muli
