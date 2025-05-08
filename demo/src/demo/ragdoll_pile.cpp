#include "demo.h"

namespace muli
{

extern void CreateRagdoll(World* world, float headX, float headY, float scale);

class RagdollPile : public Demo
{
public:
    float t0 = 0;

    RagdollPile(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, identity, RigidBody::static_body);
        camera.scale.Set(2.0f);
        camera.position.y = (screenBounds.y * 2.0f) / 2;
        t0 = game.GetTime();
    }

    int32 count = 0;

    float x = 0.0f;
    float y = 4.0f;
    float dx = 2.5;
    float dy = 0.2;

    void Step() override
    {
        float t1 = game.GetTime();

        if (count < 100 && t0 + 0.3f < t1)
        {
            CreateRagdoll(world, x, y, 0.5f);
            x += dx;
            y += dy;

            if (x > 5 || x < -5)
            {
                dx = -dx;
            }

            t0 = t1;
            ++count;
        }

        Demo::Step();
    }

    static Demo* Create(Game& game)
    {
        return new RagdollPile(game);
    }
};

static int index = register_demo("Ragdoll pile", RagdollPile::Create, 60);

} // namespace muli
