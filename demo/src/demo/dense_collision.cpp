#include "demo.h"

namespace muli
{

class DenseCollision : public Demo
{
public:
    DenseCollision(Game& game)
        : Demo(game)
    {
        settings.world_bounds.min.y = -100.0f;
        settings.apply_gravity = false;

        float size = 15.0f;
        float r = 0.25f;
        float spread = 10.0f;

        RigidBody* b = world->CreateRandomConvexPolygon(spread / 2.0f, 7);
        b->SetPosition(-25.0, 0.0f);
        b->SetLinearVelocity(30.0f, 0.0f);
        b->SetAngularVelocity(1.0f);
        b->SetLinearDamping(0.1f);
        b->SetContinuous(true);

        for (int i = 0; i < 500; ++i)
        {
            RigidBody* c = world->CreateCircle(r);
            c->SetPosition(LinearRand(0.0f, spread * 1.414f), LinearRand(0.0f, spread * 0.9f) - spread / 2.0f);
            c->SetLinearDamping(0.1f);
        }

        camera.position.Set(-5.0f, 0.0f);
        camera.scale.Set(6.0f, 6.0f);
    }

    static Demo* Create(Game& game)
    {
        return new DenseCollision(game);
    }
};

DemoFrame dense_collision{ "Dense collision", DenseCollision::Create };

} // namespace muli
