#include "demo.h"

namespace muli
{

class DenseCollision : public Demo
{
public:
    DenseCollision(Game& game)
        : Demo(game)
    {
        settings.VALID_REGION.min.y = -50.0f;
        settings.APPLY_GRAVITY = false;

        float size = 15.0f;
        float r = 0.25f;
        float spread = 10.0f;

        Polygon* b = world->CreateRandomConvexPolygon(spread / 2.0f, 7);
        b->SetPosition(-25.0, 0.0f);
        b->SetLinearVelocity(12.0f, 0.0f);
        b->SetAngularVelocity(1.0f);

        for (int i = 0; i < 500; i++)
        {
            Circle* c = world->CreateCircle(r);
            c->SetPosition(LinearRand(0.0f, spread * 1.414f), LinearRand(0.0f, spread * 0.9f) - spread / 2.0f);
        }

        camera.position = { -5.0f, 0.0f };
        camera.scale = { 6.0f, 6.0f };
    }

    static Demo* Create(Game& game)
    {
        return new DenseCollision(game);
    }
};

DemoFrame dense_collision{ "Dense collision", DenseCollision::Create };

} // namespace muli
