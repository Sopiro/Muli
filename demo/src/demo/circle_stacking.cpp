#include "demo.h"

namespace muli
{

class CircleStacking : public Demo
{
public:
    CircleStacking(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, RigidBody::Type::static_body);

        float xStart = -3.0f;
        float yStart = 1.0f;
        float size = 0.3f;
        float gap = 0.3f;

        int32 rows = 10;

        for (int32 i = 0; i < rows; ++i)
        {
            for (int32 j = i; j < rows; ++j)
            {
                RigidBody* c = world->CreateCircle(size);
                c->SetPosition(xStart + (gap + size * 2.0f) * i, yStart + (gap + size * 2.0f) * j);
            }
        }
    }

    static Demo* Create(Game& game)
    {
        return new CircleStacking(game);
    }
};

DemoFrame circle_stacking{ "Circle stacking", CircleStacking::Create };

} // namespace muli
