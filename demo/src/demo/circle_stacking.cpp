#include "demo.h"

namespace muli
{

class CircleStacking : public Demo
{
public:
    CircleStacking(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateBox(100.0f, 0.4f, RigidBody::Type::Static);

        float xStart = -3.0f;
        float yStart = 1.0f;
        float size = 0.3f;
        float gap = 0.3f;

        int rows = 10;

        for (int i = 0; i < rows; ++i)
        {
            for (int j = i; j < rows; ++j)
            {
                Circle* c = world->CreateCircle(size);
                c->SetPosition(xStart + (gap + size * 2) * i, yStart + (gap + size * 2) * j);
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
