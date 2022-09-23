#include "demo.h"

namespace muli
{

class Pyramid : public Demo
{
public:
    Pyramid(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateBox(100.0f, 0.4f, RigidBody::Type::Static);

        int32 rows = 15;
        float boxSize = 0.4f;
        float xGap = 0.0625f * boxSize / 0.5f;
        float yGap = 0.125f * boxSize / 0.5f;
        float xStart = -(rows - 1.0f) * (boxSize + xGap) / 2.0f;
        float yStart = 0.2f + boxSize / 2.0f + yGap;

        for (int y = 0; y < rows; y++)
        {
            for (int x = 0; x < rows - y; x++)
            {
                RigidBody* b = world->CreateBox(boxSize);
                b->SetPosition(xStart + y * (boxSize + xGap) / 2.0f + x * (boxSize + xGap), yStart + y * (boxSize + yGap));
            }
        }
    }

    static Demo* Create(Game& game)
    {
        return new Pyramid(game);
    }
};

DemoFrame pyramid{ "Pyramid", Pyramid::Create };

} // namespace muli
