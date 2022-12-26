#include "demo.h"
#include "game.h"

namespace muli
{

class Terrain : public Demo
{
public:
    Terrain(Game& game)
        : Demo(game)
    {
        float d = 0.2f;

        auto f = [](float x) { return Sin(x) * 0.5f + 0.8f; };

        RigidBody* b = world->CreateEmptyBody(RigidBody::Type::static_body);
        b->UserFlag |= UserFlag::remove_outline;

        // Capsule terrain
        for (float x0 = -10.0f; x0 < 10.0f; x0 += d)
        {
            float y0 = f(x0);
            float x1 = x0 + d;
            float y1 = f(x1);

            Capsule capsule{ Vec2{ x0, y0 }, Vec2{ x1, y1 }, 0.05f };

            b->CreateCollider(&capsule);
        }

        // Pyramid
        int32 rows = 15;
        float boxSize = 0.3f;
        float xGap = 0.0625f * boxSize / 0.5f;
        float yGap = 0.125f * boxSize / 0.5f;
        float xStart = -(rows - 1.0f) * (boxSize + xGap) / 2.0f;
        float yStart = 3.0f + boxSize / 2.0f + yGap;

        for (int32 y = 0; y < rows; ++y)
        {
            for (int32 x = 0; x < rows - y; ++x)
            {
                b = world->CreateBox(boxSize);
                b->SetPosition(xStart + y * (boxSize + xGap) / 2.0f + x * (boxSize + xGap), yStart + y * (boxSize + yGap));
            }
        }
    }

    static Demo* Create(Game& game)
    {
        return new Terrain(game);
    }
};

DemoFrame terrain{ "Terrain", Terrain::Create };

} // namespace muli
