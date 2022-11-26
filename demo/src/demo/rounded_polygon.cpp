#include "demo.h"
#include "game.h"

namespace muli
{

class RoundedPolygon : public Demo
{
public:
    RoundedPolygon(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateBox(100.0f, 0.4f, RigidBody::Type::static_body);

        int rows = 12;
        float size = 0.2f;
        float xGap = 0.3f;
        float yGap = 0.2f;
        float xStart = -(rows - 1) * (size + xGap) / 2.0f;
        float yStart = 1.0f;

        for (int y = 0; y < rows; ++y)
        {
            for (int x = 0; x < rows - y; ++x)
            {
                RigidBody* b = world->CreateRandomConvexPolygon(size, 6, RigidBody::Type::dynamic_body, 0.08f);

                b->SetPosition(xStart + y * (size + xGap) / 2 + x * (size + xGap), yStart + y * (size + yGap));
                b->SetLinearVelocity(b->GetPosition() * LinearRand(0.5f, 0.7f));
                b->userFlag |= UserFlag::RENDER_POLYGON_RADIUS;
            }
        }
    }

    static Demo* Create(Game& game)
    {
        return new RoundedPolygon(game);
    }
};

DemoFrame rounded_polygon{ "Rounded polygon", RoundedPolygon::Create };

} // namespace muli
