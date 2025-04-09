#include "demo.h"

namespace muli
{

class RoundedPolygon : public Demo
{
public:
    RoundedPolygon(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, identity, RigidBody::static_body);

        int32 rows = 12;
        float size = 0.2f;
        float xGap = 0.3f;
        float yGap = 0.2f;
        float xStart = -(rows - 1) * (size + xGap) / 2.0f;
        float yStart = 1.0f;

        for (int32 y = 0; y < rows; ++y)
        {
            for (int32 x = 0; x < rows - y; ++x)
            {
                RigidBody* b = world->CreateRandomConvexPolygon(size, 6, identity, RigidBody::dynamic_body, 0.08f);

                b->SetPosition(xStart + y * (size + xGap) / 2 + x * (size + xGap), yStart + y * (size + yGap));
                b->SetLinearVelocity(b->GetPosition() * Rand(0.5f, 0.7f));
                UserFlag::SetFlag(b, UserFlag::render_polygon_radius, true);
            }
        }
    }

    static Demo* Create(Game& game)
    {
        return new RoundedPolygon(game);
    }
};

static int index = register_demo("Rounded polygons", RoundedPolygon::Create, 36);

} // namespace muli
