#include "demo.h"

namespace muli
{

class ConvexPolygons1000 : public Demo
{
public:
    ConvexPolygons1000(Game& game)
        : Demo(game)
    {
        float size = 15.0f;
        float halfSize = size / 2.0f;
        float wallWidth = 0.4f;
        float wallRadius = wallWidth / 2.0f;

        world->CreateCapsule(Vec2{ -halfSize, -halfSize }, Vec2{ halfSize, -halfSize }, wallRadius, RigidBody::Type::static_body);
        world->CreateCapsule(Vec2{ halfSize, -halfSize }, Vec2{ halfSize, halfSize }, wallRadius, RigidBody::Type::static_body);
        world->CreateCapsule(Vec2{ halfSize, halfSize }, Vec2{ -halfSize, halfSize }, wallRadius, RigidBody::Type::static_body);
        world->CreateCapsule(Vec2{ -halfSize, halfSize }, Vec2{ -halfSize, -halfSize }, wallRadius, RigidBody::Type::static_body);

        float r = 0.27f;
        for (int32 i = 0; i < 1000; ++i)
        {
            RigidBody* b = world->CreateRandomConvexPolygon(r, 7);
            b->SetPosition(
                Rand(0.0f, size - wallWidth) - (size - wallWidth) / 2.0f, Rand(0.0f, size - wallWidth) - (size - wallWidth) / 2.0f
            );
        }

        camera.position = { 0.0f, 0.0f };
        camera.scale = { 3.f, 3.f };
    }

    static Demo* Create(Game& game)
    {
        return new ConvexPolygons1000(game);
    }
};

static int index = register_demo("1000 Convex polygons", ConvexPolygons1000::Create, 17);

} // namespace muli
