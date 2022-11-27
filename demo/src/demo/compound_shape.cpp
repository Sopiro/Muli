#include "demo.h"

namespace muli
{

class CompoundShape : public Demo
{
public:
    CompoundShape(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateBox(100.0f, 0.4f, RigidBody::Type::static_body, 0.0f);
        ground->SetPosition(0, -0.2f);

        RigidBody* b = world->CreateEmptyBody();

        Vec2 vertices[4] = { Vec2{ 2.0f, 0.0f }, Vec2{ 3.0f, 0.0f }, Vec2{ 2.0f, 3.0f }, Vec2{ 3.0f, 3.0f } };
        PolygonShape p1{ vertices, 4, true, 0.0f };
        b->AddCollider(&p1, 2.0f);

        b->SetPosition(0, 0);
    }

    static Demo* Create(Game& game)
    {
        return new CompoundShape(game);
    }
};

DemoFrame compound_shape{ "Compound shape", CompoundShape::Create };

} // namespace muli
