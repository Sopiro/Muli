#include "demo.h"

namespace muli
{

class CompoundShape : public Demo
{
public:
    CompoundShape(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, identity, RigidBody::static_body);

        RigidBody* b = world->CreateEmptyBody();

        float adjust = 0.7f;
        float radius = 0.1f;
        // 우
        b->CreateCircleCollider(0.5f, Vec2{ -3.0f + adjust, 3.0f });
        b->CreateCapsuleCollider(Vec2{ -3.9f + adjust, 2.0f }, Vec2{ -2.1f + adjust, 2.0f }, radius);
        b->CreateCapsuleCollider(Vec2{ -3.0f + adjust, 2.0f }, Vec2{ -3.0f + adjust, 1.0f }, radius);

        // 왁
        b->CreateCircleCollider(0.5f, Vec2{ 0.0f, 3.0f });
        b->CreateCapsuleCollider(Vec2{ -0.6f, 2.0f }, Vec2{ 0.6f, 2.0f }, radius);
        b->CreateCapsuleCollider(Vec2{ 0.0f, 2.3f }, Vec2{ 0.0f, 2.0f }, radius);
        b->CreateCapsuleCollider(Vec2{ 1.0f, 3.0f }, Vec2{ 1.0f, 2.0f }, radius);
        b->CreateCapsuleCollider(Vec2{ 1.0f, 2.5f }, Vec2{ 1.5f, 2.5f }, radius);
        b->CreateCapsuleCollider(Vec2{ -0.2f, 1.5f }, Vec2{ 1.0f, 1.5f }, radius);
        b->CreateCapsuleCollider(Vec2{ 1.0f, 1.5f }, Vec2{ 1.0f, 1.0f }, radius);

        // 굳
        b->CreateCapsuleCollider(Vec2{ 3.1f - adjust, 3.4f }, Vec2{ 4.4f - adjust, 3.4f }, radius);
        b->CreateCapsuleCollider(Vec2{ 4.4f - adjust, 3.4f }, Vec2{ 4.4f - adjust, 3.0f }, radius);
        b->CreateCapsuleCollider(Vec2{ 3.0f - adjust, 2.5f }, Vec2{ 4.5f - adjust, 2.5f }, radius);
        b->CreateCapsuleCollider(Vec2{ 3.75f - adjust, 2.5f }, Vec2{ 3.75f - adjust, 2.0f }, radius);
        b->CreateCapsuleCollider(Vec2{ 3.25f - adjust, 1.5f }, Vec2{ 4.25f - adjust, 1.5f }, radius);
        b->CreateCapsuleCollider(Vec2{ 3.25f - adjust, 1.5f }, Vec2{ 3.25f - adjust, 1.0f }, radius);
        b->CreateCapsuleCollider(Vec2{ 3.25f - adjust, 1.0f }, Vec2{ 4.25f - adjust, 1.0f }, radius);

        b->SetPosition(0.0f, 3.0f);

        for (int32 i = 0; i < 100; ++i)
        {
            float w = Rand(0.1f, 0.4f);
            float h = Rand(0.1f, 0.4f);
            float r = Rand(0.02f, 0.08f);

            Vec2 pos = RandVec2(Vec2{ -5.0f }, Vec2{ 5.0f });
            pos.y += 15.0f;

            float angle = Rand(0.0f, pi);

            b = world->CreateBox(w, h, identity, RigidBody::dynamic_body, r);
            b->SetPosition(pos);
            b->SetRotation(angle);
            UserFlag::SetFlag(b, UserFlag::render_polygon_radius, true);
        }
    }

    static Demo* Create(Game& game)
    {
        return new CompoundShape(game);
    }
};

static int index = register_demo("Compound shapes", CompoundShape::Create, 42);

} // namespace muli
