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

        RigidBody* b = world->CreateEmptyBody();

        float adjust = 0.7f;

        // 우
        b->CreateCircleCollider(0.5f, Vec2{ -3.0f + adjust, 3.0f });
        b->CreateCapsuleCollider(Vec2{ -3.9f + adjust, 2.0f }, Vec2{ -2.1f + adjust, 2.0f }, 0.1f);
        b->CreateCapsuleCollider(Vec2{ -3.0f + adjust, 2.0f }, Vec2{ -3.0f + adjust, 1.0f }, 0.1f);

        // 왁
        b->CreateCircleCollider(0.5f, Vec2{ 0.0f, 3.0f });
        b->CreateCapsuleCollider(Vec2{ -0.6f, 2.0f }, Vec2{ 0.6f, 2.0f }, 0.1f);
        b->CreateCapsuleCollider(Vec2{ 0.0f, 2.3f }, Vec2{ 0.0f, 2.0f }, 0.1f);
        b->CreateCapsuleCollider(Vec2{ 1.0f, 3.0f }, Vec2{ 1.0f, 2.0f }, 0.1f);
        b->CreateCapsuleCollider(Vec2{ 1.0f, 2.5f }, Vec2{ 1.5f, 2.5f }, 0.1f);
        b->CreateCapsuleCollider(Vec2{ -0.2f, 1.5f }, Vec2{ 1.0f, 1.5f }, 0.1f);
        b->CreateCapsuleCollider(Vec2{ 1.0f, 1.5f }, Vec2{ 1.0f, 1.0f }, 0.1f);

        // 굳
        b->CreateCapsuleCollider(Vec2{ 3.1f - adjust, 3.4f }, Vec2{ 4.4f - adjust, 3.4f }, 0.1f);
        b->CreateCapsuleCollider(Vec2{ 4.4f - adjust, 3.4f }, Vec2{ 4.4f - adjust, 3.0f }, 0.1f);
        b->CreateCapsuleCollider(Vec2{ 3.0f - adjust, 2.5f }, Vec2{ 4.5f - adjust, 2.5f }, 0.1f);
        b->CreateCapsuleCollider(Vec2{ 3.75f - adjust, 2.5f }, Vec2{ 3.75f - adjust, 2.0f }, 0.1f);
        b->CreateCapsuleCollider(Vec2{ 3.25f - adjust, 1.5f }, Vec2{ 4.25f - adjust, 1.5f }, 0.1f);
        b->CreateCapsuleCollider(Vec2{ 3.25f - adjust, 1.5f }, Vec2{ 3.25f - adjust, 1.0f }, 0.1f);
        b->CreateCapsuleCollider(Vec2{ 3.25f - adjust, 1.0f }, Vec2{ 4.25f - adjust, 1.0f }, 0.1f);

        b->SetPosition(0.0f, 3.0f);

        for (int32 i = 0; i < 100; ++i)
        {
            float w = LinearRand(0.1f, 0.4f);
            float h = LinearRand(0.1f, 0.4f);
            float r = LinearRand(0.02f, 0.08f);

            Vec2 pos = LinearRand(Vec2{ -5.0f }, Vec2{ 5.0f });
            pos.y += 15.0f;

            float angle = LinearRand(0.0f, MULI_PI);

            b = world->CreateBox(w, h, RigidBody::Type::dynamic_body, r);
            b->SetPosition(pos);
            b->SetRotation(angle);
            b->UserFlag |= UserFlag::render_polygon_radius;
        }
    }

    static Demo* Create(Game& game)
    {
        return new CompoundShape(game);
    }
};

DemoFrame compound_shape{ "Compound shapes", CompoundShape::Create };

} // namespace muli
