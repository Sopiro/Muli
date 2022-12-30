#include "demo.h"
#include "game.h"

namespace muli
{

class Breakable : public Demo,
                  public ContactListener
{
    RigidBody* b;
    Collider* c1;
    Collider* c2;

public:
    Breakable(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, RigidBody::Type::static_body);

        b = world->CreateEmptyBody();

        Polygon t1({ Vec2{ 0.0f, 0.0f }, Vec2{ 0.0f, 1.0f }, Vec2{ -0.75f, 1.5f } });
        Polygon t2({ Vec2{ 0.0f, 0.0f }, Vec2{ 0.0f, 1.0f }, Vec2{ 0.75f, 1.5f } });
        c1 = b->CreateCollider(&t1);
        c2 = b->CreateCollider(&t2);

        c1->ContactListener = this;
        c2->ContactListener = this;

        b->SetPosition(0.0f, 5.0f);
        b->SetAngularVelocity(LinearRand(-10.0f, 10.0f));
    }

    static Demo* Create(Game& game)
    {
        return new Breakable(game);
    }

    bool broke = false;

    virtual void OnContactBegin(Collider* me, Collider* other, Contact* contact) override {}

    virtual void OnContactTouching(Collider* me, Collider* other, Contact* contact) override
    {
        if (broke)
        {
            return;
        }

        Polygon t1({ Vec2{ 0.0f, 0.0f }, Vec2{ 0.0f, 1.0f }, Vec2{ -0.75f, 1.5f } });
        Polygon t2({ Vec2{ 0.0f, 0.0f }, Vec2{ 0.0f, 1.0f }, Vec2{ 0.75f, 1.5f } });

        RigidBody* b1 = world->CreateEmptyBody();
        RigidBody* b2 = world->CreateEmptyBody();
        b1->CreateCollider(&t1);
        b2->CreateCollider(&t2);

        b1->SetTransform(b->GetTransform());
        b2->SetTransform(b->GetTransform());
        b1->SetLinearVelocity(b->GetLinearVelocity());
        b1->SetAngularVelocity(b->GetAngularVelocity());
        b2->SetLinearVelocity(b->GetLinearVelocity());
        b2->SetAngularVelocity(b->GetAngularVelocity());

        world->BufferDestroy(b);

        game.RegisterRenderBody(b1);
        game.RegisterRenderBody(b2);

        broke = true;
    }

    virtual void OnContactEnd(Collider* me, Collider* other, Contact* contact) override {}
};

DemoFrame breakable{ "Breakable", Breakable::Create };

} // namespace muli
