#include "demo.h"

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
        ground->GetColliderList()->ContactListener = this;

        b = world->CreateEmptyBody();

        Polygon t1({ Vec2{ 0.0f, 0.0f }, Vec2{ 0.0f, 1.0f }, Vec2{ -0.75f, 1.5f } });
        Polygon t2({ Vec2{ 0.0f, 0.0f }, Vec2{ 0.0f, 1.0f }, Vec2{ 0.75f, 1.5f } });
        c1 = b->CreateCollider(&t1);
        c2 = b->CreateCollider(&t2);

        b->SetPosition(0.0f, 5.0f);
        b->SetAngularVelocity(Rand(-10.0f, 10.0f));
    }

    static Demo* Create(Game& game)
    {
        return new Breakable(game);
    }

    virtual void OnContactBegin(Collider* me, Collider* other, Contact* contact) override {}
    virtual void OnContactTouching(Collider* me, Collider* other, Contact* contact) override {}
    virtual void OnContactEnd(Collider* me, Collider* other, Contact* contact) override {}
    virtual void OnPreSolve(Collider* me, Collider* other, Contact* contact) override
    {
        static int32 broke;
        RigidBody* b = other->GetBody();
        if (b->GetColliderCount() == 1 || b->UserData == &broke)
        {
            return;
        }

        Collider* c = b->GetColliderList();
        while (c)
        {
            RigidBody* e = world->CreateEmptyBody();
            e->CreateCollider(c->GetShape(), c->GetDensity(), c->GetMaterial());
            e->SetTransform(b->GetTransform());
            e->SetLinearVelocity(b->GetLinearVelocity());
            e->SetAngularVelocity(b->GetAngularVelocity());

            c = c->GetNext();
        }

        b->UserData = &broke;
        world->BufferDestroy(b);
    }

    virtual void OnPostSolve(Collider* me, Collider* other, Contact* contact) override {}
};

static int index = register_demo("Breakable", Breakable::Create, 45);

} // namespace muli
