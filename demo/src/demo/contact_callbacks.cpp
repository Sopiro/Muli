#include "demo.h"
#include "window.h"

namespace muli
{

static bool disable_contacts = false;

class ContactCallbacks : public Demo,
                         public ContactListener
{
public:
    ContactCallbacks(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, identity, RigidBody::Type::static_body);

        int32 rows = 12;
        float size = 0.25f;
        float xGap = 0.2f;
        float yGap = 0.15f;
        float xStart = -(rows - 1) * (size + xGap) / 2.0f;
        float yStart = 1.0f;

        for (int32 y = 0; y < rows; ++y)
        {
            for (int32 x = 0; x < rows - y; ++x)
            {
                RigidBody* b = world->CreateRandomConvexPolygon(size, 6);
                b->SetPosition(xStart + y * (size + xGap) / 2 + x * (size + xGap), yStart + y * (size + yGap));
                b->SetLinearVelocity(b->GetPosition() * Rand(0.5f, 0.7f));
            }
        }

        RigidBody* b = world->CreateCapsule(Vec2{ -5, 8 }, Vec2{ 5, 8 }, 0.05f);

        Collider* c = b->GetColliderList();
        c->ContactListener = this;
    }

    virtual void OnContactBegin(Collider* me, Collider* other, Contact* contact) override
    {
        RigidBody* body = other->GetBody();
        contact->SetEnabled(!disable_contacts);

        if (body->GetType() != RigidBody::Type::static_body)
        {
            world->BufferDestroy(body);
        }
    }

    virtual void OnContactTouching(Collider* me, Collider* other, Contact* contact) override {}
    virtual void OnContactEnd(Collider* me, Collider* other, Contact* contact) override {}
    virtual void OnPreSolve(Collider* me, Collider* other, Contact* contact) override {}

    struct ContactRecord
    {
        Vec2 pos = { max_value, max_value };
        Vec2 normal = Vec2::zero;
        float impulse = 0.0f;
    };
    ContactRecord rec[100];
    int32 p = 0;

    virtual void OnPostSolve(Collider* me, Collider* other, Contact* contact) override
    {
        const ContactManifold& m = contact->GetContactManifold();
        for (int32 i = 0; i < m.contactCount; ++i)
        {
            ContactRecord r;
            r.pos = m.contactPoints[i].p;
            r.normal = m.contactNormal;
            r.impulse = contact->GetNormalImpulse(i);
            rec[p++] = r;
            p = p % 100;
        }
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Contact callbacks", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            if (ImGui::Checkbox("Disable contacts", &disable_contacts))
            {
                game.RestartDemo();
            }
        }
        ImGui::End();

        for (int32 i = 0; i < 100; ++i)
        {
            ContactRecord& r = rec[i];

            Vec2 t = Cross(1.0f, r.normal);

            Vec2 p1 = r.pos;
            Vec2 p2 = p1 + r.normal * r.impulse * 0.03f;
            Vec2 t0 = p2 - r.normal * 0.035f;
            Vec2 t1 = t0 + t * 0.02f;
            Vec2 t2 = t0 - t * 0.02f;

            renderer.DrawPoint(p1);
            renderer.DrawLine(p1, p2);
            renderer.DrawLine(p2, t1);
            renderer.DrawLine(p2, t2);
        }
    }

    static Demo* Create(Game& game)
    {
        return new ContactCallbacks(game);
    }
};

static int index = register_demo("Contact callbacks", ContactCallbacks::Create, 43);

} // namespace muli
