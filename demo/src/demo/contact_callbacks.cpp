#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli
{

static bool disable_contact = false;

class ContactCallbacks : public Demo,
                         public ContactListener
{
public:
    ContactCallbacks(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, RigidBody::Type::static_body);

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
                b->SetLinearVelocity(b->GetPosition() * RandRange(0.5f, 0.7f));
            }
        }

        RigidBody* b = world->CreateCapsule(Vec2{ -5, 8 }, Vec2{ 5, 8 }, 0.05f);

        Collider* c = b->GetColliderList();
        c->ContactListener = this;
    }

    virtual void OnContactBegin(Collider* me, Collider* other, Contact* contact) override
    {
        RigidBody* body = other->GetBody();
        contact->SetEnabled(!disable_contact);

        if (body->GetType() != RigidBody::Type::static_body)
        {
            world->BufferDestroy(body);
        }
    }

    virtual void OnContactTouching(Collider* me, Collider* other, Contact* contact) override {}

    virtual void OnContactEnd(Collider* me, Collider* other, Contact* contact) override {}

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get().GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Contact callbacks", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            if (ImGui::Checkbox("Disable contact", &disable_contact))
            {
                game.RestartDemo();
            }
        }
        ImGui::End();
    }

    static Demo* Create(Game& game)
    {
        return new ContactCallbacks(game);
    }
};

DemoFrame contact_callbacks{ "Contact callbacks", ContactCallbacks::Create };

} // namespace muli
