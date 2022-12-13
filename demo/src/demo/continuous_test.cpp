#include "demo.h"
#include "game.h"
#include "muli/time_of_impact.h"

namespace muli
{

class ContinuousTest : public Demo
{
public:
    ContinuousTest(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateBox(100.0f, 0.4f, RigidBody::Type::static_body);

        float start = 0.5f;
        float size = 0.3f;
        float gap = 0.25f;

        world->CreateCapsule(Vec2{ 0.0f, 2.0f }, Vec2{ 0.0f, 1.5f }, 0.1f, RigidBody::Type::static_body);

        RigidBody* b;
        // b = world->CreateBox(0.2f);
        // b->SetPosition(-3, 1);
        // b->SetContinuous(true);
        // b->SetLinearVelocity(100.0f, 0.0f);

        b = world->CreateCapsule(2.8f, 0.05f, true);
        b->SetPosition(LinearRand(-0.5f, 0.5f), 7.2f);
        b->SetLinearVelocity(0.0f, -100.0f);
        b->SetAngularVelocity(LinearRand(-20.0f, 20.0f));
        b->SetContinuous(true);

        settings.continuous = true;
    }

    void Render() override
    {
        std::vector<Vec2>& pl = game.GetPointList();
        std::vector<Vec2>& ll = game.GetLineList();

        for (RigidBody* b = world->GetBodyList(); b; b = b->GetNext())
        {
            if (!b->IsContinuous())
            {
                continue;
            }

            const Sweep& s = b->GetSweep();

            pl.push_back(s.c0);
            pl.push_back(s.c);

            ll.push_back(s.c0);
            ll.push_back(s.c);
        }
    }

    ~ContinuousTest() {}

    static Demo* Create(Game& game)
    {
        return new ContinuousTest(game);
    }
};

DemoFrame continuous_test{ "Continuous test", ContinuousTest::Create };

} // namespace muli
