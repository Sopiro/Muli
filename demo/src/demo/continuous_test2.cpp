#include "demo.h"
#include "game.h"
#include "muli/time_of_impact.h"

namespace muli
{

class ContinuousTest2 : public Demo
{
public:
    ContinuousTest2(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateBox(100.0f, 0.4f, RigidBody::Type::static_body);

        float start = 0.5f;
        float size = 0.3f;
        float gap = 0.25f;

        RigidBody* b;

        // b = world->CreateCircle(0.1f);
        b = world->CreateBox(0.3f);
        b->SetRotation(LinearRand(-MULI_PI / 2.0f, MULI_PI / 2.0f));
        b->SetPosition(-1.0f, 3.0f);
        b->SetContinuous(true);

        b->SetLinearVelocity(100.0f, 0.0f);

        world->CreateCapsule(Vec2{ 3.6f, 6.0f }, Vec2{ 3.6f, 0.0f }, 0.05f, RigidBody::Type::static_body);
        world->CreateCapsule(Vec2{ 3.6f, 6.0f }, Vec2{ -1.0f, 6.0f }, 0.05f, RigidBody::Type::static_body);

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

    ~ContinuousTest2() {}

    static Demo* Create(Game& game)
    {
        return new ContinuousTest2(game);
    }
};

DemoFrame continuous_test2{ "Continuous test 2", ContinuousTest2::Create };

} // namespace muli
