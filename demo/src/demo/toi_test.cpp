#include "demo.h"
#include "game.h"
#include "muli/time_of_impact.h"

namespace muli
{

class TOITest : public Demo
{
public:
    RigidBody* b;
    RigidBody* w;

    TOITest(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateBox(100.0f, 0.4f, RigidBody::Type::static_body);

        float start = 0.5f;
        float size = 0.3f;
        float gap = 0.25f;

        // b = world->CreateCircle(0.1f);
        b = world->CreateBox(0.2f);
        b->SetPosition(0, 3);
        b->SetContinuous(true);

        b->SetLinearVelocity(100.0f, 0.0f);

        w = world->CreateCapsule(Vec2{ 3, 5 }, Vec2{ 3, 0 }, 0.05f, RigidBody::static_body);

        options.show_aabb = true;
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

    ~TOITest()
    {
        options.show_aabb = false;
    }

    static Demo* Create(Game& game)
    {
        return new TOITest(game);
    }
};

DemoFrame toi_test{ "TOI test", TOITest::Create };

} // namespace muli
