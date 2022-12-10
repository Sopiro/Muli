#include "demo.h"
#include "game.h"

namespace muli
{

class TOITest : public Demo
{
public:
    RigidBody* b;

    TOITest(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateBox(100.0f, 0.4f, RigidBody::Type::static_body);

        float start = 0.5f;
        float size = 0.3f;
        float gap = 0.25f;

        b = world->CreateBox(size);
        b->SetContinuous(true);

        b = world->CreateCircle(size / 1.414f);
        b->SetPosition(3, 3);
        b->SetContinuous(true);
    }

    void Render() override
    {
        Sweep s = b->GetSweep();

        std::vector<Vec2>& pl = game.GetPointList();
        std::vector<Vec2>& ll = game.GetLineList();

        pl.push_back(s.c0);
        pl.push_back(s.c);

        ll.push_back(s.c0);
        ll.push_back(s.c);
    }

    static Demo* Create(Game& game)
    {
        return new TOITest(game);
    }
};

DemoFrame toi_test{ "TOI test", TOITest::Create };

} // namespace muli
