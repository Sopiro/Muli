#include "demo.h"
#include "game.h"

namespace muli
{

class Cloth : public Demo
{
public:
    Cloth(Game& game)
        : Demo(game)
    {
        options.drawOutlineOnly = true;

        const uint32 rows = 24;
        const uint32 cols = (uint32)(rows * 1.4f);
        float radius = 0.02f;

        float width = 4.5f;

        float gap = width / rows;

        Circle* circles[rows][cols];

        float yStart = 2.0f;

        for (uint32 j = 0; j < rows; j++)
        {
            for (uint32 i = 0; i < cols; i++)
            {
                Circle* c = world->CreateCircle(radius);

                float x = ((i - (cols - 1) / 2.0f) / (float)cols) * cols * gap;
                float y = (j / (float)rows) * rows * gap + yStart;

                c->SetPosition(x, y);
                c->SetCollisionFilter(CollisionFilter{ 1, (1 << 1), 0xffffffff ^ (1 << 1) });

                circles[j][i] = c;
            }
        }

        for (uint32 j = 0; j < rows; j++)
        {
            for (uint32 i = 0; i < cols; i++)
            {
                Circle* c00 = circles[j][i];

                if (j + 1 < rows)
                {
                    Circle* c10 = circles[j + 1][i];
                    world->CreateDistanceJoint(c00, c10, -1.0f, 1.0f, 0.7f, 1.0f);
                }
                if (i + 1 < cols)
                {
                    Circle* c01 = circles[j][i + 1];
                    world->CreateDistanceJoint(c00, c01, -1.0f, 1.0f, 0.7f, 1.0f);
                }
            }
        }

        RigidBody* tl = circles[rows - 1][0];
        RigidBody* ml = circles[rows - 1][(int32)(cols / 3.0f) - 1];
        RigidBody* mr = circles[rows - 1][(int32)(cols * 2.0f / 3.0f)];
        RigidBody* tr = circles[rows - 1][cols - 1];

        Circle* w1 = world->CreateCircle(0.1f, RigidBody::Type::Static);
        Circle* w2 = world->CreateCircle(0.1f, RigidBody::Type::Static);
        Circle* w3 = world->CreateCircle(0.1f, RigidBody::Type::Static);
        Circle* w4 = world->CreateCircle(0.1f, RigidBody::Type::Static);
        w1->SetPosition(tl->GetPosition() + Vec2{ -gap, gap });
        w2->SetPosition(ml->GetPosition() + Vec2{ 0.0f, gap });
        w3->SetPosition(mr->GetPosition() + Vec2{ 0.0f, gap });
        w4->SetPosition(tr->GetPosition() + Vec2{ gap, gap });

        world->CreateDistanceJoint(w1, tl, -1.0f, 10.0f, 1.0f, tl->GetMass());
        world->CreateDistanceJoint(w2, ml, -1.0f, 10.0f, 1.0f, ml->GetMass());
        world->CreateDistanceJoint(w3, mr, -1.0f, 10.0f, 1.0f, mr->GetMass());
        world->CreateDistanceJoint(w4, tr, -1.0f, 10.0f, 1.0f, tr->GetMass());
    }

    ~Cloth()
    {
        options.drawOutlineOnly = false;
    }

    static Demo* Create(Game& game)
    {
        return new Cloth(game);
    }
};

DemoFrame cloth{ "Cloth", Cloth::Create };

} // namespace muli
