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

        const uint32 rows = 20;
        const uint32 cols = 32;
        float radius = 0.02f;

        float width = 4.0f;

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
                    world->CreateDistanceJoint(c00, c10, -1.0f, 10.0f, 0.7f, c10->GetMass());
                }
                if (i + 1 < cols)
                {
                    Circle* c01 = circles[j][i + 1];
                    world->CreateDistanceJoint(c00, c01, -1.0f, 10.0f, 0.7f, c01->GetMass());
                }
            }
        }

        RigidBody* tl = circles[rows - 1][0];
        RigidBody* tr = circles[rows - 1][cols - 1];

        Circle* w1 = world->CreateCircle(0.1f, RigidBody::Type::Static);
        Circle* w2 = world->CreateCircle(0.1f, RigidBody::Type::Static);
        w1->SetPosition(tl->GetPosition() + Vec2{ -gap, gap });
        w2->SetPosition(tr->GetPosition() + Vec2{ gap, gap });

        world->CreateDistanceJoint(w1, tl, -1.0f, 10.0f, 1.0f, tl->GetMass());
        world->CreateDistanceJoint(w2, tr, -1.0f, 10.0f, 1.0f, tr->GetMass());
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
