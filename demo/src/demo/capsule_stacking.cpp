#include "demo.h"

namespace muli
{

class CapsuleStacking : public Demo
{
public:
    CapsuleStacking(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, RigidBody::Type::static_body);

        float xStart = -3.5f;
        float yStart = 1.0f;
        float size = 0.3f;
        float gap = 0.3f;

        int32 rows = 11;

        for (int32 i = 0; i < rows; ++i)
        {
            for (int32 j = 0; j < rows; ++j)
            {
                RigidBody* c = world->CreateCapsule(size, size / 2.0f);
                c->SetPosition(xStart + (gap + size * 2) * i, yStart + (gap + size * 2) * j);
                int32 k = Abs(rows / 2 - i) + 2;
                if (j % k == 0)
                {
                    c->SetRotation(pi / 2.0f);
                }
            }
        }
    }

    static Demo* Create(Game& game)
    {
        return new CapsuleStacking(game);
    }
};

DemoFrame capsule_stacking{ "Capsule stacking", CapsuleStacking::Create };

} // namespace muli
