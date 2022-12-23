#include "demo.h"

namespace muli
{

class Restitutions : public Demo
{
public:
    Restitutions(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateBox(100.0f, 0.4f, RigidBody::Type::static_body);

        int32 count = 11;
        float gap = 0.5f;
        float size = 0.3f;

        float xStart = -(count - 1) / 2 * gap;
        float yStart = 6.0f;

        bool r = LinearRand(0.0f, 1.0f) > 0.5f;

        RigidBody* b;
        for (int32 i = 0; i < count; ++i)
        {
            b = r ? world->CreateBox(size) : world->CreateCircle(size / 2.0f);
            b->SetPosition(xStart + gap * i, yStart);
            float attenuation = (count - i) / (float)count;
            b->SetRestitution(1.0f - attenuation * attenuation);
        }
    }

    static Demo* Create(Game& game)
    {
        return new Restitutions(game);
    }
};

DemoFrame restitutions{ "Restitutions", Restitutions::Create };

} // namespace muli
