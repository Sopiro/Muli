#include "demo.h"

namespace muli
{

class Restitutions : public Demo
{
public:
    Restitutions()
    {
        settings.APPLY_GRAVITY = true;
        RigidBody* ground = world->CreateBox(100.0f, 0.4f, RigidBody::Type::Static);

        int count = 11;
        float gap = 0.5f;
        float size = 0.3f;

        float xStart = -(count - 1) / 2 * gap;
        float yStart = 6.0f;

        bool r = LinearRand(0.0f, 1.0f) > 0.5f;

        RigidBody* b;
        for (int i = 0; i < count; i++)
        {
            if (r)
                b = world->CreateBox(size);
            else
                b = world->CreateCircle(size / 2.0f);
            b->SetPosition(xStart + gap * i, yStart);
            float attenuation = (count - i) / (float)count;
            b->SetRestitution(1.0f - attenuation * attenuation);
        }
    }

    static Demo* Create()
    {
        return new Restitutions;
    }
};

DemoFrame restitutions{ "Restitutions", Restitutions::Create };

} // namespace muli
