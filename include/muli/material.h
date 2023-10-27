#pragma once

#include "collision_filter.h"
#include "settings.h"

namespace muli
{

struct Material
{
    float friction = default_friction;
    float restitution = default_restitution;
    float restitutionTreshold = default_restitution_treshold;
    float surfaceSpeed = default_surface_speed;
};

constexpr Material default_material{};

// Material properties mixing rules

inline float MixFriction(float f1, float f2)
{
    return Sqrt(f1 * f2);
}

inline float MixRestitution(float r1, float r2)
{
    return Max(r1, r2);
}

inline float MixRestitutionTreshold(float t1, float t2)
{
    return Min(t1, t2);
}

} // namespace muli
