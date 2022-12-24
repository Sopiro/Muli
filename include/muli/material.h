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
    CollisionFilter filter{};
};

constexpr Material default_material{};

} // namespace muli
