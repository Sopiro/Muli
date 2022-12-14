#pragma once

#include "collision_filter.h"
#include "settings.h"

namespace muli
{

struct Material
{
    float friction;
    float restitution;
    float surfaceSpeed;
    CollisionFilter filter;
};

constexpr Material default_material{ default_friction, default_restitution, default_surface_speed, CollisionFilter{} };

} // namespace muli
