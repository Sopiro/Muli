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

constexpr Material default_material{ DEFAULT_FRICTION, DEFAULT_RESTITUTION, DEFAULT_SURFACESPEED, CollisionFilter{} };

} // namespace muli
