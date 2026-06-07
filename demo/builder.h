#pragma once

#include "muli/world.h"

namespace muli
{

Body* CreateRandomConvexPolygon(
    World* world,
    float length,
    int32 vertexCount = 0,
    const Transform& tf = identity,
    Body::Type type = Body::dynamic_body,
    float radius = default_radius,
    float density = default_density
);

Body* CreateRegularPolygon(
    World* world,
    float length,
    int32 vertexCount = 0,
    float initialAngle = 0,
    const Transform& tf = identity,
    Body::Type type = Body::dynamic_body,
    float radius = default_radius,
    float density = default_density
);

} // namespace muli
