#pragma once

#include "muli/world.h"

namespace muli
{

RigidBody* CreateRandomConvexPolygon(
    World* world,
    float length,
    int32 vertexCount = 0,
    const Transform& tf = identity,
    RigidBody::Type type = RigidBody::dynamic_body,
    float radius = default_radius,
    float density = default_density
);

RigidBody* CreateRegularPolygon(
    World* world,
    float length,
    int32 vertexCount = 0,
    float initialAngle = 0,
    const Transform& tf = identity,
    RigidBody::Type type = RigidBody::dynamic_body,
    float radius = default_radius,
    float density = default_density
);

} // namespace muli
