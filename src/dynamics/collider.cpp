#include "muli/collider.h"
#include "muli/aabb_tree.h"
#include "muli/callbacks.h"
#include "muli/world.h"

namespace muli
{

ContactListener defaultListener;

Collider::Collider()
    : OnDestroy{ nullptr }
    , ContactListener{ &defaultListener }
    , next{ nullptr }
    , node{ AABBTree::nullNode }
    , enabled{ true }
{
}

Collider::~Collider()
{
    if (OnDestroy)
    {
        OnDestroy->OnColliderDestroy(this);
    }

    body = nullptr;
    next = nullptr;
}

void Collider::Create(RigidBody* inBody, Shape* inShape, const Transform& tf, float inDensity, const Material& inMaterial)
{
    body = inBody;
    shape = body->world->CloneShape(inShape, tf);
    density = inDensity;
    material = inMaterial;
}

void Collider::Destroy(World* world)
{
    world->FreeShape(shape);
    shape = nullptr;
}

} // namespace muli