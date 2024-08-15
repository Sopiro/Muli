#include "muli/collider.h"
#include "muli/aabb_tree.h"
#include "muli/callbacks.h"
#include "muli/capsule.h"
#include "muli/circle.h"
#include "muli/polygon.h"

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

void Collider::Create(Allocator* allocator, RigidBody* _body, Shape* _shape, float _density, const Material& _material)
{
    body = _body;
    shape = _shape->Clone(allocator);
    density = _density;
    material = _material;
}

void Collider::Destroy(Allocator* allocator)
{
    shape->~Shape();

    switch (shape->GetType())
    {
    case Shape::Type::circle:
        allocator->Free(shape, sizeof(Circle));
        break;
    case Shape::Type::capsule:
        allocator->Free(shape, sizeof(Capsule));
        break;
    case Shape::Type::polygon:
        allocator->Free(shape, sizeof(Polygon));
        break;
    default:
        MuliAssert(false);
        break;
    }

    shape = nullptr;
}

} // namespace muli