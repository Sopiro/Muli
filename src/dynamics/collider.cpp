#include "muli/collider.h"
#include "muli/capsule_shape.h"
#include "muli/circle_shape.h"
#include "muli/polygon_shape.h"

namespace muli
{

Collider::Collider()
    : next{ nullptr }
    , node{ 0 }
{
}

void Collider::Create(
    PredefinedBlockAllocator* allocator, RigidBody* _body, Shape* _shape, float _density, const Material& _material)
{
    body = _body;
    shape = _shape->Clone(allocator);
    density = _density;
    material = _material;
}

void Collider::Destroy(PredefinedBlockAllocator* allocator)
{
    shape->~Shape();

    switch (shape->GetType())
    {
    case Shape::Type::circle:
        allocator->Free(shape, sizeof(CircleShape));
        break;
    case Shape::Type::capsule:
        allocator->Free(shape, sizeof(CapsuleShape));
        break;
    case Shape::Type::polygon:
        allocator->Free(shape, sizeof(PolygonShape));
        break;
    default:
        muliAssert(false);
        break;
    }
}

} // namespace muli