#include "muli/collision.h"
#include "muli/capsule.h"
#include "muli/circle.h"
#include "muli/polygon.h"
#include "muli/polytope.h"
#include "muli/rigidbody.h"
#include "muli/shape.h"

namespace muli
{

static constexpr Vec2 origin{ 0.0f };

static bool detection_function_initialized = false;
DetectionFunction* detection_function_map[Shape::Type::shape_count][Shape::Type::shape_count];
void InitializeDetectionFunctionMap();

/*
 * Returns support point in 'Minkowski Difference' set
 * Minkowski Sum: A ⊕ B = {Pa + Pb| Pa ∈ A, Pb ∈ B}
 * Minkowski Difference : A ⊖ B = {Pa - Pb| Pa ∈ A, Pb ∈ B}
 * CSO stands for Configuration Space Object
 *
 * 'dir' should be normalized
 */
inline SupportPoint CSOSupport(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, const Vec2& dir)
{
    SupportPoint supportPoint;
    supportPoint.pointA.id = a->GetSupport(MulT(tfA.rotation, dir));
    supportPoint.pointB.id = b->GetSupport(MulT(tfB.rotation, -dir));
    supportPoint.pointA.p = Mul(tfA, a->GetVertex(supportPoint.pointA.id));
    supportPoint.pointB.p = Mul(tfB, b->GetVertex(supportPoint.pointB.id));
    supportPoint.point = supportPoint.pointA.p - supportPoint.pointB.p;

    return supportPoint;
}

bool GJK(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, GJKResult* result)
{
    Simplex simplex;

    // Random initial search direction
    Vec2 direction = tfB.position - tfA.position;
    SupportPoint support = CSOSupport(a, tfA, b, tfB, direction);
    simplex.AddVertex(support);

    Vec2 save[max_simplex_vertex_count];
    int32 saveCount;

    for (int32 k = 0; k < gjk_max_iteration; ++k)
    {
        simplex.Save(save, &saveCount);
        simplex.Advance(origin);

        if (simplex.count == 3)
        {
            break;
        }

        direction = simplex.GetSearchDirection();

        // Simplex contains origin
        if (Dot(direction, direction) == 0.0f)
        {
            break;
        }

        support = CSOSupport(a, tfA, b, tfB, direction);

        // Check duplicate vertices
        for (int32 i = 0; i < saveCount; ++i)
        {
            if (save[i] == support.point)
            {
                goto end;
            }
        }

        simplex.AddVertex(support);
    }

end:
    Vec2 closest = simplex.GetClosestPoint();
    float distance = Length(closest);

    result->simplex = simplex;
    result->direction = Normalize(direction);
    result->distance = distance;

    return distance < gjk_tolerance;
}

void EPA(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, const Simplex& simplex, EPAResult* result)
{
    Polytope polytope{ simplex };
    PolytopeEdge edge{ 0, max_value, zero_vec2 };

    for (int32 k = 0; k < epa_max_iteration; ++k)
    {
        edge = polytope.GetClosestEdge();
        Vec2 supportPoint = CSOSupport(a, tfA, b, tfB, edge.normal).point;
        float newDistance = Dot(edge.normal, supportPoint);

        if (Abs(edge.distance - newDistance) > epa_tolerance)
        {
            // Insert the support vertex so that it expands our polytope
            polytope.vertices.Insert(edge.index + 1, supportPoint);
        }
        else
        {
            // We finally reached the closest outer edge!
            break;
        }
    }

    result->contactNormal = edge.normal;
    result->penetrationDepth = edge.distance;
}

static void ClipEdge(Edge* e, const Vec2& p, const Vec2& dir, bool removeClippedPoint)
{
    float d1 = Dot(e->p1.p - p, dir);
    float d2 = Dot(e->p2.p - p, dir);

    if (d1 >= 0 && d2 >= 0)
    {
        return;
    }

    if (d1 < 0)
    {
        if (removeClippedPoint)
        {
            e->p1 = e->p2;
        }
        else
        {
            e->p1.p = e->p1.p + (e->p2.p - e->p1.p) * (-d1 / (Abs(d1) + Abs(d2)));
        }
    }
    else if (d2 < 0)
    {
        if (removeClippedPoint)
        {
            e->p2 = e->p1;
        }
        else
        {
            e->p2.p = e->p2.p + (e->p1.p - e->p2.p) * (-d2 / (Abs(d1) + Abs(d2)));
        }
    }
}

static void FindContactPoints(
    const Vec2& n, const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    Edge edgeA = a->GetFeaturedEdge(tfA, n);
    Edge edgeB = b->GetFeaturedEdge(tfB, -n);

    edgeA.Translate(n * a->GetRadius());
    edgeB.Translate(-n * b->GetRadius());

    Edge* ref = &edgeA; // Reference edge
    Edge* inc = &edgeB; // Incident edge
    manifold->contactNormal = n;
    manifold->featureFlipped = false;

    float aPerpendicularness = Abs(Dot(edgeA.tangent, n));
    float bPerpendicularness = Abs(Dot(edgeB.tangent, n));

    if (bPerpendicularness < aPerpendicularness)
    {
        ref = &edgeB;
        inc = &edgeA;
        manifold->contactNormal = -n;
        manifold->featureFlipped = true;
    }

    ClipEdge(inc, ref->p1.p, ref->tangent, false);
    ClipEdge(inc, ref->p2.p, -ref->tangent, false);
    ClipEdge(inc, ref->p1.p, -manifold->contactNormal, true);

    // To ensure consistent warm starting, the contact point id is always set based on Shape A
    if (inc->GetLength2() <= contact_merge_threshold)
    {
        // If two points are closer than the threshold, merge them into one point
        manifold->contactPoints[0].id = edgeA.p1.id;
        manifold->contactPoints[0].p = inc->p1.p;
        manifold->contactCount = 1;
    }
    else
    {
        manifold->contactPoints[0].id = edgeA.p1.id;
        manifold->contactPoints[0].p = inc->p1.p;
        manifold->contactPoints[1].id = edgeA.p2.id;
        manifold->contactPoints[1].p = inc->p2.p;
        manifold->contactCount = 2;
    }

    manifold->referencePoint = ref->p1;
}

bool CircleVsCircle(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    Vec2 pa = Mul(tfA, a->GetCenter());
    Vec2 pb = Mul(tfB, b->GetCenter());
    Vec2 d = pb - pa;

    float ra = a->GetRadius();
    float rb = b->GetRadius();
    float radii = ra + rb;

    float distance2 = d.Length2();
    if (distance2 > radii * radii || distance2 == 0.0f)
    {
        return false;
    }

    float distance = Sqrt(distance2);
    Vec2 normal = d / distance;

    manifold->contactNormal = normal;
    manifold->contactTangent.Set(-normal.y, normal.x);
    manifold->contactPoints[0].id = 0;
    manifold->contactPoints[0].p = pb - normal * rb;
    manifold->referencePoint.id = 0;
    manifold->referencePoint.p = pa + normal * ra;
    manifold->contactCount = 1;
    manifold->penetrationDepth = radii - distance;
    manifold->featureFlipped = false;

    return true;
}

bool CapsuleVsCircle(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    const Capsule* c = static_cast<const Capsule*>(a);
    Vec2 va = c->GetVertexA();
    Vec2 vb = c->GetVertexB();
    Vec2 l = vb - va;

    Vec2 pb = Mul(tfB, b->GetCenter());
    Vec2 localP = MulT(tfA, pb);

    Vec2 bp = localP - vb;
    Vec2 ap = localP - va;

    float u = Dot(bp, -l);
    float v = Dot(ap, l);

    // Find closest point depending on the Voronoi region
    Vec2 normal;
    float distance;
    int32 index;

    if (v <= 0.0f) // Region A
    {
        normal = ap;
        distance = normal.Normalize();
        index = 0;
    }
    else if (u <= 0.0f) // Region B
    {
        normal = bp;
        distance = normal.Normalize();
        index = 1;
    }
    else // Region AB
    {
        normal = Normalize(Cross(1.0f, l));
        distance = Dot(ap, normal);
        if (distance < 0.0f)
        {
            normal = -normal;
            distance = -distance;
        }
        index = 0;
    }

    float ra = a->GetRadius();
    float rb = b->GetRadius();
    float radii = ra + rb;

    if (distance > radii)
    {
        return false;
    }

    normal = Mul(tfA.rotation, normal);
    Vec2 point = Mul(tfA, (index ? vb : va));

    manifold->contactNormal = normal;
    manifold->contactTangent.Set(-normal.y, normal.x);
    manifold->penetrationDepth = radii - distance;
    manifold->contactPoints[0].id = 0;
    manifold->contactPoints[0].p = pb - normal * rb;
    manifold->referencePoint.id = index;
    manifold->referencePoint.p = point + normal * ra;
    manifold->contactCount = 1;
    manifold->featureFlipped = false;

    return true;
}

bool PolygonVsCircle(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    const Polygon* p = static_cast<const Polygon*>(a);
    const Vec2* vertices = p->GetVertices();
    const Vec2* normals = p->GetNormals();
    int32 vertexCount = p->GetVertexCount();

    float ra = a->GetRadius();
    float rb = b->GetRadius();
    float radii = ra + rb;

    Vec2 pb = Mul(tfB, b->GetCenter());
    Vec2 localP = MulT(tfA, pb);

    int32 index = 0;
    float minSeparation = Dot(normals[index], localP - vertices[index]);

    for (int32 i = 1; i < vertexCount; ++i)
    {
        float separation = Dot(normals[i], localP - vertices[i]);
        if (separation > radii)
        {
            return false;
        }

        if (separation > minSeparation)
        {
            minSeparation = separation;
            index = i;
        }
    }

    // Circle center is inside the polygon
    if (minSeparation < 0.0f)
    {
        Vec2 normal = Mul(tfA.rotation, normals[index]);
        Vec2 point = Mul(tfA, vertices[index]);

        manifold->contactNormal = normal;
        manifold->contactTangent.Set(-normal.y, normal.x);
        manifold->penetrationDepth = radii - minSeparation;
        manifold->contactPoints[0].id = 0;
        manifold->contactPoints[0].p = pb - normal * rb;
        manifold->referencePoint.id = index;
        manifold->referencePoint.p = point + normal * ra;
        manifold->contactCount = 1;
        manifold->featureFlipped = false;

        return true;
    }

    Vec2 v0 = vertices[index];
    Vec2 v1 = vertices[(index + 1) % vertexCount];

    Vec2 v0p = localP - v0;
    Vec2 v1p = localP - v1;

    float u = Dot(v1p, v0 - v1);
    float v = Dot(v0p, v1 - v0);

    Vec2 normal;
    float distance;

    if (v <= 0.0f) // Region v0
    {
        normal = v0p;
        distance = normal.Normalize();
    }
    else if (u <= 0.0f) // Region v1
    {
        normal = v1p;
        distance = normal.Normalize();
        index = (index + 1) % vertexCount;
    }
    else // Inside the region
    {
        normal = normals[index];
        distance = Dot(normal, v0p);
    }

    if (distance > radii)
    {
        return false;
    }

    normal = Mul(tfA.rotation, normal);
    Vec2 point = Mul(tfA, vertices[index]);

    manifold->contactNormal = normal;
    manifold->contactTangent.Set(-normal.y, normal.x);
    manifold->penetrationDepth = radii - distance;
    manifold->contactPoints[0].id = 0;
    manifold->contactPoints[0].p = pb - normal * rb;
    manifold->referencePoint.id = index;
    manifold->referencePoint.p = point + normal * ra;
    manifold->contactCount = 1;
    manifold->featureFlipped = false;

    return true;
}

// This works for all possible shape pairs
bool ConvexVsConvex(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    GJKResult gjkResult;
    bool collide = GJK(a, tfA, b, tfB, &gjkResult);

    Simplex& simplex = gjkResult.simplex;

    float ra = a->GetRadius();
    float rb = b->GetRadius();
    float radii = ra + rb;

    if (collide == false)
    {
        switch (simplex.count)
        {
        case 1: // vertex vs. vertex collision
            if (gjkResult.distance < radii)
            {
                Vec2 normal = Normalize(origin - simplex.vertices[0].point);

                ContactPoint supportA = simplex.vertices[0].pointA;
                ContactPoint supportB = simplex.vertices[0].pointB;
                supportA.p += normal * ra;
                supportB.p -= normal * rb;

                manifold->contactNormal = normal;
                manifold->contactTangent.Set(-normal.y, normal.x);
                manifold->contactPoints[0] = supportB;
                manifold->contactCount = 1;
                manifold->referencePoint = supportA;
                manifold->penetrationDepth = radii - gjkResult.distance;
                manifold->featureFlipped = false;

                return true;
            }
            else
            {
                return false;
            }
        case 2: // vertex vs. edge collision
            if (gjkResult.distance < radii)
            {
                Vec2 normal = Normalize(Cross(1.0f, simplex.vertices[1].point - simplex.vertices[0].point));
                Vec2 k = origin - simplex.vertices[0].point;
                if (Dot(normal, k) < 0)
                {
                    normal = -normal;
                }

                manifold->contactNormal = normal;
                manifold->penetrationDepth = radii - gjkResult.distance;
            }
            else
            {
                return false;
            }
        }
    }
    else
    {
        // Expand to a full simplex if the gjk termination simplex has vertices less than 3
        // We need a full n-simplex to start EPA (actually it's rare case)
        switch (simplex.count)
        {
        case 1:
        {
            SupportPoint support = CSOSupport(a, tfA, b, tfB, Vec2{ 1.0f, 0.0f });
            if (support.point == simplex.vertices[0].point)
            {
                support = CSOSupport(a, tfA, b, tfB, Vec2{ -1.0f, 0.0f });
            }

            simplex.AddVertex(support);
        }

            [[fallthrough]];

        case 2:
        {
            Vec2 normal = Normalize(Cross(1.0f, simplex.vertices[1].point - simplex.vertices[0].point));
            SupportPoint support = CSOSupport(a, tfA, b, tfB, normal);

            if (simplex.vertices[0].point == support.point || simplex.vertices[1].point == support.point)
            {
                simplex.AddVertex(CSOSupport(a, tfA, b, tfB, -normal));
            }
            else
            {
                simplex.AddVertex(support);
            }
        }
        }

        EPAResult epaResult;
        EPA(a, tfA, b, tfB, simplex, &epaResult);

        manifold->contactNormal = epaResult.contactNormal;
        manifold->penetrationDepth = epaResult.penetrationDepth;
    }

    FindContactPoints(manifold->contactNormal, a, tfA, b, tfB, manifold);
    manifold->contactTangent.Set(-manifold->contactNormal.y, manifold->contactNormal.x);

    return true;
}

bool DetectCollision(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    if (detection_function_initialized == false)
    {
        InitializeDetectionFunctionMap();
    }

    static ContactManifold default_manifold;
    if (manifold == nullptr)
    {
        manifold = &default_manifold;
    }

    Shape::Type shapeA = a->GetType();
    Shape::Type shapeB = b->GetType();

    if (shapeB > shapeA)
    {
        muliAssert(detection_function_map[shapeB][shapeA] != nullptr);

        bool collide = detection_function_map[shapeB][shapeA](b, tfB, a, tfA, manifold);
        manifold->featureFlipped = true;

        return collide;
    }
    else
    {
        muliAssert(detection_function_map[shapeA][shapeB] != nullptr);

        return detection_function_map[shapeA][shapeB](a, tfA, b, tfB, manifold);
    }
}

void InitializeDetectionFunctionMap()
{
    if (detection_function_initialized)
    {
        return;
    }

    detection_function_map[Shape::Type::circle][Shape::Type::circle] = &CircleVsCircle;

    detection_function_map[Shape::Type::capsule][Shape::Type::circle] = &CapsuleVsCircle;
    detection_function_map[Shape::Type::capsule][Shape::Type::capsule] = &ConvexVsConvex;

    detection_function_map[Shape::Type::polygon][Shape::Type::circle] = &PolygonVsCircle;
    detection_function_map[Shape::Type::polygon][Shape::Type::capsule] = &ConvexVsConvex;
    detection_function_map[Shape::Type::polygon][Shape::Type::polygon] = &ConvexVsConvex;

    detection_function_initialized = true;
}

} // namespace muli
