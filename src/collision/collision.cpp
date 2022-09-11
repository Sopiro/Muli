#include "spe/collision.h"
#include "spe/box.h"
#include "spe/circle.h"
#include "spe/edge.h"
#include "spe/polygon.h"
#include "spe/polytope.h"
#include "spe/rigidbody.h"
#include "spe/simplex.h"

#define APPLY_AXIS_WEIGHT 0

namespace spe
{
static constexpr Vec2 weightAxis{ 0.0f, 1.0f };

struct SupportResult
{
    Vec2 vertex;
    int32_t index;
};

// Returns the fardest vertex in the 'dir' direction
// 'dir' should be normalized and a local vector
static SupportResult support(RigidBody* b, Vec2 dir)
{
    RigidBody::Shape shape = b->GetShape();
    if (shape == RigidBody::Shape::ShapePolygon)
    {
        Polygon* p = static_cast<Polygon*>(b);
        const std::vector<Vec2>& vertices = p->GetVertices();

        int32_t idx = 0;
        float maxValue = dot(dir, vertices[idx]);

        for (int32_t i = 1; i < vertices.size(); i++)
        {
            float value = dot(dir, vertices[i]);
            if (value > maxValue)
            {
                idx = i;
                maxValue = value;
            }
        }

        return { vertices[idx], idx };
    }
    else if (shape == RigidBody::Shape::ShapeCircle)
    {
        Circle* c = static_cast<Circle*>(b);

        return { dir * c->GetRadius(), -1 };
    }
    else
    {
        throw std::exception("Not a supported shape");
    }
}

/*
 * Returns support point in 'Minkowski Difference' set
 * Minkowski Sum: A ⊕ B = {Pa + Pb| Pa ∈ A, Pb ∈ B}
 * Minkowski Difference : A ⊖ B = {Pa - Pb| Pa ∈ A, Pb ∈ B}
 * CSO stands for Configuration Space Object
 */
//'dir' should be normalized
static Vec2 cso_support(RigidBody* b1, RigidBody* b2, Vec2 dir)
{
    const Vec2 localDirP1 = mul_t(b1->GetRotation(), dir);
    const Vec2 localDirP2 = mul_t(b2->GetRotation(), -dir);

    const Vec2 supportP1 = b1->GetTransform() * support(b1, localDirP1).vertex;
    const Vec2 supportP2 = b2->GetTransform() * support(b2, localDirP2).vertex;

    return supportP1 - supportP2;
}

struct GJKResult
{
    bool collide;
    Simplex simplex;
};

static GJKResult gjk(RigidBody* b1, RigidBody* b2, bool earlyReturn = true)
{
    const Vec2 origin{ 0.0f };
    Vec2 dir(1.0f, 0.0f); // Random initial direction

    bool collide = false;
    Simplex simplex{};

    Vec2 supportPoint = cso_support(b1, b2, dir);
    simplex.AddVertex(supportPoint);

    for (uint32_t k = 0; k < GJK_MAX_ITERATION; k++)
    {
        ClosestResult closest = simplex.GetClosest(origin);

        if (spe::dist2(closest.point, origin) < GJK_TOLERANCE)
        {
            collide = true;
            break;
        }

        if (simplex.Count() != 1)
        {
            // Rebuild the simplex with vertices that are used(involved) to calculate closest distance
            simplex.Shrink(closest.contributors, closest.count);
        }

        dir = origin - closest.point;
        float dist = spe::length(dir);
        dir.Normalize();

        supportPoint = cso_support(b1, b2, dir);

        // If the new support point is not further along the search direction than the closest point,
        // two objects are not colliding so you can early return here.
        if (earlyReturn && dist > dot(dir, supportPoint - closest.point))
        {
            collide = false;
            break;
        }

        if (simplex.ContainsVertex(supportPoint))
        {
            collide = false;
            break;
        }
        else
        {
            simplex.AddVertex(supportPoint);
        }
    }

    return { collide, simplex };
}

struct EPAResult
{
    float penetrationDepth;
    Vec2 contactNormal;
};

static EPAResult epa(RigidBody* b1, RigidBody* b2, Simplex& gjkResult)
{
    Polytope polytope{ gjkResult };

    ClosestEdgeInfo closestEdge{ 0, FLT_MAX, Vec2{ 0.0f } };

    for (uint32_t i = 0; i < EPA_MAX_ITERATION; i++)
    {
        closestEdge = polytope.GetClosestEdge();
        const Vec2 supportPoint = cso_support(b1, b2, closestEdge.normal);
        const float newDistance = dot(closestEdge.normal, supportPoint);

        if (spe::abs(closestEdge.distance - newDistance) > EPA_TOLERANCE)
        {
            // Insert the support vertex so that it expands our polytope
            polytope.vertices.Insert(closestEdge.index + 1, supportPoint);
        }
        else
        {
            // If you didn't expand edge, it means you reached the closest outer edge
            break;
        }
    }

    return { closestEdge.distance, closestEdge.normal };
}

static Edge find_featured_edge(Polygon* b, const Vec2& dir)
{
    const Vec2 localDir = mul_t(b->GetRotation(), dir);
    const SupportResult farthest = support(b, localDir);

    Vec2 curr = farthest.vertex;
    int32_t idx = farthest.index;

    const Transform& t = b->GetTransform();

    RigidBody::Shape shape = b->GetShape();
    if (shape == RigidBody::Shape::ShapeCircle)
    {
        curr = t * curr;
        const Vec2 tangent = cross(1.0f, dir) * TANGENT_MIN_LENGTH;

        return Edge{ curr, curr + tangent };
    }
    else if (shape == RigidBody::Shape::ShapePolygon)
    {
        Polygon* p = static_cast<Polygon*>(b);

        const std::vector<Vec2>& vertices = p->GetVertices();
        int32_t vertexCount = static_cast<int32_t>(p->VertexCount());

        const Vec2& prev = vertices[(idx - 1 + vertexCount) % vertexCount];
        const Vec2& next = vertices[(idx + 1) % vertexCount];

        Vec2 e1 = (curr - prev).Normalized();
        Vec2 e2 = (curr - next).Normalized();

        bool w = dot(e1, localDir) <= dot(e2, localDir);

        if (w)
        {
            return Edge{ t * prev, t * curr, (idx - 1 + vertexCount) % vertexCount, idx };
        }
        else
        {
            return Edge{ t * curr, t * next, idx, (idx + 1) % vertexCount };
        }
    }
    else
    {
        throw std::exception("Not a supported shape");
    }
}

static void clip_edge(Edge* edge, const Vec2& p, const Vec2& dir, bool removeClippedPoint = false)
{
    float d1 = dot(edge->p1.position - p, dir);
    float d2 = dot(edge->p2.position - p, dir);

    if (d1 >= 0 && d2 >= 0)
    {
        return;
    }

    float s = spe::abs(d1) + spe::abs(d2);

    if (d1 < 0)
    {
        if (removeClippedPoint)
        {
            edge->p1 = edge->p2;
        }
        else
        {
            edge->p1.position = edge->p1.position + (edge->p2.position - edge->p1.position) * (-d1 / s);
        }
    }
    else if (d2 < 0)
    {
        if (removeClippedPoint)
        {
            edge->p2 = edge->p1;
        }
        else
        {
            edge->p2.position = edge->p2.position + (edge->p1.position - edge->p2.position) * (-d2 / s);
        }
    }
}

static void find_contact_points(const Vec2& n, Polygon* a, Polygon* b, ContactManifold* out)
{
    Edge edgeA = find_featured_edge(a, n);
    Edge edgeB = find_featured_edge(b, -n);

    Edge* ref = &edgeA; // Reference edge
    Edge* inc = &edgeB; // Incidence edge
    out->bodyA = a;
    out->bodyB = b;
    out->contactNormal = n;
    out->featureFlipped = false;

    float aPerpendicularness = spe::abs(dot(edgeA.dir, n));
    float bPerpendicularness = spe::abs(dot(edgeB.dir, n));

    if (bPerpendicularness < aPerpendicularness)
    {
        ref = &edgeB;
        inc = &edgeA;
        out->bodyA = b;
        out->bodyB = a;
        out->contactNormal = -n;
        out->featureFlipped = true;
    }

    clip_edge(inc, ref->p1.position, ref->dir);
    clip_edge(inc, ref->p2.position, -ref->dir);
    clip_edge(inc, ref->p1.position, -out->contactNormal, true);

    // If two points are closer than threshold, merge them into one point
    if (inc->Length() <= CONTACT_MERGE_THRESHOLD)
    {
        out->contactPoints[0] = ContactPoint{ inc->p1.position, inc->p1.id };
        out->numContacts = 1;
    }
    else
    {
        out->contactPoints[0] = ContactPoint{ inc->p1.position, inc->p1.id };
        out->contactPoints[1] = ContactPoint{ inc->p2.position, inc->p2.id };
        out->numContacts = 2;
    }

    out->referenceEdge = *ref;
}

static bool circle_vs_circle(Circle* a, Circle* b, ContactManifold* out)
{
    float d = dist2(a->GetPosition(), b->GetPosition());
    const float r2 = a->GetRadius() + b->GetRadius();

    if (d > r2 * r2)
    {
        return false;
    }
    else
    {
        if (out == nullptr) return true;

        d = spe::sqrt(d);

        out->bodyA = a;
        out->bodyB = b;
        out->contactNormal = (b->GetPosition() - a->GetPosition()).Normalized();
        out->contactPoints[0] = ContactPoint{ b->GetPosition() + (-out->contactNormal * b->GetRadius()), -1 };
        out->referenceEdge = Edge{ a->GetPosition() + (out->contactNormal * a->GetRadius()),
                                   a->GetPosition() + (out->contactNormal * a->GetRadius()) };
        out->numContacts = 1;
        out->penetrationDepth = r2 - d;
        out->featureFlipped = false;

        // Apply axis weight to improve coherence
        if (APPLY_AXIS_WEIGHT && dot(out->contactNormal, weightAxis) < 0.0f)
        {
            RigidBody* tmp = out->bodyA;
            out->bodyA = out->bodyB;
            out->bodyB = tmp;
            out->contactNormal *= -1;
            out->featureFlipped = out->bodyA != a;
        }
        out->contactTangent = Vec2{ -out->contactNormal.y, out->contactNormal.x };

        return true;
    }
}

static bool convex_vs_circle(Polygon* a, Circle* b, ContactManifold* out)
{
    const Vec2& pa = a->GetPosition();
    const Vec2& pb = b->GetPosition();

    Vec2 a2b = (pb - pa).Normalized();
    out->referenceEdge = get_farthest_edge(a, a2b);
    UV w = compute_uv(out->referenceEdge.p1.position, out->referenceEdge.p2.position, b->GetPosition());

    const Vec2& closest = w.v <= 0 ? out->referenceEdge.p1.position
                                   : (w.v >= 1 ? out->referenceEdge.p2.position
                                               : lerp_vector(out->referenceEdge.p1.position, out->referenceEdge.p2.position, w));

    Vec2 d = pb - closest;
    float distance = d.Length();

    if (distance > b->GetRadius())
    {
        return false;
    }
    else
    {
        out->bodyA = a;
        out->bodyB = b;
        out->contactNormal = -out->referenceEdge.normal;
        out->contactTangent = out->contactNormal.Skew();
        out->penetrationDepth = b->GetRadius() - distance;
        out->contactPoints[0] = ContactPoint{ b->GetPosition() + out->contactNormal * -b->GetRadius(), -1 };
        out->numContacts = 1;

        return true;
    }
}

static bool convex_vs_convex(Polygon* a, Polygon* b, ContactManifold* out)
{
    GJKResult gjkResult = gjk(a, b);

    if (!gjkResult.collide)
    {
        return false;
    }
    else
    {
        if (out == nullptr) return true;

        // If the gjk termination simplex has vertices less than 3, expand to full simplex
        // Because EPA needs a full n-simplex to get started
        Simplex& simplex = gjkResult.simplex;
        switch (simplex.Count())
        {
        case 1:
        {
            const Vec2& v = simplex.vertices[0];
            Vec2 randomSupport = cso_support(a, b, Vec2{ 1.0f, 0.0f });

            if (randomSupport == v)
            {
                randomSupport = cso_support(a, b, Vec2{ -1.0f, 0.0f });
            }

            simplex.AddVertex(randomSupport);
        }
        case 2:
        {
            Edge e{ simplex.vertices[0], simplex.vertices[1] };
            Vec2 normalSupport = cso_support(a, b, e.normal);

            if (simplex.ContainsVertex(normalSupport))
            {
                simplex.AddVertex(cso_support(a, b, -e.normal));
            }
            else
            {
                simplex.AddVertex(normalSupport);
            }
        }
        }

        EPAResult epaResult = epa(a, b, gjkResult.simplex);

        find_contact_points(epaResult.contactNormal, a, b, out);

        // Apply axis weight to improve coherence
        if (APPLY_AXIS_WEIGHT && dot(out->contactNormal, weightAxis) < 0.0f)
        {
            RigidBody* tmp = out->bodyA;
            out->bodyA = out->bodyB;
            out->bodyB = tmp;
            out->contactNormal *= -1;
            out->featureFlipped = out->bodyA != a;
        }
        out->contactTangent = Vec2{ -out->contactNormal.y, out->contactNormal.x };
        out->penetrationDepth = epaResult.penetrationDepth;

        return true;
    }
}

bool detect_collision(RigidBody* a, RigidBody* b, ContactManifold* out)
{
    out->numContacts = 0;
    out->penetrationDepth = 0.0f;

    RigidBody::Shape shapeA = a->GetShape();
    RigidBody::Shape shapeB = b->GetShape();

    // Circle vs. Circle collision
    if (shapeA == RigidBody::Shape::ShapeCircle && shapeB == RigidBody::Shape::ShapeCircle)
    {
        return circle_vs_circle(static_cast<Circle*>(a), static_cast<Circle*>(b), out);
    }
    // Convex vs. Circle collision
    else if (shapeA == RigidBody::Shape::ShapePolygon && shapeB == RigidBody::Shape::ShapeCircle)
    {
        return convex_vs_circle(static_cast<Polygon*>(a), static_cast<Circle*>(b), out);
    }
    else if (shapeA == RigidBody::Shape::ShapeCircle && shapeB == RigidBody::Shape::ShapePolygon)
    {
        return convex_vs_circle(static_cast<Polygon*>(b), static_cast<Circle*>(a), out);
        out->featureFlipped = true;
    }
    else
    {
        return convex_vs_convex(static_cast<Polygon*>(a), static_cast<Polygon*>(b), out);
    }
}

bool test_point_inside(RigidBody* b, const Vec2& p)
{
    Vec2 localP = mul_t(b->GetTransform(), p);

    if (b->GetShape() == RigidBody::Shape::ShapeCircle)
    {
        return spe::length(localP) <= static_cast<Circle*>(b)->GetRadius();
    }
    else if (b->GetShape() == RigidBody::Shape::ShapePolygon)
    {
        Polygon* p = static_cast<Polygon*>(b);
        const std::vector<Vec2>& vertices = p->GetVertices();

        float dir = cross(vertices[0] - localP, vertices[1] - localP);

        for (uint32_t i = 1; i < vertices.size(); i++)
        {
            float nDir = cross(vertices[i] - localP, vertices[(i + 1) % vertices.size()] - localP);

            if (dir * nDir < 0) return false;
        }

        return true;
    }
    else
    {
        throw std::exception("Not a supported shape");
    }
}

float compute_distance(RigidBody* a, RigidBody* b)
{
    GJKResult gr = gjk(a, b, false);

    if (gr.collide)
    {
        return 0.0f;
    }
    else
    {
        ClosestResult cr = gr.simplex.GetClosest({ 0.0f, 0.0f });

        return spe::length(cr.point);
    }
}

float compute_distance(RigidBody* b, const Vec2& p)
{
    return dist(get_closest_point(b, p), p);
}

Vec2 get_closest_point(RigidBody* b, const Vec2& p)
{
    Vec2 localP = mul_t(b->GetTransform(), p);

    if (test_point_inside(b, localP))
    {
        return p;
    }
    else
    {
        Vec2 dir = (localP - b->GetPosition()).Normalized();

        if (b->GetType() == RigidBody::Shape::ShapeCircle)
        {
            Vec2 localR = b->GetPosition() + dir * static_cast<Circle*>(b)->GetRadius();
            return b->GetTransform() * localR;
        }
        else if (b->GetType() == RigidBody::Shape::ShapePolygon)
        {
            Polygon* poly = static_cast<Polygon*>(b);
            const std::vector<Vec2>& v = poly->GetVertices();

            SupportResult sr = support(poly, dir);
            Simplex s;

            s.AddVertex(v[sr.index]);
            s.AddVertex(v[(sr.index + 1) % v.size()]);
            s.AddVertex(v[(sr.index + 2) % v.size()]);

            ClosestResult cr = s.GetClosest(localP);
            return b->GetTransform() * cr.point;
        }
        else
        {
            throw std::exception("Not a supported shape");
        }
    }
}

Edge get_farthest_edge(Polygon* p, const Vec2& dir)
{
    const Vec2 localDir = mul_t(p->GetRotation(), dir);
    const SupportResult farthest = support(p, localDir);

    Vec2 curr = farthest.vertex;
    int32_t idx = farthest.index;

    const Transform& t = p->GetTransform();

    const std::vector<Vec2>& vertices = p->GetVertices();
    int32_t vertexCount = static_cast<int32_t>(p->VertexCount());

    const Vec2& prev = vertices[(idx - 1 + vertexCount) % vertexCount];
    const Vec2& next = vertices[(idx + 1) % vertexCount];

    bool w = cross(localDir, curr) > 0;

    if (w)
    {
        return Edge{ t * prev, t * curr, (idx - 1 + vertexCount) % vertexCount, idx };
    }
    else
    {
        return Edge{ t * curr, t * next, idx, (idx + 1) % vertexCount };
    }
}

} // namespace spe
