#include <algorithm>
#include <random>

#include "muli/geometry.h"
#include "muli/hash.h"

namespace muli
{

void ComputeConvexHull(const Vec2* vertices, int32 vertexCount, Vec2* outVertices, int32* outVertexCount)
{
    if (vertexCount < 3)
    {
        memcpy(outVertices, vertices, vertexCount * sizeof(Vec2));
        *outVertexCount = vertexCount;
        return;
    }

    // Find the lowest vertex
    int32 index = 0;
    for (int32 i = 1; i < vertexCount; ++i)
    {
        if (vertices[i].y < vertices[index].y)
        {
            index = i;
        }
    }
    Vec2 bottom = vertices[index];

    Vec2* sorted = new Vec2[vertexCount];
    memcpy(sorted, vertices, vertexCount * sizeof(Vec2));
    std::swap(sorted[index], sorted[0]);

    // Sort the vertices based on the angle related to the lowest vertex.
    std::sort(sorted + 1, sorted + vertexCount, [&](const Vec2& a, const Vec2& b) -> bool {
        Vec2 ra = a - bottom;
        Vec2 rb = b - bottom;

        float angleA = Atan2(ra.y, ra.x);
        float angleB = Atan2(rb.y, rb.x);

        if (angleA == angleB)
        {
            return Dist2(bottom, a) < Dist2(bottom, b);
        }
        else
        {
            return angleA < angleB;
        }
    });

    // Discard overlapped bottom vertices
    int32 i = 0;
    while (i < vertexCount)
    {
        Vec2& v0 = sorted[i];
        Vec2& v1 = sorted[(i + 1) % vertexCount];

        if (v0 == v1)
        {
            ++i;
        }
        else
        {
            break;
        }
    }

    // Initialize stack
    int32 sp = 0;
    *outVertexCount = vertexCount - i;
    outVertices[sp++] = sorted[i++];
    outVertices[sp++] = sorted[i++];

    while (i < vertexCount)
    {
        Vec2& v = sorted[i];
        int32 l = sp;

        if (v == outVertices[l - 1])
        {
            ++i;
            continue;
        }

        Vec2 d1 = outVertices[l - 1] - outVertices[l - 2];
        Vec2 d2 = v - outVertices[l - 1];

        if (Cross(d1, d2) <= 0)
        {
            --sp;

            if (l < 3)
            {
                outVertices[sp++] = v;
                break;
            }
        }
        else
        {
            outVertices[sp++] = v;
            ++i;
        }
    }

    delete[] sorted;
}

std::vector<Vec2> ComputeConvexHull(std::span<const Vec2> vertices)
{
    if (vertices.size() < 3)
    {
        return std::vector<Vec2>(vertices.begin(), vertices.end());
    }

    size_t index = 0;
    for (size_t i = 1; i < vertices.size(); ++i)
    {
        if (vertices[i].y < vertices[index].y)
        {
            index = i;
        }
    }
    Vec2 bottom = vertices[index];

    std::vector<Vec2> sorted(vertices.begin(), vertices.end());
    std::swap(sorted[index], sorted[0]);

    std::sort(sorted.begin() + 1, sorted.end(), [&](const Vec2& a, const Vec2& b) -> bool {
        Vec2 ra = a - bottom;
        Vec2 rb = b - bottom;

        float angleA = Atan2(ra.y, ra.x);
        float angleB = Atan2(rb.y, rb.x);

        if (angleA == angleB)
        {
            return Dist2(bottom, a) < Dist2(bottom, b);
        }
        else
        {
            return angleA < angleB;
        }
    });

    // Discard overlapped bottom vertices
    size_t i = 0;
    while (i < sorted.size())
    {
        Vec2& v0 = sorted[i];
        Vec2& v1 = sorted[(i + 1) % sorted.size()];

        if (v0 == v1)
        {
            i++;
        }
        else
        {
            break;
        }
    }

    std::vector<Vec2> s;
    s.push_back(sorted[i++]);
    s.push_back(sorted[i++]);

    while (i < sorted.size())
    {
        Vec2& v = sorted[i];
        size_t l = s.size();

        if (v == s[l - 1])
        {
            ++i;
            continue;
        }

        Vec2 d1 = s[l - 1] - s[l - 2];
        Vec2 d2 = v - s[l - 1];

        if (Cross(d1, d2) <= 0)
        {
            s.pop_back();

            if (l < 3)
            {
                s.push_back(v);
                break;
            }
        }
        else
        {
            s.push_back(v);
            ++i;
        }
    }

    return s;
}

static inline Circle ComputeCircle2(const Vec2& a, const Vec2& b)
{
    Vec2 center{ (a.x + b.x) / 2, (a.y + b.y) / 2 };
    float radius = Dist(a, b) / 2;

    return Circle{ radius, center };
}

static inline Circle ComputeCircle3(const Vec2& a, const Vec2& b, const Vec2& c)
{
    // Solve linear equations to find circumcircle
    float d = 2 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));

    float nx =
        ((a.x * a.x + a.y * a.y) * (b.y - c.y) + (b.x * b.x + b.y * b.y) * (c.y - a.y) + (c.x * c.x + c.y * c.y) * (a.y - b.y));
    float ny =
        ((a.x * a.x + a.y * a.y) * (c.x - b.x) + (b.x * b.x + b.y * b.y) * (a.x - c.x) + (c.x * c.x + c.y * c.y) * (b.x - a.x));

    Vec2 center{ nx / d, ny / d };
    float radius = Dist(center, a);

    return Circle{ radius, center };
}

// Welzl algorithm (https://en.wikipedia.org/wiki/Smallest-circle_problem)
// Compute minimum circle containing all points
// P: original all vertices
// R: vertices (n <= 3) to define minimum circle
static inline Circle Welzl(const std::vector<Vec2>& P, std::vector<Vec2>& R, int32 n)
{
    if (n == 0 || R.size() == 3)
    {
        // Handle trivial cases
        switch (R.size())
        {
        case 0:
            return Circle{ 0 };
        case 1:
            return Circle{ 0, R[0] };
        case 2:
            return ComputeCircle2(R[0], R[1]);

        default:
            return ComputeCircle3(R[0], R[1], R[2]);
        }
    }

    // Select random point.
    Vec2 p = P[n - 1];

    // Find minimum circle without current point p.
    Circle D = Welzl(P, R, n - 1);

    // If the returned circle also contains current point,
    // it is the minimum circle for the whole of P and is returned.
    if (D.TestPoint(identity, p))
    {
        return D;
    }
    else
    {
        // Otherwise, current point must lie on the boundary of the resulting circle.
        // It recurses, but with the set R of points known to be on the boundary.

        R.push_back(p);
        Circle newCircle = Welzl(P, R, n - 1);
        R.pop_back();

        return newCircle;
    }
}

Circle ComputeCircle(std::span<const Vec2> points)
{
    std::vector<Vec2> R;
    int32 n = int32(points.size());

    std::vector<Vec2> copy(points.begin(), points.end());
    std::shuffle(copy.begin(), copy.end(), std::default_random_engine{});

    return Welzl(copy, R, n);
}

struct TriEdge
{
    Vec2 v[2];

    TriEdge() = default;
    TriEdge(const Vec2& v0, const Vec2& v1)
        : v{ v0, v1 }
    {
    }

    Vec2& operator[](int32 index)
    {
        MuliAssert(0 <= index && index < 2);
        return v[index];
    }

    const Vec2& operator[](int32 index) const
    {
        MuliAssert(0 <= index && index < 2);
        return v[index];
    }

    bool Intersect(const TriEdge& other) const
    {
        // Test intersection (v0, v1) vs. (other.v0, other.v1)
        // Note it's working on open intervals

        Vec2 d = v[1] - v[0];
        if (Cross(d, other[0] - v[0]) * Cross(d, other[1] - v[0]) >= 0)
        {
            return false;
        }
        d = other[1] - other[0];

        if (Cross(d, v[0] - other[0]) * Cross(d, v[1] - other[0]) >= 0)
        {
            return false;
        }

        return true;
    }

    struct Hasher
    {
        size_t operator()(const TriEdge& e) const
        {
            return Hash(e[0], e[1]);
        }
    };
};

static inline bool operator==(const TriEdge& a, const TriEdge& b)
{
    return a[0] == b[0] && a[1] == b[1];
}

static inline TriEdge operator~(const TriEdge& e)
{
    return TriEdge{ e[1], e[0] };
}

struct Tri
{
    Vec2 v[3];

    Tri() = default;
    Tri(const Vec2& v0, const Vec2& v1, const Vec2& v2)
        : v{ v0, v1, v2 }
    {
    }

    Vec2& operator[](int32 index)
    {
        MuliAssert(0 <= index && index < 3);
        return v[index];
    }

    const Vec2& operator[](int32 index) const
    {
        MuliAssert(0 <= index && index < 3);
        return v[index];
    }

    bool HasEdge(const TriEdge& edge) const
    {
        if (edge[0] == v[0] && edge[1] == v[1]) return true;
        if (edge[0] == v[1] && edge[1] == v[0]) return true;

        if (edge[0] == v[1] && edge[1] == v[2]) return true;
        if (edge[0] == v[2] && edge[1] == v[1]) return true;

        if (edge[0] == v[2] && edge[1] == v[0]) return true;
        if (edge[0] == v[0] && edge[1] == v[2]) return true;

        return false;
    }

    bool HasVertex(const Vec2& p) const
    {
        return v[0] == p || v[1] == p || v[2] == p;
    }

    Vec2 GetCenter() const
    {
        return (v[0] + v[1] + v[2]) / 3;
    }

    std::array<TriEdge, 3> GetEdges() const
    {
        return { TriEdge{ v[0], v[1] }, TriEdge{ v[1], v[2] }, TriEdge{ v[2], v[0] } };
    }

    TriEdge GetEdge(int32 index) const
    {
        index %= 3;

        switch (index)
        {
        case 0:
            return TriEdge(v[0], v[1]);
        case 1:
            return TriEdge(v[1], v[2]);
        default:
            return TriEdge(v[2], v[0]);
        }
    }

    int32 GetIndex(const Vec2& p) const
    {
        if (p == v[0])
            return 0;
        else if (p == v[1])
            return 1;
        else if (p == v[2])
            return 2;
        else
            return -1;
    }

    struct Hasher
    {
        size_t operator()(const Tri& t) const
        {
            return Hash(t);
        }
    };
};

static inline bool operator==(const Tri& a, const Tri& b)
{
    return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}

static inline bool RayCastEdge(Vec2 o, Vec2 d, const TriEdge& edge)
{
    Vec2 e = edge[1] - edge[0];
    Vec2 o2a = edge[0] - o;

    float c = Cross(d, e);

    if (c == 0)
    {
        return false;
    }

    float t = Cross(o2a, e) / c;
    float u = Cross(o2a, d) / c;

    return (t >= 0 && u >= 0 && u <= 1);
}

static inline bool Contains(const std::vector<TriEdge>& edges, const Vec2& p)
{
    // TODO: Utilize BVH to accelerate raycasting
    int32 count = 0;
    for (const TriEdge& e : edges)
    {
        if (RayCastEdge(p, Vec2(1, 0), e))
        {
            ++count;
        }
    }

    return count % 2 == 1;
}

struct Poly
{
    std::vector<Vec2> v;

    Poly() = default;

    Poly(const Tri& tri)
        : v{ tri[0], tri[1], tri[2] }
    {
    }

    Vec2& operator[](int32 index)
    {
        MuliAssert(index > 0);
        index %= v.size();

        return v[index];
    }

    const Vec2& operator[](int32 index) const
    {
        MuliAssert(index >= 0);
        index %= v.size();

        return v[index];
    }

    TriEdge GetEdge(int32 index) const
    {
        index %= v.size();

        return TriEdge(v[index], v[(index + 1) % v.size()]);
    }

    int32 VertexCount() const
    {
        return int32(v.size());
    }

    int32 GetIndex(const Vec2& p) const
    {
        for (int32 i = 0; i < VertexCount(); ++i)
        {
            if (v[i] == p)
            {
                return i;
            }
        }

        return -1;
    }

    bool IsConvex() const
    {
        int32 count = VertexCount();
        if (count < 3)
        {
            return false;
        }

        for (int32 i0 = count - 1, i1 = 0; i1 < count; i0 = i1, ++i1)
        {
            Vec2 e0 = v[i1] - v[i0];
            Vec2 e1 = v[(i1 + 1) % count] - v[i1];

            if (Cross(e0, e1) <= 0)
            {
                return false;
            }
        }

        return true;
    }
};

static Poly Merge(const Poly& p1, const Poly& p2, const TriEdge& e)
{
    Poly merged;

    int32 i0 = p1.GetIndex(e[1]);
    int32 i1 = (i0 - 1 + p1.VertexCount()) % p1.VertexCount();

    for (int32 i = i0; i != i1; i = (i + 1) % p1.VertexCount())
    {
        merged.v.push_back(p1[i]);
    }

    i0 = p2.GetIndex(e[0]);
    i1 = (i0 - 1 + p2.VertexCount()) % p2.VertexCount();

    for (int32 i = i0; i != i1; i = (i + 1) % p2.VertexCount())
    {
        merged.v.push_back(p2[i]);
    }

    MuliAssert(merged.VertexCount() == (p1.VertexCount() + p2.VertexCount() - 2));

    return merged;
}

// The Bowyer–Watson algorithm (https://en.wikipedia.org/wiki/Bowyer%E2%80%93Watson_algorithm)
// + Brute force constraint resolution
static std::vector<Tri> ComputeTriangulation(std::span<const Vec2> v, std::span<const std::vector<Vec2>> constraints)
{
    std::vector<TriEdge> constraintEdges;

    struct Vec2Hasher
    {
        size_t operator()(const Vec2& v) const
        {
            return Hash(v);
        }
    };

    // Collect all vertices
    std::unordered_set<Vec2, Vec2Hasher> vertices(v.begin(), v.end());

    for (const std::vector<Vec2>& hole : constraints)
    {
        if (hole.size() > 1)
        {
            for (size_t i0 = hole.size() - 1, i1 = 0; i1 < hole.size(); i0 = i1, ++i1)
            {
                constraintEdges.emplace_back(hole[i0], hole[i1]);
            }
        }

        vertices.insert(hole.begin(), hole.end());
    }

    if (vertices.size() < 3)
    {
        return {};
    }

    // Prepare super triangle
    AABB bounds{ Vec2{ max_value, max_value }, -Vec2{ max_value, max_value } };

    for (const Vec2& p : vertices)
    {
        bounds = AABB::Union(bounds, p);
    }

    Vec2 extents = bounds.GetExtents();

    const float margin = 0.1f;
    Tri super{ Vec2{ bounds.min.x - extents.x - margin, bounds.min.y - margin },
               Vec2{ bounds.max.x + extents.x + margin, bounds.min.y - margin },
               Vec2{ bounds.min.x + extents.x / 2, bounds.max.y + extents.y + margin } };

    std::unordered_set<Tri, Tri::Hasher> tris;
    tris.insert(super);

    for (const Vec2& p : vertices)
    {
        std::vector<Tri> badTris;

        for (const Tri& t : tris)
        {
            Circle c = ComputeCircle3(t[0], t[1], t[2]);
            if (c.TestPoint(identity, p))
            {
                badTris.push_back(t);
            }
        }

        std::unordered_set<TriEdge, TriEdge::Hasher> poly;

        for (const Tri& t : badTris)
        {
            std::array<TriEdge, 3> edges = t.GetEdges();

            for (const TriEdge& e : edges)
            {
                if (poly.contains(~e))
                {
                    // Remove shared edge
                    poly.erase(~e);
                    continue;
                }

                poly.insert(e);
            }
        }

        for (const Tri& t : badTris)
        {
            tris.erase(t);
        }

        for (const TriEdge& e : poly)
        {
            // Guaranteed to be CCW
            tris.emplace(e[0], e[1], p);
        }
    }

    std::unordered_map<TriEdge, const Tri*, TriEdge::Hasher> edge2Tri;

    for (const Tri& t : tris)
    {
        for (const TriEdge& e : t.GetEdges())
        {
            MuliAssert(!edge2Tri.contains(e));
            edge2Tri.emplace(e, &t);
        }
    }

    for (const TriEdge& ce : constraintEdges)
    {
        if (edge2Tri.contains(ce))
        {
            continue;
        }

        while (true)
        {
            std::vector<TriEdge> badEdges;
            std::unordered_set<TriEdge, TriEdge::Hasher> resolved;

            // Collect all bad edges
            for (const Tri& t : tris)
            {
                for (const TriEdge& e : t.GetEdges())
                {
                    if (e.Intersect(ce))
                    {
                        badEdges.push_back(e);
                    }
                }
            }

            if (badEdges.size() == 0)
            {
                break;
            }

            // Resolve all bad edges
            for (const TriEdge& be : badEdges)
            {
                if (resolved.contains(be))
                {
                    continue;
                }

                const Tri* t1 = edge2Tri[be];
                const Tri* t2 = edge2Tri[~be];

                // Build CCW quadrilateral
                Vec2 v0 = be[0];
                Vec2 v1 = (*t2)[(t2->GetIndex(v0) + 1) % 3];
                Vec2 p2 = be[1];
                Vec2 p3 = (*t1)[(t1->GetIndex(v0) + 2) % 3];

                // Check convexity and skip if it's concave
                float s = Cross(v1 - v0, p2 - v1);
                if (s * Cross(p2 - v1, p3 - p2) < 0) continue;
                if (s * Cross(p3 - p2, v0 - p3) < 0) continue;
                if (s * Cross(v0 - p3, v1 - v0) < 0) continue;

                resolved.insert(~be);

                // Flip edge
                for (const TriEdge& e : t1->GetEdges())
                {
                    edge2Tri.erase(e);
                }

                for (const TriEdge& e : t2->GetEdges())
                {
                    edge2Tri.erase(e);
                }

                tris.erase(*t1);
                tris.erase(*t2);

                t1 = &(*(tris.emplace(v0, v1, p3).first));
                t2 = &(*(tris.emplace(v1, p2, p3).first));

                for (const TriEdge& e : t1->GetEdges())
                {
                    edge2Tri.emplace(e, t1);
                }

                for (const TriEdge& e : t2->GetEdges())
                {
                    edge2Tri.emplace(e, t2);
                }
            }

            // Repeat until there are no bad edges
        }
    }

    std::vector<Tri> res;
    res.reserve(tris.size());

    for (const Tri& t : tris)
    {
        // Discard triangle containing super triangle vertices
        if (t.HasVertex(super[0]) || t.HasVertex(super[1]) || t.HasVertex(super[2]))
        {
            continue;
        }

        // Discard triangle lies outside the shape
        if (constraintEdges.size() > 0 && !Contains(constraintEdges, t.GetCenter()))
        {
            continue;
        }

        res.emplace_back(t[0], t[1], t[2]);
    }

    return res;
}

std::vector<Polygon> ComputeTriangles(std::span<const Vec2> vertices, std::span<const std::vector<Vec2>> constraints)
{
    std::vector<Tri> triangles = ComputeTriangulation(vertices, constraints);

    std::vector<Polygon> res;
    res.reserve(triangles.size());

    for (const Tri& t : triangles)
    {
        res.emplace_back(t.v, 3);
    }

    return res;
}

std::vector<Polygon> ComputeDecomposition(std::span<const std::vector<Vec2>> constraints)
{
    std::vector<Tri> triangles = ComputeTriangulation({}, constraints);
    std::vector<Poly> polys(triangles.begin(), triangles.end());

    std::unordered_map<TriEdge, Poly*, TriEdge::Hasher> edge2Poly;

    while (true)
    {
    repeat:
        for (size_t index = 0; index < polys.size(); ++index)
        {
            Poly& p = polys[index];

            for (int32 i = 0; i < p.VertexCount(); ++i)
            {
                TriEdge e = p.GetEdge(i);

                if (!edge2Poly.contains(~e))
                {
                    edge2Poly.emplace(e, &p);
                    continue;
                }
                // Current polygon contains sharing edge

                Poly* other = edge2Poly[~e];

                Poly merged = Merge(*other, p, ~e);
                if (!merged.IsConvex())
                {
                    edge2Poly.emplace(e, &p);
                    continue;
                }
                // Merged polygon is convex, so we replace it with the merged one

                edge2Poly.clear();

                polys[index] = std::move(polys.back());
                polys.pop_back();

                *other = std::move(polys.back());
                polys.pop_back();

                polys.push_back(std::move(merged));
                goto repeat;
            }
        }

        break;
    }

    std::vector<Polygon> res;
    res.reserve(polys.size());

    for (const Poly& p : polys)
    {
        res.emplace_back(p.v.data(), p.VertexCount());
    }

    return res;
}

} // namespace muli