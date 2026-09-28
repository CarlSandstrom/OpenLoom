#include "Meshing/Core/3D/General/RobustPredicates3D.h"
#include "Meshing/Core/3D/General/ExactArithmetic3D.h"

#include <cmath>
#include <optional>

namespace Meshing
{

namespace
{

using Expansion = ExactArithmetic3D::Expansion;

// ---------------------------------------------------------------------------
// Adaptive fast path: plain-double arithmetic is ~1000x cheaper than the
// exact expansion arithmetic in ExactArithmetic3D — correct for the vast
// majority of calls — only genuinely near-degenerate input needs the exact
// fallback. Each of these computes the same determinant in plain double
// alongside a conservative error bound (SAFETY_FACTOR relative to the sum of
// magnitudes of the determinant's expanded terms — deliberately generous,
// ~1e8 times larger than the actual worst-case floating-point rounding for a
// bounded-depth determinant like these, so it trades a bit of fast-path
// coverage for being unambiguously safe rather than chasing Shewchuk's
// tightest published bounds). Returns nullopt when the fast result isn't
// trustworthy, signaling the caller to fall back to the exact path.
// ---------------------------------------------------------------------------

constexpr double SAFETY_FACTOR = 1e-8;

std::optional<int> fastOrientationSign(const Point3D& p0, const Point3D& p1, const Point3D& p2, const Point3D& p3)
{
    const double ax = p0.x() - p3.x(), ay = p0.y() - p3.y(), az = p0.z() - p3.z();
    const double bx = p1.x() - p3.x(), by = p1.y() - p3.y(), bz = p1.z() - p3.z();
    const double cx = p2.x() - p3.x(), cy = p2.y() - p3.y(), cz = p2.z() - p3.z();

    const double t0 = ax * (by * cz);
    const double t1 = ax * (bz * cy);
    const double t2 = ay * (bx * cz);
    const double t3 = ay * (bz * cx);
    const double t4 = az * (bx * cy);
    const double t5 = az * (by * cx);

    const double det = t0 - t1 - t2 + t3 + t4 - t5;
    const double permanent = std::abs(t0) + std::abs(t1) + std::abs(t2) + std::abs(t3) + std::abs(t4) + std::abs(t5);
    const double bound = SAFETY_FACTOR * permanent;

    if (std::abs(det) <= bound)
        return std::nullopt;
    return det > 0.0 ? 1 : -1;
}

int exactOrientationSign(const Point3D& p0, const Point3D& p1, const Point3D& p2, const Point3D& p3)
{
    // Signed volume of (p0,p1,p2,p3), via rows relative to p3.
    Expansion m[3][3];
    const Point3D relativeTo[3] = {p0, p1, p2};
    for (int i = 0; i < 3; ++i)
    {
        m[i][0] = ExactArithmetic3D::expansionFromDifference(relativeTo[i].x(), p3.x());
        m[i][1] = ExactArithmetic3D::expansionFromDifference(relativeTo[i].y(), p3.y());
        m[i][2] = ExactArithmetic3D::expansionFromDifference(relativeTo[i].z(), p3.z());
    }

    return ExactArithmetic3D::expansionSign(ExactArithmetic3D::det3x3(m));
}

} // namespace

int RobustPredicates3D::orientationSign(const Point3D& p0,
                                        const Point3D& p1,
                                        const Point3D& p2,
                                        const Point3D& p3)
{
    if (const std::optional<int> fast = fastOrientationSign(p0, p1, p2, p3))
        return *fast;
    return exactOrientationSign(p0, p1, p2, p3);
}

bool RobustPredicates3D::segmentCrossesTriangle(const Point3D& p,
                                                const Point3D& q,
                                                const Point3D& a,
                                                const Point3D& b,
                                                const Point3D& c)
{
    // p and q must be strictly on opposite sides of the triangle's plane.
    // Equal signs also correctly rejects the coplanar case (both 0).
    const int sideP = orientationSign(a, b, c, p);
    const int sideQ = orientationSign(a, b, c, q);
    if (sideP == sideQ)
        return false;

    // The plane crossing point (never computed explicitly) is inside the
    // triangle iff segment pq passes the same side of all 3 edges, taken in
    // a consistent winding order around the triangle -- i.e. these three
    // orientation tests all agree in sign. Any of them landing on exactly 0
    // means the crossing touches an edge or vertex rather than the
    // triangle's interior, so it's rejected rather than treated as a match.
    const int edgeAB = orientationSign(p, q, a, b);
    const int edgeBC = orientationSign(p, q, b, c);
    const int edgeCA = orientationSign(p, q, c, a);
    if (edgeAB == 0 || edgeBC == 0 || edgeCA == 0)
        return false;

    return edgeAB == edgeBC && edgeBC == edgeCA;
}

} // namespace Meshing
