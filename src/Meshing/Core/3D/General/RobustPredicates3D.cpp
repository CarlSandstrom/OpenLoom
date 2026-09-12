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

std::optional<int> fastInsphereSign(const Point3D& p0, const Point3D& p1, const Point3D& p2, const Point3D& p3,
                                    const Point3D& queryPoint)
{
    const Point3D vertices[4] = {p0, p1, p2, p3};
    double d[4][4];
    for (int i = 0; i < 4; ++i)
    {
        d[i][0] = vertices[i].x() - queryPoint.x();
        d[i][1] = vertices[i].y() - queryPoint.y();
        d[i][2] = vertices[i].z() - queryPoint.z();
        d[i][3] = d[i][0] * d[i][0] + d[i][1] * d[i][1] + d[i][2] * d[i][2];
    }

    auto det3 = [](const double m[3][3])
    {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    };

    double minor[4];
    for (int skipRow = 0; skipRow < 4; ++skipRow)
    {
        double sub[3][3];
        int r = 0;
        for (int row = 0; row < 4; ++row)
        {
            if (row == skipRow)
                continue;
            sub[r][0] = d[row][0];
            sub[r][1] = d[row][1];
            sub[r][2] = d[row][2];
            ++r;
        }
        minor[skipRow] = det3(sub);
    }

    double det = 0.0;
    double permanent = 0.0;
    for (int i = 0; i < 4; ++i)
    {
        double term = d[i][3] * minor[i];
        permanent += std::abs(term);
        det += (i % 2 == 0) ? -term : term;
    }
    const double bound = SAFETY_FACTOR * permanent;

    if (std::abs(det) <= bound)
        return std::nullopt;
    return det > 0.0 ? 1 : -1;
}

int exactOrientationSign(const Point3D& p0, const Point3D& p1, const Point3D& p2, const Point3D& p3)
{
    // Signed volume of (p0,p1,p2,p3), via rows relative to p3 (same
    // row-subtraction identity used in exactInsphereSign).
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

int exactInsphereSign(const Point3D& p0, const Point3D& p1, const Point3D& p2, const Point3D& p3,
                      const Point3D& queryPoint)
{
    // Standard in-sphere determinant, built from coordinates shifted relative to
    // queryPoint (eliminates the "1" column of the textbook 5x5 form — subtracting
    // one row from the others is determinant-invariant, see class comment). Row i
    // is [dx_i, dy_i, dz_i, dx_i^2+dy_i^2+dz_i^2] for i in {p0,p1,p2,p3}.
    const Point3D vertices[4] = {p0, p1, p2, p3};

    Expansion d[4][4];
    for (int i = 0; i < 4; ++i)
    {
        d[i][0] = ExactArithmetic3D::expansionFromDifference(vertices[i].x(), queryPoint.x());
        d[i][1] = ExactArithmetic3D::expansionFromDifference(vertices[i].y(), queryPoint.y());
        d[i][2] = ExactArithmetic3D::expansionFromDifference(vertices[i].z(), queryPoint.z());
        d[i][3] = ExactArithmetic3D::squaredNormRelative(d[i][0], d[i][1], d[i][2]);
    }

    // 4x4 determinant via cofactor expansion along the last (squared-distance) column.
    Expansion minor[4];
    for (int skipRow = 0; skipRow < 4; ++skipRow)
    {
        Expansion sub[3][3];
        int r = 0;
        for (int row = 0; row < 4; ++row)
        {
            if (row == skipRow)
                continue;
            sub[r][0] = d[row][0];
            sub[r][1] = d[row][1];
            sub[r][2] = d[row][2];
            ++r;
        }
        minor[skipRow] = ExactArithmetic3D::det3x3(sub);
    }

    // det = -d[0][3]*minor0 + d[1][3]*minor1 - d[2][3]*minor2 + d[3][3]*minor3
    Expansion determinant;
    for (int i = 0; i < 4; ++i)
    {
        Expansion term = ExactArithmetic3D::expansionMul(d[i][3], minor[i]);
        if (i % 2 == 0)
            term = ExactArithmetic3D::expansionNegate(term);
        determinant = ExactArithmetic3D::expansionAdd(determinant, term);
    }

    return ExactArithmetic3D::expansionSign(determinant);
}

} // namespace

bool RobustPredicates3D::insidePointCircumsphere(const Point3D& p0,
                                                 const Point3D& p1,
                                                 const Point3D& p2,
                                                 const Point3D& p3,
                                                 const Point3D& queryPoint)
{
    const int orientation = orientationSign(p0, p1, p2, p3);

    // A degenerate (zero-volume) tetrahedron has no valid circumsphere.
    if (orientation == 0)
        return false;

    const std::optional<int> fast = fastInsphereSign(p0, p1, p2, p3, queryPoint);
    const int insphereSign = fast ? *fast : exactInsphereSign(p0, p1, p2, p3, queryPoint);

    // See class comment: insphereSign agrees with orientationSign exactly when
    // queryPoint is inside the sphere. Treat exactly-on-sphere (insphereSign
    // == 0) as inside — inclusive, like the Bowyer-Watson cavity this feeds
    // needs: boundary-discretized points from a circular/cylindrical edge are
    // routinely exactly cospherical by construction (regular-polygon
    // vertices sharing one circle), and a strict "on sphere is not conflicting"
    // reading leaves some of them out of the cavity, producing an
    // inconsistent triangulation (a boundary face with only one neighboring
    // tetrahedron) rather than a genuinely ambiguous/wrong answer.
    if (insphereSign == 0)
        return true;
    return insphereSign == orientation;
}

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
