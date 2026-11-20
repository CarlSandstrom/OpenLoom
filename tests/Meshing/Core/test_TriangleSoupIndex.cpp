#include <gtest/gtest.h>

#include "Common/Types.h"
#include "Meshing/Core/3D/General/RobustPredicates3D.h"
#include "Meshing/Core/3D/General/TriangleSoupIndex.h"

#include <cmath>
#include <vector>

using namespace Meshing;

namespace
{

// A square on the slanted plane x = SLOPE * z, spanning [MINIMUM, MINIMUM + SIDE]
// in y and z, tessellated into DIVISIONS x DIVISIONS cells of two triangles each.
// The awkward numbers keep the grid's cell boundaries off round values, as a
// CAD model's coordinates are.
constexpr double SLOPE = 0.1;
constexpr double MINIMUM = -2.6764428489921919;
constexpr double SIDE = 5.3528856979843838;
constexpr int DIVISIONS = 8;

double planeX(double z)
{
    return SLOPE * z;
}

std::vector<TriangleSoupIndex::Triangle> slantedSquare()
{
    std::vector<TriangleSoupIndex::Triangle> triangles;
    auto vertex = [](int row, int column)
    {
        double y = MINIMUM + SIDE * row / DIVISIONS;
        double z = MINIMUM + SIDE * column / DIVISIONS;
        return Point3D(planeX(z), y, z);
    };
    for (int row = 0; row < DIVISIONS; ++row)
        for (int column = 0; column < DIVISIONS; ++column)
        {
            triangles.push_back({vertex(row, column), vertex(row + 1, column), vertex(row + 1, column + 1)});
            triangles.push_back({vertex(row, column), vertex(row + 1, column + 1), vertex(row, column + 1)});
        }
    return triangles;
}

bool anyTriangleCrossed(const std::vector<TriangleSoupIndex::Triangle>& triangles, const Point3D& a, const Point3D& b)
{
    for (auto& vertices : triangles)
        if (RobustPredicates3D::segmentCrossesTriangle(a, b, vertices[0], vertices[1], vertices[2]))
            return true;
    return false;
}

} // namespace

// The grid is a filter in front of the exact predicate: it must never hide a
// triangle the segment crosses. Segments almost parallel to a cell boundary
// plane, lying on it or a few ulps either side, are where a cell walk decides
// its next cell from nearly cancelling floating-point parameters.
TEST(TriangleSoupIndexTest, FindCrossingNearest_SegmentAlongCellBoundaries_FindsEveryCrossing)
{
    auto triangles = slantedSquare();
    TriangleSoupIndex index;
    index.build(triangles);

    // Every cell boundary of this soup's grid (5 cells along each axis) lies
    // on one of these, the bounds included.
    std::vector<double> positions;
    for (int step = 0; step <= 10; ++step)
        positions.push_back(MINIMUM + SIDE * step / 10);

    // The segment ends on (y, z) and arrives from a start displaced by these:
    // along the boundary plane, or crossing it at a grazing angle from either side.
    std::vector<double> drifts = {0.0, 1e-13, -1e-13, 2e-15, -2e-15};

    int checked = 0;
    for (double y : positions)
        for (double z : positions)
            for (double yDrift : drifts)
                for (double zDrift : drifts)
                {
                    Point3D a(287.05078500293052, y - yDrift, z - zDrift);
                    Point3D b(-4.9827360213250103, y, z);
                    if (!anyTriangleCrossed(triangles, a, b))
                        continue;

                    ++checked;
                    auto crossing = index.findCrossingNearest(a, b, a);
                    ASSERT_TRUE(crossing.has_value()) << "y " << y << " z " << z << " drifts " << yDrift << " " << zDrift;
                    EXPECT_NEAR(crossing->point.y(), y, 1e-9);
                    EXPECT_NEAR(crossing->point.z(), z, 1e-9);
                }

    EXPECT_GT(checked, 1000);
}

// A planar CAD face's tessellation is flat only up to rounding: its coordinates
// along the normal differ by an ulp or two. The grid must not cut that sliver
// of extent into cells narrower than the rounding of the coordinates
// themselves, or a walk entering from far away lands in the wrong cell.
TEST(TriangleSoupIndexTest, FindCrossingNearest_SoupFlatUpToRounding_FindsCrossing)
{
    // The bounds, triangle count and segments are modelled on a HexNutSurfaceMesh
    // side face (OPE-226). The vertices are not that face's, so this does not
    // reproduce the miss found there; HexNutSurfaceMesh's golden guards that.
    constexpr int DIVISIONS = 128;
    double planeX = 6.5;
    double aboveX = std::nextafter(planeX, INFINITY);
    double belowX = std::nextafter(planeX, -INFINITY);
    double yMinimum = -3.7900090229976344;
    double ySide = 3.7746433229199861 - yMinimum;
    double zMinimum = -0.031220472440946168;
    double zSide = 6.5199606299212602 - zMinimum;

    std::vector<TriangleSoupIndex::Triangle> triangles;
    for (int row = 0; row < DIVISIONS; ++row)
        for (int column = 0; column < DIVISIONS; ++column)
        {
            double y = yMinimum + ySide * row / DIVISIONS;
            double z = zMinimum + zSide * column / DIVISIONS;
            double yNext = yMinimum + ySide * (row + 1) / DIVISIONS;
            double zNext = zMinimum + zSide * (column + 1) / DIVISIONS;
            double x = (row + column) % 2 == 0 ? aboveX : belowX;
            triangles.push_back({Point3D(x, y, z), Point3D(planeX, yNext, z), Point3D(x, yNext, zNext)});
            triangles.push_back({Point3D(x, y, z), Point3D(x, yNext, zNext), Point3D(planeX, y, zNext)});
        }
    TriangleSoupIndex index;
    index.build(triangles);

    std::vector<std::array<Point3D, 2>> segments = {
        {Point3D(287.05078500293052, 2.6764428489920817, 5.4166666666666856),
         Point3D(4.9827360213250103, 2.6764428489921919, 5.4166666666666661)},
        {Point3D(285.55161167528951, -0.80858830602949183, 4.6346153846153868),
         Point3D(5.927777777777778, -0.80858830602936871, 4.634615384615385)},
        {Point3D(285.21663103683727, -1.4312558979838741, 4.2909157560507651),
         Point3D(5.7788412720664457, -1.4312558979837251, 4.2909157560507731)},
        {Point3D(283.70155742461048, -2.6764428489923375, 1.0833333333333144),
         Point3D(4.9827360213250103, -2.6764428489921941, 1.0833333333333333)}};

    for (auto& [a, b] : segments)
    {
        ASSERT_TRUE(anyTriangleCrossed(triangles, a, b));
        auto crossing = index.findCrossingNearest(a, b, a);
        ASSERT_TRUE(crossing.has_value()) << "segment ending at y " << b.y();
        EXPECT_NEAR(crossing->point.x(), planeX, 1e-9);
    }
}

TEST(TriangleSoupIndexTest, FindCrossingNearest_FlatSoup_FindsCrossing)
{
    std::vector<TriangleSoupIndex::Triangle> triangles = {
        {Point3D(0.0, 0.0, 0.0), Point3D(1.0, 0.0, 0.0), Point3D(0.0, 1.0, 0.0)},
        {Point3D(1.0, 0.0, 0.0), Point3D(1.0, 1.0, 0.0), Point3D(0.0, 1.0, 0.0)}};
    TriangleSoupIndex index;
    index.build(triangles);

    auto crossing = index.findCrossingNearest(Point3D(0.3, 0.6, -1.0), Point3D(0.3, 0.6, 2.0), Point3D(0.3, 0.6, -1.0));

    ASSERT_TRUE(crossing.has_value());
    EXPECT_NEAR(crossing->point.x(), 0.3, 1e-12);
    EXPECT_NEAR(crossing->point.y(), 0.6, 1e-12);
    EXPECT_NEAR(crossing->point.z(), 0.0, 1e-12);
}

TEST(TriangleSoupIndexTest, FindCrossingNearest_SegmentMissingSoup_ReturnsNothing)
{
    TriangleSoupIndex index;
    index.build(slantedSquare());

    auto crossing = index.findCrossingNearest(Point3D(-10.0, 10.0, 0.0), Point3D(10.0, 10.0, 0.0), Point3D(0.0, 10.0, 0.0));

    EXPECT_FALSE(crossing.has_value());
}
