#include <gtest/gtest.h>

#include "Meshing/Core/3D/General/SizingFieldBuilder3D.h"
#include "Readers/OpenCascade/TopoDS_ShapeConverter.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeWedge.hxx>
#include <TopoDS_Shape.hxx>
#include <cmath>
#include <cstddef>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <memory>

using Meshing::Point3D;
using Meshing::SizingFieldBuilder3D;
using Meshing::SizingFieldSettings3D;

namespace
{

std::unique_ptr<Readers::TopoDS_ShapeConverter> convert(const TopoDS_Shape& shape)
{
    return std::make_unique<Readers::TopoDS_ShapeConverter>(shape);
}

} // namespace

TEST(SizingFieldBuilder3DTest, CylinderSizeMatchesTheChordDeviationInversion)
{
    // On a cylinder of radius R the larger principal curvature is 1/R, so
    // holding chord deviation to `tolerance` implies h = sqrt(8*tolerance*R).
    // Evaluated on the curved wall the field must reproduce that, since no
    // other constraint is tighter there.
    constexpr double RADIUS = 3.0;
    constexpr double HEIGHT = 8.0;
    constexpr double TOLERANCE = 0.05;

    auto converter = convert(
        BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)),
                                 RADIUS,
                                 HEIGHT)
            .Shape());

    SizingFieldSettings3D settings;
    settings.chordDeviationTolerance = TOLERANCE;

    const auto field =
        SizingFieldBuilder3D::build(converter->getGeometryCollection(), converter->getTopology(), settings);

    const double expected = std::sqrt(8.0 * TOLERANCE * RADIUS);

    // Mid-height on the wall, where the caps' own constraints are furthest away.
    const double actual = field.evaluate(Point3D(RADIUS, 0.0, HEIGHT / 2.0));

    EXPECT_GT(actual, 0.0);
    EXPECT_LE(actual, expected * 1.05);
}

TEST(SizingFieldBuilder3DTest, TighterChordToleranceAsksForSmallerElements)
{
    constexpr double RADIUS = 3.0;
    constexpr double HEIGHT = 8.0;

    auto converter = convert(
        BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)),
                                 RADIUS,
                                 HEIGHT)
            .Shape());

    SizingFieldSettings3D coarse;
    coarse.chordDeviationTolerance = 0.2;
    SizingFieldSettings3D fine;
    fine.chordDeviationTolerance = 0.01;

    const auto coarseField =
        SizingFieldBuilder3D::build(converter->getGeometryCollection(), converter->getTopology(), coarse);
    const auto fineField =
        SizingFieldBuilder3D::build(converter->getGeometryCollection(), converter->getTopology(), fine);

    const Point3D wall(RADIUS, 0.0, HEIGHT / 2.0);
    EXPECT_LT(fineField.evaluate(wall), coarseField.evaluate(wall));
}

TEST(SizingFieldBuilder3DTest, ThinBoxIsSizedByItsThicknessNotItsCurvature)
{
    // A box is flat everywhere, so curvature contributes nothing at all.
    // Only local feature size can bound element size here, and it must scale
    // with the thin dimension rather than the long ones.
    constexpr double THICKNESS = 0.2;

    auto thin = convert(BRepPrimAPI_MakeBox(10.0, 10.0, THICKNESS).Shape());
    auto cube = convert(BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape());

    const auto thinField =
        SizingFieldBuilder3D::build(thin->getGeometryCollection(), thin->getTopology());
    const auto cubeField =
        SizingFieldBuilder3D::build(cube->getGeometryCollection(), cube->getTopology());

    const double thinSize = thinField.evaluate(Point3D(5.0, 5.0, THICKNESS / 2.0));
    const double cubeSize = cubeField.evaluate(Point3D(5.0, 5.0, 5.0));

    EXPECT_LT(thinSize, cubeSize);
    // Two elements across a 0.2 gap; the envelope may only relax that with
    // distance, never tighten it, so mid-plate must still be near thickness/2.
    EXPECT_LT(thinSize, THICKNESS);
}

TEST(SizingFieldBuilder3DTest, ClosedSeamCurveIsNotMistakenForAThinFeature)
{
    // A closed curve's first and last samples occupy the same point in space
    // while sitting a full circumference apart along the curve. Measuring the
    // route between them the long way round makes the seam look like a gap of
    // zero width, which collapses the whole field onto its minimum-size floor.
    // On this cylinder nothing is finer than the curved wall demands, so the
    // smallest source must be the wall's own curvature answer.
    constexpr double RADIUS = 3.0;
    constexpr double TOLERANCE = 0.1;

    auto converter = convert(
        BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)), RADIUS, 8.0)
            .Shape());

    SizingFieldSettings3D settings;
    settings.chordDeviationTolerance = TOLERANCE;

    const auto field =
        SizingFieldBuilder3D::build(converter->getGeometryCollection(), converter->getTopology(), settings);

    EXPECT_NEAR(field.getMinimumSourceSize(), std::sqrt(8.0 * TOLERANCE * RADIUS), 1e-6);
}

TEST(SizingFieldBuilder3DTest, FieldIsGradientLimitedAcrossTheWholeModel)
{
    auto converter = convert(
        BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)), 3.0, 8.0)
            .Shape());

    SizingFieldSettings3D settings;
    settings.gradientLimit = 0.25;

    const auto field =
        SizingFieldBuilder3D::build(converter->getGeometryCollection(), converter->getTopology(), settings);

    const Point3D a(3.0, 0.0, 0.0);
    const Point3D b(0.0, 3.0, 8.0);

    EXPECT_LE(std::abs(field.evaluate(a) - field.evaluate(b)),
              settings.gradientLimit * (a - b).norm() + 1e-12);
}

TEST(SizingFieldBuilder3DTest, WedgeCornerDoesNotMakeTheFieldSamplingDependent)
{
    // Two curves leaving a shared corner at angle theta, sampled at distance
    // r along each, are 2*r*sin(theta/2) apart with a route of 2*r along the
    // geometry -- so the "straight line is a shortcut" test fires for every
    // such pair once theta is small, all the way into the corner, while the
    // separation itself tends to zero there. Local feature size is therefore
    // unbounded below in a wedge, and what it reports is the probe density
    // rather than the shape.
    //
    // A wedge tapered to a ridge has exactly that: the two sloping edges of
    // each gable meet at the ridge corner at a few degrees. The field's
    // finest size is a property of the geometry, so refining the probe
    // density must not move it. Before corner-only pairs were excluded it
    // halved every time samplesPerCurve doubled, which silently turned any
    // consumer deriving a length from it into a function of the sampling.
    constexpr double WIDTH = 0.5;
    constexpr double DEPTH = 4.0;
    constexpr double HEIGHT = 5.0;

    auto converter = convert(BRepPrimAPI_MakeWedge(WIDTH, DEPTH, HEIGHT, 0.0).Shape());

    const auto minimumSizeAt = [&converter](std::size_t samplesPerCurve)
    {
        SizingFieldSettings3D settings;
        settings.samplesPerCurve = samplesPerCurve;
        return SizingFieldBuilder3D::build(converter->getGeometryCollection(),
                                           converter->getTopology(),
                                           settings)
            .getMinimumSourceSize();
    };

    const double coarse = minimumSizeAt(16);
    const double medium = minimumSizeAt(32);
    const double fine = minimumSizeAt(64);

    EXPECT_GT(coarse, 0.0);
    EXPECT_NEAR(medium, coarse, 0.05 * coarse);
    EXPECT_NEAR(fine, coarse, 0.05 * coarse);
}
