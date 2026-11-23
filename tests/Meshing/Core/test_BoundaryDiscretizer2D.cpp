#include <gtest/gtest.h>

#include "Common/Types.h"
#include "Geometry/2D/Base/Corner2D.h"
#include "Geometry/2D/Base/GeometryCollection2D.h"
#include "Geometry/2D/Base/IEdge2D.h"
#include "Geometry/2D/Base/LinearEdge2D.h"
#include "Meshing/Core/2D/General/BoundaryDiscretizer2D.h"
#include "Meshing/Core/2D/General/MeshingContext2D.h"
#include "Topology2D/Corner2D.h"
#include "Topology2D/Edge2D.h"
#include "Topology2D/Topology2D.h"

#include <cmath>
#include <functional>
#include <memory>
#include <numbers>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

using namespace Meshing;

namespace
{

// An edge given by a parametric function on [parameterMin, parameterMax].
class FunctionEdge2D : public Geometry2D::IEdge2D
{
public:
    FunctionEdge2D(std::string id, std::function<Point2D(double)> function, double parameterMin, double parameterMax) :
        id_(std::move(id)),
        function_(std::move(function)),
        parameterMin_(parameterMin),
        parameterMax_(parameterMax)
    {
    }

    Point2D getPoint(double parameter) const override { return function_(parameter); }
    Point2D getStartPoint() const override { return function_(parameterMin_); }
    Point2D getEndPoint() const override { return function_(parameterMax_); }
    std::pair<double, double> getParameterBounds() const override { return {parameterMin_, parameterMax_}; }
    double getLength() const override { return (getEndPoint() - getStartPoint()).norm(); }
    std::string getId() const override { return id_; }

private:
    std::string id_;
    std::function<Point2D(double)> function_;
    double parameterMin_;
    double parameterMax_;
};

// The curve "curve", closed by the straight edge "chord" back to its start.
MeshingContext2D makeClosedCurveContext(std::function<Point2D(double)> function, double parameterMin, double parameterMax)
{
    auto curve = std::make_unique<FunctionEdge2D>("curve", std::move(function), parameterMin, parameterMax);
    Point2D start = curve->getStartPoint();
    Point2D end = curve->getEndPoint();

    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("start", start));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("end", end));
    geometry->addEdge(std::move(curve));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("chord", end, start));

    std::unordered_map<std::string, Topology2D::Corner2D> corners;
    corners.emplace("start", Topology2D::Corner2D("start", std::set<std::string>{"chord", "curve"}));
    corners.emplace("end", Topology2D::Corner2D("end", std::set<std::string>{"curve", "chord"}));
    std::unordered_map<std::string, Topology2D::Edge2D> edges;
    edges.emplace("curve", Topology2D::Edge2D("curve", "start", "end"));
    edges.emplace("chord", Topology2D::Edge2D("chord", "end", "start"));

    auto topology = std::make_unique<Topology2D::Topology2D>(corners, edges, std::vector<std::string>{"curve", "chord"});
    return MeshingContext2D(std::move(geometry), std::move(topology));
}

size_t countInteriorPoints(const DiscretizationResult2D& discretization, const std::string& edgeId)
{
    return discretization.edgeIdToPointIndicesMap.at(edgeId).size() - 2;
}

} // namespace

// The angle limit applies to the tangent's accumulated turn, as in
// BoundaryDiscretizer3D. y = 0.3 sin(x) over a period swings its tangent by
// +/-16.7 degrees -- never 45 degrees from any earlier tangent -- but turns
// about 67 degrees in total, so a 45-degree limit must place a point.
TEST(BoundaryDiscretizer2D, AngleMode_CountsTheAccumulatedTurnOfAnSCurve)
{
    auto context = makeClosedCurveContext([](double t)
                                          { return Point2D(t, 0.3 * std::sin(t)); }, 0.0, 2.0 * std::numbers::pi);
    BoundaryDiscretizer2D discretizer(context, Geometry2D::DiscretizationSettings2D(std::nullopt, std::numbers::pi / 4.0));

    auto discretization = discretizer.discretize();

    EXPECT_EQ(countInteriorPoints(discretization, "curve"), 1u);
}

// Only the end vertex itself is skipped, not points within an absolute distance
// of a corner: on a micrometre arc every interior point lies that close.
TEST(BoundaryDiscretizer2D, AngleMode_KeepsPointsNearTheCornersOfATinyArc)
{
    constexpr double RADIUS = 1e-6;
    auto context = makeClosedCurveContext([](double t)
                                          { return Point2D(RADIUS * std::cos(t), RADIUS * std::sin(t)); },
                                          0.0, std::numbers::pi / 2.0);
    BoundaryDiscretizer2D discretizer(context, Geometry2D::DiscretizationSettings2D(std::nullopt, std::numbers::pi / 8.0));

    auto discretization = discretizer.discretize();

    EXPECT_EQ(countInteriorPoints(discretization, "curve"), 3u);
}
