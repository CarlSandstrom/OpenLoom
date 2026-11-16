#include "Common/Logging.h"
#include "Common/Types.h"
#include "Export/VtkExporter.h"
#include "Geometry/2D/Base/Corner2D.h"
#include "Geometry/2D/Base/GeometryCollection2D.h"
#include "Geometry/2D/Base/LinearEdge2D.h"
#include "Meshing/Core/2D/BoundaryDiscretizer2D.h"
#include "Meshing/Core/2D/ConstrainedDelaunay2D.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Meshing/Core/2D/ShewchukRefiner2D.h"
#include "Meshing/Data/2D/Mesh2DQualitySettings.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Topology2D/Topology2D.h"
#include "spdlog/spdlog.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

using namespace Meshing;

namespace
{
constexpr auto OUTPUT_FILE_NAME = "NearCoincidentConstraintEdges.vtu";

constexpr double SQUARE_SIZE = 100.0;
constexpr double CRACK_UPPER_ENTRY_Y = 50.5;
constexpr double CRACK_LOWER_ENTRY_Y = 49.5;
constexpr double CRACK_TIP_X = 90.0;
constexpr double CRACK_TIP_Y = 50.0;

void addCorner(Geometry2D::GeometryCollection2D& geometry, const std::string& id, const Point2D& position)
{
    geometry.addCorner(std::make_unique<Geometry2D::Corner2D>(id, position));
}

void addEdge(Geometry2D::GeometryCollection2D& geometry,
             const std::string& id,
             const Point2D& start,
             const Point2D& end)
{
    geometry.addEdge(std::make_unique<Geometry2D::LinearEdge2D>(id, start, end));
}

std::unique_ptr<Geometry2D::GeometryCollection2D> createGeometry()
{
    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();

    Point2D origin(0.0, 0.0);
    Point2D bottomRight(SQUARE_SIZE, 0.0);
    Point2D topRight(SQUARE_SIZE, SQUARE_SIZE);
    Point2D topLeft(0.0, SQUARE_SIZE);

    // c_upper and c_lower are the crack entry points on the left boundary;
    // c_tip is where the two crack walls nearly coincide.
    Point2D crackUpper(0.0, CRACK_UPPER_ENTRY_Y);
    Point2D crackTip(CRACK_TIP_X, CRACK_TIP_Y);
    Point2D crackLower(0.0, CRACK_LOWER_ENTRY_Y);

    addCorner(*geometry, "c0", origin);
    addCorner(*geometry, "c1", bottomRight);
    addCorner(*geometry, "c2", topRight);
    addCorner(*geometry, "c3", topLeft);

    addCorner(*geometry, "c_upper", crackUpper);
    addCorner(*geometry, "c_tip", crackTip);
    addCorner(*geometry, "c_lower", crackLower);

    // Outer square edges (counter-clockwise)
    addEdge(*geometry, "e0", origin, bottomRight);
    addEdge(*geometry, "e1", bottomRight, topRight);
    addEdge(*geometry, "e2", topRight, topLeft);
    addEdge(*geometry, "e3", topLeft, crackUpper);

    // Crack wall edges, nearly coincident, meeting at ~0.64 degrees at the tip
    addEdge(*geometry, "e_upper", crackUpper, crackTip);
    addEdge(*geometry, "e_lower", crackTip, crackLower);

    // Left-lower segment from crack bottom entry to domain origin
    addEdge(*geometry, "e4", crackLower, origin);

    return geometry;
}

std::unique_ptr<Topology2D::Topology2D> createTopology()
{
    std::unordered_map<std::string, Topology2D::Corner2D> topologyCorners;
    topologyCorners.emplace("c0", Topology2D::Corner2D("c0", {"e0", "e4"}));
    topologyCorners.emplace("c1", Topology2D::Corner2D("c1", {"e1", "e0"}));
    topologyCorners.emplace("c2", Topology2D::Corner2D("c2", {"e2", "e1"}));
    topologyCorners.emplace("c3", Topology2D::Corner2D("c3", {"e3", "e2"}));
    topologyCorners.emplace("c_upper", Topology2D::Corner2D("c_upper", {"e_upper", "e3"}));
    topologyCorners.emplace("c_tip", Topology2D::Corner2D("c_tip", {"e_lower", "e_upper"}));
    topologyCorners.emplace("c_lower", Topology2D::Corner2D("c_lower", {"e4", "e_lower"}));

    std::unordered_map<std::string, Topology2D::Edge2D> topologyEdges;
    topologyEdges.emplace("e0", Topology2D::Edge2D("e0", "c0", "c1"));
    topologyEdges.emplace("e1", Topology2D::Edge2D("e1", "c1", "c2"));
    topologyEdges.emplace("e2", Topology2D::Edge2D("e2", "c2", "c3"));
    topologyEdges.emplace("e3", Topology2D::Edge2D("e3", "c3", "c_upper"));
    topologyEdges.emplace("e_upper", Topology2D::Edge2D("e_upper", "c_upper", "c_tip"));
    topologyEdges.emplace("e_lower", Topology2D::Edge2D("e_lower", "c_tip", "c_lower"));
    topologyEdges.emplace("e4", Topology2D::Edge2D("e4", "c_lower", "c0"));

    std::vector<std::string> boundaryEdgeLoop = {"e0", "e1", "e2", "e3", "e_upper", "e_lower", "e4"};

    return std::make_unique<Topology2D::Topology2D>(topologyCorners, topologyEdges, boundaryEdgeLoop);
}
} // namespace

int main()
{
    Common::initializeLogging();

    // Stress test: 100x100 square with a very thin crack entering from the left boundary.
    //
    // The crack runs from (0, 50.5) and (0, 49.5) on the left edge to a shared tip at
    // (90, 50.0). The two crack-wall constraint edges are nearly coincident — they diverge
    // by only 1 unit over 90 units of length (aspect ratio 90:1), meeting at the tip with
    // an interior angle of ~0.64°. At the tip, the two edges are nearly parallel, producing
    // a near-degenerate vertex configuration. The crack interior (the thin triangular region)
    // is included in the mesh. This stresses CDT constraint edge insertion near nearly-
    // coincident segments and the Shewchuk refiner's behaviour adjacent to the acute tip.

    spdlog::info("Creating near-coincident constraint edges stress test");

    MeshingContext2D context(createGeometry(), createTopology());

    BoundaryDiscretizer2D discretizer(context);
    auto discretization = discretizer.discretize();

    spdlog::info("Generating constrained Delaunay triangulation...");
    ConstrainedDelaunay2D::triangulate(context, discretization);

    spdlog::info("Refining mesh with ShewchukRefiner2D...");
    ShewchukRefiner2D refiner(context, Meshing::Mesh2DQualitySettings{});
    refiner.refine();

    spdlog::info("Near-coincident constraint edges mesh generation complete!");

    Export::VtkExporter exporter;
    exporter.exportMesh(context.getMeshData(), OUTPUT_FILE_NAME);

    return 0;
}
