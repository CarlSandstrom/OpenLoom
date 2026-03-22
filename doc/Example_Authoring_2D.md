# 2D Example Authoring Guide

Quick reference for writing new 2D stress-test examples. Read this instead of reverse-engineering existing examples.

---

## File Layout

- Source: `src/Examples/2D/<Name>.cc`
- Register: add `<Name>.cc` to `EXAMPLE_SOURCES_2D` in `src/Examples/2D/CMakeLists.txt`
- Output: `build/src/Examples/2D/<Name>` (executable), `<Name>.vtu` (mesh output)

---

## Standard Includes

```cpp
#include "Export/VtkExporter.h"
#include "Geometry/2D/Base/Corner2D.h"
#include "Geometry/2D/Base/GeometryCollection2D.h"
#include "Geometry/2D/Base/LinearEdge2D.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DCorner.h"   // curved corners
#include "Geometry/2D/OpenCascade/OpenCascade2DEdge.h"     // curved edges
#include "Meshing/Core/2D/ConstrainedDelaunay2D.h"
#include "Meshing/Core/2D/EdgeDiscretizer2D.h"
#include "Meshing/Core/2D/Mesh2DQualitySettings.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Meshing/Core/2D/ShewchukRefiner2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Topology2D/Topology2D.h"
#include "Common/Logging.h"
#include "spdlog/spdlog.h"

// Only when using OCC curved geometry (circles, arcs):
#include <Geom2d_Circle.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <gp_Ax2d.hxx>
#include <gp_Circ2d.hxx>
#include <gp_Dir2d.hxx>
#include <gp_Pnt2d.hxx>

#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

using namespace Meshing;
```

---

## main() Skeleton

```cpp
int main()
{
    Common::initLogging();

    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();
    std::unordered_map<std::string, Topology2D::Corner2D> topoCorners;
    std::unordered_map<std::string, Topology2D::Edge2D> topoEdges;

    // 1. Build geometry + topology (see helpers below)
    std::vector<std::string> outerEdgeLoop;
    std::vector<std::vector<std::string>> holeEdgeLoops; // empty = no holes

    // 2. Create topology
    auto topology = std::make_unique<Topology2D::Topology2D>(
        topoCorners, topoEdges, outerEdgeLoop, holeEdgeLoops);

    // 3. Create context
    MeshingContext2D context(std::move(geometry), std::move(topology));

    // 4. Discretize
    EdgeDiscretizer2D discretizer(context);
    auto discretization = discretizer.discretize();

    // 5. CDT
    ConstrainedDelaunay2D mesher(context, discretization);
    mesher.triangulate();

    // 6. Refine (optional but typical)
    ShewchukRefiner2D refiner(context, Meshing::Mesh2DQualitySettings{});
    refiner.refine();

    // 7. Export
    Export::VtkExporter exporter;
    exporter.exportMesh(context.getMeshData(), "OutputName.vtu");

    return 0;
}
```

---

## Reusable Helpers

Both helpers below are copy-pasted verbatim across all examples (no shared library — just copy them).

### Square boundary

```cpp
void addSquare(Geometry2D::GeometryCollection2D& geometry,
               std::unordered_map<std::string, Topology2D::Corner2D>& topoCorners,
               std::unordered_map<std::string, Topology2D::Edge2D>& topoEdges,
               std::vector<std::string>& edgeLoop,
               const std::string& prefix,
               double size,
               Point2D origin = Point2D(0.0, 0.0))
{
    std::vector<Point2D> corners = {
        origin,
        Point2D(origin.x() + size, origin.y()),
        Point2D(origin.x() + size, origin.y() + size),
        Point2D(origin.x(), origin.y() + size)};

    for (size_t i = 0; i < 4; ++i)
    {
        std::string cornerId = prefix + "_c" + std::to_string(i);
        std::string edgeId = prefix + "_e" + std::to_string(i);
        std::string prevEdgeId = prefix + "_e" + std::to_string((i + 3) % 4);

        geometry.addCorner(std::make_unique<Geometry2D::Corner2D>(cornerId, corners[i]));
        geometry.addEdge(std::make_unique<Geometry2D::LinearEdge2D>(
            edgeId, corners[i], corners[(i + 1) % 4]));

        topoCorners.emplace(cornerId, Topology2D::Corner2D(cornerId, {edgeId, prevEdgeId}));
        topoEdges.emplace(edgeId, Topology2D::Edge2D(
            edgeId, cornerId, prefix + "_c" + std::to_string((i + 1) % 4)));
        edgeLoop.push_back(edgeId);
    }
}
```

### Circular arc constraint

```cpp
void addCircularConstraint(Geometry2D::GeometryCollection2D& geometry,
                           std::unordered_map<std::string, Topology2D::Corner2D>& topoCorners,
                           std::unordered_map<std::string, Topology2D::Edge2D>& topoEdges,
                           const std::string& prefix,
                           double centerX,
                           double centerY,
                           double radius,
                           size_t numSegments)
{
    gp_Pnt2d center(centerX, centerY);
    gp_Ax2d axis(center, gp_Dir2d(1.0, 0.0));
    gp_Circ2d circle(axis, radius);

    for (size_t i = 0; i < numSegments; ++i)
    {
        double angle = 2.0 * M_PI * i / numSegments;
        double x = centerX + radius * std::cos(angle);
        double y = centerY + radius * std::sin(angle);

        std::string cornerId = prefix + "_c" + std::to_string(i);
        std::string edgeId   = prefix + "_e" + std::to_string(i);
        std::string prevEdgeId = prefix + "_e" + std::to_string((i + numSegments - 1) % numSegments);

        geometry.addCorner(
            std::make_unique<Geometry2D::OpenCascade2DCorner>(gp_Pnt2d(x, y), cornerId));
        topoCorners.emplace(cornerId, Topology2D::Corner2D(cornerId, {edgeId, prevEdgeId}));
    }

    for (size_t i = 0; i < numSegments; ++i)
    {
        double startAngle = 2.0 * M_PI * i / numSegments;
        double endAngle   = 2.0 * M_PI * (i + 1) / numSegments;

        Handle(Geom2d_Circle) circleGeom = new Geom2d_Circle(circle);
        Handle(Geom2d_TrimmedCurve) arc  = new Geom2d_TrimmedCurve(circleGeom, startAngle, endAngle);

        std::string edgeId      = prefix + "_e" + std::to_string(i);
        std::string startCornerId = prefix + "_c" + std::to_string(i);
        std::string endCornerId   = prefix + "_c" + std::to_string((i + 1) % numSegments);

        geometry.addEdge(std::make_unique<Geometry2D::OpenCascade2DEdge>(arc, edgeId));
        topoEdges.emplace(edgeId, Topology2D::Edge2D(edgeId, startCornerId, endCornerId));
    }
}
```

---

## Circles: Hole vs Internal Constraint

| Use case | `holeEdgeLoops` arg | Effect |
|----------|---------------------|--------|
| Excluded region (void) | Pass each circle's edge loop | Flood-fill removes triangles inside |
| Internal partition | Pass empty vector `{}` | Full domain meshed; circles create constraint edges only |

**Collecting a hole loop** (one loop per circle):
```cpp
std::vector<std::string> holeLoop;
for (size_t i = 0; i < numSegments; ++i)
    holeLoop.push_back(prefix + "_e" + std::to_string(i));
holeEdgeLoops.push_back(std::move(holeLoop));
```

---

## numSegments Guidelines

| Value | Chord length (r=5) | When to use |
|-------|--------------------|-------------|
| 8     | ~3.8 units | Quick basic test |
| 16    | ~1.95 units | Standard stress test |
| 24    | ~1.30 units | Higher fidelity |
| 32    | ~0.98 units | Maximum stress (e.g. tightly packed rings) |

---

## Twin Edges (periodic boundary)

Add after building geometry/topology, before `Topology2D` construction. See `SquareWithCircleAndTwinEdges.cc` for full example. Requires `TwinManager` and `BoundarySplitSynchronizer`; pass the synchronizer to `refiner.setOnBoundarySplit(...)`.

---

## Running

```bash
# Normal run
SPDLOG_LEVEL=info ./build/src/Examples/2D/<Name>

# With mesh integrity checks (slow — O(n²) — avoid for large examples)
CHECK_MESH_EACH_ITERATION=1 SPDLOG_LEVEL=info ./build/src/Examples/2D/<Name>

# View output
paraview <Name>.vtu
```

---

## Existing Examples (reference)

| Example | Shape | Circles | Type | Segments | Notes |
|---------|-------|---------|------|----------|-------|
| `SimpleDelaunay2D` | Triangle | — | — | — | Minimal CDT |
| `RectangleWithHole2D` | Rectangle | 1 hole | Excluded | 8 | Basic hole |
| `RectangleWithCrack` | Rectangle | — | — | — | Crack constraint |
| `SquareWithCircularHole` | Square | 2 holes | Excluded | 8 | Multi-hole |
| `SquareWithInternalCircles` | Square | 2 | Internal | 8 | No exclusion |
| `SquareWithCircleAndTwinEdges` | Square | 1 hole | Excluded | 12 | Twin edge sync |
| `ThinSliverDomain` | Sliver | — | — | — | Extreme aspect ratio |
| `StarPolygon` | Star | — | — | — | Sharp angles; no refiner |
| `NestedConcentricRings` | Square | 4 concentric | Internal | 32 | Tight annular channels |
| `ManyCirclesInSquare` | Square | 25 (5×5) | Excluded | 16 | Multi-hole flood-fill |
| `SpiralChannel` | Square | — | — | — | Spiral constraint |
