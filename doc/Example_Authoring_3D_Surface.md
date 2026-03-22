# 3D Surface Meshing — Example Authoring Guide

Quick reference for writing new 3D surface meshing examples. Read this instead of reverse-engineering existing examples.

---

## File Layout

- Source: `src/Examples/3D/Surface/<Name>.cc`
- Register: add `<Name>.cc` to `EXAMPLE_SOURCES_3D_SURFACE` in `src/Examples/3D/Surface/CMakeLists.txt`
- Output: `build/src/Examples/3D/Surface/<Name>` (executable), `<Name>.vtu` (mesh output)

---

## Pipeline Overview

```
CAD shape (OCC)
  → TopoDS_ShapeConverter          (extract geometry + topology)
  → SurfaceMeshingContext3D ctor   (S1: discretize edges + initial 2D Delaunay per face)
  → refineSurfaces()               (S2–S3: quality refinement)
  → buildSurfaceMesh()             (assemble global SurfaceMesh3D)
  → VtkExporter::writeSurfaceMesh  (write .vtu)
```

---

## Standard Includes

```cpp
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/Surface/SurfaceMeshingContext3D.h"
#include "Meshing/Core/3D/Surface/SurfaceMesh3DQualitySettings.h"
#include "Meshing/Core/3D/Surface/SurfaceMesher3D.h"   // only if using high-level API
#include "Readers/OpenCascade/TopoDS_ShapeConverter.h"
#include "Export/VtkExporter.h"
#include "Common/Logging.h"
#include "spdlog/spdlog.h"

// OpenCASCADE shape creation
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <numbers>    // std::numbers::pi
```

---

## Two Usage Patterns

### Pattern A — High-Level (`SurfaceMesher3D`)

Runs the full S1–S3 pipeline in one call. Use when you just want the final mesh.

```cpp
int main()
{
    Common::initLogging();

    // 1. Build CAD shape
    TopoDS_Shape shape = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();

    // 2. Convert
    Readers::TopoDS_ShapeConverter converter(shape);

    // 3. Settings
    Geometry3D::DiscretizationSettings3D settings(std::nullopt, std::numbers::pi / 8.0, 2);

    // 4. Run full pipeline
    Meshing::SurfaceMesher3D mesher(
        converter.getGeometryCollection(),
        converter.getTopology(),
        settings,
        Meshing::SurfaceMesh3DQualitySettings{});
    auto surfaceMesh = mesher.mesh();  // S1 + S2 + S3, one shot

    // 5. Export
    Export::VtkExporter exporter;
    exporter.writeSurfaceMesh(surfaceMesh, "MyExample.vtu");

    return 0;
}
```

### Pattern B — Low-Level (`SurfaceMeshingContext3D`)

Gives access to intermediate S1 results and allows exporting before/after refinement.

```cpp
int main()
{
    Common::initLogging();

    TopoDS_Shape shape = /* ... */;
    Readers::TopoDS_ShapeConverter converter(shape);
    Geometry3D::DiscretizationSettings3D settings(std::nullopt, std::numbers::pi / 8.0, 2);

    // S1 runs at construction
    Meshing::SurfaceMeshingContext3D context(
        converter.getGeometryCollection(),
        converter.getTopology(),
        settings,
        Meshing::SurfaceMesh3DQualitySettings{});

    // Optional: inspect/export S1 output
    const auto& discResult = context.getDiscretizationResult();
    const auto subfacets = context.getFacetTriangulationManager().getAllSubfacets();

    Export::VtkExporter exporter;
    exporter.writeEdgeMesh(discResult, "MyExampleEdges.vtu");                        // boundary edges only
    exporter.writeSurfaceMesh(discResult, subfacets, "MyExampleInitial.vtu");        // pre-refinement

    // S2–S3
    context.refineSurfaces();
    auto surfaceMesh = context.buildSurfaceMesh();
    exporter.writeSurfaceMesh(surfaceMesh, "MyExampleRefined.vtu");

    return 0;
}
```

---

## CAD Shape Construction

All shapes are built with OpenCASCADE. Common primitives:

```cpp
// Box (axis-aligned, corner at origin)
TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();

// Cylinder
gp_Ax2 axis(gp_Pnt(cx, cy, cz), gp_Dir(0.0, 0.0, 1.0));
TopoDS_Shape cylinder = BRepPrimAPI_MakeCylinder(axis, radius, height).Shape();

// Boolean subtraction (cut a hole)
TopoDS_Shape result = BRepAlgoAPI_Cut(box, cylinder).Shape();

// Boolean union
TopoDS_Shape result = BRepAlgoAPI_Fuse(shapeA, shapeB).Shape();
```

To load from STEP file instead: use `Readers::StepReader3D` (see `MeshStepFile2D.cc` for pattern, adapting to 3D).

---

## Discretization Settings

```cpp
// Angle-based (adaptive, recommended): inserts points where tangent changes > angle
Geometry3D::DiscretizationSettings3D settings(
    std::nullopt,             // numSegmentsPerEdge = auto
    std::numbers::pi / 8.0,  // maxAngle = 22.5° (use pi/4 = 45° for coarser, pi/16 for finer)
    2);                       // surface samples per UV direction (interior points per face)

// Fixed-count (uniform): divides every edge into N equal segments
Geometry3D::DiscretizationSettings3D settings(8, 2);
```

**Guideline**: `π/8` (22.5°) is the standard used in all existing examples. Increase `numSamplesPerSurfaceDirection` (e.g. 4) for surfaces with little curvature that still need interior seed points.

---

## Quality Settings

```cpp
Meshing::SurfaceMesh3DQualitySettings quality;
// Defaults are fine for most examples:
//   circumradiusToEdgeRatio = 2.0  (~30° min angle)
//   minAngleDegrees         = 30.0
//   elementLimit            = 50000
//   chordDeviationTolerance = 0.0  (disabled)
```

Enable chord deviation for geometric fidelity on curved surfaces:
```cpp
quality.chordDeviationTolerance = 0.05;  // refine until chord error < 0.05 units
```

---

## Export Methods

```cpp
Export::VtkExporter exporter;

// S1 output: boundary edge polylines (no faces)
exporter.writeEdgeMesh(discResult, "edges.vtu");

// S1 output: initial triangulation (before refinement), needs subfacets from FacetTriangulationManager
const auto subfacets = context.getFacetTriangulationManager().getAllSubfacets();
exporter.writeSurfaceMesh(discResult, subfacets, "initial.vtu");

// Final output: fully refined and assembled SurfaceMesh3D
exporter.writeSurfaceMesh(surfaceMesh, "refined.vtu");
```

---

## Running

```bash
# Normal run
SPDLOG_LEVEL=info ./build/src/Examples/3D/Surface/<Name>

# With mesh integrity checks (slow for large meshes)
CHECK_MESH_EACH_ITERATION=1 SPDLOG_LEVEL=info ./build/src/Examples/3D/Surface/<Name>

# View output
paraview <Name>.vtu
```

---

## Existing Examples

| Example | Shape | Pattern | Exports | Notes |
|---------|-------|---------|---------|-------|
| `CylinderSurfaceMesh` | Cylinder | Both A and B | Edges + final | Shows seam-edge handling |
| `SurfaceMeshEdges` | Box − cylinder | B (low-level) | Edges + initial + refined | Shows S1/S3 intermediate outputs |
| `BoxWithHoleSurface` | Box − cylinder | B (low-level) | Edges + initial + refined | Minimal complete example |
