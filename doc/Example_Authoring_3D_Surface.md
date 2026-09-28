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
  → TopoDS_ShapeConverter              (extract geometry + topology)
  → SurfaceMesher3D ctor               (holds an RCDTMesher)
  → mesh()                             (discretize, protect, refine, extract, smooth → SurfaceMesh3D)
  → VtkExporter::writeSurfaceMesh      (write .vtu)
```

`BoundaryDiscretizer3D::discretize` can be called on its own to export the boundary discretization — it is the same call the mesher makes first.

---

## Standard Includes

```cpp
#include "Common/Logging.h"
#include "Export/TsvExporter.h"   // only for a golden model
#include "Export/VtkExporter.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"   // only to export the edges
#include "Meshing/Data/3D/DiscretizationResult3D.h"  // only to export the edges
#include "Meshing/Core/3D/Surface/SurfaceMesher3D.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"
#include "Readers/OpenCascade/TopoDS_ShapeConverter.h"

// OpenCASCADE shape creation
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <numbers>    // std::numbers::pi
```

---

## Minimal Example

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

    // 4. Optional: export the boundary discretization on its own
    const auto discretization = Meshing::BoundaryDiscretizer3D::discretize(
        converter.getGeometryCollection(), converter.getTopology(), settings);
    Export::VtkExporter exporter;
    exporter.writeEdgeMesh(*discretization, "MyExampleEdges.vtu");

    // 5. Mesh
    Meshing::SurfaceMesher3D mesher(
        converter.getGeometryCollection(),
        converter.getTopology(),
        settings,
        Meshing::SurfaceMesh3DQualitySettings{});
    auto surfaceMesh = mesher.mesh();

    // 6. Export
    exporter.writeSurfaceMesh(surfaceMesh, "MyExample.vtu");

    return 0;
}
```

For a volume mesh use `VolumeMesher3D` the same way (see `src/Examples/3D/Volume/`).

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

There is no 3D STEP reader class yet (`StepReader2D` is 2D only). `TopoDS_ShapeConverter` takes any `TopoDS_Shape`, so a STEP file can be read with OCC's `STEPControl_Reader` and its shape passed in.

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
//   minAngleDegrees         = 30.0   (facet shape criterion)
//   chordDeviationTolerance = 0.1    (facet distance criterion; 0 disables)
//   smoothingIterations     = 5
//   minimumEdgeLength       = unset  (derived from the geometry)
```

Tighten chord deviation for geometric fidelity on curved surfaces:
```cpp
quality.chordDeviationTolerance = 0.05;  // refine until chord error < 0.05 units
```

See `doc/Surface_Mesh_Quality.md` for what each criterion does.

---

## Export Methods

```cpp
Export::VtkExporter exporter;

// Boundary edge polylines (no faces), coloured by EdgeID
exporter.writeEdgeMesh(*discretization, "edges.vtu");

// Final SurfaceMesh3D, coloured by SurfaceID
exporter.writeSurfaceMesh(surfaceMesh, "mesh.vtu");

// Golden models also write TSV, which scripts/refactor-check.sh diffs
Export::TsvExporter::writeDiscretization(*discretization, "edges");
Export::TsvExporter::writeSurfaceMesh(surfaceMesh, "mesh");
```

---

## Running

```bash
SPDLOG_LEVEL=info ./build/src/Examples/3D/Surface/<Name>

# View output
paraview <Name>.vtu
```

`CHECK_MESH_EACH_ITERATION=1` does nothing in 3D.

---

## Existing Examples

| Example | Shape | Exports | Notes |
|---------|-------|---------|-------|
| `CylinderSurfaceMesh` | Cylinder | Edges + mesh | Seam-edge handling; golden (fast tier) |
| `HexNutSurfaceMesh` | Hexagonal nut | Edges + mesh | Planar faces plus a periodic bore; golden (fast tier) |
| `BoxWithHoleSurface` | Box − cylinder | Edges + mesh | Minimal complete example; golden (full tier) |
| `SaddleSurfaceMesh` | Hyperbolic paraboloid solid | Edges + mesh | Crease-protection benchmark; golden (full tier) |
| `TorusSurfaceMesh` | Torus | Edges + mesh | Doubly periodic single face |
| `SharpCreaseBracket` | Bent bracket | Edges + mesh | 20° crease |
| `ThinFinSurfaceMesh` | 100×10×0.5 box | Edges + mesh | Thin plate; slow (OPE-213) |
| `HexNutChamferedSurfaceMesh` | Hex nut, chamfered bore | Edges + mesh | Many coplanar boundary points (OPE-173) |
