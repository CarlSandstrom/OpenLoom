# Surface Mesh Quality

This document describes how triangle quality is evaluated and enforced during 3D surface mesh refinement. RCDT applies CGAL Mesh_3's facet criteria in `SurfaceDelaunayRefiner`. The legacy UV-space quality controllers (`Shewchuk2DQualityController`, `SurfaceMeshQualityController`) used by the old per-face mesher are documented at the end for reference.

---

## RCDT facet criteria

`SurfaceFacetCriteria` (`src/Meshing/Core/3D/RCDT/SurfaceFacetCriteria.h`) reads its bounds from `SurfaceMesh3DQualitySettings`:

| Setting | Criterion |
|-------|-------------|
| `minAngleDegrees` (30) | **shape**: a restricted facet is bad when sin² of its smallest angle is below sin²(`minAngleDegrees`) — CGAL's `facet_angle` |
| `chordDeviationTolerance` (0.1) | **distance**: bad when the facet's orthocentre lies further than this from its surface Delaunay ball centre (where its dual edge crosses the surface) — CGAL's `facet_distance` |
| — | **same patch**: bad when two of its surface-interior vertices lie on different surfaces — CGAL's `FACET_VERTICES_ON_SAME_SURFACE_PATCH` |

The first failing criterion, then how badly it fails, orders the refinement queue: worst first.

**Exemption at protecting balls** (`ProtectionExemption`, CGAL's `Facet_criterion_visitor_with_features`): a facet touching protected (weighted) vertices is exempt from the shape criterion, and when much smaller than the balls also from the distance criterion. Protection already guarantees the mesh at a crease; refining there would only place points inside the balls, where they are refused. Triangles under 30° therefore remain next to creases, as in CGAL's own output.

---

## How SurfaceDelaunayRefiner uses them

`SurfaceDelaunayRefiner` (`src/Meshing/Core/3D/RCDT/SurfaceDelaunayRefiner.h`):

1. Classify every face of the seeded tetrahedralization with `WeightedDualRestriction`; queue the restricted facets that fail a criterion.
2. Take the worst facet and insert its surface Delaunay ball centre by Bowyer-Watson — unless no tetrahedron beside the facet conflicts with it, or it is hidden by or coincident with an existing vertex; then drop the facet.
3. Reclassify only the faces the insertion created or destroyed.
4. Repeat until the queue is empty or `maxRefinementIterations` insertions have been made.

There is no size floor and no curve splitting: the protecting balls are fixed after seeding. For volumes, `TetrahedronDelaunayRefiner` runs below this level and bounds `tetCircumradiusToShortestEdgeRatio` (default 2.5); see `doc/RCDT_Techniques.md`.

After refinement, `SurfaceMeshSmoother` moves interior vertices toward their neighbours' centroid and re-projects them onto their surface; it improves most of the exempt triangles at the creases.

---

## Legacy UV-Space Quality Controllers

The following quality infrastructure belongs to the legacy UV-space surface mesher (`SurfaceMesher3D` / `SurfaceMeshingContext3D`) and the 2D mesher. They are retained for the 2D meshing pipeline and for reference.

### Quality Controller Interface

`IQualityController2D` (`src/Meshing/Interfaces/IQualityController2D.h`) is the abstraction used by `ShewchukRefiner2D`:

```cpp
bool isMeshAcceptable(const MeshData2D& data) const;
bool isTriangleAcceptable(const TriangleElement& element) const;
double getTargetElementQuality() const;
std::size_t getElementLimit() const;
```

- `isMeshAcceptable` — returns true when the whole mesh satisfies the quality goal or when the element count has reached `getElementLimit()`.
- `isTriangleAcceptable` — returns true when a single triangle satisfies the quality goal.
- `getTargetElementQuality` — returns the circumradius-to-shortest-edge bound used to sort triangles by distance from the target.
- `getElementLimit` — safety cap on the number of triangles per face.

### Shewchuk2DQualityController — UV-space angle quality

`Shewchuk2DQualityController` (`src/Meshing/Core/2D/Shewchuk2DQualityController.h/.cpp`) evaluates triangles in UV (parametric) space. Used by the 2D mesher and the legacy UV-space surface mesher.

**Constructor parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `meshData` | `const MeshData2D&` | The face's UV-space mesh |
| `circumradiusToShortestEdgeRatioBound` | `double` | Maximum allowed circumradius / shortest-edge ratio (default 2.0) |
| `minAngleThresholdRadians` | `double` | Minimum interior angle in radians (default corresponds to 30°) |
| `elementLimit` | `size_t` | Safety cap on triangle count (default 50 000) |

A triangle is acceptable if its circumradius-to-shortest-edge ratio and minimum interior angle both satisfy their bounds, computed in flat UV space.

### SurfaceMeshQualityController — 3D quality with optional chord deviation

`SurfaceMeshQualityController` (`src/Meshing/Core/3D/Surface/SurfaceMeshQuality.h/.cpp`) lifts triangles from UV space to 3D before evaluating them. Used by the legacy UV-space surface mesher's second refinement phase.

**Constructor parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `meshData` | `const MeshData2D&` | The face's UV-space mesh |
| `surface` | `const ISurface3D&` | The CAD surface for UV→3D evaluation |
| `circumradiusToShortestEdgeRatioBound` | `double` | Maximum ratio in 3D |
| `minAngleThresholdRadians` | `double` | Minimum interior angle in 3D |
| `elementLimit` | `size_t` | Safety cap |
| `chordDeviationTolerance` | `double` | Maximum chord height; 0 disables (default 0.0) |

A triangle is acceptable when its 3D circumradius ratio, minimum angle, and chord deviation (sampled at the centroid and three edge midpoints) all satisfy their bounds.

### Two-phase refinement in the legacy surface mesher

`SurfaceMeshingContext3D::refineSurfaces` runs two sequential phases:

- **Phase 1 — Angle quality**: `Shewchuk2DQualityController` drives `ShewchukRefiner2D` on each face until UV-space angle criteria are met. Cross-face boundary splits are propagated via `TwinManager`.

- **Phase 2 — Chord deviation** (when `chordDeviationTolerance > 0`): `SurfaceMeshQualityController` (with angle bounds disabled) drives `ShewchukRefiner2D` to subdivide triangles whose flat approximation deviates from the CAD surface.

Both phases repeat until a complete sweep over all faces produces no new cross-face boundary splits.
