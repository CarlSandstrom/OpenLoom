# Surface Mesh Quality

This document describes how triangle quality is evaluated and enforced during 3D surface mesh refinement. The primary mechanism is `RCDTQualitySettings` used by `RCDTRefiner`. The legacy UV-space quality controllers (`Shewchuk2DQualityController`, `SurfaceMeshQualityController`) used by the old per-face mesher are documented at the end for reference.

---

## RCDT Quality Settings

`RCDTQualitySettings` (`src/Meshing/Core/3D/RCDT/RCDTQualitySettings.h`) is a plain struct that carries the two quality bounds consumed by `RCDTRefiner`:

```cpp
struct RCDTQualitySettings
{
    double maximumCircumradiusToShortestEdgeRatio = 1.0;
    double maximumChordDeviation                  = 0.1;
};
```

| Field | Description |
|-------|-------------|
| `maximumCircumradiusToShortestEdgeRatio` | Maximum allowed ratio of the circumradius of a restricted triangle to its shortest edge, measured in 3D ambient space |
| `maximumChordDeviation` | Maximum allowed distance from the flat triangle to the actual CAD surface, in model units |

A restricted triangle fails quality when either bound is exceeded. Both checks are evaluated in 3D ambient space against the actual CAD surface geometry.

---

## How RCDTRefiner Uses Quality Settings

`RCDTRefiner` (`src/Meshing/Core/3D/RCDT/RCDTRefiner.h`) drives refinement via a two-priority loop. Quality settings are consulted through `RestrictedTriangulation::getBadTriangles`.

**Main loop (`refine`):**
```
while any work remains:
    refineStep()
```

**Each refinement step (`refineStep`):**

1. **Priority 1 — split encroached curve segments**: A `CurveSegment` is encroached if any mesh vertex other than its endpoints lies inside or on its diametral sphere (the sphere whose diameter equals the segment length). If any encroached segments exist, split the first one at its arc-length midpoint via Bowyer-Watson. Update `RestrictedTriangulation` incrementally after insertion.

2. **Priority 2 — split bad restricted triangles** (only when no encroached segments remain): Call `restrictedTriangulation.getBadTriangles(settings, meshData, geometry)` to find restricted faces that violate `RCDTQualitySettings`. For each bad triangle:
   - Compute its circumcircle center (in the surface's tangent plane).
   - If inserting that point would encroach a curve segment, do not insert; split the encroached segment instead (back to Priority 1).
   - Otherwise, insert the circumcircle center via Bowyer-Watson and update `RestrictedTriangulation`. The bad triangle is eliminated because its circumsphere is no longer empty.

The circumcenter demotion rule (Priority 2 falling back to Priority 1) is the key termination guarantee: it prevents arbitrarily small insertions near curve segments.

**Quality evaluation in `getBadTriangles`:**

For each restricted face (a face of the tetrahedralization restricted to some surface S):
1. Lift the three nodes to 3D and compute the circumradius and shortest edge length in ambient space.
2. If `circumradius / shortestEdge > maximumCircumradiusToShortestEdgeRatio`, the triangle is bad.
3. Sample the distance from the flat triangle to the CAD surface at the circumcenter and edge midpoints. If any sample exceeds `maximumChordDeviation`, the triangle is bad.

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
