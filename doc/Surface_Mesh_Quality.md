# Surface Mesh Quality

This document describes how triangle quality is evaluated and enforced during 3D surface mesh refinement. It covers the quality interface, the two concrete controllers, how the Shewchuk refiner uses them, and the two-phase structure of `refineSurfaces`.

---

## Quality Controller Interface

`IQualityController2D` (in `src/Meshing/Interfaces/IQualityController2D.h`) is the single abstraction that the refiner talks to. Any quality strategy must implement four methods:

```cpp
bool isMeshAcceptable(const MeshData2D& data) const;
bool isTriangleAcceptable(const TriangleElement& element) const;
double getTargetElementQuality() const;
std::size_t getElementLimit() const;
```

- `isMeshAcceptable` — returns true when the whole mesh satisfies the quality goal **or** when the element count has reached `getElementLimit()`. The refiner calls this at the top of every iteration to decide whether to stop.
- `isTriangleAcceptable` — returns true when a single triangle satisfies the quality goal. The refiner calls this to identify which triangles need refinement.
- `getTargetElementQuality` — returns the circumradius-to-shortest-edge bound. Used by `ElementQuality2D` to sort triangles by how far they are from the target.
- `getElementLimit` — safety cap on the number of triangles per face. When reached, refinement stops regardless of quality.

---

## Shewchuk2DQualityController — UV-space angle quality

`Shewchuk2DQualityController` (`src/Meshing/Core/2D/Shewchuk2DQualityController.h/.cpp`) evaluates triangles entirely in UV (parametric) space. All distances and angles are computed from the flat 2D coordinates stored in `MeshData2D`.

**Constructor parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `meshData` | `const MeshData2D&` | The face's UV-space mesh |
| `circumradiusToShortestEdgeRatioBound` | `double` | Maximum allowed circumradius / shortest-edge ratio (default 2.0) |
| `minAngleThresholdRadians` | `double` | Minimum interior angle in radians (default corresponds to 30°) |
| `elementLimit` | `size_t` | Safety cap on triangle count (default 50 000) |

**Triangle acceptance rule:**

A triangle is acceptable if and only if:
1. Its circumradius-to-shortest-edge ratio ≤ `circumradiusToShortestEdgeRatioBound`, and
2. Its minimum interior angle ≥ `minAngleThresholdRadians`.

Both metrics are computed in flat UV space using `ElementGeometry2D` and `ElementQuality2D`.

**When to use:** Phase 1 of `refineSurfaces`. This controller is fast and produces reliable angle-quality meshes. It is the primary driver of refinement.

---

## SurfaceMeshQualityController — 3D quality with optional chord deviation

`SurfaceMeshQualityController` (`src/Meshing/Core/3D/Surface/SurfaceMeshQuality.h/.cpp`) lifts triangles from UV space to 3D before evaluating them. It requires access to the CAD surface via `ISurface3D::getPoint(u, v)`.

**Constructor parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `meshData` | `const MeshData2D&` | The face's UV-space mesh |
| `surface` | `const ISurface3D&` | The CAD surface for UV→3D evaluation |
| `circumradiusToShortestEdgeRatioBound` | `double` | Maximum allowed circumradius / shortest-edge ratio in 3D |
| `minAngleThresholdRadians` | `double` | Minimum interior angle in 3D, in radians |
| `elementLimit` | `size_t` | Safety cap on triangle count |
| `chordDeviationTolerance` | `double` | Maximum allowed chord height in model units; 0 disables (default 0.0) |

**Triangle acceptance rule:**

A triangle is acceptable if and only if **all** of the following hold:

1. **Non-degenerate:** shortest 3D edge length ≥ 1e-10.
2. **Circumradius ratio:** 3D circumradius / shortest 3D edge ≤ `circumradiusToShortestEdgeRatioBound`.
3. **Minimum angle:** minimum interior angle of the 3D triangle ≥ `minAngleThresholdRadians`.
4. **Chord deviation** (only when `chordDeviationTolerance > 0`): the distance from the flat triangle to the actual CAD surface, sampled at four UV locations — the triangle centroid and each of the three edge midpoints — must not exceed `chordDeviationTolerance` at any sample point.

For the chord deviation check, each sample is computed as:

```
deviation(u, v, flatPoint) = |surface.getPoint(u, v) − flatPoint|
```

where `flatPoint` is the corresponding point on the flat linear triangle (barycentric centroid or edge midpoint interpolated from the 3D vertex positions).

**When to use:** Phase 2 of `refineSurfaces` (chord deviation pass). In this phase the angle bounds are set to effectively disabled values (`circumradiusToShortestEdgeRatioBound = 1e9`, `minAngleThresholdRadians = 0.0`) so that only chord deviation drives refinement.

---

## How ShewchukRefiner2D uses the quality controller

`ShewchukRefiner2D` (`src/Meshing/Core/2D/ShewchukRefiner2D.h/.cpp`) implements Ruppert's algorithm. It holds a `const IQualityController2D*` and drives refinement as follows:

**Main loop (`refine`):**
```
while !qualityController.isMeshAcceptable(mesh):
    refineStep()
```

**Each refinement step (`refineStep`):**
1. **Priority 1 — encroached segments:** Scan all constrained segments for encroachment (a mesh node falls inside the diametral circle of the segment). If any are found, split the first encroached segment at its midpoint. After a successful boundary split, fire the `onBoundarySplit` callback so twin faces can mirror the split.
2. **Priority 2 — poor-quality triangles:** Sort all triangles by `ElementQuality2D` (worst first). For each triangle, check `qualityController.isTriangleAcceptable`. On the first failing triangle:
   a. Compute its circumcenter.
   b. If the circumcenter would encroach a visible constrained segment, split that segment instead (back to Priority 1).
   c. Otherwise, insert the circumcenter via Bowyer-Watson.

**Safety guards:**
- Hard iteration limit (10 000) prevents infinite loops.
- Triangles that survive circumcenter insertion (circumcenter blocked by a constraint) are added to an `unrefinableTriangles_` set and skipped in future iterations.
- Triangles below minimum area/edge thresholds (1e-12 / 1e-8) are skipped.
- After every 10 successful refinements, exterior triangles are re-classified and removed.
- A final exterior-triangle cleanup runs after the loop exits.

---

## Two-phase refinement in refineSurfaces

`SurfaceMeshingContext3D::refineSurfaces` (`src/Meshing/Core/3D/Surface/SurfaceMeshingContext3D.cpp`) orchestrates the per-face refinement in two sequential phases.

**Signature:**
```cpp
void refineSurfaces(double circumradiusToEdgeRatio = 2.0,
                    double minAngleDegrees = 30.0,
                    size_t elementLimit = 50000,
                    double chordDeviationTolerance = 0.0);
```

### Phase 1 — Angle quality (always runs)

For each face, a `Shewchuk2DQualityController` is constructed with the supplied `circumradiusToEdgeRatio`, `minAngleDegrees`, and `elementLimit`. `ShewchukRefiner2D` is run until the mesh is acceptable.

Passes repeat until a complete sweep over all faces produces no new cross-face boundary splits. A cross-face split occurs when a boundary segment shared with an adjacent face is split: the split is immediately applied to the twin face via the `onBoundarySplit` callback and `TwinManager`, and the outer loop runs another pass to give the twin face a chance to re-refine.

### Phase 2 — Chord deviation (runs only when `chordDeviationTolerance > 0`)

For each face, a `SurfaceMeshQualityController` is constructed with:
- `circumradiusToShortestEdgeRatioBound = 1e9` (effectively disabled)
- `minAngleThresholdRadians = 0.0` (effectively disabled)
- the supplied `chordDeviationTolerance`

`ShewchukRefiner2D` is run on each face with this controller. The same cross-face boundary-split synchronisation applies: the phase repeats until a full sweep produces no new cross-face splits.

Phase 2 only subdivides triangles whose flat approximation deviates from the actual curved surface by more than `chordDeviationTolerance`. It does not re-check angle quality; the angle-quality invariant is maintained because Delaunay circumcenter insertion is generally angle-preserving for interior points.

### Default behaviour

With `chordDeviationTolerance = 0.0` (the default), Phase 2 is skipped entirely and behaviour is identical to the pre-S2.4 pipeline.

---

## Parameter summary

| Parameter | Where | Default | Effect |
|-----------|-------|---------|--------|
| `circumradiusToEdgeRatio` | `refineSurfaces` | 2.0 | Max circumradius/shortest-edge in UV space (Phase 1) |
| `minAngleDegrees` | `refineSurfaces` | 30.0 | Min interior angle in UV space, degrees (Phase 1) |
| `elementLimit` | `refineSurfaces` | 50 000 | Triangle count cap per face (both phases) |
| `chordDeviationTolerance` | `refineSurfaces` | 0.0 | Max chord height in model units; 0 = disabled (Phase 2) |
