# OpenLoom Implementation Progress

This document tracks the full implementation status of OpenLoom: the 2D constrained Delaunay mesher (complete), the 3D surface mesher (in progress), and the 3D volume mesher (partially implemented).

---

# Part I — 2D Mesher

A standalone 2D constrained Delaunay mesher with Shewchuk refinement.

**Pipeline:** `EdgeDiscretizer2D` → `DiscretizationResult2D` → `MeshingContext2D` → `ConstrainedDelaunay2D` → optional `ShewchukRefiner2D` → `VtkExporter`

**Status: COMPLETE**

---

## Step A — Core Data Structures

| Sub-step | Description | File(s) | Status |
|----------|-------------|---------|--------|
| A1 | Geometric primitives (`Circle2D`, `ConstrainedSegment2D`, `EdgeRole`) | `2D/GeometryStructures2D.h` | Done |
| A2 | Discretization result type (`DiscretizationResult2D`: sampled points, t-parameters, corner/edge index maps) | `2D/DiscretizationResult2D.h` | Done |
| A3 | `MeshData2D` / `MeshMutator2D` — node + triangle storage with controlled mutation | `Meshing/Data/MeshData2D.h/.cpp`, `MeshMutator2D.h/.cpp` | Done |

---

## Step B — Unconstrained Delaunay Triangulation

| Sub-step | Description | File(s) | Status |
|----------|-------------|---------|--------|
| B1 | `Delaunay2D`: incremental Bowyer-Watson insertion of unordered point sets | `2D/Delaunay2D.h/.cpp` | Done |
| B2 | `MeshOperations2D`: Bowyer-Watson insertion, Lawson edge flipping, exterior removal, constrained segment splitting | `2D/MeshOperations2D.h/.cpp` | Done |
| B3 | `MeshQueries2D`: conflict/cavity queries, encroachment detection, interior/exterior classification (ray-casting + BFS) | `2D/MeshQueries2D.h/.cpp` | Done |
| B4 | `GeometryUtilities2D`: pure static geometry (orientation, segment intersection, point-in-circle, super-triangle) | `2D/GeometryUtilities2D.h/.cpp` | Done |
| B5 | `ElementGeometry2D`: per-element computations (circumcircle, area, angles, centroid) | `2D/ElementGeometry2D.h/.cpp` | Done |
| B6 | `ElementQuality2D`: quality metrics (shortest/longest edge, circumradius-to-edge ratio, sorted quality lists) | `2D/ElementQuality2D.h/.cpp` | Done |

---

## Step C — Constrained Delaunay Triangulation

| Sub-step | Description | File(s) | Status |
|----------|-------------|---------|--------|
| C1 | `EdgeDiscretizer2D`: samples CAD boundary edges into `DiscretizationResult2D` (curvature-adaptive or fixed count) | `2D/EdgeDiscretizer2D.h/.cpp` | Done |
| C2 | `ConstrainedDelaunay2D`: extends `Delaunay2D` with edge constraint enforcement and exterior triangle removal | `2D/ConstrainedDelaunay2D.h/.cpp` | Done |
| C3 | `MeshingContext2D`: top-level context owning geometry, topology, and `MeshData2D`; `fromSurface()` factory creates a UV-space context for a CAD face | `2D/MeshingContext2D.h/.cpp` | Done |

---

## Step D — Shewchuk Quality Refinement

| Sub-step | Description | File(s) | Status |
|----------|-------------|---------|--------|
| D1 | `IQualityController2D`: interface for pluggable quality criteria | `2D/IQualityController2D.h` | Done |
| D2 | `Shewchuk2DQualityController`: circumradius-to-shortest-edge bound, min-angle threshold, element limit | `2D/Shewchuk2DQualityController.h/.cpp` | Done |
| D3 | `ConstraintChecker2D`: diametral circle encroachment test for constrained segments | `2D/ConstraintChecker2D.h/.cpp` | Done |
| D4 | `ShewchukRefiner2D`: priority-driven refinement loop — (1) split encroached segments, (2) insert circumcenters of poor triangles; supports `BoundarySplitCallback` for twin-edge sync | `2D/ShewchukRefiner2D.h/.cpp` | Done |
| D5 | `BoundarySplitSynchronizer`: callback implementation that propagates segment splits to twin edges via `TwinManager` | `2D/BoundarySplitSynchronizer.h/.cpp` | Done |

---

## Step E — Verification & Debug

| Sub-step | Description | File(s) | Status |
|----------|-------------|---------|--------|
| E1 | `MeshVerifier`: CCW orientation check, no-overlap check; returns `VerificationResult` | `2D/MeshVerifier.h/.cpp` | Done |
| E2 | `MeshDebugUtils2D`: compile-flag-gated conditional export + verify at each iteration | `2D/MeshDebugUtils2D.h/.cpp` | Done |

---

## Step F — Export & Examples

| Sub-step | Description | File(s) | Status |
|----------|-------------|---------|--------|
| F1 | VTK `.vtu` export for 2D meshes | `Export/VtkExporter` | Done |
| F2 | `SimpleDelaunay2D` — basic unconstrained triangulation | `Examples2D/SimpleDelaunay2D.cc` | Done |
| F3 | `RectangleWithHole2D` — constrained mesh with hole | `Examples2D/RectangleWithHole2D.cc` | Done |
| F4 | `RectangleWithCrack` — interior edge constraint | `Examples2D/RectangleWithCrack.cc` | Done |
| F5 | `SquareWithCircularHole` — curved boundary sampling | `Examples2D/SquareWithCircularHole.cc` | Done |
| F6 | `SquareWithInternalCircles` — multiple interior constraints | `Examples2D/SquareWithInternalCircles.cc` | Done |
| F7 | `SquareWithCircleAndTwinEdges` — periodic twin-edge demo | `Examples2D/SquareWithCircleAndTwinEdges.cc` | Done |
| F8 | `MeshStepFile2D` — CAD STEP file import and mesh | `Examples2D/MeshStepFile2D.cc` | Done |
| F9 | Unit tests: constrained triangulation, refinement, geometry, quality, constraint checking | `tests/Meshing/Core/test_*.cpp` | Done |

---

---

# Part II — 3D Surface Mesher

Produces a quality triangle mesh of all CAD surfaces. Each face is meshed independently in its UV (parametric) space using the 2D Shewchuk refiner, with inter-face conformity enforced on shared edges via `TwinManager`.

**Pipeline:** `TwinTableGenerator` (topology) → `BoundaryDiscretizer3D` (populates `TwinManager`) → `FacetTriangulationManager` (per-face `MeshData2D`, no `MeshData3D`) → `ShewchukRefiner2D` per face → assembly → `SurfaceMesh3D`

**Output:** `SurfaceMesh3D` — a conforming triangle mesh of all boundary surfaces, with node-pairing metadata for periodic/synchronized boundaries.

---

## Prerequisites

- [x] **`MeshDebugUtils3D`** — Phase-aware export + verify utility (modeled on `MeshDebugUtils2D`)
  - Accepts a `MeshingPhase3D` enum to select which invariants to check
  - Each phase accumulates checks from all previous phases
  - Files: `src/Meshing/Core/3D/General/MeshDebugUtils3D.h/.cpp`
  - Status: **Done**

---

## Phase 0 — Folder Restructuring

Before surface mesher work begins, the existing flat `3D/` folder is split into three sub-folders.

| Task | Description | Status |
|------|-------------|--------|
| 0.1 | Create `3D/General/`, `3D/Surface/`, `3D/Volume/` | Done |
| 0.2 | Move general 3D infrastructure to `3D/General/`: `GeometryStructures3D`, `GeometryUtilities3D`, `DiscretizationResult3D`, `BoundaryDiscretizer3D`, `ConstraintRegistrar3D`, `ConstraintChecker3D`, `MeshingContext3D`, `MeshOperations3D`, `MeshQueries3D`, `MeshVerifier3D`, `ElementGeometry3D`, `ElementQuality3D`, `MeshDebugUtils3D` | Done |
| 0.3 | Decouple `BoundaryDiscretizer3D` from `MeshingContext3D` (currently depends on volume context; should work without tet data) | Done |
| 0.4 | Extend `ElementGeometry3D` to cover triangle-in-3D operations (area, normal, circumcircle in plane) — needed by surface mesher | Done |
| 0.5 | Move `FacetTriangulation`, `FacetTriangulationManager` to `3D/Surface/` | Done |
| 0.6 | Move the four volume-specific files to `3D/Volume/`: `Delaunay3D`, `ConstrainedDelaunay3D`, `ShewchukRefiner3D`, `Shewchuk3DQualityController` | Done |
| 0.7 | Update all `CMakeLists.txt` and `#include` paths; verify build passes | Done |

**Status: Complete**

---

## Step S1 — Surface Meshing Context and Infrastructure

| Sub-step | Description | File(s) | Status |
|----------|-------------|---------|--------|
| S1.1 | Define `SurfaceMesh3D` result type (nodes, triangles, per-face groups, edge node pairing) | `3D/Surface/SurfaceMesh3D.h` | Done |
| S1.2 | `TwinTableGenerator`: inspect topology, compute `EdgeTwinTable` (edgeId → adjacent surfaces + orientations) | `3D/General/TwinTableGenerator.h/.cpp` | Done |
| S1.3 | Extend `BoundaryDiscretizer3D`: accept `EdgeTwinTable`, assign global node IDs from counter, populate `TwinManager` with segment-level twin groups | `3D/General/BoundaryDiscretizer3D` | Done |
| S1.4 | `FacetTriangulationManager`: surface-mesher initialisation using `DiscretizationResult3D` directly (no `MeshData3D`); each face gets a `MeshData2D` via `FacetTriangulation` | `3D/General/FacetTriangulationManager` | Done |
| S1.5 | `SurfaceMeshingContext3D`: owns geometry + topology + `FacetTriangulationManager` + `TwinManager`; no tet data. Exposes `buildSurfaceMesh()` → `MeshData3D` of initial surface triangulation | `3D/Surface/SurfaceMeshingContext3D.h/.cpp` | Done |

**Implementation note (S1.4):** `FacetTriangulation::initialize()` uses `ConstrainedDelaunay2D` (not bare `Delaunay2D`) so boundary edge constraints are registered, enforced, and exterior triangles removed for each face. The per-face `DiscretizationResult2D` is built by `FacetDiscretization2DBuilder` (extracted from `FacetTriangulationManager`) using local (face-scoped) point indices from the 3D discretization result. `ConstrainedDelaunay2D` exposes `getPointIndexToNodeIdMap()` so callers can build 3D↔2D node ID mappings after triangulation.

**Implementation note (periodic surfaces):** Two bugs affecting cylindrical/toroidal faces were fixed. Bug 1: `initializeForSurfaceMesher` now uses `collectBoundaryPointIndices`, excluding interior surface sample points so the constrained Delaunay starts with boundary-only vertices. Bug 2: `buildFaceDiscretization2D` processes seam twin edges first to build a `realCornerToShiftedLocal` map, then uses it to close circular edges at U+uPeriod instead of doubling back to U=0.

**Refactoring — SeamCollection:** A new `Topology3D::SeamCollection` class owns the original↔twin edge mapping for periodic surfaces, exposing `isSeamTwin()`, `getOriginalEdgeId()`, and `getSeamTwinEdgeIds()`. `Topology3D` holds and exposes `SeamCollection`; `isValid()` skips seam twin edges (their synthetic `_seam` corner IDs intentionally don't exist in `corners_`). `Edge3D` is restored to a plain topological type with no seam knowledge. `TopoDS_ShapeConverter` builds `SeamCollection` during `createEdges()`. All consumers (`BoundaryDiscretizer3D`, `FacetTriangulationManager`, `MeshingContext2D`) query `SeamCollection` instead of checking edge name suffixes.

**Validation gate:** Each CAD face has an initialized `FacetTriangulation` (`MeshData2D`) with boundary nodes seeded from edge discretization. `TwinManager` knows all shared-edge segment pairs.

**Validation:** `SurfaceMeshEdges` example (box-with-hole) exports:
  - `SurfaceMeshEdges.vtu` — discretized boundary edges (color by EdgeID); verified in ParaView
  - `SurfaceMesh3D.vtu` — initial surface triangulation (color by SurfaceID); all 8 faces covered including cylinder

**Status: Complete**

---

## Step S2 — Per-Face Quality Refinement (UV Space)

`FacetTriangulation` already holds a `MeshingContext2D`. This step adds the quality refinement pass using `ShewchukRefiner2D` in UV space.

| Sub-step | Description | File(s) | Status |
|----------|-------------|---------|--------|
| S2.1 | Run `ShewchukRefiner2D` on each `FacetTriangulation` context in UV space | `FacetTriangulation`, `ShewchukRefiner2D` | Done |
| S2.2 | Quality criterion: use 3D arc-length metric instead of flat UV distance (pull-back metric via the surface's first fundamental form / metric tensor) | `3D/Surface/SurfaceMeshQuality.h/.cpp` (new) | **TODO** |
| S2.3 | Respect edge constraints: boundary edges of each face (on shared topology edges) must not be modified | `FacetTriangulation` | Done |
| S2.4 | Geometric deviation check: for each triangle evaluate the 3D distance from the triangle midpoint (and edge midpoints) to the actual CAD surface by evaluating the surface at the UV midpoint; refine any triangle whose chord deviation exceeds a user-specified tolerance | `3D/Surface/SurfaceMeshQuality.h/.cpp` | **TODO** |

**Note on S2.2:** For mildly curved CAD surfaces the UV distance approximation is acceptable and can be used initially. For tightly curved faces (small fillets, tight bends) the pull-back metric correction matters. Can be deferred to a later iteration.

**Note on CAD abstraction (S2.2, S2.4):** Both sub-steps require querying the underlying surface geometry (metric tensor for S2.2, surface point evaluation for S2.4). These queries must go through an abstraction layer (e.g. `ISurface`) rather than calling CAD-library APIs directly. OCC is the current backend but must not be assumed. The `ISurface` interface should expose `evaluate(u, v) → Point3D` and optionally `metric(u, v) → (du, dv)` for the first fundamental form coefficients.

**Validation gate:** Each face has a quality triangle mesh in its `FacetTriangulation`. No triangle violates the angle bound. Boundary edges match the seeded edge discretization.

**Implementation note (S2.1):** `SurfaceMeshingContext3D::refineSurfaces()` iterates over all surfaces, retrieves each face's `MeshingContext2D` from its `FacetTriangulation`, constructs a `Shewchuk2DQualityController` and `ShewchukRefiner2D`, wires in a `BoundarySplitSynchronizer`, then calls `refiner.refine()`. After all faces are refined, `resolveRefinementNodes()` is called per face to lift UV refinement nodes onto the 3D surface and assign global 3D node IDs. These are stored in `SurfaceMeshingContext3D::refinementNodes_` and included by `getSurfaceMesh3D()`.

**Status: IN PROGRESS — S2.1, S2.3 Done**

---

## Step S3 — Inter-Face Conformity (TwinManager)

Adjacent CAD faces share topology edges. Conformity is enforced via `TwinManager`: when a segment on a shared edge is split during per-face refinement, the split propagates to all twin segments on adjacent faces. The `TwinManager` is populated in S1.3 and consulted throughout S2.

| Sub-step | Description | File(s) | Status |
|----------|-------------|---------|--------|
| S3.1 | `TwinManager` class: tracks segment twin groups, resolves split parameters, maps cross-mesh node IDs | `Common/TwinManager.h/.cpp` | Done |
| S3.2 | Wire `TwinManager` into boundary split propagation: on each split, call `hasTwins()`, propagate to all twin segments | `BoundarySplitSynchronizer`, `SurfaceMeshingContext3D` | Done |
| S3.3 | `MeshVerifier::verifyTwinConsistency()`: check all twin members have equal subdivision counts and all recorded vertex IDs exist in their meshes | `MeshVerifier` (2D) | **TODO** |

**Validation gate:** For every shared topology edge, all adjacent face `MeshData2D` instances have identical node sequences along that edge. `verifyTwinConsistency()` passes. No cracks visible in ParaView output.

**Implementation note (S3.2):** `BoundarySplitSynchronizer` is the `BoundarySplitCallback` implementation that propagates segment splits to twin edges via `TwinManager`. It is wired into `ShewchukRefiner2D` via `refiner.setOnBoundarySplit(sync)` inside `refineSurfaces()`. When the refiner splits a boundary segment, the callback resolves the twin segments through `TwinManager` and performs matching splits on all adjacent faces.

**Status: IN PROGRESS — S3.1, S3.2 Done**

---

## Step S4 — Surface Mesher API and Output

| Sub-step | Description | File(s) | Status |
|----------|-------------|---------|--------|
| S4.1 | `SurfaceMesher3D` top-level class: takes geometry + topology + settings, runs S1–S3, returns `SurfaceMesh3D` | `3D/Surface/SurfaceMesher3D.h/.cpp` (new) | **TODO** (needs S2/S3) |
| S4.2 | Assemble `SurfaceMesh3D` from all `FacetTriangulation` results (global node numbering, per-face groups) | — | **TODO** |
| S4.3 | VTK export of surface mesh (triangle surface, `.vtu`) | `Export/VtkExporter` | Done |
| S4.4 | Example: export surface triangulation to ParaView | `src/Examples3D/SurfaceMeshEdges.cc` | Done (initial triangulation) |

**S4.3 implementation:** Two `VtkExporter::writeSurfaceMesh` overloads exist. The original takes `(disc3D, subfacets, path)` and writes only discretization points (suitable before refinement). A second overload takes `(MeshData3D, subfacets, path)` and writes all nodes from `MeshData3D` (including refinement nodes added by `resolveRefinementNodes()`), sorted by ID so the VTK point index equals the node ID. Use the `MeshData3D` overload after calling `refineSurfaces()` + `getSurfaceMesh3D()`.

**S4.4 implementation:** `SurfaceMeshEdges.cc` exports both `SurfaceMeshEdges.vtu` (edges) and `SurfaceMesh3D.vtu` (initial surface triangulation, before refinement). `CylinderSurfaceMesh.cc` (new) demonstrates the full pipeline for a cylinder: edge discretization → initial triangulation → `refineSurfaces()` → `getSurfaceMesh3D()` → export; produces 249 nodes and 431 triangles with zero warnings.

**Validation gate:** Exported surface mesh displays correctly in ParaView. All CAD faces covered. No cracks or duplicate nodes on shared edges.

**Status: S4.3 and S4.4 Done (initial + cylinder refinement) — S4.1/S4.2 await remaining S2/S3**

---

---

# Part III — 3D Volume Mesher

Takes a `SurfaceMesh3D` from Part II as input. Fills the interior with quality tetrahedra. The surface is **fixed** — no new nodes are inserted on the boundary during volume refinement.

**Design note:** The volume mesher must recover the given surface triangulation as constrained faces in the tetrahedralization (Steps V3–V4). This is the classical conforming Delaunay embedding problem. Whether Steiner points are permitted on the surface during recovery is an open design decision to be resolved during V3–V4 implementation.

---

## Step V1 — Input Processing

| Sub-step | Description | File(s) | Status |
|----------|-------------|---------|--------|
| V1.1 | Refactor `MeshingContext3D` to accept `SurfaceMesh3D` as input (replaces internal `BoundaryDiscretizer3D` call) | `MeshingContext3D` | **TODO** (refactor) |
| V1.2 | Extract constrained subsegments from surface mesh edges lying on CAD topology edges | `ConstraintRegistrar3D` | Done (needs adaptation) |
| V1.3 | Extract constrained subfacets from surface mesh triangles | `ConstraintRegistrar3D` | Done (needs adaptation) |

**Status: NEEDS REFACTORING**

---

## Step V2 — Initial Delaunay Tetrahedralization (Unconstrained)

| Sub-step | Description | File(s) | Status |
|----------|-------------|---------|--------|
| V2.1 | Create bounding tetrahedron | `MeshOperations3D::createBoundingTetrahedron` | Done |
| V2.2 | Incremental Bowyer-Watson insertion | `MeshOperations3D::insertVertexBowyerWatson` | Done |
| V2.2.1 | Find seed tetrahedron containing point | `MeshQueries3D::findConflictingTetrahedra` | Done |
| V2.2.2 | Circumsphere test (BFS flood fill) | `MeshQueries3D::findConflictingTetrahedra` | Done |
| V2.2.3 | Extract cavity boundary | `MeshQueries3D::findCavityBoundary` | Done |
| V2.2.4 | Remove conflicting tetrahedra | `MeshOperations3D` (internal) | Done |
| V2.2.5 | Retriangulate cavity | `MeshOperations3D::retriangulate` | Done |
| V2.3 | Remove bounding tetrahedron | `MeshOperations3D::removeBoundingTetrahedron` | Done |

**Validation gate (`MeshingPhase3D::InitialDelaunay`):**
- [ ] No degenerate tetrahedra (volume > 0)
- [ ] No inverted tetrahedra
- [ ] All node references valid
- [ ] No NaN/Inf coordinates
- [ ] All surface mesh nodes present as tet mesh nodes

**Status: COMPLETE**

---

## Step V3 — Segment Recovery

Recover surface mesh edges (on CAD topology edges) as edges in the tetrahedralization. The surface is fixed — no new nodes may be added on the boundary.

| Sub-step | Description | File(s) | Status |
|----------|-------------|---------|--------|
| V3.1 | Check if each constrained edge exists as a tet mesh edge | `MeshQueries3D::edgeExistsInMesh` | Done |
| V3.2 | Identify missing constrained edges | `ConstraintChecker3D` | Done |
| V3.3 | Recover missing edges via cavity re-triangulation (flip-based, no splitting) | `MeshOperations3D` | **TODO** |
| V3.4 | **Wire into pipeline: loop until all constrained edges present** | `ShewchukRefiner3D` | **TODO** |

**Note:** Unlike the original plan, the surface is now fixed so recovery must be flip-based rather than split-based. If a constrained edge cannot be recovered without inserting a boundary Steiner point, this is a design decision point to resolve during implementation.

**Validation gate (`MeshingPhase3D::SegmentRecovery`):**
- [ ] All checks from `InitialDelaunay` pass
- [ ] Every constrained edge exists as a tet mesh edge

**Status: INFRASTRUCTURE EXISTS — recovery strategy needs revision for fixed surface**

---

## Step V4 — Facet Recovery

Recover surface mesh triangles as constrained faces in the tetrahedralization.

| Sub-step | Description | File(s) | Status |
|----------|-------------|---------|--------|
| V4.1 | Check if each surface triangle exists as a tet mesh face | `MeshQueries3D::faceExistsInMesh` | Done |
| V4.2 | Identify missing constrained faces | — | **TODO** |
| V4.3 | Recover missing faces via cavity re-triangulation (flip-based) | — | **TODO** |
| V4.4 | **Wire into pipeline: loop until all surface triangles present as tet faces** | `ShewchukRefiner3D` | **TODO** |

**Validation gate (`MeshingPhase3D::FacetRecovery`):**
- [ ] All checks from `SegmentRecovery` pass
- [ ] Every surface triangle exists as a tet mesh face

**Status: QUERY EXISTS — recovery implementation TODO**

---

## Step V5 — Exterior Removal

| Sub-step | Description | File(s) | Status |
|----------|-------------|---------|--------|
| V5.1 | Facets partition mesh into interior/exterior (precondition from V4) | — | — |
| V5.2 | Flood-fill classify tetrahedra | `MeshOperations3D::classifyTetrahedraInteriorExterior` | Done |
| V5.3 | Remove exterior tetrahedra | `MeshOperations3D::classifyTetrahedraInteriorExterior` | Done |
| V5.4 | **Integrate into pipeline** (call after V4 completes) | pipeline orchestrator | **TODO** |

**Validation gate (`MeshingPhase3D::ExteriorRemoved`):**
- [ ] All checks from `FacetRecovery` pass
- [ ] No tetrahedra exist outside the boundary
- [ ] Mesh element count decreased (exterior tets removed)

**Status: IMPLEMENTED — needs pipeline integration**

---

## Step V6 — Shewchuk's 3D Delaunay Refinement

Interior-only quality refinement. New nodes are inserted only inside the domain. Encroachment checks still apply to prevent interior circumcenters from violating surface constraints.

| Sub-step | Description | File(s) | Status |
|----------|-------------|---------|--------|
| V6.1 | **Priority 1:** Split encroached subsegments | `ShewchukRefiner3D` | **TODO** (stub exists) |
| V6.1.1 | Diametral sphere encroachment test | `ConstraintChecker3D::isSubsegmentEncroached` | Done |
| V6.1.2 | Split at midpoint | `MeshOperations3D` | Done |
| V6.1.3–4 | Repeat until none encroached | `ShewchukRefiner3D` | **TODO** |
| V6.2 | **Priority 2:** Split encroached subfacets | `ShewchukRefiner3D` | **TODO** (stub exists) |
| V6.2.1 | Equatorial sphere encroachment test | `ConstraintChecker3D::isSubfacetEncroached` | Done |
| V6.2.2 | Projection Lemma ordering | — | **TODO** |
| V6.2.3 | Circumcenter encroachment check before insertion | `MeshQueries3D::findEncroachingSubsegments` | Done |
| V6.2.4 | Insert circumcenter into tetrahedralization | `MeshOperations3D::splitConstrainedSubfacet` | Done |
| V6.3 | **Priority 3:** Split skinny tetrahedra | `ShewchukRefiner3D::handleSkinnyTetrahedron` | **Partial** |
| V6.3.1 | Circumradius-to-shortest-edge ratio test | `ElementQuality3D`, `Shewchuk3DQualityController` | Done |
| V6.3.2 | Select worst tetrahedron | `ElementQuality3D::getSkinnyTetrahedraSortedByQuality` | Done |
| V6.3.3 | Compute circumcenter | `ElementGeometry3D::computeCircumscribingSphere` | Done |
| V6.3.4.1 | Check subsegment encroachment before insertion | `ShewchukRefiner3D::handleSkinnyTetrahedron` | **TODO** (skipped) |
| V6.3.4.2 | Check subfacet encroachment before insertion | `ShewchukRefiner3D::handleSkinnyTetrahedron` | **TODO** (skipped) |
| V6.3.4.3 | Insert circumcenter if no encroachment | `ShewchukRefiner3D::handleSkinnyTetrahedron` | Done |

**What exists:** Priority 3 works for the simple case (insert circumcenter without encroachment checks). The TODO comments in `ShewchukRefiner3D.cpp:82` and `ShewchukRefiner3D.cpp:144` mark where Priority 1, 2, and encroachment checks need to go.

**Validation gate (`MeshingPhase3D::Refined`):**
- [ ] All checks from `ExteriorRemoved` pass
- [ ] No encroached subsegments remain
- [ ] No encroached subfacets remain
- [ ] All tetrahedra satisfy quality bound B (circumradius/shortest-edge < B)

**Status: PARTIALLY IMPLEMENTED — Priority 3 only, encroachment checks missing**

---

## Step V7 — Post-Processing

| Sub-step | Description | File(s) | Status |
|----------|-------------|---------|--------|
| V7.1.1 | Identify slivers (near-zero volume, extreme dihedral angles) | — | **TODO** |
| V7.1.2 | Split slivers at circumcenter using V6.3 mechanism | — | **TODO** |
| V7.1.3 | Prioritize by circumradius-to-edge ratio | — | **TODO** |
| V7.2.1 | Optimization-based vertex smoothing | — | **TODO** |
| V7.2.2 | Topological transforms (2-3, 3-2 flips) | — | **TODO** |

**Validation gate (`MeshingPhase3D::PostProcessed`):**
- [ ] All checks from `Refined` pass
- [ ] No tetrahedra with dihedral angles below threshold (e.g., 19 degrees)

**Status: NOT STARTED**

---

---

# Recommended Implementation Order

### Done
1. ~~Part I: Full 2D mesher (A–F)~~ **Done**
2. ~~Phase 0: Folder restructuring (3D)~~ **Done**
3. ~~Step S1: Surface meshing context and infrastructure~~ **Done**
4. ~~S3.1: `TwinManager` class~~ **Done**
5. ~~S4.3/S4.4 (initial): VTK export + edge example~~ **Done**
6. ~~S2.1: `ShewchukRefiner2D` per face in UV space~~ **Done**
7. ~~S3.2: `BoundarySplitSynchronizer` wired into refinement loop~~ **Done**
8. ~~`CylinderSurfaceMesh` example: full pipeline validation (init → refine → export)~~ **Done**

### In Progress / Next Steps

#### Phase I-B: Per-face quality meshing (Step S2)
7. ~~Run `ShewchukRefiner2D` per face in UV space (S2.1)~~ **Done**
8. Quality criterion with 3D arc-length metric / pull-back metric (S2.2)
9. Geometric deviation check per triangle against actual CAD surface (S2.4)

#### Phase I-C: Inter-face conformity via TwinManager (Step S3)
10. ~~Wire twin split propagation into boundary splits via `BoundarySplitSynchronizer` (S3.2)~~ **Done**
11. `MeshVerifier::verifyTwinConsistency()` (S3.3)
12. Validate: no cracks between faces in ParaView output

#### Phase I-D: Surface mesher API (Step S4)
12. `SurfaceMesher3D` top-level class (S4.1)
13. Assemble `SurfaceMesh3D` with global node numbering (S4.2)

#### Phase II-A: Volume input refactoring (Step V1)
14. Refactor `MeshingContext3D` to accept `SurfaceMesh3D` as input (V1.1)

#### Phase II-B: Segment and facet recovery (Steps V3–V4)
15. Implement flip-based edge recovery for fixed surface (V3.3–V3.4)
16. Implement flip-based face recovery (V4.2–V4.4)

#### Phase II-C: Exterior removal integration (Step V5)
17. Call `classifyTetrahedraInteriorExterior()` after facet recovery (V5.4)

#### Phase II-D: Complete refinement loop (Step V6)
18. Add Priority 1 and Priority 2 to `refineStep()`
19. Add encroachment checks to `handleSkinnyTetrahedron()`

#### Phase II-E: Post-processing (Step V7)
20. Sliver detection and removal
21. (Optional) Smoothing and topological flips

---

---

# Key Files Reference

### 2D Mesher
| File | Role |
|------|------|
| `2D/GeometryStructures2D.h` | `Circle2D`, `ConstrainedSegment2D`, `EdgeRole` |
| `2D/DiscretizationResult2D.h` | Sampled points + edge index maps from `EdgeDiscretizer2D` |
| `2D/GeometryUtilities2D.h/.cpp` | Static geometry: orientation, intersection, point-in-circle |
| `2D/ElementGeometry2D.h/.cpp` | Per-element: circumcircle, area, angles, centroid |
| `2D/ElementQuality2D.h/.cpp` | Quality metrics: circumradius-to-edge ratio, sorted lists |
| `2D/Delaunay2D.h/.cpp` | Unconstrained incremental Bowyer-Watson triangulation |
| `2D/ConstrainedDelaunay2D.h/.cpp` | Constrained triangulation with exterior removal |
| `2D/EdgeDiscretizer2D.h/.cpp` | CAD edge → sampled `DiscretizationResult2D` |
| `2D/MeshingContext2D.h/.cpp` | Top-level context; `fromSurface()` for UV-space CAD face meshing |
| `2D/MeshOperations2D.h/.cpp` | Bowyer-Watson, Lawson flipping, segment splitting, exterior removal |
| `2D/MeshQueries2D.h/.cpp` | Conflict/cavity queries, encroachment, interior/exterior classification |
| `2D/ConstraintChecker2D.h/.cpp` | Diametral circle encroachment test |
| `2D/IQualityController2D.h` | Quality controller interface |
| `2D/Shewchuk2DQualityController.h/.cpp` | Circumradius-to-edge bound + min-angle threshold |
| `2D/ShewchukRefiner2D.h/.cpp` | Refinement loop: encroachment splits + circumcenter insertion |
| `2D/BoundarySplitSynchronizer.h/.cpp` | `BoundarySplitCallback` for twin-edge synchronization |
| `2D/MeshVerifier.h/.cpp` | CCW orientation + overlap checks |
| `2D/MeshDebugUtils2D.h/.cpp` | Flag-gated conditional export + verify |
| `Meshing/Data/MeshData2D.h/.cpp` | Node + triangle storage |
| `Meshing/Data/MeshMutator2D.h/.cpp` | Controlled mesh mutation |

### 3D/General (shared infrastructure)
| File | Role |
|------|------|
| `GeometryStructures3D.h` | Common data types (`ConstrainedSubsegment3D`, `ConstrainedSubfacet3D`) |
| `GeometryUtilities3D.h/.cpp` | Shared geometric utilities |
| `DiscretizationResult3D.h` | Result type from `BoundaryDiscretizer3D` |
| `EdgeTwinTable.h` | `EdgeTwinTable` type and `TwinOrientation` enum (edgeId → adjacent surfaces + orientations) |
| `BoundaryDiscretizer3D.h/.cpp` | Edge and surface parametric sampling |
| `FacetDiscretization2DBuilder.h/.cpp` | Builds per-face `DiscretizationResult2D` from `DiscretizationResult3D`; handles seam twin edges |
| `FacetTriangulation.h/.cpp` | UV-space 2D triangulation for one CAD face |
| `FacetTriangulationManager.h/.cpp` | Manages all per-face triangulations |
| `TwinTableGenerator.h/.cpp` | Topology-only: produces `EdgeTwinTable` |
| `ConstraintRegistrar3D.h/.cpp` | Subsegment/subfacet registration |
| `ConstraintChecker3D.h/.cpp` | Encroachment tests (diametral/equatorial sphere) |
| `MeshingContext3D.h/.cpp` | Owns 3D mesh data and operations lifecycle |
| `MeshOperations3D.h/.cpp` | Mutation operations (Bowyer-Watson, cavity retriangulation, splitting) |
| `MeshQueries3D.h/.cpp` | Read-only queries (edge/face existence, encroachment, cavity finding) |
| `MeshVerifier3D.h/.cpp` | Mesh integrity checks |
| `ElementGeometry3D.h/.cpp` | Circumsphere, tet volume, triangle area/normal in 3D, circumcircle in plane |
| `ElementQuality3D.h/.cpp` | Quality metrics for tetrahedra and triangles in 3D |
| `MeshDebugUtils3D.h/.cpp` | Phase-aware debug export + verification |

### 3D/Surface (surface mesher)
| File | Role |
|------|------|
| `SurfaceMeshingContext3D.h/.cpp` | Surface meshing context (geometry + topology + `FacetTriangulationManager` + `TwinManager`) |
| `SurfaceMesher3D.h/.cpp` | Top-level surface mesher API (**TODO**) |
| `SurfaceMesh3D.h` | Result type (nodes, triangles, per-face groups, edge node pairing) |
| `SurfaceMeshQuality.h/.cpp` | UV-space quality with optional 3D pull-back metric (**TODO**) |

### 3D/Volume (volume mesher)
| File | Role |
|------|------|
| `Delaunay3D.h/.cpp` | Unconstrained Bowyer-Watson tetrahedralization |
| `ConstrainedDelaunay3D.h/.cpp` | Top-level constrained Delaunay volume mesher |
| `ShewchukRefiner3D.h/.cpp` | Shewchuk refinement loop (Priority 1/2/3) |
| `Shewchuk3DQualityController.h/.cpp` | Circumradius-to-edge quality bound configuration |

### Topology
| File | Role |
|------|------|
| `src/Topology/SeamCollection.h/.cpp` | Owns original↔twin edge mapping for periodic surfaces |

### Common
| File | Role |
|------|------|
| `src/Common/TwinManager.h/.cpp` | Segment-level twin groups; cross-mesh node ID mapping |
| `src/Common/DebugFlags.h` | Runtime debug flags (`OPENLOOM_DEBUG_ENABLED`) |
| `tests/Meshing/Core/test_ShewchukRefiner3D.cpp` | Volume refiner tests |

---

---

# Future Improvements

- [ ] **No abbreviations in identifiers** *(do soon)* — All variable, parameter, and function names must use full words. Examples: `disc2D` → `discretization2D`, `globalPtIndices` → `globalPointIndices`, `ctx` → `context`, `mgr` → `manager`. Sweep all files under `src/` and rename abbreviated identifiers.
- [ ] **`FacetTriangulation` — split into data and logic classes** *(do soon)* — `FacetTriangulation` currently mixes algorithmic logic with data ownership (e.g. `node3DTo2DMap_`, UV-space `MeshingContext2D`). Split into a plain data container (`FacetTriangulationData` or similar) and a separate logic class. Doing this before S2 quality refinement work will keep the logic class focused and testable. Files: `3D/Surface/FacetTriangulation.h/.cpp`.
- [ ] **Topology3D string IDs → `size_t` indices** — `Edge3D`, `Surface3D`, `Corner3D` currently use `std::string` IDs both for self-identity and for cross-referencing neighbours. Switch to `size_t` indices, store entities in `std::vector` instead of `unordered_map`, and drop `getId()` (only 2 production call sites). Requires updating `TopoDS_ShapeConverter` on ingestion and all consumers of `getStartCornerId()`, `getBoundaryEdgeIds()`, etc. Separately, add `using NodeID = size_t;` and `using ElementID = size_t;` aliases for mesh data so function signatures are self-documenting and grep-able.
- [ ] **Metric adaptation in surface mesher (S2.2)** — For highly curved CAD surfaces, use the OCC pull-back metric (`GeomLProp_SLProps`) in the UV-space quality criterion to avoid angle distortion. Defer until basic surface mesher works.
- [ ] **`TwinSurfaces` — periodic 3D surface mesh (FEM sense)** — Extend `TwinTableGenerator` to accept user-declared surface pairs (S1 ↔ S2 with a UV→UV mapping). Declaring twin surfaces implies that all corresponding boundary edges are also twin edges. Interior refinement propagation requires facet-level twinning in `TwinManager` (complement to the current segment-level twinning). Use case: inlet/outlet faces of a periodic pipe mesh.
- [ ] **`TwinEdges` for 2D periodic meshes (FEM sense)** — `TwinTableGenerator2D` accepts user-declared boundary edge pairs. `TwinManager` already handles segment-level splits. Enables generating periodic FEM meshes where two boundary edges are discretized identically.
- [ ] **Parallelize verification loops in `MeshDebugUtils3D`** — The subsegment and subfacet presence checks iterate over all constraints sequentially. For large meshes these are embarrassingly parallel. Add optional OpenMP as done in `MeshVerifier` (2D overlap checks).
