# 3D Mesher Implementation Progress

**Strategy (updated Feb 2026):** The original plan targeted the volume mesher directly. The revised plan builds a standalone **3D surface mesher first**, then a **volume mesher** that takes the surface mesh as input. This mirrors how major meshers (Gmsh, TetGen) separate surface and volume meshing, and delivers the surface mesher as an independent milestone with standalone value.

The codebase will be reorganized from a flat `3D/` folder into:
- `3D/General/` — shared infrastructure (geometry structures, boundary discretization, constraint registration)
- `3D/Surface/` — surface mesher (UV-space quality meshing, inter-face conformity)
- `3D/Volume/` — volume mesher (tetrahedralization, Shewchuk refinement)

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
| 0.4 | Extend `ElementGeometry3D` to cover triangle-in-3D operations (area, normal, circumcircle in plane) — needed by surface mesher | **TODO** |
| 0.5 | Move `FacetTriangulation`, `FacetTriangulationManager` to `3D/Surface/` | Done |
| 0.6 | Move the four volume-specific files to `3D/Volume/`: `Delaunay3D`, `ConstrainedDelaunay3D`, `ShewchukRefiner3D`, `Shewchuk3DQualityController` | Done |
| 0.7 | Update all `CMakeLists.txt` and `#include` paths; verify build passes | Done |

**Status: IN PROGRESS — 0.4 remaining**

---

# Part I — 3D Surface Mesher

Produces a quality triangle mesh of all CAD surfaces. Each face is meshed independently in its UV (parametric) space using the 2D Shewchuk refiner, with inter-face conformity enforced on shared edges.

**Output:** `SurfaceMesh3D` — a conforming triangle mesh of all boundary surfaces, with node-pairing metadata for periodic/synchronized boundaries.

---

## Step S1 — Surface Meshing Context and Infrastructure

| Sub-step | Description | File(s) | Status |
|----------|-------------|---------|--------|
| S1.1 | Define `SurfaceMesh3D` result type (nodes, triangles, per-face groups, edge node pairing) | `3D/Surface/SurfaceMesh3D.h` | **TODO** |
| S1.2 | Create `SurfaceMeshingContext3D` — lightweight context holding geometry + topology + per-face `FacetTriangulation`, no tet data | `3D/Surface/SurfaceMeshingContext3D.h/.cpp` | **TODO** |
| S1.3 | Wire in `BoundaryDiscretizer3D` (after Phase 0 decoupling) to seed per-face triangulations with boundary nodes | — | **TODO** |

**Validation gate:** Each CAD face has an initialized `FacetTriangulation` with boundary nodes seeded from edge discretization.

**Status: NOT STARTED**

---

## Step S2 — Per-Face Quality Refinement (UV Space)

`FacetTriangulation` already holds a `MeshingContext2D`. This step adds the quality refinement pass using `ShewchukRefiner2D` in UV space.

| Sub-step | Description | File(s) | Status |
|----------|-------------|---------|--------|
| S2.1 | Run `ShewchukRefiner2D` on each `FacetTriangulation` context in UV space | `FacetTriangulation`, `ShewchukRefiner2D` | **TODO** |
| S2.2 | Quality criterion: use 3D arc-length metric instead of flat UV distance (pull-back metric via OCC `GeomLProp_SLProps`) | `3D/Surface/SurfaceMeshQuality.h/.cpp` (new) | **TODO** |
| S2.3 | Respect edge constraints: boundary edges of each face (on shared topology edges) must not be modified | `FacetTriangulation` | **TODO** |

**Note on S2.2:** For mildly curved CAD surfaces the UV distance approximation is acceptable and can be used initially. For tightly curved faces (small fillets, tight bends) the pull-back metric correction matters. Can be deferred to a later iteration.

**Validation gate:** Each face has a quality triangle mesh in its `FacetTriangulation`. No triangle violates the angle bound. Boundary edges match the seeded edge discretization.

**Status: NOT STARTED**

---

## Step S3 — Inter-Face Conformity (Shared Edge Synchronization)

Adjacent CAD faces share topology edges. After independent per-face refinement, the same edge may have different node spacings on each side. This step enforces a common discretization on all shared edges.

| Sub-step | Description | File(s) | Status |
|----------|-------------|---------|--------|
| S3.1 | Build shared-edge registry: for each topology edge, list the `(face, UV-coords)` pairs it belongs to | `3D/Surface/SharedEdgeRegistry.h/.cpp` (new) | **TODO** |
| S3.2 | After per-face refinement, collect all nodes on each shared edge across all adjacent faces | — | **TODO** |
| S3.3 | Merge node lists: take the union of node positions along each shared edge, re-insert missing nodes into affected face triangulations | — | **TODO** |
| S3.4 | Re-run local Delaunay refinement around modified edges to restore triangle quality | — | **TODO** |

**Note:** This is the "synchronized mesh" mechanism. It generalizes to periodic meshes: instead of merging nodes from geometrically adjacent faces, nodes between geometrically opposite faces are paired based on a user-defined mapping. The interface should be designed with this extension in mind.

**Validation gate:** For every topology edge, all adjacent face triangulations have identical node sequences along that edge (same 3D positions, same count). No cracks visible in ParaView output.

**Status: NOT STARTED**

---

## Step S4 — Surface Mesher API and Output

| Sub-step | Description | File(s) | Status |
|----------|-------------|---------|--------|
| S4.1 | `SurfaceMesher3D` top-level class: takes geometry + topology + settings, runs S1–S3, returns `SurfaceMesh3D` | `3D/Surface/SurfaceMesher3D.h/.cpp` (new) | **TODO** |
| S4.2 | Assemble `SurfaceMesh3D` from all `FacetTriangulation` results (global node numbering, per-face groups) | — | **TODO** |
| S4.3 | VTK export of surface mesh (triangle surface, `.vtu`) | `Export/VtkExporter` or new exporter | **TODO** |
| S4.4 | Example: `SurfaceMesh3DExample` — mesh a STEP file surface, export to ParaView | `src/Examples/` | **TODO** |

**Validation gate:** Exported surface mesh displays correctly in ParaView. All CAD faces covered. No cracks or duplicate nodes on shared edges.

**Status: NOT STARTED**

---

# Part II — 3D Volume Mesher

Takes a `SurfaceMesh3D` from Part I as input. Fills the interior with quality tetrahedra. The surface is **fixed** — no new nodes are inserted on the boundary during volume refinement.

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

**Algorithm ref:** Sections 2.1–2.3

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

## Recommended Implementation Order

### Phase 0: Folder restructuring
1. ~~Create `3D/General/`, `3D/Surface/`, `3D/Volume/` and move files~~ **Done**
2. ~~Decouple `BoundaryDiscretizer3D` from `MeshingContext3D`~~ **Done**
3. Extend `ElementGeometry3D` with triangle-in-3D operations

### Phase I-A: Surface mesher infrastructure (Step S1)
4. Define `SurfaceMesh3D` result type
5. Create `SurfaceMeshingContext3D`
6. Wire existing `FacetTriangulation`/`FacetTriangulationManager` into surface context

### Phase I-B: Per-face quality meshing (Step S2)
7. Run `ShewchukRefiner2D` per face in UV space
8. Validate: each face mesh satisfies angle bound, boundary edges unchanged

### Phase I-C: Inter-face conformity (Step S3)
9. Build `SharedEdgeRegistry`
10. Merge node lists across adjacent faces
11. Validate: no cracks between faces in ParaView output

### Phase I-D: Surface mesher API (Step S4)
12. `SurfaceMesher3D` top-level class
13. VTK export + example

### Phase II-A: Volume input refactoring (Step V1)
14. Refactor `MeshingContext3D` to accept `SurfaceMesh3D` as input

### Phase II-B: Segment and facet recovery (Steps V3–V4)
15. Implement flip-based edge and face recovery for fixed surface
16. Wire into pipeline

### Phase II-C: Exterior removal integration (Step V5)
17. Call `classifyTetrahedraInteriorExterior()` after facet recovery

### Phase II-D: Complete refinement loop (Step V6)
18. Add Priority 1 and Priority 2 to `refineStep()`
19. Add encroachment checks to `handleSkinnyTetrahedron()`

### Phase II-E: Post-processing (Step V7)
20. Sliver detection and removal
21. (Optional) Smoothing and topological flips

---

## Key Files Reference

### 3D/General (shared infrastructure)
| File | Role |
|------|------|
| `GeometryStructures3D.h` | Common data types (`ConstrainedSubsegment3D`, `ConstrainedSubfacet3D`) |
| `GeometryUtilities3D.h/.cpp` | Shared geometric utilities |
| `DiscretizationResult3D.h` | Result type from `BoundaryDiscretizer3D` |
| `BoundaryDiscretizer3D.h/.cpp` | Edge and surface parametric sampling |
| `ConstraintRegistrar3D.h/.cpp` | Subsegment/subfacet registration |
| `ConstraintChecker3D.h/.cpp` | Encroachment tests (diametral/equatorial sphere — geometric, not algorithm-specific) |
| `MeshingContext3D.h/.cpp` | Owns 3D mesh data and operations lifecycle |
| `MeshOperations3D.h/.cpp` | Mutation operations (Bowyer-Watson, cavity retriangulation, splitting) |
| `MeshQueries3D.h/.cpp` | Read-only queries (edge/face existence, encroachment, cavity finding) |
| `MeshVerifier3D.h/.cpp` | Mesh integrity checks |
| `ElementGeometry3D.h/.cpp` | Geometric computations: circumsphere, tet volume, triangle area/normal in 3D, circumcircle in plane |
| `ElementQuality3D.h/.cpp` | Quality metrics for tetrahedra and triangles in 3D |
| `MeshDebugUtils3D.h/.cpp` | Phase-aware debug export + verification |

### 3D/Surface (surface mesher)
| File | Role |
|------|------|
| `FacetTriangulation.h/.cpp` | UV-space 2D triangulation for one CAD face |
| `FacetTriangulationManager.h/.cpp` | Manages all per-face triangulations |
| `SurfaceMeshingContext3D.h/.cpp` | Surface meshing context (geometry + topology + facet triangulations) |
| `SurfaceMesher3D.h/.cpp` | Top-level surface mesher API |
| `SurfaceMesh3D.h` | Result type (nodes, triangles, per-face groups, edge node pairing) |
| `SharedEdgeRegistry.h/.cpp` | Inter-face conformity enforcement on shared topology edges |
| `SurfaceMeshQuality.h/.cpp` | UV-space quality metrics with optional 3D pull-back metric |

### 3D/Volume (volume mesher)
| File | Role |
|------|------|
| `Delaunay3D.h/.cpp` | Unconstrained Bowyer-Watson tetrahedralization |
| `ConstrainedDelaunay3D.h/.cpp` | Top-level constrained Delaunay volume mesher |
| `ShewchukRefiner3D.h/.cpp` | Shewchuk refinement loop (Priority 1/2/3) |
| `Shewchuk3DQualityController.h/.cpp` | Circumradius-to-edge quality bound configuration |

### Common
| File | Role |
|------|------|
| `src/Common/DebugFlags.h` | Runtime debug flags (`CMESH_DEBUG_ENABLED`) |
| `tests/Meshing/Core/test_ShewchukRefiner3D.cpp` | Volume refiner tests |

---

## Future Improvements

- [ ] **Metric adaptation in surface mesher (S2.2)** — For highly curved CAD surfaces, use the OCC pull-back metric (`GeomLProp_SLProps`) in the UV-space quality criterion to avoid angle distortion. Defer until basic surface mesher works.
- [ ] **Periodic mesh support (S3 extension)** — The `SharedEdgeRegistry` and inter-face node-pairing mechanism generalizes to periodic BCs. Instead of matching geometrically adjacent faces, pair nodes between user-specified opposite faces. Design the pairing interface early so it doesn't need to be retrofitted.
- [ ] **Parallelize verification loops in `MeshDebugUtils3D`** — The subsegment and subfacet presence checks iterate over all constraints sequentially. For large meshes these are embarrassingly parallel. Add optional OpenMP as done in `MeshVerifier` (2D overlap checks).
