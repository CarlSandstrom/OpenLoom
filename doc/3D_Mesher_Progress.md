# 3D Mesher Implementation Progress

Tracks implementation of the Shewchuk 3D Delaunay refinement algorithm as described in `doc/Flowcharts/3D algorithm.md`. Each step has a validation gate that must pass before proceeding.

## Prerequisites

- [x] **`MeshDebugUtils3D`** — Phase-aware export + verify utility (modeled on `MeshDebugUtils2D`)
  - Accepts a `MeshingPhase3D` enum to select which invariants to check
  - Each phase accumulates checks from all previous phases
  - Status: **Not yet created**

---

## Step 1 — Input Processing

**Algorithm ref:** Sections 1.1–1.4

| Sub-step | Description | File(s) | Status |
|----------|-------------|---------|--------|
| 1.1 | Receive PLC from CAD geometry/topology | `MeshingContext3D` | Done |
| 1.2 | Discretize edges (parametric sampling) | `BoundaryDiscretizer3D` | Done |
| 1.3 | Discretize surfaces | `BoundaryDiscretizer3D` | Done |
| 1.4.1 | Extract subsegment list from topology edges | `MeshQueries3D::extractConstrainedSubsegments`, `ConstraintRegistrar3D` | Done |
| 1.4.2 | Create independent 2D Delaunay per facet | `FacetTriangulationManager`, `FacetTriangulation` | Done |

**Validation gate:** All input vertices discretized. Each topology edge produces a chain of subsegments. Each surface has a valid facet triangulation.

**Status: COMPLETE**

---

## Step 2 — Initial Delaunay Tetrahedralization (Unconstrained)

**Algorithm ref:** Sections 2.1–2.3

| Sub-step | Description | File(s) | Status |
|----------|-------------|---------|--------|
| 2.1 | Create bounding tetrahedron | `MeshOperations3D::createBoundingTetrahedron` | Done |
| 2.2 | Incremental Bowyer-Watson insertion | `MeshOperations3D::insertVertexBowyerWatson` | Done |
| 2.2.1 | Find seed tetrahedron containing point | `MeshQueries3D::findConflictingTetrahedra` | Done |
| 2.2.2 | Circumsphere test (BFS flood fill) | `MeshQueries3D::findConflictingTetrahedra` | Done |
| 2.2.3 | Extract cavity boundary | `MeshQueries3D::findCavityBoundary` | Done |
| 2.2.4 | Remove conflicting tetrahedra | `MeshOperations3D` (internal) | Done |
| 2.2.5 | Retriangulate cavity | `MeshOperations3D::retriangulate` | Done |
| 2.3 | Remove bounding tetrahedron | `MeshOperations3D::removeBoundingTetrahedron` | Done |

**Validation gate (`MeshingPhase3D::InitialDelaunay`):**
- [ ] No degenerate tetrahedra (volume > 0)
- [ ] No inverted tetrahedra
- [ ] All node references valid
- [ ] No NaN/Inf coordinates
- [ ] All input vertices present as mesh nodes

Note: Constraints (subsegments, subfacets) may be missing from the mesh — this is expected.

**Status: COMPLETE**

---

## Step 3 — Segment Recovery

**Algorithm ref:** Sections 3.1–3.4

| Sub-step | Description | File(s) | Status |
|----------|-------------|---------|--------|
| 3.1 | Check if each subsegment exists as mesh edge | `MeshQueries3D::edgeExistsInMesh` | Done |
| 3.2 | Identify missing (encroached) subsegments | `ConstraintChecker3D` | Done |
| 3.3.1 | Compute midpoint on parent edge curve | `MeshOperations3D::splitConstrainedSubsegment` | Done |
| 3.3.2 | Insert midpoint via Bowyer-Watson | `MeshOperations3D::insertVertexBowyerWatson` | Done |
| 3.3.3 | Replace subsegment with two children in constraint list | `MeshOperations3D::splitConstrainedSubsegment` | Done |
| 3.3.4 | Insert midpoint into affected facet triangulations | `FacetTriangulationManager` | **TODO** |
| 3.4 | **Wire into refiner: loop until all subsegments present** | `ShewchukRefiner3D` | **TODO** |

**What exists:** The building blocks (`splitConstrainedSubsegment`, `edgeExistsInMesh`, `ConstraintChecker3D`) are all implemented and tested individually. What's missing is the orchestration loop in `ShewchukRefiner3D` and the facet triangulation update when splitting a subsegment that lies on a facet boundary.

**Validation gate (`MeshingPhase3D::SegmentRecovery`):**
- [ ] All checks from `InitialDelaunay` pass
- [ ] Every subsegment in `MeshData3D::getConstrainedSubsegments()` exists as a mesh edge (`edgeExistsInMesh` returns true)

**Status: IN PROGRESS — infrastructure done, orchestration TODO**

---

## Step 4 — Facet Recovery

**Algorithm ref:** Sections 4.1–4.4

| Sub-step | Description | File(s) | Status |
|----------|-------------|---------|--------|
| 4.1 | Compare facet triangulations against tet faces | `MeshQueries3D::faceExistsInMesh` | Done (query exists) |
| 4.2.1 | Projection Lemma: project encroaching vertex onto facet | — | **TODO** |
| 4.2.2 | If projection inside facet, split containing subfacet first | — | **TODO** |
| 4.3.1 | Compute subfacet circumcenter in facet plane | `MeshOperations3D::splitConstrainedSubfacet` | Done |
| 4.3.2 | Check if circumcenter encroaches any subsegment | `MeshQueries3D::findEncroachingSubsegments` | Done |
| 4.3.3 | Insert circumcenter into facet triangulation + tetrahedralization | `FacetTriangulationManager`, `MeshOperations3D` | Partially done |
| 4.4 | **Wire into refiner: loop until all subfacets present** | `ShewchukRefiner3D` | **TODO** |

**What exists:** `splitConstrainedSubfacet`, `faceExistsInMesh`, and encroachment queries. Missing: Projection Lemma implementation, cocircularity handling, and the orchestration loop in the refiner.

**Validation gate (`MeshingPhase3D::FacetRecovery`):**
- [ ] All checks from `SegmentRecovery` pass
- [ ] Every subfacet in each facet triangulation exists as a mesh face (`faceExistsInMesh` returns true)

**Status: IN PROGRESS — infrastructure partially done, orchestration TODO**

---

## Step 5 — Exterior Removal

**Algorithm ref:** Sections 5.1–5.3

| Sub-step | Description | File(s) | Status |
|----------|-------------|---------|--------|
| 5.1 | Facets partition mesh into interior/exterior | (precondition from Step 4) | — |
| 5.2 | Flood-fill classify tetrahedra | `MeshOperations3D::classifyTetrahedraInteriorExterior` | Done |
| 5.3 | Remove exterior tetrahedra | `MeshOperations3D::classifyTetrahedraInteriorExterior` | Done |
| — | **Integrate into pipeline** (call after Step 4 completes) | `ShewchukRefiner3D` or pipeline orchestrator | **TODO** |

**What exists:** `classifyTetrahedraInteriorExterior()` is implemented but not called automatically in the refinement pipeline.

**Validation gate (`MeshingPhase3D::ExteriorRemoved`):**
- [ ] All checks from `FacetRecovery` pass
- [ ] No tetrahedra exist outside the PLC boundary
- [ ] Mesh element count decreased (exterior tets removed)

**Status: IMPLEMENTED — needs pipeline integration**

---

## Step 6 — Shewchuk's 3D Delaunay Refinement

**Algorithm ref:** Sections 6.1–6.4

| Sub-step | Description | File(s) | Status |
|----------|-------------|---------|--------|
| 6.1 | **Priority 1:** Split encroached subsegments | `ShewchukRefiner3D` | **TODO** (stub exists) |
| 6.1.1 | Diametral sphere encroachment test | `ConstraintChecker3D::isSubsegmentEncroached` | Done |
| 6.1.2 | Split at midpoint via Bowyer-Watson | `MeshOperations3D::splitConstrainedSubsegment` | Done |
| 6.1.3–4 | Repeat until none encroached | `ShewchukRefiner3D` | **TODO** |
| 6.2 | **Priority 2:** Split encroached subfacets | `ShewchukRefiner3D` | **TODO** (stub exists) |
| 6.2.1 | Equatorial sphere encroachment test | `ConstraintChecker3D::isSubfacetEncroached` | Done |
| 6.2.2 | Projection Lemma ordering | — | **TODO** (same as 4.2) |
| 6.2.3 | Circumcenter encroachment check before insertion | `MeshQueries3D::findEncroachingSubsegments` | Done |
| 6.2.4 | Insert circumcenter into facet + tetrahedralization | `MeshOperations3D::splitConstrainedSubfacet` | Done |
| 6.3 | **Priority 3:** Split skinny tetrahedra | `ShewchukRefiner3D::handleSkinnyTetrahedron` | **Partial** |
| 6.3.1 | Circumradius-to-shortest-edge ratio test | `ElementQuality3D`, `Shewchuk3DQualityController` | Done |
| 6.3.2 | Select worst tetrahedron | `ElementQuality3D::getSkinnyTetrahedraSortedByQuality` | Done |
| 6.3.3 | Compute circumcenter | `ElementGeometry3D::computeCircumscribingSphere` | Done |
| 6.3.4.1 | Check subsegment encroachment before insertion | `ShewchukRefiner3D::handleSkinnyTetrahedron` | **TODO** (skipped) |
| 6.3.4.2 | Check subfacet encroachment before insertion | `ShewchukRefiner3D::handleSkinnyTetrahedron` | **TODO** (skipped) |
| 6.3.4.3 | Insert circumcenter if no encroachment | `ShewchukRefiner3D::handleSkinnyTetrahedron` | Done |

**What exists:** Priority 3 works for the simple case (insert circumcenter without encroachment checks). The TODO comments in `ShewchukRefiner3D.cpp:82` and `ShewchukRefiner3D.cpp:144` mark where Priority 1, 2, and encroachment checks need to go.

**Validation gate (`MeshingPhase3D::Refined`):**
- [ ] All checks from `ExteriorRemoved` pass
- [ ] No encroached subsegments remain
- [ ] No encroached subfacets remain
- [ ] All tetrahedra satisfy quality bound B (circumradius/shortest-edge < B)

**Status: PARTIALLY IMPLEMENTED — Priority 3 only, encroachment checks missing**

---

## Step 7 — Post-Processing

**Algorithm ref:** Sections 7.1–7.2

| Sub-step | Description | File(s) | Status |
|----------|-------------|---------|--------|
| 7.1.1 | Identify slivers (near-zero volume, extreme dihedral angles) | — | **TODO** |
| 7.1.2 | Split slivers at circumcenter using Step 6.3 mechanism | — | **TODO** |
| 7.1.3 | Prioritize by circumradius-to-edge ratio | — | **TODO** |
| 7.2.1 | Optimization-based vertex smoothing | — | **TODO** |
| 7.2.2 | Topological transforms (2-3, 3-2 flips) | — | **TODO** |

**Validation gate (`MeshingPhase3D::PostProcessed`):**
- [ ] All checks from `Refined` pass
- [ ] No tetrahedra with dihedral angles below threshold (e.g., 19 degrees)

**Status: NOT STARTED**

---

## Recommended Implementation Order

The steps below respect dependencies — each builds on the previous.

### Phase A: Validation infrastructure
1. Create `MeshDebugUtils3D` with `MeshingPhase3D` enum
2. Add phase-specific checks that extend `MeshVerifier3D`
3. Wire into existing examples to confirm Steps 1–2 pass validation

### Phase B: Segment recovery (Step 3)
4. Add facet triangulation update when splitting a boundary subsegment (3.3.4)
5. Implement segment recovery loop in `ShewchukRefiner3D` (3.4)
6. Test: all subsegments exist as mesh edges after recovery

### Phase C: Facet recovery (Step 4)
7. Implement Projection Lemma (4.2)
8. Implement facet recovery loop in `ShewchukRefiner3D` (4.4)
9. Test: all subfacets exist as mesh faces after recovery

### Phase D: Exterior removal integration (Step 5)
10. Call `classifyTetrahedraInteriorExterior()` in pipeline after facet recovery
11. Test: no exterior tetrahedra remain, mesh is domain-only

### Phase E: Complete refinement loop (Step 6)
12. Add Priority 1 to `refineStep()` (encroached subsegments)
13. Add Priority 2 to `refineStep()` (encroached subfacets)
14. Add encroachment checks to `handleSkinnyTetrahedron()` (6.3.4)
15. Test: full three-priority loop on real geometry

### Phase F: Post-processing (Step 7)
16. Sliver detection (dihedral angle metric)
17. Sliver removal via circumcenter splitting
18. (Optional) Smoothing and topological flips

---

## Key Files Reference

| File | Role |
|------|------|
| `src/Meshing/Core/3D/ShewchukRefiner3D.h/.cpp` | Main refinement loop — most work happens here |
| `src/Meshing/Core/3D/MeshOperations3D.h/.cpp` | Mutation operations (Bowyer-Watson, splitting) |
| `src/Meshing/Core/3D/MeshQueries3D.h/.cpp` | Read-only queries (edge/face existence, encroachment) |
| `src/Meshing/Core/3D/ConstraintChecker3D.h/.cpp` | Encroachment tests |
| `src/Meshing/Core/3D/FacetTriangulationManager.h/.cpp` | Per-surface 2D Delaunay management |
| `src/Meshing/Core/3D/MeshVerifier3D.h/.cpp` | Basic mesh integrity checks |
| `src/Meshing/Core/3D/MeshingContext3D.h/.cpp` | Owns mesh data + operations lifecycle |
| `src/Meshing/Core/3D/ElementQuality3D.h/.cpp` | Quality metrics |
| `src/Meshing/Core/3D/ElementGeometry3D.h/.cpp` | Geometric computations (circumsphere, volume) |
| `src/Common/DebugFlags.h` | Runtime debug flags (`CMESH_DEBUG_ENABLED`) |
| `tests/Meshing/Core/test_ShewchukRefiner3D.cpp` | Refiner tests (includes TODO stubs for encroachment) |
