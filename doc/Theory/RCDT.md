Here is the algorithm as implemented in OpenLoom, built from the paper and the flowchart documents.

---

## What problem RCDT solves

The old approach (UV-space mesher) triangulated each CAD face independently in its parametric (u,v) space, then stitched faces together via `TwinManager`. This broke on curved/periodic surfaces (cylinders, tori) because OCC's seam parameterisation creates coordinate discontinuities.

RCDT sidesteps this entirely. **It never works in UV space.** It builds a single 3D Delaunay tetrahedralization over all boundary vertices, and surface triangles *emerge* from it implicitly — no explicit constraint recovery, no stitching, no UV pathologies.

---

## Two key definitions

### Restricted face
A triangular face of the 3D tetrahedralization is **restricted to surface S** if:
1. All three of its nodes lie on S (or on curves/corners adjacent to S), **and**
2. The two tetrahedra sharing that face lie on **opposite sides** of S (one has a positive signed distance to S, the other negative).

The union of all restricted faces for surface S *is* the surface triangulation — automatically conforming, automatically shared across adjacent surfaces.

### Curve segment
A `CurveSegment [a, b]` is a straight mesh edge whose endpoints `a` and `b` are consecutive samples along a topology edge (CAD curve). A vertex `v` **encroaches** a segment if `v` lies strictly inside its diametral sphere (the sphere with diameter `|b − a|` centred at the midpoint). Endpoints are never considered encroaching their own segment.

---

## Phase 1 — Build Initial (`RCDTContext::buildInitial`)

**Goal:** Get a valid Delaunay tetrahedralization and identify the initial surface triangulation.

**R1.1 — Discretize boundary** (`BoundaryDiscretizer3D`)
- Walk every corner, edge, and surface interior in the CAD topology.
- Sample 3D points parametrically. Store results in `DiscretizationResult3D` with point arrays and index maps per corner/edge/surface.

**R1.2 — Build unconstrained Delaunay** (`Delaunay3D`, Bowyer-Watson)
- Create a bounding tetrahedron that encloses all points.
- Insert every discretized point incrementally:
  1. Find the seed tetrahedron containing the point.
  2. BFS flood-fill to find all tetrahedra whose circumsphere contains the point — the *conflict set*.
  3. Extract the *cavity boundary*: faces that appear in exactly one conflicting tet.
  4. Remove conflicting tets.
  5. Connect each cavity boundary face to the new point.
- Remove the bounding tetrahedron.

No constraints are enforced here. The tetrahedralization is purely Delaunay.

**R1.3 — Build restricted triangulation** (`RestrictedTriangulation::buildFrom`)
- For every tetrahedral face, resolve each node's geometry IDs to the set of CAD surface IDs it touches (direct surface membership, or via curve/corner adjacency — `effectiveSurfaceIds`).
- A face is a **candidate** for surface S if all three nodes touch S.
- **Confirm** with `SurfaceProjector::crossesSurface`: the opposite vertices of the two adjacent tets must have opposite sign of signed distance to S.
- Store all confirmed faces in `restrictedFaces_` (a map from `FaceKey` → `surfaceId`).

After R1.3, the restricted triangulation exists but may be coarse — areas with no interior sample points will have large triangles.

**R1.4 — Build curve segment manager** (`CurveSegmentOperations::buildCurveSegments`)
- For each topology edge, read the ordered node sequence from `DiscretizationResult3D`.
- Register one `CurveSegment` per consecutive node pair, recording the edge ID and parametric range `[tStart, tEnd]`.
- `CurveSegmentManager` is now the authoritative list of constrained boundary segments.

---

## Phase 2 — Refine (`RCDTRefiner::refine`)

This is the core loop. It runs until both conditions are satisfied: no encroached curve segments and no bad restricted triangles.

Each call to `refineStep()` picks the **highest-priority action** available.

---

### Priority 1 — Split encroached curve segments

**When:** Any mesh vertex (other than the endpoints of the segment) lies inside or on its diametral sphere.

**Steps:**
1. Scan `CurveSegmentManager::findEncroached` — iterate all segments, check each vertex against the diametral sphere. Use `excludeNodeId` so endpoints skip their own segments.
2. For the first encroached segment `[a, b]`:
   - Compute the split point via `CurveSegmentOperations::computeSplitPoint` — the 3D point on the CAD curve at the arc-length midpoint of `[tStart, tEnd]`.
3. **Pre-compute cavity interior faces** — before calling Bowyer-Watson, record which faces are shared by two tets in the cavity. These will be removed from `RestrictedTriangulation` (they no longer exist after insertion).
4. Insert the split point via Bowyer-Watson (`insertAndUpdate`):
   - Run Bowyer-Watson; collect new tetrahedra adjacent to the new node.
   - Remove stale restricted faces (cavity interior faces).
   - Re-classify all faces of the new tets using the restriction test (R1.3 steps 1–4). Add newly restricted faces.
5. Replace `[a, b]` with two sub-segments `[a, m]` and `[m, b]` in `CurveSegmentManager`.
6. Return `true` (work was done).

**Why Priority 1 terminates:** Each split places a vertex on the curve, strictly reducing the maximum segment length. A split point cannot immediately re-encroach the two segments it creates.

---

### Priority 2 — Split bad restricted triangles

**When:** No encroached segments exist, but some restricted face fails `RCDTQualitySettings`.

A restricted face is **bad** if either:
- `circumradius / shortestEdge > maximumCircumradiusToShortestEdgeRatio` (measured in 3D)
- The chord deviation from the CAD surface exceeds `maximumChordDeviation` at the circumcenter or edge midpoints

**Steps:**
1. Call `RestrictedTriangulation::getBadTriangles(settings, meshData, geometry)` — returns a list sorted by severity, each annotated with `surfaceId` and `circumcircleCenter`.
2. For the worst bad triangle:
   - The `circumcircleCenter` is computed in the tangent plane of the surface at the triangle centroid.
3. **Circumcenter demotion** (key rule): Check whether inserting the circumcircle centre would encroach any `CurveSegment`. If yes — **do not insert it**. Instead, split those encroached segments (Priority 1). Return `true`.
4. If no encroachment: insert the circumcircle centre via `insertAndUpdate` (same as step 4 of Priority 1). The bad triangle is eliminated because a vertex now lies inside its circumsphere.
5. Return `true`.

**Why circumcenter demotion is the termination guarantee:** Without it, inserting a circumcenter near a curve boundary could create a tiny new segment, whose encroachment forces another insertion, ad infinitum. Demotion ensures that insertions near features are always preceded by sufficient segment refinement, breaking the size-reduction cycle.

---

### Loop terminates when
`refineStep()` returns `false` — no encroached segments and no bad restricted triangles remain.

---

## Phase 3 — Build Surface Mesh (`RCDTContext::buildSurfaceMesh`)

1. Collect all faces from `RestrictedTriangulation::getRestrictedFaces()`.
2. For each restricted face, read the three node positions from `MeshData3D`.
3. Assemble `SurfaceMesh3D`: one `Node3D` per unique node ID, one `TriangleElement` per restricted face.
4. Adjacent CAD surfaces automatically share boundary nodes — because those nodes are the same mesh nodes in the single shared tetrahedralization. No stitching needed.

---

## Volume extension (three priorities)

For volume meshing, Phase 2 adds a third priority, only active when Priority 1 and 2 are both exhausted:

**Priority 3 — Split skinny tetrahedra**
- A tet is *skinny* if its circumradius/shortest-edge ratio exceeds the volume quality bound.
- Compute its circumcenter.
- **Cascade demotion:** if inserting would encroach a curve segment → Priority 1. If it would create a bad restricted triangle → Priority 2. Otherwise insert.

After refinement, flood-fill from outside the domain to classify interior vs exterior tets, then remove all exterior ones.

---

## Data structures that must stay consistent after every insertion

| Structure | What changes |
|-----------|-------------|
| `MeshData3D` | New node added; new tets added; old tets removed |
| `RestrictedTriangulation` | Cavity interior faces removed; new faces of new tets re-classified and added/skipped |
| `CurveSegmentManager` | Replaced `[a,b]` with `[a,m]` and `[m,b]` on segment splits |

These three must be updated atomically in `insertAndUpdate`. If any one is stale, the refinement loop will make incorrect decisions — which is the most common source of bugs in this pipeline.