# 3D Surface Mesh Generation by RCDT

Based on the algorithm in "Restricted Constrained Delaunay Triangulations".

## Key Idea

Rather than triangulating each CAD face independently in UV space, RCDT works in
ambient 3D space. A single 3D Delaunay tetrahedralization is built over all input
vertices. A face of that tetrahedralization is **restricted** to a CAD surface S if
all three of its nodes lie on S and the two adjacent tetrahedra are on opposite sides
of S. The union of all restricted faces for surface S is the surface triangulation.

There is no explicit surface constraint recovery phase. The surface triangulation
emerges from the tetrahedralization once the vertex set is refined to place vertices
where they are needed.

---

## Phase 1 — Build Initial (`RCDTContext::buildInitial`)

### R1.1 Boundary discretization (`BoundaryDiscretizer3D`)

Walks the topology and samples 3D points along every corner, edge, and surface
interior — identical to the discretization step of the legacy UV-space mesher. Produces
`DiscretizationResult3D` with point arrays and index maps per corner, edge, and surface.

### R1.2 Delaunay tetrahedralization (`Delaunay3D`)

Insert all discretized boundary points into an unconstrained 3D Delaunay
tetrahedralization via Bowyer-Watson. No constraint awareness at this stage.

### R1.3 Build restricted triangulation (`RestrictedTriangulation::buildFrom`)

Classify every face of the tetrahedralization:

1. For each tetrahedral face, retrieve the geometry IDs of its three nodes.
2. Resolve each node's geometry IDs to the set of CAD surface IDs it touches (direct
   surface membership, or edge membership → adjacent surfaces).
3. A face is restricted to surface S if all three nodes touch S.
4. Confirm restriction with `SurfaceProjector.crossesSurface`: the signed distances
   of the two adjacent tetrahedra's opposite vertices to S must have opposite signs,
   meaning the face lies on S rather than inside or outside it.
5. Store restricted faces in `RestrictedTriangulation::restrictedFaces_` (a map from
   `FaceKey` to `surfaceId`).

After this step, `restrictedFaces_` contains the initial surface triangulation derived
from the as-inserted vertex set. It may be coarse or miss areas with no interior points.

### R1.4 Build curve segment manager (`CurveSegmentOperations::buildCurveSegments`)

Populate `CurveSegmentManager` from the topology edges:

- For each topology edge (excluding seam twin edges), read the ordered point-index
  sequence from `DiscretizationResult3D::edgeIdToPointIndicesMap`.
- Convert point indices to mesh node IDs via `pointIndexToNodeIdMap`.
- Record the parametric value of each point along the edge curve.
- Register one `CurveSegment` per consecutive pair of nodes, associated with the
  edge's geometry and parametric bounds.

`CurveSegmentManager` is the authoritative list of constrained segments on curve
boundaries. Splitting a segment updates this list.

---

## Phase 2 — Refine (`RCDTContext::refine` → `RCDTRefiner::refine`)

A two-priority loop. Each iteration selects the highest-priority action available.

### R2.1 Priority 1 — Split encroached curve segments

A `CurveSegment` `[a, b]` is **encroached** if any mesh vertex other than `a` or `b`
lies strictly inside or on its diametral sphere (the sphere whose diameter equals `|b − a|`,
centred at the midpoint).

- **R2.1.1** Scan `CurveSegmentManager` for encroached segments.
- **R2.1.2** For the first encroached segment, compute its split point via
  `CurveSegmentOperations::computeSplitPoint` — the 3D point on the edge curve at the
  arc-length midpoint of the segment's parametric range.
- **R2.1.3** Pre-compute the cavity interior faces (faces shared by exactly two
  tetrahedra in the Bowyer-Watson cavity) before insertion, so `RestrictedTriangulation`
  can remove stale restricted faces incrementally.
- **R2.1.4** Insert the split point via Bowyer-Watson (`RCDTRefiner::insertAndUpdate`):
  - Run Bowyer-Watson; collect the new tetrahedra adjacent to the new node.
  - Remove the cavity interior faces from `RestrictedTriangulation`.
  - Re-classify all faces of the new tetrahedra: for each face adjacent to the new node,
    run the restriction test (R1.3 step 3–4). Add newly restricted faces to
    `RestrictedTriangulation`.
- **R2.1.5** Replace the original `CurveSegment` with two sub-segments in
  `CurveSegmentManager`.
- **R2.1.6** Repeat until no encroached segments remain.

Convergence is guaranteed: each split places a vertex on the curve, reducing the maximum
segment length. Once segments are short enough, no vertex can fall in their diametral sphere.

### R2.2 Priority 2 — Split bad restricted triangles (only when no encroached segments exist)

A restricted face is **bad** if it fails `RCDTQualitySettings`:

- `circumradius / shortestEdge > maximumCircumradiusToShortestEdgeRatio` (measured in 3D)
- chord deviation from the CAD surface exceeds `maximumChordDeviation` at the
  circumcenter or any edge midpoint

`RestrictedTriangulation::getBadTriangles` returns a list of bad faces sorted by severity,
each annotated with its `surfaceId` and circumcircle centre.

For each bad triangle:

- **R2.2.1** Compute the circumcircle centre in the tangent plane of the surface at the
  triangle centroid (stored in `BadRestrictedTriangle::circumcircleCenter`).
- **R2.2.2** **Circumcenter demotion**: if inserting the circumcircle centre would
  encroach any `CurveSegment`, do not insert it. Instead, split all encroached segments
  (return to R2.1). This rule prevents infinitely small insertions near curve boundaries
  and is the key to termination.
- **R2.2.3** Otherwise, insert the circumcircle centre via `insertAndUpdate` (same as
  R2.1.4). The bad triangle is eliminated because a vertex now lies inside its circumsphere.
- **R2.2.4** Repeat until no bad restricted triangles remain.

### R2.3 Termination

The loop terminates when no encroached segments and no bad restricted triangles exist.
Termination is guaranteed because:

- Every segment split reduces the maximum segment length; segments cannot be re-encroached
  by their own split points.
- The circumcenter demotion rule (R2.2.2) ensures that circumcenters inserted near features
  are always preceded by sufficient segment refinement, preventing size-reduction cycles.

---

## Phase 3 — Build Surface Mesh (`RCDTContext::buildSurfaceMesh`)

1. Collect all restricted faces from `RestrictedTriangulation::getRestrictedFaces()`.
2. For each restricted face, read the three node positions from `MeshData3D`.
3. Assemble a `SurfaceMesh3D`: one `Node3D` per unique node ID appearing in the
   restricted faces, one `TriangleElement` per restricted face.
4. Return the `SurfaceMesh3D`.

The output is a watertight triangulation of all input CAD surfaces. Adjacent CAD surfaces
automatically share the nodes that lie on their common topology edges, because those nodes
are the same mesh nodes in the single shared tetrahedralization.

---

## Key Invariants

| After step | Invariant |
|------------|-----------|
| R1.2 | Tetrahedralization is Delaunay; all boundary vertices inserted |
| R1.3 | `restrictedFaces_` holds the initial surface triangulation |
| R1.4 | `CurveSegmentManager` holds one segment per consecutive boundary node pair |
| During R2 | `restrictedFaces_` is current: updated after every Bowyer-Watson insertion |
| After R2 | No encroached segments; all restricted triangles satisfy `RCDTQualitySettings` |

---

## Data Structures

| Structure | Purpose | Lifetime |
|-----------|---------|----------|
| `DiscretizationResult3D` | All sampled 3D points; corner/edge/surface index maps | Built in R1.1, read-only after |
| `MeshData3D` (via `MeshingContext3D`) | The 3D Delaunay tetrahedralization | Built in R1.2, refined during R2 |
| `RestrictedTriangulation` | Map from `FaceKey` → `surfaceId` for all restricted faces | Built in R1.3, updated after each insertion |
| `CurveSegmentManager` | All constrained curve segments; updated as segments are split | Built in R1.4, mutated during R2 |
| `BadRestrictedTriangle` | A failing restricted face with its `circumcircleCenter` | Computed on demand in R2.2 |

---

## Comparison with the Legacy UV-Space Mesher

| Aspect | Legacy (UV-space) | RCDT (ambient-space) |
|--------|-------------------|----------------------|
| Working space | Per-face UV parametric space | 3D ambient space |
| Surface constraint recovery | Explicit: `FacetTriangulationManager`, `TwinManager` | Implicit: restricted faces emerge from tetrahedralization |
| Shared-edge synchronisation | `TwinManager` + cross-face callbacks | Automatic: shared nodes are the same mesh nodes |
| Quality evaluation | UV-space angles (Phase 1), 3D chord deviation (Phase 2) | 3D circumradius ratio and chord deviation in one pass |
| Quality interface | `IQualityController2D` strategy | `RCDTQualitySettings` struct |
