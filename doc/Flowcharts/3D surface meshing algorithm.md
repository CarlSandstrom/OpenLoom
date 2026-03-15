# 3D Surface Mesh Generation

## Overview

The surface mesher triangulates each CAD face independently in its own UV parametric
space. Each face gets a 2D constrained Delaunay triangulation that is refined to meet
quality criteria. The resulting triangles are mapped back to 3D coordinates at export.

The central challenge is **shared edges**: when face A and face B share a topology edge,
both faces have boundary segments covering that edge. Quality refinement splits those
boundary segments to insert new nodes. Every split on one face must produce a matching
split at the same 3D position on the adjacent face — otherwise the two triangulations
disagree on where their shared boundary is, producing gaps or T-junctions.

---

## S1. Initialization

### S1.1 Boundary discretization (`BoundaryDiscretizer3D`)

Walks the topology and samples 3D points along every corner, edge, and surface interior:

- **Corners**: one `Point3D` per topology corner; records `cornerId → pointIndex` in
  `cornerIdToPointIndexMap`.
- **Edge interiors**: samples points along each edge (angle-based or fixed-count) and
  stores the complete ordered sequence `[startCorner, interior..., endCorner]` as point
  indices in `edgeIdToPointIndicesMap`. The sequence always runs in the edge's canonical
  direction (parameter `tMin → tMax`).
- **Seam twin edges**: a periodic seam edge (e.g. the closing seam of a cylinder) has no
  independent 3D geometry; its point sequence is the reverse of the original seam edge's
  sequence, stored under the twin edge ID.

**Key invariant**: every edge's point index sequence is stored in canonical
(start → end) order. Adjacent faces that share an edge both index into the same
`DiscretizationResult3D::points` array, so the same 3D positions are referenced
by both faces.

### S1.2 Per-face triangulation setup (`FacetTriangulationManager`)

For each topology surface:

1. **Collect boundary points** (`collectBoundaryPointIndices`): gather the global point
   indices for the surface's corners and non-seam-twin boundary edges. Seam twin edges
   are excluded here — they are handled by UV-shifted copies in the next step.

2. **Build UV-space discretization** (`FacetDiscretization2DBuilder`):
   - Project each 3D point onto the surface's UV plane via `surface.projectPoint()`.
   - Normalize each edge's raw parameter values to `[0, 1]`.
   - **Seam twin edges (phase 1)**: for each seam twin edge, create UV-shifted copies of
     all its points by adding `uPeriod = uMax - uMin` to each U coordinate. These shifted
     copies represent the same 3D positions at the opposite side of the periodic boundary.
   - **All other edges (phase 2)**: translate global point sequences to local (0-based)
     indices. Closed-circle edges (where start and end share one global point) close at
     `U + uPeriod` using the seam-shifted copy.
   - The result is a `DiscretizationResult2D` with local point indices and 2D UV
     coordinates, plus a `localIndexToNode3DId` vector mapping each local index to its
     3D node ID. In the surface-mesher path, the mapping is the identity (point index
     equals 3D node ID).

3. **Run constrained Delaunay 2D** (`ConstrainedDelaunay2D`): registers boundary
   segments, enforces them, and removes exterior triangles. After this step each face
   has a valid 2D constrained triangulation covering its UV domain.

4. **Build edge node sequences**: `FacetTriangulation::initialize()` builds
   `edgeIdToNode2DSeq_` — a map from edge ID to the ordered sequence of **2D node IDs**
   along that edge, in the same canonical order as the 3D discretization. These sequences
   are used in the next step to register twin pairs.

### S1.3 Twin manager construction (`FacetTriangulationManager::buildTwinManager`)

The `TwinManager` tracks which boundary segment on one face corresponds to which segment
on an adjacent face.

#### Cross-surface twins

For each topology edge shared by exactly two surfaces (from `TwinTableGenerator`):

- Retrieve the 2D node sequence for that edge from each face's `FacetTriangulation`.
- **Both sequences are in canonical (start → end) order** — because `BoundaryDiscretizer3D`
  always builds sequences in canonical order, and `FacetDiscretization2DBuilder` preserves
  that order when projecting to UV space. This means `seq0[i]` and `seq1[i]` represent the
  same 3D point on the shared edge, even though the two faces traverse the edge from
  opposite sides in 3D.
- Register segment pairs by matching index: for each `i`, call
  `registerTwin(surfaceId0, seq0[i], seq0[i+1], surfaceId1, seq1[i], seq1[i+1])`.

> **Why canonical order always holds**: face 0 sees the edge running from corner A to
> corner B; face 1 may traverse it from B to A in 3D, but `BoundaryDiscretizer3D`
> always samples from `tMin` to `tMax`, and `FacetDiscretization2DBuilder` reads the
> same sequence directly. There is no orientation correction to apply.
>
> If this invariant is ever broken — for example by reversing a sequence during UV
> projection — twin pairing will be completely wrong: segment `i` on face 0 will be
> paired with segment `N-i` on face 1, matching nodes that are at opposite ends of
> the shared edge. The mesh will crack at every shared boundary.

#### Seam twins (same-surface periodic boundaries)

For each seam pair (`origEdgeId`, `twinEdgeId`) on a surface:

- Retrieve `origSeq` and `twinSeq` from the face's `FacetTriangulation`.
- The seam and its UV-shifted twin traverse the same 3D points in **opposite directions**
  (the twin is the reverse of the original in 3D, which is how a periodic surface closes).
  Therefore: `origSeq[i]` pairs with `twinSeq[N-i]`, so segment `i` of the original
  pairs with segment `(N-1-i)` of the twin running backwards:
  `registerTwin(surfaceId, origSeq[i], origSeq[i+1], surfaceId, twinSeq[N-i], twinSeq[N-i-1])`.

#### TwinManager internals

`registerTwin(A, n1, n2, B, m1, m2)` stores all four directed entries:

```
(A, n1, n2) → (B, m1, m2)
(A, n2, n1) → (B, m2, m1)
(B, m1, m2) → (A, n1, n2)
(B, m2, m1) → (A, n2, n1)
```

This means a lookup succeeds regardless of which direction the refiner traverses a
segment. When a segment is split, `recordSplit` removes the four old entries and
registers two new pairs for the sub-segments, keeping the map current throughout
refinement.

---

## S2. Quality Refinement (`refineSurfaces`)

Refinement runs `ShewchukRefiner2D` on every face in passes until no cross-face
boundary split occurs in a complete pass.

### S2.1 The convergence loop

```
anyNewSplits ← true
while anyNewSplits:
    anyNewSplits ← false
    for each surface in topology order:
        hadCrossFaceSplit ← refineOneFace(surface)
        if hadCrossFaceSplit: anyNewSplits ← true
```

A single pass over all faces is often sufficient (most splits happen on faces that
haven't been refined yet in the current pass, so the split is already present when the
twin face is refined). A second pass is needed when face A splits face B's boundary
after B has already run in the current pass — B then re-runs in the next pass and
quality-refines the area around the new node.

Subsequent passes converge quickly: the Shewchuk refiner completes in 0 iterations on
faces that already satisfy quality criteria.

### S2.2 Per-face refinement (`refineOneFace`)

For each face:

1. Create `Shewchuk2DQualityController` with the caller's quality bounds.
2. Create `ShewchukRefiner2D` over the face's `MeshingContext2D`.
3. Register the boundary-split callback (see S2.3).
4. Call `refiner.refine()` — the standard Shewchuk two-priority loop:
   split encroached boundary segments first, then split poor-quality triangles
   by inserting circumcenters.
5. Call `facetTriang->resolveRefinementNodes(nextNode3DId)`: any 2D node that was
   inserted by the refiner and has no 3D peer yet is assigned a new 3D ID, lifted to
   3D via `surface.getPoint(u, v)`, and appended to `refinementNodes_`.
6. Return whether any cross-face boundary split occurred.

### S2.3 The boundary-split callback

This callback fires every time the 2D refiner splits a boundary segment `(n1, n2)` at
a new midpoint node `mid`. It is responsible for keeping the twin face consistent.

```
onBoundarySplit(n1, n2, mid):
    twinOpt ← twinManager.getTwin(surfaceId, n1, n2)
    if not twinOpt: return              # interior or non-twin boundary segment

    (twinSurfaceId, m1, m2) ← twinOpt

    # Compute 3D position and register 3D node on the source face
    uv ← faceContext.getMeshData().getNode(mid).getCoordinates()
    node3DId ← nextNode3DId++
    refinementNodes_.push_back(sourceSurface.getPoint(uv))
    facetTriang.registerNode(mid, node3DId)
    facetTriang.updateEdgeNodeAfterSplit(n1, n2, mid)

    if twinSurfaceId == surfaceId:
        # Seam twin — both sides live in this face's own MeshData2D.
        # Apply immediately; safe because the refiner is operating on the
        # same context and seam splits do not change quality on other faces.
        splitSeamTwin(m1, m2, node3DId)
    else:
        # Cross-surface twin — apply to the adjacent face immediately.
        applyCrossFaceSplit(twinFacet, twinSurfaceId, m1, m2, node3DId)
        hadCrossFaceSplit ← true   # triggers another pass if needed
```

#### Seam twin application

Looks up the common edge geometry for `(m1, m2)` in the same face's context, calls
`splitConstrainedSegment`, registers the new node, updates the edge sequence, and calls
`recordSplit` to update the TwinManager.

#### Cross-surface twin application (`applyCrossFaceSplit`)

1. Look up the common edge geometry for `(m1, m2)` in the **twin** face's context.
2. Call `splitConstrainedSegment` on the twin face — this inserts the split point into
   the twin's 2D constrained triangulation at the correct parametric position.
3. Register `twinMid ↔ node3DId` on the twin face via `registerNode`. Both faces will
   now reference the same 3D node ID for this point.
4. Call `updateEdgeNodeAfterSplit(m1, m2, twinMid)` on the twin face to keep its edge
   node sequence current.
5. Call `twinManager.recordSplit(...)` to replace the old `(n1,n2)↔(m1,m2)` entries
   with two new sub-segment pairs: `(n1,mid)↔(m1,twinMid)` and `(mid,n2)↔(twinMid,m2)`.

**Why immediate application is mandatory**: the Shewchuk refiner may subsequently split
the sub-segments `(n1, mid)` or `(mid, n2)` within the same refinement of face A. Each
such sub-split will call this callback and look up the TwinManager for `(surfaceId, n1, mid)`.
If `recordSplit` has not yet been called, that lookup returns nothing and the sub-split
is silently dropped — leaving the twin face with a stale, unsplit boundary that no longer
corresponds to face A's boundary.

### S2.4 Required invariants during refinement

The following must hold at all times during and after each callback invocation:

| Invariant | Maintained by |
|-----------|---------------|
| `TwinManager` is current for all existing boundary segments | `recordSplit` called in every callback before returning |
| Edge node sequences reflect all splits | `updateEdgeNodeAfterSplit` called on both source and twin face |
| Both faces share the same 3D node ID for a shared boundary point | `registerNode` called on twin face with the source's `node3DId` |
| `refinementNodes_` contains a 3D position for every new boundary node | `refinementNodes_.push_back(...)` called in callback before `resolveRefinementNodes` |

---

## S3. Export (`getSurfaceMesh3D`)

1. Add nodes `0 .. N-1`: one `Node3D` per point in `discretizationResult_->points`.
2. Add nodes `N .. M-1`: one `Node3D` per point in `refinementNodes_` (inserted during S2).
3. For each face's `FacetTriangulation`, translate each 2D triangle's node IDs to 3D node
   IDs via `node2DTo3DMap_` and emit a `TriangleElement`.

The output `MeshData3D` is a watertight triangulation: adjacent faces share the same 3D
node IDs at their common boundary, so there are no gaps or duplicate nodes at shared edges.

---

## Data structures

| Structure | Purpose | Lifetime |
|-----------|---------|----------|
| `DiscretizationResult3D` | All sampled 3D points; corner/edge/surface index maps | Built in S1.1, read-only after that |
| `TwinManager` | Directed lookup: segment on face A ↔ segment on face B; updated as splits occur | Built in S1.3, mutated during S2 |
| `FacetTriangulation` | Owns `MeshingContext2D`; maps `node3DId ↔ node2DId`; tracks edge node sequences | Created in S1.2, refined in S2 |
| `EdgeTwinTable` | Maps shared edge ID → two adjacent surface IDs | Built and consumed in S1.3 |
| `refinementNodes_` | 3D positions for nodes inserted during refinement | Populated during S2, consumed in S3 |

---

## Common mistakes

**Reversing a sequence during UV projection**: both edge node sequences for a shared
edge must remain in canonical (start → end) order after UV projection. If a sequence is
reversed (e.g. to "match" a face's local winding), segment `i` on face A will be paired
with segment `N-i` on face B — completely wrong pairings. The fix is to never reverse;
let the geometry handle orientation naturally.

**Not calling `recordSplit` immediately**: if `recordSplit` is deferred (e.g. batched for
later), sub-splits of the same segment within the same refiner pass will not find their
twins. Always call `recordSplit` inside the callback before returning.

**Not calling `updateEdgeNodeAfterSplit` on the twin face**: the edge node sequence on the
twin face must be updated whenever a split is applied to it. Without this, the sequence
drifts out of sync with the actual mesh — downstream verification passes will fail, and
further splits may not find the correct neighboring segments.

**Applying splits only to the current face**: a cross-surface split must be applied to the
twin face immediately inside the callback, not after the refiner finishes. Post-refinement
application adds the constraint but leaves large triangles adjacent to the new node — the
refiner will not re-run on that face unless the convergence loop triggers another pass.
