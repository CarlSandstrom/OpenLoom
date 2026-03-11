# 3D Surface Mesh Generation

## Key Difference from Volume Meshing

The surface mesher triangulates each CAD face independently in its own UV parameter space — no 3D tetrahedralization is involved. Each face gets a 2D constrained Delaunay triangulation that is refined to meet quality criteria. The resulting triangles are mapped back to 3D coordinates only at export time. Shared edges between adjacent faces are kept consistent by synchronizing boundary splits across the two surface triangulations.

## S1. Input Processing

- **S1.1** Receive geometry (`GeometryCollection3D`) and topology (`Topology3D`): corners, edges, and surfaces from CAD
- **S1.2** Build the twin table (`TwinTableGenerator::generate`):
  - For each topology edge adjacent to exactly 2 surfaces (a shared interior edge), record a twin entry:
    - The first adjacent surface takes `Same` orientation (point sequence runs start → end)
    - The second adjacent surface takes `Reversed` orientation (point sequence runs end → start)
  - Boundary edges (one adjacent surface) have no twin — they are not entered in the table
  - The twin table drives `TwinManager` construction in S1.3 and boundary-split synchronization in S2
- **S1.3** Discretize boundaries (`BoundaryDiscretizer3D::discretize`):
  - **S1.3.1** Sample corner points: one `Point3D` per topology corner, stored in `result_.points`; record `cornerId → pointIndex` in `cornerIdToPointIndexMap`
  - **S1.3.2** Sample edge interior points (non-seam-twin edges only):
    - Angle-based mode: walk 1000 uniform parameter steps; insert a point whenever the accumulated tangent-angle change since the last inserted point reaches `maxAngle`. Straight edges produce no interior points
    - Fixed-count mode: divide the edge into `numSegments` equal parameter intervals; insert `numSegments - 1` interior points
    - Corner endpoints are annotated with the edge's `tMin`/`tMax` parameters and edge ID; interior points carry their parameter value and edge ID
    - The full ordered point index sequence (start corner, interiors, end corner) is stored in `edgeIdToPointIndicesMap`
  - **S1.3.3** Populate seam twin edge sequences: a seam twin edge has no 3D geometry but shares the same 3D points as its original seam edge. Copy the original edge's point index sequence and reverse it, then store under the twin edge ID in `edgeIdToPointIndicesMap`
  - **S1.3.4** Build the `TwinManager` from the twin table: for each shared edge, walk its segment pairs and call `registerTwin(s0From, s0To, s1From, s1To)` with orientations corrected per the twin table entry. This creates a lookup from any segment on one surface to the corresponding segment on the adjacent surface
- **S1.4** Initialize per-surface facet triangulations (`FacetTriangulationManager::createForSurfaceMesher` → `initializeForSurfaceMesher`):
  - For each topology surface:
    - **S1.4.1** Collect boundary point indices: gather all global point indices for the surface's corners and non-seam-twin boundary edges; exclude seam twin edges (they share 3D points with the original seam and are handled by UV-shifted copies below)
    - **S1.4.2** Build the UV-space discretization (`buildFaceDiscretization2D`):
      - Project each collected 3D point onto the surface UV plane via `surface.projectPoint()`
      - Normalize each edge's raw parameter values to `[0, 1]` (relative position along the edge)
      - Copy `cornerId → localIndex` mapping (translating global indices to local)
      - **Phase 1 — Seam twin edges first**: for each seam twin edge, create a UV-shifted copy of every point along that edge by adding `uPeriod = uMax - uMin` to its U coordinate. These shifted copies represent the same 3D positions but at the far side of the periodic boundary. Record `globalPointIndex → shiftedLocalIndex` in `realCornerToShiftedLocal`; replace the original seam edge ID with the twin edge ID in the shifted copies' geometry metadata
      - **Phase 2 — Non-seam edges**: translate each edge's global point sequence to local indices. For a closed-circle edge (start and end share the same global index, e.g. the top or bottom circle of a cylinder), replace the final local index with the seam-shifted copy from Phase 1 so the edge closes at `U + uPeriod` instead of doubling back to `U`
    - **S1.4.3** Run constrained Delaunay 2D (`ConstrainedDelaunay2D::triangulate`): register boundary segments, insert all vertices, enforce constraints, and remove exterior triangles — identical to the 2D algorithm (see `2D algorithm.md`)
    - **S1.4.4** Build bidirectional node ID mappings: for each local point index, record `node3DId ↔ node2DId` so that 2D triangle node IDs can be translated to 3D IDs at export time

## S2. Surface Quality Refinement (`refineSurfaces`)

For each topology surface (independently):

- **S2.1** Create a `Shewchuk2DQualityController` with the caller-supplied quality bounds (circumradius-to-edge ratio, minimum angle)
- **S2.2** Create a `ShewchukRefiner2D` over the surface's `MeshingContext2D`
- **S2.3** Register a `BoundarySplitSynchronizer` as the refiner's `onBoundarySplit` callback:
  - Whenever the 2D refiner splits a boundary segment `(n1, n2)` at a new midpoint node `mid`:
    - **S2.3.1** Look up whether `(n1, n2)` has a twin in `TwinManager`
    - **S2.3.2** If a twin `(t1, t2)` exists, find that constrained segment in the current surface's 2D mesh data (searching both endpoint orderings)
    - **S2.3.3** Determine the common edge geometry (`findCommonGeometryId`) and split the twin segment via `splitConstrainedSegment`, inserting a new midpoint on the twin side at the same parametric position
    - **S2.3.4** Record the split in `TwinManager` (`recordSplit`) so that future refinement splits on either side can find updated twin pairs
- **S2.4** Run `ShewchukRefiner2D::refine()` — the same two-priority loop as the 2D algorithm (see `2D algorithm.md`, step 5): split encroached boundary segments first, then split poor-quality triangles by inserting circumcenters
- **S2.5** After all surfaces are refined, resolve refinement nodes (`resolveRefinementNodes`):
  - Walk every 2D node in the surface's mesh; any node not yet in `node2DTo3DMap` was inserted during refinement and has no 3D peer yet
  - Evaluate its 3D position via `surface.getPoint(u, v)`
  - Assign the next sequential 3D node ID (`nextNode3DId++`) and update both mappings
  - Collect the 3D points into `refinementNodes_`

## S3. Mesh Export (`getSurfaceMesh3D`)

- **S3.1** Create a `MeshData3D` and a `MeshMutator3D`
- **S3.2** Add all discretization points as 3D nodes with IDs `0 .. N-1` (in the order they appear in `discretizationResult_.points`)
- **S3.3** Add all refinement nodes with IDs `N .. M-1` (in the order they were resolved in S2.5)
- **S3.4** Collect all subfacets from all facet triangulations:
  - For each surface, iterate over every triangle in the 2D mesh
  - Translate its three 2D node IDs to 3D node IDs via `node2DTo3DMap`
  - Emit a `TriangleElement` with those three 3D node IDs
- **S3.5** Return the assembled `MeshData3D`

## Key Invariants

- After S1.3: every topology corner and edge is represented as a sequence of 3D points; shared edges between surfaces have consistent point sequences (same 3D points, opposing orientation)
- After S1.4: each surface has an independent 2D constrained Delaunay triangulation whose boundary matches the discretized edges; the `node3DId ↔ node2DId` mappings cover all pre-refinement nodes
- During S2: boundary-split synchronization guarantees that adjacent surface triangulations stay consistent — a split on one side immediately produces a matching split on the other, preventing T-junction artifacts at shared edges
- After S2: all surfaces satisfy the quality criteria; every 2D refinement node has a resolved 3D position in `refinementNodes_`
- After S3: the output `MeshData3D` contains only triangle elements; node IDs are globally unique across all surfaces; the mesh is a watertight (gap-free) triangulation of all CAD faces

## Data Structure Summary

| Structure | Purpose | Lifetime |
|-----------|---------|----------|
| `EdgeTwinTable` | Maps shared edge ID → two adjacent surfaces with orientations | Built in S1.2, consumed by S1.3.4 |
| `DiscretizationResult3D` | All sampled 3D points plus corner/edge/surface index maps | Built in S1.3, read throughout |
| `TwinManager` | Bidirectional lookup: segment on surface A ↔ corresponding segment on surface B; updated as splits occur | Built in S1.3.4, updated during S2 |
| `FacetTriangulation` per surface | Owns a `MeshingContext2D` (2D Delaunay mesh + geometry + topology); maps `node3DId ↔ node2DId` | Created in S1.4, refined in S2 |
| `refinementNodes_` | 3D positions for nodes inserted during refinement (no pre-existing 3D peer) | Populated at end of S2, consumed in S3 |

## Surface-Mesher vs Volume-Mesher Differences

| Aspect | Surface mesher | Volume mesher |
|--------|---------------|---------------|
| 3D tetrahedralization | None | Full Bowyer-Watson tetrahedralization |
| Initial face points | Boundary points only (corners + edges) | Boundary + interior surface samples |
| `FacetTriangulationManager` path | `createForSurfaceMesher` | `createForVolumeMesher` |
| `pointIndex → node3DId` mapping | Identity (index == ID) | Via `pointIndexToNodeIdMap` from tetrahedralization |
| Refinement node 3D evaluation | Deferred to `resolveRefinementNodes` after all surfaces | Circumcenter inserted into both the 2D mesh and the 3D tetrahedralization simultaneously |
| Output | `MeshData3D` with triangle elements only | `MeshData3D` with tetrahedral elements (surface triangles are implicit) |
