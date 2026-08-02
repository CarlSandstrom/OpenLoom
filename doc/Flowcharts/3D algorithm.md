# 3D Tetrahedral Mesh Generation by RCDT Refinement

Based on the algorithm in "Restricted Constrained Delaunay Triangulations".

## Key Difference from the Shewchuk Approach

Shewchuk's algorithm recovers surface constraints explicitly: a separate facet recovery
phase tracks subsegments and subfacets, inserts circumcenters to recover missing faces,
and uses a strict priority ordering to guarantee termination.

RCDT takes a different route. Surface constraints are never "recovered" — they emerge
implicitly from the tetrahedralization. A face of the 3D Delaunay tetrahedralization is
**restricted** to a CAD surface S if all three of its nodes lie on S and the two adjacent
tetrahedra are on opposite sides of S. Refining the vertex set (by splitting encroached
segments and bad restricted triangles) drives the restricted triangulation to converge to
the input geometry without a dedicated recovery pass.

The refinement loop has three priorities:

1. Split encroached curve segments (highest)
2. Split bad restricted surface triangles
3. Split skinny tetrahedra (lowest)

---

## 1. Input Processing

- **1.1** Receive Piecewise Linear Complex (PLC): vertices, constraining curve segments
  along topology edges, and bounding CAD surfaces.
- **1.2** Discretize edges: sample vertices along each topology edge respecting curved
  CAD geometry via parametric evaluation (`BoundaryDiscretizer3D`).
- **1.3** Discretize surface interiors: sample vertices on each topology surface.
- **1.4** Build initial constraint data structures:
  - **1.4.1** Register one `CurveSegment` per consecutive pair of boundary nodes along
    each topology edge (`CurveSegmentManager`). Each segment records its parametric
    range on the parent edge curve.
  - **1.4.2** No per-surface triangulation is built; surface constraints will emerge from
    the restricted triangulation in step 3.

## 2. Initial Delaunay Tetrahedralization

Create an **unconstrained** Delaunay tetrahedralization of all input vertices. Surface
constraints are not enforced here; they will emerge in step 3.

- **2.1** Create a bounding tetrahedron enclosing all input vertices with a large margin.
- **2.2** Insert each vertex incrementally via Bowyer-Watson:
  - **2.2.1** Find a seed tetrahedron containing the point.
  - **2.2.2** Find all tetrahedra whose circumsphere contains the new point (BFS from seed).
  - **2.2.3** Extract the cavity boundary (faces appearing exactly once among conflicting
    tetrahedra).
  - **2.2.4** Remove all conflicting tetrahedra.
  - **2.2.5** Retriangulate: connect each boundary face to the inserted vertex.
- **2.3** Remove the bounding tetrahedron and all tetrahedra sharing its vertices.

## 3. Build Restricted Triangulation

Classify every face of the tetrahedralization to identify surface constraints:

- **3.1** For each tetrahedral face, resolve each node's geometry IDs to the set of CAD
  surface IDs it touches (direct surface membership, or edge membership → adjacent
  surfaces via `RestrictedTriangulation::effectiveSurfaceIds`).
- **3.2** A face is a candidate for surface S if all three nodes touch S.
- **3.3** Confirm with `SurfaceProjector.crossesSurface`: the opposite vertices of the
  two adjacent tetrahedra must lie on opposite sides of S (opposite sign of signed distance).
- **3.4** Store all confirmed restricted faces in `RestrictedTriangulation`.

After this step the restricted triangulation exists but may be coarse or incomplete where
the initial vertex set is sparse.

## 4. RCDT Quality Refinement

A three-priority loop. Each iteration selects the highest-priority action available.

### 4.1 Priority 1 — Split encroached curve segments

A `CurveSegment` `[a, b]` is encroached if any mesh vertex other than its endpoints lies
inside or on its diametral sphere.

- **4.1.1** Scan `CurveSegmentManager` for encroached segments.
- **4.1.2** For the first encroached segment, compute the split point at the arc-length
  midpoint of the segment's parametric range on the parent edge curve
  (`CurveSegmentOperations::computeSplitPoint`).
- **4.1.3** Pre-compute cavity interior faces before Bowyer-Watson insertion.
- **4.1.4** Insert the split point via Bowyer-Watson. After insertion, remove cavity
  interior faces from `RestrictedTriangulation` and re-classify all new faces adjacent
  to the inserted node.
- **4.1.5** Replace the original segment with two sub-segments in `CurveSegmentManager`.
- **4.1.6** Repeat until no encroached segments remain.

### 4.2 Priority 2 — Split bad restricted surface triangles (only when no encroached segments)

A restricted face is bad if either quality bound in `RCDTQualitySettings` is exceeded
(`maximumCircumradiusToShortestEdgeRatio` or `maximumChordDeviation`), measured in 3D
ambient space against the CAD surface.

- **4.2.1** Call `RestrictedTriangulation::getBadTriangles` to collect failing faces with
  their circumcircle centres.
- **4.2.2** For the worst bad triangle, attempt to insert its circumcircle centre.
- **4.2.3** **Circumcenter demotion**: if inserting the circumcircle centre would
  encroach any curve segment, do not insert it; split all encroached segments instead
  (return to 4.1). This prevents unbounded refinement near curve features.
- **4.2.4** Otherwise, insert the circumcircle centre via Bowyer-Watson and update
  `RestrictedTriangulation`. The bad triangle is eliminated because its circumsphere is
  no longer empty.
- **4.2.5** Repeat until no bad restricted triangles remain.

### 4.3 Priority 3 — Split skinny tetrahedra (only when no encroached segments or bad restricted triangles)

A tetrahedron is skinny if its circumradius-to-shortest-edge ratio exceeds the volume
quality bound.

- **4.3.1** Find the tetrahedron with the worst ratio.
- **4.3.2** Compute its circumcenter.
- **4.3.3** Check cascade:
  - If the circumcenter would encroach a curve segment → do not insert; split the
    segment instead (return to 4.1).
  - If the circumcenter would create a bad restricted triangle → do not insert; handle
    that triangle instead (return to 4.2).
  - Otherwise → insert the circumcenter via Bowyer-Watson and update
    `RestrictedTriangulation`. The skinny tetrahedron is eliminated.
- **4.3.4** Repeat until all tetrahedra satisfy the volume quality bound.

### 4.4 Termination

The strict priority ordering guarantees termination when the quality bounds are not
too aggressive:

- Every segment split reduces maximum segment length; sub-segments cannot be
  immediately re-encroached by their own split point.
- The circumcenter demotion rule (4.2.3, 4.3.3) ensures insertions near curve features
  are always preceded by sufficient segment refinement, breaking the parent-child cycle
  that would otherwise cause non-termination.

## 5. Exterior Removal

- **5.1** Once refinement is complete, the restricted faces partition the tetrahedralization
  into interior and exterior regions.
- **5.2** Flood-fill from tetrahedra adjacent to the bounding tetrahedron's removal region
  to identify exterior tetrahedra (those outside the PLC domain).
- **5.3** Remove all exterior tetrahedra.

## 6. Output

- The **volume mesh** (all remaining tetrahedra) is accessible via `MeshingContext3D`.
- The **surface mesh** (all restricted faces) can be assembled via
  `RestrictedTriangulation::getRestrictedFaces()`.

---

## Key Invariants

| After step | Invariant |
|------------|-----------|
| 2 | Tetrahedralization is Delaunay; all boundary vertices inserted |
| 3 | `RestrictedTriangulation` holds the initial surface triangulation |
| 3 | `CurveSegmentManager` holds one segment per consecutive boundary node pair |
| During 4 | `RestrictedTriangulation` is current: updated after every insertion |
| After 4.1 | No encroached curve segments |
| After 4.2 | No bad restricted triangles |
| After 4.3 | All tetrahedra satisfy the volume quality bound |
| After 5 | Mesh contains only interior tetrahedra |

## Data Structure Summary

| Structure | Purpose | Lifetime |
|-----------|---------|----------|
| `CurveSegmentManager` | Constrained curve segments; updated as segments are split | Created in 1.4.1, mutated during 4.1 |
| `MeshData3D` (via `MeshingContext3D`) | The 3D Delaunay tetrahedralization | Created in step 2, refined throughout |
| `RestrictedTriangulation` | Map from tetrahedral face → surface ID for all restricted faces | Built in step 3, updated after every insertion |

The key architectural difference from Shewchuk: there is no separate subsegment list or
per-surface facet triangulation. The restricted triangulation replaces both. Surface
constraints emerge from the tetrahedralization rather than being explicitly recovered.
