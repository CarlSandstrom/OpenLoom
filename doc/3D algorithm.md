# 3D Tetrahedral Mesh Generation by Delaunay Refinement

## 1. Input Processing

- **1.1** Receive Piecewise Linear Complex (PLC): vertices, constraining segments, and planar constraining facets from CAD geometry and topology
- **1.2** Discretize edges: sample vertices along edges respecting curved CAD geometry via parametric evaluation
- **1.3** Discretize surfaces: sample vertices on facets
- **1.4** Extract constraint lists:
  - **1.4.1** Segments → initial subsegment list (each segment starts as one subsegment)
  - **1.4.2** For each facet, create an independent 2D Delaunay triangulation of its vertices → initial subfacet list. These facet triangulations are maintained separately from the tetrahedralization throughout the entire algorithm

## 2. Initial Delaunay Tetrahedralization

- **2.1** Create a bounding tetrahedron that encloses all input vertices with large margin
- **2.2** Insert each vertex incrementally via 3D Bowyer-Watson:
  - **2.2.1** Find a seed tetrahedron containing the point
  - **2.2.2** Circumsphere test: find all tetrahedra whose circumsphere contains the new point (BFS flood fill from seed; no constraint-awareness at this stage since constraints are not yet recovered)
  - **2.2.3** Extract cavity boundary: triangular faces appearing exactly once among conflicting tetrahedra form the boundary; faces appearing twice are interior and are removed
  - **2.2.4** Remove all conflicting tetrahedra
  - **2.2.5** Retriangulate: create new tetrahedra connecting each boundary face to the inserted vertex
- **2.3** Remove the bounding tetrahedron and all tetrahedra sharing its vertices

## 3. Segment Recovery (Priority 1 — "Stitching")

This phase ensures every input segment is represented as a contiguous chain of mesh edges.

- **3.1** For each subsegment, check if it exists as a mesh edge
- **3.2** If a subsegment is missing from the mesh, it is necessarily encroached (by the Delaunay property: the tetrahedralization always connects a vertex to its nearest neighbor)
- **3.3** Split the missing subsegment at its midpoint:
  - **3.3.1** Compute the midpoint (on the parent edge's parametric curve for curved geometry)
  - **3.3.2** Insert the midpoint via Bowyer-Watson (step 2.2)
  - **3.3.3** Replace the original subsegment with two new subsegments in the constraint list
  - **3.3.4** Also insert the midpoint into every facet triangulation (step 1.4.2) that contains this subsegment
- **3.4** Repeat from 3.1 for each resulting subsegment until all subsegments are present as mesh edges. Convergence is guaranteed: once vertex spacing along a segment is small enough, the Delaunay property forces the edges to appear

## 4. Facet Recovery (Priority 2)

This phase ensures every input facet is represented as a union of triangular mesh faces. Facets are recovered after all segments (step 3), so facet boundaries are already present in the mesh.

- **4.1** For each facet, compare its independent 2D Delaunay triangulation (step 1.4.2) against the faces of the tetrahedralization. Identify subfacets that are missing from the tetrahedralization
- **4.2** For each missing subfacet f of facet F, apply the Projection Lemma:
  - **4.2.1** If a vertex p encroaches f but the projection proj_F(p) lies outside F, then p also encroaches a boundary subsegment of F → split that subsegment instead (return to step 3.3). The Projection Lemma guarantees the encroachment on f is resolved as a side effect
  - **4.2.2** If proj_F(p) lies inside F, then split the subfacet g of F that contains proj_F(p) first (not f). This ordering yields a tighter quality bound
- **4.3** Split the selected subfacet at its circumcenter:
  - **4.3.1** Compute the circumcenter of the subfacet triangle (in the facet plane)
  - **4.3.2** Check if the circumcenter would encroach any subsegment. If yes, split all encroached subsegments instead (return to step 3.3)
  - **4.3.3** Otherwise, insert the circumcenter into both the facet's 2D triangulation and the 3D tetrahedralization via Bowyer-Watson (step 2.2)
- **4.4** Repeat from 4.1 until no subfacets are missing. Handle cocircularity degeneracies: if a facet's Delaunay triangulation is non-unique, correct it to match the tetrahedralization

## 5. Exterior Removal

- **5.1** Once all segments and facets are recovered (steps 3-4 complete), every facet is a union of mesh faces. These faces partition the tetrahedralization into interior and exterior regions
- **5.2** Identify exterior tetrahedra: those lying inside the convex hull of the input vertices but outside the facet-bounded PLC domain
- **5.3** Remove all exterior tetrahedra before any quality refinement. This prevents overrefinement from spurious small angles formed between the PLC boundary and the convex hull

## 6. Shewchuk's 3D Delaunay Refinement

A three-priority loop. Each iteration selects the highest-priority action available:

- **6.1** Priority 1 — Split encroached subsegments:
  - **6.1.1** A subsegment is encroached if any vertex (other than its endpoints) lies inside or on its diametral sphere (the smallest sphere enclosing the subsegment; centered at its midpoint, radius = half its length)
  - **6.1.2** Split at the midpoint: insert via Bowyer-Watson (step 2.2), replace with two new subsegments, update all affected facet triangulations
  - **6.1.3** The new subsegments may or may not be encroached; if they are, they will be caught in the next iteration
  - **6.1.4** Repeat until no encroached subsegments remain
- **6.2** Priority 2 — Split encroached subfacets (only when no encroached subsegments exist):
  - **6.2.1** A subfacet is encroached if any non-coplanar vertex lies inside or on its equatorial sphere (the smallest sphere passing through the subfacet's three vertices; centered in the facet plane)
  - **6.2.2** Apply the Projection Lemma ordering (step 4.2) to choose which subfacet to split first
  - **6.2.3** Compute the circumcenter of the subfacet. If it would encroach any subsegment, do not insert it; split those subsegments instead (return to 6.1)
  - **6.2.4** Otherwise, insert the circumcenter into both the facet triangulation and the tetrahedralization
  - **6.2.5** Repeat until no encroached subfacets remain
- **6.3** Priority 3 — Split skinny tetrahedra (only when no encroached subsegments or subfacets exist):
  - **6.3.1** A tetrahedron is skinny if its circumradius-to-shortest-edge ratio exceeds bound B (B > 2 guarantees termination; in practice B ~ 1.2 often works)
  - **6.3.2** Select the tetrahedron with the worst (largest) ratio
  - **6.3.3** Compute its circumcenter
  - **6.3.4** Check encroachment:
    - **6.3.4.1** If the circumcenter would encroach any subsegment → do not insert; split all encroached subsegments instead (return to 6.1)
    - **6.3.4.2** If the circumcenter would encroach any subfacet (but no subsegment) → do not insert; split all encroached subfacets instead (return to 6.2)
    - **6.3.4.3** Otherwise → insert the circumcenter via Bowyer-Watson (step 2.2). The skinny tetrahedron is eliminated because its circumsphere is no longer empty
  - **6.3.5** Repeat until all tetrahedra satisfy the quality bound
- **6.4** Termination is guaranteed when B > 2 and the PLC satisfies the projection condition (incident segments and facets separated by at least 90 degrees). The proof follows from the dataflow graph: vertex insertion radii cannot diminish without bound because no cycle of parent-child relationships has a product less than one

## 7. Post-Processing

- **7.1** Sliver removal (heuristic, no theoretical guarantee):
  - **7.1.1** Slivers are tetrahedra with acceptable circumradius-to-edge ratio but near-zero volume and extreme dihedral angles (near 0 or 180 degrees). They arise from near-coplanar vertex configurations
  - **7.1.2** Target slivers by treating tetrahedra with dihedral angles below a threshold (e.g. 19-21 degrees) as skinny, and split at their circumcenters using the same mechanism as step 6.3. In practice this eliminates most slivers
  - **7.1.3** Prioritize by circumradius-to-edge ratio (not dihedral angle) so that slivers are addressed last, reducing tet count and improving termination likelihood
- **7.2** Mesh smoothing and optimization (optional):
  - **7.2.1** Optimization-based vertex smoothing to improve worst dihedral angles
  - **7.2.2** Topological transforms (2-3 and 3-2 flips) to improve local connectivity
  - **7.2.3** These techniques can remove boundary slivers that Delaunay refinement alone cannot handle

## Key Invariants

- After step 3: every input segment is a chain of mesh edges
- After step 4: every input facet is a union of mesh faces
- After step 5: mesh contains only interior tetrahedra
- During step 6: the strict priority ordering (subsegments > subfacets > skinny tets) guarantees that subfacet circumcenters lie in their containing facet (when no subsegments are encroached), and tetrahedron circumcenters lie in the mesh (when no subfacets are encroached)
- The Projection Lemma (steps 4.2, 6.2.2) prevents cross-feature cascade: encroachment caused by a vertex near one facet never triggers unbounded refinement on an incident facet
