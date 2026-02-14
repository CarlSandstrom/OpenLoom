# 2D Constrained Delaunay Triangulation by Delaunay Refinement

## 1. Input Processing

- **1.1** Receive geometry and topology: edges (straight or curved) and surfaces defining the 2D domain, including holes
- **1.2** Discretize edges: sample vertices along each edge, respecting curved CAD geometry via parametric evaluation
- **1.3** Extract constrained segments from the discretized edges:
  - **1.3.1** Each pair of consecutive vertices along a discretized edge becomes a constrained segment
  - **1.3.2** Classify each segment as BOUNDARY (domain perimeter) or INTERIOR (hole boundary or internal constraint)

## 2. Initial Delaunay Triangulation

- **2.1** Create a super triangle that encloses all input vertices with large margin (bounding box × 100 scale)
- **2.2** Insert each vertex incrementally via Bowyer-Watson:
  - **2.2.1** Find all conflicting triangles: triangles whose circumcircle contains the new point
    - **2.2.1.1** Circumcircle test: compute the circumcenter and circumradius of each triangle; the point conflicts if its distance to the circumcenter is less than the circumradius
    - **2.2.1.2** If constrained segments already exist, conflict detection uses BFS flood fill from a seed triangle containing the point, never crossing constrained edges. This prevents the cavity from spanning across domain boundaries
    - **2.2.1.3** Seed triangle selection: find a conflicting triangle that geometrically contains the point via orientation sign tests (all three signed areas have the same sign)
    - **2.2.1.4** Star-shapedness verification: the BFS-restricted cavity may not be star-shaped w.r.t. the insertion point (constrained edges can clip the conflict region asymmetrically). Iteratively check each cavity boundary edge: compute the signed area of the triangle formed by the insertion point and the boundary edge. If any boundary edge produces a negative signed area (inverted triangle), remove the owning cavity triangle and recheck. This guarantees that Bowyer-Watson retriangulation produces only valid, correctly-oriented triangles
  - **2.2.2** Extract cavity boundary: edges appearing exactly once among conflicting triangles form the boundary; edges appearing twice are interior and are removed
  - **2.2.3** Remove all conflicting triangles
  - **2.2.4** Retriangulate: create new triangles connecting each cavity boundary edge to the inserted vertex. Skip degenerate triangles (area below tolerance)
- **2.3** Remove the super triangle and all triangles sharing its vertices

## 3. Constraint Edge Recovery

This phase ensures every constrained segment from step 1.3 is represented as a mesh edge. Unlike the 3D algorithm (which splits missing segments), the 2D algorithm recovers edges by local retriangulation.

- **3.1** For each constrained segment, check if it exists as a mesh edge
- **3.2** If a segment is missing:
  - **3.2.1** Find all triangles whose edges properly intersect the constraint segment (geometric intersection test using orientation predicates)
  - **3.2.2** Remove these intersecting triangles and extract the cavity boundary polygon
  - **3.2.3** Trace the boundary into a closed polygon using edge adjacency
  - **3.2.4** Split the polygon into two sub-polygons at the constraint segment endpoints (left side and right side of the constraint)
  - **3.2.5** Triangulate each sub-polygon via ear clipping:
    - **3.2.5.1** Ensure counter-clockwise winding
    - **3.2.5.2** Find an "ear": a convex vertex whose triangle contains no other polygon vertices
    - **3.2.5.3** Clip the ear (create triangle, remove vertex from polygon)
    - **3.2.5.4** Repeat until three vertices remain (final triangle)
- **3.3** Repeat from 3.1 until all constrained segments are present as mesh edges

## 4. Interior/Exterior Classification

- **4.1** Find an interior seed triangle via ray casting:
  - **4.1.1** For each triangle, compute its centroid
  - **4.1.2** Cast a horizontal ray in the +X direction from the centroid
  - **4.1.3** Count crossings with BOUNDARY constrained segments only (interior constraints are ignored)
  - **4.1.4** Odd crossing count → point is inside the domain → triangle is a valid seed
- **4.2** Flood fill from the seed triangle via BFS:
  - **4.2.1** Traverse to neighboring triangles across shared edges
  - **4.2.2** Never cross BOUNDARY constrained edges (these form the domain perimeter and hole boundaries)
  - **4.2.3** All reached triangles are classified as interior
- **4.3** Remove all triangles not reached by the flood fill (exterior triangles and triangles inside holes)

## 5. Shewchuk's 2D Delaunay Refinement

A two-priority loop. Each iteration selects the highest-priority action available:

- **5.1** Priority 1 — Split encroached segments:
  - **5.1.1** A constrained segment is encroached if any vertex (other than its endpoints) lies inside or on its diametral circle (the circle with the segment as diameter)
  - **5.1.2** For each encroached segment, compute the parametric midpoint on the parent edge curve: t_mid = (t_start + t_end) / 2
  - **5.1.3** Evaluate the midpoint position on the parent edge geometry (preserves curved boundaries)
  - **5.1.4** Insert the midpoint via Bowyer-Watson (step 2.2, with constraint-aware conflict detection per 2.2.1.2)
  - **5.1.5** Replace the original segment with two new sub-segments in the constraint list
  - **5.1.6** Enforce the two new sub-segments as mesh edges (step 3.2 if needed)
  - **5.1.7** Apply Lawson edge flipping to restore the Delaunay property locally (step 5.3)
  - **5.1.8** Repeat until no encroached segments remain
- **5.2** Priority 2 — Split poor-quality triangles (only when no encroached segments exist):
  - **5.2.1** Quality criteria — a triangle is acceptable if both conditions hold:
    - **5.2.1.1** Circumradius-to-shortest-edge ratio ≤ bound B
    - **5.2.1.2** Minimum interior angle ≥ threshold θ
  - **5.2.2** Select the triangle with the worst quality (largest ratio) among all failing triangles
  - **5.2.3** Compute its circumcenter
  - **5.2.4** Safety checks — skip the triangle if:
    - **5.2.4.1** The triangle is too small (area or shortest edge below tolerance)
    - **5.2.4.2** The circumcenter falls outside the domain or inside a hole
    - **5.2.4.3** The circumcenter coincides with an existing mesh node (distance below edge length tolerance). This occurs when multiple bad triangles near a constraint edge share the same circumcenter location; Bowyer-Watson cannot insert a point at an existing vertex because the circumcircle containment test degenerates (the point lies ON the circumcircle rather than inside it)
  - **5.2.5** Encroachment check: if the circumcenter would encroach any constrained segment, do not insert it; split the encroached segment instead (return to 5.1)
  - **5.2.6** Otherwise, insert the circumcenter via Bowyer-Watson (step 2.2). The bad triangle is eliminated because its circumcircle is no longer empty
  - **5.2.7** If the original bad triangle survives insertion (circumcenter landed on the wrong side of a constraint), mark it as unrefinable and skip it in future iterations
  - **5.2.8** Repeat until all triangles satisfy the quality criteria or only unrefinable triangles remain
- **5.3** Lawson edge flipping (used after segment splitting in 5.1.7):
  - **5.3.1** Collect all non-constrained edges from newly created triangles
  - **5.3.2** For each edge shared by two triangles forming a quadrilateral:
    - **5.3.2.1** Check if the quadrilateral is convex (orientation test on the opposite vertices)
    - **5.3.2.2** Check the Delaunay criterion: is the opposite vertex inside the circumcircle of the adjacent triangle?
    - **5.3.2.3** If both conditions hold, flip the diagonal: replace the two triangles with two new triangles using the other diagonal
    - **5.3.2.4** Push the four outer edges of the flipped quadrilateral onto the stack (unless constrained)
  - **5.3.3** Repeat until the stack is empty (all local edges satisfy the Delaunay property)
- **5.4** Termination: the refinement loop ends when no encroached segments exist and all triangles are acceptable. A safety limit of 10,000 iterations prevents infinite loops. Early exit after 10 consecutive iterations with no progress

## 6. Post-Processing

- **6.1** Final flood fill: remove any triangles that migrated into hole regions during refinement (re-apply step 4.2)

## Key Invariants

- After step 2: the triangulation satisfies the Delaunay property (empty circumcircle) but does not respect constraints
- After step 3: every constrained segment from step 1.3 is present as a mesh edge
- After step 4: the mesh contains only interior triangles; exterior and hole regions are removed
- During step 5: the strict priority ordering (encroached segments before bad triangles) ensures that segment splitting resolves encroachment before quality refinement proceeds. Circumcenters of bad triangles that would violate constraints are redirected to segment splits, preventing constraint violations
- The constraint-aware BFS in conflict detection (step 2.2.1.2) ensures that Bowyer-Watson insertion never creates triangles spanning across constrained edges. The star-shapedness verification (step 2.2.1.4) additionally guarantees that the restricted cavity produces only valid triangles, even when constrained edges clip the conflict region into a non-convex shape
