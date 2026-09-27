# Meshing Techniques Inventory

**Last updated:** 2026-09-27

Every technique the mesh generator relies on, grouped by the job it does, with
the class that implements it and the reason it is there. Written as a reading
aid: the *why* of each piece, not its API. Paths are relative to `src/Meshing/`.

Two things are worth holding onto while reading:

- The **restriction oracle** (section 4) is the only part currently known to be
  running outside its stated preconditions. Its acceptance hedges — the
  protected-edge shortcut, the phase test, the periodic-surface skip — are
  compensation for that, not part of the textbook algorithm.
- Everything in **section 6** is post-hoc repair of what section 4 got wrong.

---

## 1. Numerical foundation

### Exact geometric predicates
`Core/3D/General/RobustPredicates3D`, `Core/3D/General/ExactArithmetic3D`

Shewchuk-style adaptive expansion arithmetic: a plain-double fast path with a
conservative error bound, falling back to exact expansions only when the sign
is not comfortably determined. Three predicates:

- `insidePointCircumsphere` — evaluates the 4x4 in-sphere determinant directly
  and never forms an explicit circumcenter or radius, so a near-flat tetrahedron
  cannot corrupt it through a nearly-singular linear solve.
- `orientationSign` — sign of the signed volume.
- `segmentCrossesTriangle` — built entirely from `orientationSign`.

Exactness, not merely extra precision, is the requirement. This codebase's
discretization of circles (cylinder caps, sphere and torus seams) produces
points that are *algebraically* exactly cospherical. A first attempt using
double-double (~32 digits) resolved those ties inconsistently — ~23% disagreement
in a targeted fuzz test — because no fixed-width approximation is guaranteed to
compute zero for a quantity that is genuinely zero.

### Weighted (regular) predicate
`Core/3D/General/RegularPredicates3D`

The in-orthosphere / power-distance test: the same determinant with each row's
quadratic term reduced by that point's weight. Equivalent to lifting to the 4D
paraboloid at `(x, y, z, x²+y²+z²−w)`. Reduces term for term to the unweighted
test when every weight is zero. Needed because protecting balls (section 3) make
the curve-network points weighted. Orientation is unaffected by weights, so
callers reuse `RobustPredicates3D::orientationSign` directly.

### Bowyer-Watson incremental insertion
`Core/3D/General/MeshOperations3D`, `Core/3D/Volume/Delaunay3D`, `Core/2D/MeshOperations2D`

Locate the seed element containing the point, BFS flood-fill the conflict set
(elements whose circumsphere/orthosphere contains it), extract the cavity
boundary (faces appearing in exactly one conflicting element), remove the
conflicting elements, connect the new point to each cavity boundary face.

### The bounding super-tetrahedron is deliberately kept
`Core/3D/Volume/Delaunay3D`

It stays in the mesh through all of refinement rather than being removed after
the initial build, for two reasons: every convex-hull face keeps a neighbour to
fall back on when a newly inserted vertex turns out coplanar with it, and every
restricted face keeps tetrahedra on *both* sides, so the dual Voronoi edge the
restriction test needs is always defined. It is removed at the end, by
`AmbientTetrahedronRemover`.

---

## 2. Deciding where points go before meshing

### Boundary discretization
`Core/3D/General/BoundaryDiscretizer3D`

Samples corners and curves only — **no surface-interior points are pre-seeded**.
Surface interiors are populated entirely by the refinement loop. The default
criterion is a tangent-angle bound, which is scale-free: it bounds how far a
segment may *turn*, never how long it may be. On the saddle's parabolic top arc
that permits chords of 3.79 next to chords of 0.20 on one curve.

### Sizing field h(x)  *(opt-in, largely unwired — see OPE-181)*
`Core/3D/General/SizingField3D`, `Core/3D/General/SizingFieldBuilder3D`

    h(x) = min over i of ( h_i + g * |x - x_i| )

over point sources `(x_i, h_i)` with gradient limit `g`. That closed form does
not approximate a gradient-limited field, it *is* one: each term is
g-Lipschitz, a min of g-Lipschitz functions is g-Lipschitz, and among all such
functions bounded by the sources it is the largest. So no background grid, no
fast-marching or relaxation sweep, and no resolution parameter.

Sources combine the two independent reasons elements must be small:

- **Curvature**, inverted through the chord tolerance: a chord of length `h`
  across radius `1/κ` stands off by about `h²κ/8`, so `h = sqrt(8·tol/κ)`.
- **Local feature size**, divided by how many elements should span the gap.

The smaller wins at each sample. Distances are ambient, not geodesic — two
sheets far apart across the surface but close through space limit each other,
which is conservative and, for feature size, exactly right. A *missing* source
is harmless (neighbours still cover it, less tightly); a *wrong, too-small*
source drags down a whole neighbourhood, so unevaluable geometry (a parametric
pole, a cone apex, a degenerate patch) emits nothing rather than a guess.

When supplied, it also bounds discretization segment length: a point is emitted
as soon as *either* the tangent has turned by the angle bound *or* accumulated
arclength reaches `h(x)`.

### Local feature size
`Core/3D/General/LocalFeatureSize3D`

How far each sample is from the nearest *unrelated* part of the boundary.
Curvature says how fast geometry bends; feature size says how close other
geometry is. A flat plate 0.2 units thick has zero curvature everywhere and
still cannot take 1-unit elements — triangles would bridge across it.

"Unrelated" is not a topological adjacency test, which fails on precisely the
case the measure exists for: the two faces of a thin fin *are* adjacent (they
share the tip edge) yet need small elements in the middle. The discriminator is
instead: two samples constrain each other when the route between them along the
geometry is much longer than the straight line, so the straight line is a
genuine shortcut through space. A right-angle junction has route `2r` against
separation `sqrt(2)·r` and is correctly ignored; a fin measured at depth `L` has
route about `2L` against the fin thickness and passes easily.

### Minimum edge length
`Core/3D/RCDT/MinimumEdgeLengthEstimator`

Median nearest-neighbour spacing divided by 10 — median rather than minimum
because a periodic curve's discretization leaves an unrepresentative short
"remainder" segment near its seam vertex.

This one number is the refinement size floor, the floor
`CurveProtectionSubdivider` subdivides against, and the cell size the
tessellation oracle is scaled by. It is a **sliver guard, not a size target**:
refinement must stay free to reach the size actually being asked for, so the
floor has to sit well below it. Setting it *at* the target disables refinement
wherever the mesh has arrived and collapses the torus to degenerate triangles.

Deriving it from a sizing-field-driven discretization is circular (finer
boundary lowers the floor, which lets refinement chase deeper and rebuilds the
oracle finer at the same time), so the sizing-field variant reads a percentile
of the field's own sources instead.

---

## 3. Protecting the creases

### Protecting balls / weighted points  *(OPE-176)*
`Core/3D/RCDT/CurveProtectionScheme`

Boissonnat-Oudot curve protection. Every curve and corner sample is inserted as
a **weighted** point, its weight the squared protecting-ball radius, making the
triangulation a regular rather than an ordinary Delaunay one. Every radius
satisfies two properties:

1. **Chain connectivity** — consecutive balls along the *same* curve overlap,
   regardless of how non-uniform that curve's sampling is. A radius is based on
   the *longer* of a point's two adjacent segments, never the shorter, so both
   endpoints of any interior segment are bounded below by a fixed fraction of
   that segment's own length. A segment with a corner at one end compensates the
   interior endpoint against the corner's smaller radius to force the same
   overlap in one jump.
2. **Disjointness** — balls of unrelated features (a different curve, a
   non-adjacent corner, a non-consecutive point on the same curve) never
   overlap, and no ball swallows a point outside the curve network at all.

The effect: every crease appears as an exact edge chain in the resulting regular
triangulation. The classification ambiguity is designed out structurally rather
than repaired after the fact.

### Protection subdivider
`Core/3D/RCDT/CurveProtectionSubdivider`

When no sizing of the *existing* points can satisfy both properties — a genuine
local-feature-size conflict, confirmed on the torus's periodic seam — the
standard answer is to insert more points into the gap rather than resize two
points to do an impossible job. Repeatedly recompute weights, find the first
unresolved segment, split it at its arc-length midpoint on the true curve, and
repeat, until nothing is unresolved or the next split would fall below
`minimumEdgeLength`. A genuinely unfixable defect is left as a logged permanent
gap rather than chased forever.

### Ball exclusion at insertion
`Core/3D/RCDT/RCDTPointInserter`

No refinement point is ever inserted strictly inside a positive-weight ball, and
this is not merely a quality rule: a point hidden inside a ball finds no
conflicting tetrahedra, so Bowyer-Watson adds the node **unconnected** to the
mesh. The check applies even to split points that lie exactly on their own
curve, because the ball they fall inside may be an unrelated one.

### Curve segments and diametral-sphere encroachment
`Data/CurveSegmentManager`

A `CurveSegment [a, b]` is a straight mesh edge between consecutive samples
along one CAD curve. A vertex encroaches it if it lies inside the sphere with
diameter `|b − a|`. Endpoints are excluded explicitly (`excludeNodeId`): an
endpoint lies exactly *on* that sphere, and rounding can report it as inside,
which would trigger an unbounded self-encroachment cascade.

Splits place the new point at the **arc-length midpoint on the true CAD curve**,
not at the chord midpoint, so refinement converges onto the geometry rather than
onto its current polyline approximation.

---

## 4. Extracting the boundary from the volume — the restriction machinery

Surface constraints are never explicitly recovered. They *emerge*: the set of
restricted faces for a surface is its triangulation, automatically conforming
and automatically shared with adjacent surfaces, because all of them are faces
of one shared tetrahedralization.

### The dual-edge restriction test
`Core/3D/RCDT/DualEdgeRestrictionOracle`

The textbook definition: a face is dual to the segment joining the circumcenters
of the two tetrahedra sharing it, and the restricted Delaunay triangulation of a
surface is the set of faces whose dual edge crosses it. Under a dense enough
sample that set is homeomorphic to the surface — a provably correct boundary
extracted from a volume triangulation.

**Known to run outside its validity conditions (OPE-186).** The dual edge is
only meaningful for well-shaped tetrahedra, and the surface path never produces
them: `meshSurface()` runs with tet-quality refinement off by design and 89.8%
of tetrahedra exceed the bad-tet threshold. The test is therefore evaluated
outside its preconditions on essentially every face, permanently. That is what
the acceptance paths below compensate for, and why each is hedged with a
uniqueness requirement rather than trusted on its own. It is also why enabling
tet-quality refinement cuts the defect count by 77% without fixing anything: a
radius-edge bound provably cannot eliminate slivers in 3D, and this test needs
only one bad tetrahedron in the wrong place.

### Candidate surfaces
`Core/3D/RCDT/SurfaceCandidates`

A node's geometry id names whichever CAD entity it was sampled from, which is
often not a surface — a crease node carries a curve id, a junction node a corner
id. Each resolves to the surfaces it belongs to (a curve to the surfaces it
bounds, a corner to the surfaces meeting there), and intersecting across the
face's three nodes narrows it to the surfaces all three could lie on. An empty
intersection means the face is interior and nothing has been missed.

### Surface tessellation as an exact oracle
`Core/3D/RCDT/SurfaceTessellation`

One CAD surface's trimmed patch sampled onto a jittered UV grid and emitted as
triangles. This converts "does this segment cross a curved surface" — inherently
a floating-point near-tangent distance comparison, which misclassifies segments
whose endpoints are only fractions of a unit off the surface — into "does it
cross any triangle in a fixed tessellation", a purely point-based predicate that
is exact at any degeneracy.

Built entirely from the `ISurface3D` interface every backend already implements,
so it works identically for a plane and a NURBS patch and does not tie RCDT to
OpenCASCADE. Built once per surface at a resolution derived from
`minimumEdgeLength` and never rebuilt during refinement. Cells are not clipped
to the exact trim curve, so the tessellation may overshoot by up to one cell —
harmless, because callers separately check trimmed-boundary membership; this
only needs to have no gaps.

### Triangle soup index
`Core/3D/RCDT/TriangleSoupIndex`

A uniform 3D grid over an unstructured set of triangles, answering the exact
segment-crossing query. Each triangle is registered in every cell its bounding
box overlaps; a query visits only the cells the segment's own bounding box
overlaps and runs the expensive exact predicate only on candidates surviving
that cheap prefilter. For the short dual edges that dominate refinement that is
a handful of triangles instead of the whole soup. Knows nothing about surfaces.

### The acceptance paths in `classify()`  *(the fragile part)*

In order:

- **Convex-hull face** (one adjacent tetrahedron): the dual edge is a
  half-infinite ray from the single circumcenter outward, which necessarily
  crosses the surface when that circumcenter is on the interior side. Candidate
  membership alone suffices.
- **Trimmed-boundary membership**: all three vertices must lie within the
  surface's true trimmed boundary, since the tessellation overshoots.
- **Protected-edge shortcut**: a face with two chain-adjacent nodes carries an
  edge that overlapping protecting balls certify belongs to exactly one crease.
  But that certifies the *edge*, not this *face* — an edge is shared by the whole
  ring of tetrahedra around it. Trusting any single-candidate face touching a
  protected edge was tried and reverted (it accepted spurious faces from
  elsewhere in the ring). The shortcut therefore requires this face to be the
  **unique** candidate across the entire edge star.
- **Phase-boundary test**: accept if the two adjacent tetrahedra's centroids
  fall on different sides of the model — one inside a volume, one outside, or
  inside two different volumes — resolved by the CAD solid classifier through
  `PointPhase`. Gated on a uniqueness rule of its own, and **skipped entirely for
  periodic (seam) surfaces**: unlike an ordinary crease, a seam surface shows a
  small but persistent stream of misclassifications through this path that keeps
  refinement from ever reaching a fixed point. Root cause unknown; this is a
  scoped safety net, not a fix.
- **The dual-edge test itself**, against the tessellation.

### Three-valued classification
`Core/3D/RCDT/RestrictedFaceTypes`

`NotRestricted` / `Restricted` / `Unconfirmed`. "No surface shares all three
nodes" and "candidates existed but none could be confirmed" are different
outcomes with opposite consequences — the first is correct, the second is a hole
in the surface mesh — and a single `optional<surfaceId>` spelled them the same
way. `Unconfirmed` is where residual holes come from and is the classification's
own error signal, but it is an **upper bound** on real defects, not a count:
plenty of genuinely interior faces have three nodes sharing a surface.

### Point phase
`Core/3D/RCDT/PointPhase`

Resolves a point against every volume: `Exterior`, `InVolume`, or `Ambiguous`.
`Ambiguous` is a refusal, not a third side — reached when the CAD kernel reports
the point as on a boundary within its classification tolerance band
(`max(Precision::Confusion(), 1e-4 * diameter)`, about 1e-3 on a unit model), or
cannot decide, or when the point classifies inside two volumes at once. It is
load-bearing and must not be collapsed into a guess: the band is real geometry
at that scale, and confidently-wrong answers were measured to be more damaging
than honest refusals. Shared between the oracle and the diagnostics so a
diagnostic cannot quietly drift from what the mesher does.

### Memoization
`Core/3D/RCDT/DualEdgeRestrictionOracle`

Centroid phase is cached per tetrahedron — `classifyPointPhase` was 85% of total
runtime on the saddle, each call running `BRepClass3d_SolidClassifier::Perform`,
which on Bezier faces drops into OCC's global-optimisation machinery. The
redundancy is structural: one tetrahedron's centroid is asked for about four
times per pass, and reclassification after an insertion pulls in old,
unmodified tetrahedra on the far side of each new face.

The cache is keyed by the tetrahedron's **node set**, not its element id. An id
is only a handle and says nothing about which tetrahedron it currently names, so
a cache keyed by one needs an invalidation hook; the node set is what the
centroid is a function of, so an entry cannot go stale. Per-node
trimmed-boundary membership is cached on the same basis. Both depend on the
invariant that **node coordinates are fixed for the whole of refinement** — the
mesher only moves nodes during post-refinement smoothing.

### Incremental maintenance
`Core/3D/RCDT/RestrictedTriangulation`

After each Bowyer-Watson insertion: drop the cavity's interior faces — captured
*before* the insertion, since the insertion is what destroys them — then
reclassify every face of every new tetrahedron. The bad-face set and the
encroached-segment set are maintained the same way rather than rescanned;
rescanning encroachment cost O(nodes x segments) per step. Faces spanning a
just-split curve edge are invalidated explicitly so no stale entry survives.

---

## 5. The refinement loop

### Four priorities, one insertion per step
`Core/3D/RCDT/RCDTRefiner`

Each step takes the first priority that has work:

1. Split an encroached curve segment — `RCDTPointInserter`
2. Fix a bad restricted triangle — `RestrictedTriangleRefiner`
3. Split a skinny tetrahedron (volume meshing only) — `TetrahedronQualityRefiner`
4. Repair a non-manifold edge of the restricted set — `NonManifoldEdgeRefiner`

Within a priority, candidates are taken in the order their containers yield
them, not worst-first, so that iteration order shapes the output mesh.

### Circumcenter demotion (Shewchuk)

Priorities 2-4 never insert a point that would encroach a curve segment; they
split that segment instead. This is the termination argument. Without it, a
circumcenter inserted near a curve creates a tiny new segment whose encroachment
forces another insertion, indefinitely. Demotion guarantees that insertions near
a feature are always preceded by enough segment refinement to break the
size-reduction cycle.

### Insertion point: the restricted Voronoi vertex
`Core/3D/RCDT/SurfaceProjector`

Where the face's dual edge actually crosses the surface, found by bisection on
signed distance, so it works regardless of curvature between the endpoints. When
that bisection finds no crossing — a legitimate outcome, since restriction was
decided by the independent tessellation test on a possibly older snapshot — the
refiner falls back to projecting the circumcenter onto the surface. There is
deliberately no maximum-gap guard on that projection: early refinement's large
triangles have circumcenters legitimately far from their surface.

Computed on demand, not alongside classification: 30 bisection iterations x 3
OCC calls per bad face would otherwise be paid for every face when only one is
inserted per step.

### Quality criteria
`Data/3D/SurfaceMesh3DQualitySettings`, `Core/3D/RCDT/RCDTQualityController`

Triangle circumradius / shortest edge (default 1.0 — the value RCDT's
termination behaviour was tuned against, not the usual 2.0), minimum interior
angle, chord deviation measured at the triangle's circumcenter against the
surface it was restricted to, and tetrahedron radius-edge for volumes (default
2.5; Shewchuk's bound only guarantees termination above 2.0).

Note what these cannot do: they are all **shape** bounds, scale-invariant by
construction, so a perfectly-shaped element of any size at all satisfies them.
Controlling size requires the sizing field of section 2.

### Termination devices

The `minimumEdgeLength` floor, plus a per-priority **unrefinable set**: a
candidate refused for the floor, for ball exclusion, or for a failed geometry
lookup is recorded and never retried. Without the floor, a segment near a small
input angle is bisected forever because each half is re-encroached by the same
nearby vertex, and a triangle just past the ratio threshold spawns an equally
bad, slightly smaller sliver beside it every iteration.

The unrefinable sets are never cleared. Clearing them after every segment split
was measured on the saddle to rediscover the same unfixable candidates about 100
times over, with no gain. Entries are keyed by nodes or element id, so a
candidate an insertion restructures returns under a new key.

### Non-manifold repair
`Core/3D/RCDT/NonManifoldEdgeRefiner`

Splits the curve segment joining the defect's endpoints when there is one, which
keeps the new point exactly on the crease. Projecting onto one of the surfaces
instead was measured to *move* such a defect, not resolve it.

---

## 6. Post-processing

### Restricted-face audit  *(OPE-208, OPE-184)*
`Core/3D/RCDT/RestrictedFaceAudit`

A per-edge coverage invariant read **off the CAD topology**, not the flat "every
edge has exactly two faces" rule:

- An edge **on a curve** (its two nodes chain-adjacent along one): exactly one
  incident face per surface that curve bounds — one for a free boundary, two for
  an ordinary crease, three or more at a junction.
- An edge **not on a curve** (surface interior, or a chord skipping a curve's
  own sample points): exactly two faces, both on the same surface.

The flat count test states a closed-2-manifold requirement the target models do
not all satisfy: in a conformal multi-material model, grain boundaries meet along
triple lines where three patches share one edge. Reading the expected count off
the topology is what lets a legitimate junction and an over-acceptance flap be
told apart — and what keeps a "keep the best two, drop the rest" repair from
silently destroying triple lines.

Runs once, after refinement has converged, because both of its passes judge
faces by counts the other changes. It removes same-curve chord faces, then
over-acceptance flaps as connected components (17 defects to 1 on the saddle),
then reports whatever remains. It performs no classification at all, so it is
unaffected by replacing the restriction oracle.

The count is **not** a quality gate: it lumps holes, duplicates and junctions
into one number, and the size floor is known to hide defects behind it.

### Ambient classification and removal
`Core/3D/RCDT/AmbientTetrahedronClassifier`, `Core/3D/RCDT/AmbientTetrahedronRemover`

A single flood fill from the bounding-supertet tetrahedra, crossing only
non-restricted faces. It reaches the true exterior and any interior voids at
once, and deliberately does not distinguish them — a through-hole connects the
two without crossing a restricted face. Everything reached is stripped, along
with the bounding tetrahedron's four nodes.

Recomputed from scratch per call rather than maintained incrementally: the
surrounding loop already pays O(tet count) elsewhere in the same iteration, so
this does not change the complexity class and stays a simple reference to
optimize against later.

### Laplacian smoothing with re-projection
`Core/3D/RCDT/SurfaceMeshSmoother`

Each interior vertex moves toward the centroid of its mesh neighbours and is
re-projected onto its owning CAD surface, for a fixed number of sweeps. Vertices
on a CAD edge or corner never move — their position is fixed by the curve they
belong to. Delaunay refinement on a curved surface does not reliably converge to
FEM-quality elements on its own; this is the standard follow-up Gmsh and Netgen
use, and it changes no mesh topology.

### Tetrahedron inversion guard
`Core/3D/RCDT/TetrahedronInversionGuard`

A sweep proposes every move at once, so two moves each harmless alone can invert
a tetrahedron they share. Each tetrahedron is judged with **all four** of its
nodes at their proposed positions, and one that would invert has all of its
nodes put back. A node put back never moves again in that call, so the process
terminates and a fully-reverted tetrahedron has exactly its original
orientation. Only relevant when the surface bounds a volume mesh and shares its
nodes with tetrahedra.

---

## 7. The 2D path — a different algorithm

- **Bowyer-Watson plus explicit constraint recovery** by iterative edge swapping
  (`Core/2D/ConstrainedDelaunay2D`, `Core/2D/MeshOperations2D`). This is the
  opposite of the 3D approach, where constraints emerge from restriction and are
  never recovered.
- **Ruppert/Shewchuk refinement** (`Core/2D/ShewchukRefiner2D`): encroached
  segments first, then circumcenters of poor-quality triangles, with the same
  demotion rule as 3D.
- **Diametral-circle encroachment with a periodic frame shift**
  (`Core/2D/ConstraintChecker2D`): the candidate point is shifted into the
  segment's own periodic frame before the test, so a point near the opposite
  period boundary still registers as encroaching.
- **Boundary-split callback** (`Core/2D/BoundarySplitSynchronizer`): fires after
  each boundary segment split so a twin segment can be split identically —
  the mechanism a future periodic mesher needs.
- `CHECK_MESH_EACH_ITERATION=1` enables per-iteration integrity checks. **2D
  only** — it does nothing in 3D.

---

## 8. Verification technique

Behaviour preservation is proven by **diffing the meshes**, not by passing
tests: `./scripts/refactor-check.sh` meshes representative models and compares
every full-precision `.tsv` against `tests/golden/`. Any difference means the
change was not a refactor. `EXPORT_PHASE_DIAGNOSTICS=1` exports the
classification picture for the restriction machinery.

---

## See also

- `doc/Theory/RCDT.md` — the algorithm as a walkthrough, phase by phase
- `doc/RCDT_Restriction_Audit.md` — the 43-row inventory of what restriction
  currently decides and where
- `doc/Meshing_Core_Overview.md` — module layout and ownership
- `doc/Terminology.md` — CAD and mesh glossary
