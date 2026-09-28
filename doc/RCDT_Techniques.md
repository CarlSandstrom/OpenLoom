# Meshing Techniques Inventory

**Last updated:** 2026-09-28

Every technique the mesh generator relies on, grouped by the job it does, with
the class that implements it and the reason it is there. Written as a reading
aid: the *why* of each piece, not its API. Paths are relative to `src/Meshing/`.

The 3D mesher follows CGAL Mesh_3's design as a whole (OPE-186): protecting
balls that satisfy CGAL's conditions, restriction by the weighted dual edge
alone, and refinement that keeps inserting points until the restricted surface
is right, with no post-hoc repair. It replaced a path that decided restriction
with extra acceptance rules and repaired its output afterwards; why that path
fell short of CGAL is written up in the Linear document "Why CGAL succeeded
where our RCDT refiner did not (OPE-186)", and section 9 lists what was retired.

---

## 1. Numerical foundation

### Exact geometric predicates
`Core/3D/General/RobustPredicates3D`, `Core/3D/General/ExactArithmetic3D`

Shewchuk-style adaptive expansion arithmetic: a plain-double fast path with a
conservative error bound, falling back to exact expansions only when the sign
is not comfortably determined. Two predicates here, and the weighted one below:

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
callers reuse `RobustPredicates3D::orientationSign` directly. It evaluates the
determinant's sign and never forms an explicit centre or radius, so a near-flat
tetrahedron cannot corrupt it through a nearly-singular linear solve.

### Orthocentres
`Core/3D/General/ElementGeometry3D::computeOrthocenter`

The weighted circumcentre: the point at equal power distance `|x − p|² − w` to
every weighted vertex. In a regular triangulation this, not the ordinary
circumcentre, is an element's vertex of the dual power diagram. For a
tetrahedron flat to rounding — four points of a cyclic quad, such as matching
samples on the two circles bounding a cone — every point on the line normal to
its plane has equal power, and the minimum-norm solution picks the one in the
plane. Returning none instead left the faces around such a tetrahedron without
a dual edge: all 38 of HexNutChamfered's holes (OPE-186).

### Bowyer-Watson incremental insertion
`Core/3D/General/MeshOperations3D`, `Core/3D/Volume/Delaunay3D`, `Core/3D/RCDT/RegularConflictRegion`

Find the conflict region (elements whose orthosphere contains the point),
extract its boundary (faces appearing in exactly one conflicting element),
remove the conflicting elements, connect the new point to each boundary face.
The refiners grow the region locally from the tetrahedra next to the facet or
tetrahedron being refined, which in a regular triangulation finds the same
connected set as a full scan. A point **hidden** by an existing vertex
(`|p − v|² ≤ w_v`, inside its protecting ball) or coincident with one is
refused: it would not be a vertex of the regular triangulation.

### The bounding super-tetrahedron is deliberately kept
`Core/3D/Volume/Delaunay3D`

It stays in the mesh through all of refinement rather than being removed after
the initial build, for two reasons: every convex-hull face keeps a neighbour to
fall back on when a newly inserted vertex turns out coplanar with it, and every
restricted face keeps tetrahedra on *both* sides, so the dual edge the
restriction test needs is always defined. It is removed at the end, by
`AmbientTetrahedronRemover`.

---

## 2. Deciding where points go before meshing

### Boundary discretization
`Core/3D/General/BoundaryDiscretizer3D`

Samples corners and curves; surface interiors are populated by refinement. The
default criterion is a tangent-angle bound, which is scale-free: it bounds how
far a segment may *turn*, never how long it may be. On the saddle's parabolic
top arc that permits chords of 3.79 next to chords of 0.20 on one curve. This
spacing is the size function the protecting balls follow (section 3).

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
"remainder" segment near its seam vertex. It sets the cell size of the surface
tessellations the restriction test uses (section 4). Deriving it from a
sizing-field-driven discretization is circular (a finer boundary lowers it and
rebuilds the tessellations finer at the same time), so the sizing-field variant
reads a percentile of the field's own sources instead.

---

## 3. Protecting the creases

### Protecting balls / weighted points
`Core/3D/RCDT/ProtectingBallPlacer`

Boissonnat-Oudot curve protection, placed as CGAL Mesh_3's
`Protect_edges_sizing_field` does. Every corner and curve point is inserted as a
**weighted** point, its weight the squared ball radius, making the
triangulation a regular rather than an ordinary Delaunay one. The balls satisfy
CGAL's three conditions, pinned by tests:

1. no protected point is hidden by another ball;
2. balls that are not neighbours on a curve are disjoint;
3. neighbours along a curve overlap, so the whole curve is covered.

These are the preconditions of the restricted-Delaunay guarantees near
features. The old protection broke them on every model measured, and that —
not the restriction test — is what the old path's crease defects came from:
CGAL's own protection injected into the new path meshed the bracket cleanly at
once.

Placement:

- **Corners**: radius from the size function, capped at a third of the
  distance to the nearest other corner.
- **Curves** (`insert_balls`): between balls of radii `sp ≤ sq` a curve distance
  `d` apart, `n = round(2(d − sq)/(sp + sq))` balls with radii growing linearly
  from `sp` to `sq`, each spaced from the last by its own radius. The size is
  re-read at the midpoint for long runs, as CGAL does, **and wherever
  interpolation would oversize the middle**. CGAL's sizing fields vary slowly;
  ours, read off an angle-based discretization, falls fiftyfold along the dense
  saddle's end parabolas (1.33 → 0.028), and interpolating between the corners
  put balls of radius 0.8 at the apex.
- **Repair** (`refine_balls`): two balls that intersect without being
  neighbours are shrunk to at most their distance / 2.1, gaps between
  neighbours are repopulated, repeated to a fixed point (at most 29 rounds).
  Beyond CGAL, a neighbour hidden inside a larger ball shrinks the larger one
  to their distance — possible next to a large corner ball once the size
  function varies fast.

The protection is fixed after seeding: refinement never splits a curve, and
the points it inserts can never enter a ball (they would be hidden and are
refused, section 1).

### Curve segments
`Data/CurveSegmentManager`, `Core/3D/RCDT/CurveSegmentBuilder`

The chain of protected points along each CAD curve, recorded once at seeding.
The restricted-face audit (section 6) reads it to tell an edge on a curve from
one across a surface.

---

## 4. Restriction: extracting the boundary from the volume

Surface constraints are never explicitly recovered. They *emerge*: the set of
restricted faces for a surface is its triangulation, automatically conforming
and automatically shared with adjacent surfaces, because all of them are faces
of one shared tetrahedralization.

### The weighted dual-edge test
`Core/3D/RCDT/WeightedDualRestriction`

A face is restricted iff its dual edge in the power diagram — the segment
between the orthocentres of its two tetrahedra — crosses a surface, and it
belongs to the surface crossed. Nothing else: no gate on which surfaces its
vertices share, no shortcut routes, no inside/outside test. Where the answer is
locally wrong, refinement is expected to fix it (section 5).

This test is valid on badly shaped tetrahedra. CGAL uses it on tetrahedra with
a median circumradius/shortest edge of about 2 and flat slivers up to 1e17 and
meshes the saddle cleanly: the restricted Delaunay guarantees depend on
sampling relative to local feature size, not on tetrahedron shape.

When the dual edge crosses more than once, the crossing nearest the face's own
orthocentre wins: that point lies on the dual line, so the nearest crossing is
the one belonging to this face. The crossing is refined onto the exact CAD
surface by bisection **along the dual segment**, so it stays on the dual line;
projecting it onto the surface instead moved it sideways into a protecting ball.
It is accepted only within the surface's trimmed patch — the tessellation and
the bisection both work on the untrimmed surface, and on the bracket two flange
planes extended past their faces intersect. The accepted crossing is the centre
of the facet's **surface Delaunay ball**, the refinement point.

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
OpenCASCADE. Built once per surface at cells of half the minimum edge length and
never rebuilt. Cells are not clipped to the exact trim curve, so the
tessellation may overshoot by up to one cell — harmless, because the crossing
is checked against the true trim. What it must not have is a **gap**: the grid
used to start a fraction of a cell inside the minimum parameter edge, leaving an
uncovered band exactly where creases are, and every crossing the dense saddle
missed lay in it. The grid now starts before the minimum and ends past the
maximum.

### Triangle soup index
`Core/3D/RCDT/TriangleSoupIndex`

A uniform 3D grid over an unstructured set of triangles, answering the exact
segment-crossing query. Each triangle is registered in every cell its bounding
box overlaps; a query walks only the cells the segment passes through and runs
the exact predicate only on the triangles registered there. Knows nothing about
surfaces.

---

## 5. Refinement

Two levels, CGAL Mesh_3's `Refine_facets_3` and `Refine_cells_3`. The facet
level always comes first.

### Facet refinement
`Core/3D/RCDT/SurfaceDelaunayRefiner`, `Core/3D/RCDT/SurfaceFacetCriteria`

Every face is classified once after seeding; after each insertion, only the
faces the insertion created or destroyed are reclassified. A restricted facet is
**bad** by CGAL's criteria, in CGAL's order:

0. **shape** — sin² of its smallest angle below sin²(`minAngleDegrees`);
1. **distance** — its orthocentre further than `chordDeviationTolerance` from
   its surface Delaunay ball centre;
2. **same patch** — two surface-interior vertices on different surfaces. This
   is how CGAL handles a facet straddling a crease: it refines it rather than
   refusing to restrict it.

Bad facets are refined **worst first** — ordered by the first failing
criterion, then by how badly — by inserting the surface Delaunay ball centre.
CGAL's refusals only: a point that no tetrahedron beside the facet conflicts
with (inserting it would not remove the facet), or one that is hidden or
coincident. No size floor, no proximity guard, no curve splits. Terminates on
an empty queue or at `maxRefinementIterations`, CGAL's
`maximal_number_of_vertices`.

**Exemption at protecting balls** (`Core/3D/RCDT/ProtectionExemption`, CGAL's
`Facet_criterion_visitor_with_features`). A facet with weighted vertices is
exempt from the shape criterion when it is small relative to the balls it
touches, and from the distance criterion when much smaller still: the ratio of
the smallest sphere orthogonal to a weighted and an unweighted vertex to the
weight, below 1.0 and 0.04. It applies to one weighted vertex, or two or three
with intersecting balls. Protection already guarantees the mesh at a crease,
and refining there only places points into the balls, where they are refused.
CGAL's own final saddle mesh keeps 51 facets under 30°, all touching a protected
vertex; ours keeps its bad triangles there too.

### Tetrahedron refinement  *(volume meshing only)*
`Core/3D/RCDT/TetrahedronDelaunayRefiner`

A tetrahedron inside the domain whose weighted circumradius / shortest edge
exceeds `tetCircumradiusToShortestEdgeRatio` (default 2.5; Shewchuk's bound only
guarantees termination above 2.0) is refined at its orthocentre, worst first.

- **Inside** comes from the ambient flood fill (section 6) from the restricted
  facets, recomputed once per round. CGAL asks the domain oracle at the
  orthocentre instead; for us that is OCC's point classification, whose
  tolerance band was the source of the retired phase test's failures.
- **The surface comes first**: after every insertion the facet level refines
  until no facet is bad, and a point whose conflict region holds a restricted
  facet whose surface Delaunay ball contains it — CGAL's **encroachment** —
  refines that facet instead. Every restricted facet an insertion would destroy
  is encroached, so the surface is never broken from below.

No sliver perturbation or exudation: flat "cap" tetrahedra with all four
vertices on the boundary can remain.

---

## 6. After refinement

### Closed-boundary check
`Core/3D/RCDT/RestrictedFaceAudit`

A per-edge coverage invariant read **off the CAD topology**, not the flat "every
edge has exactly two faces" rule:

- An edge **on a curve** (its two nodes chain-adjacent along one): exactly one
  incident face per surface that curve bounds — one for a free boundary, two for
  an ordinary crease, three or more at a junction.
- An edge **not on a curve**: exactly two faces, both on the same surface.

The flat count test states a closed-2-manifold requirement the target models do
not all satisfy: in a conformal multi-material model, grain boundaries meet along
triple lines where three patches share one edge. `meshVolume()` counts the edges
missing a face and throws if there are any — the ambient flood fill below would
otherwise walk through a hole into the solid. Read-only: nothing is removed.

### Ambient classification and removal
`Core/3D/RCDT/AmbientTetrahedronClassifier`, `Core/3D/RCDT/AmbientTetrahedronRemover`

A single flood fill from the bounding-supertet tetrahedra, crossing only
non-restricted faces. It reaches the true exterior and any interior voids at
once, and deliberately does not distinguish them — a through-hole connects the
two without crossing a restricted face. Everything reached is stripped, along
with the bounding tetrahedron's four nodes.

### Laplacian smoothing with re-projection
`Core/3D/RCDT/SurfaceMeshSmoother`

Each interior vertex moves toward the centroid of its mesh neighbours and is
re-projected onto its owning CAD surface, for a fixed number of sweeps. Vertices
on a CAD edge or corner never move. Measured on the new pipeline: without it,
every triangle under 30° touches a crease (the exemption zone); with it, many of
those improve and a few interior ones appear. Removing it was worse on every
model.

### Tetrahedron inversion guard
`Core/3D/RCDT/TetrahedronInversionGuard`

A sweep proposes every move at once, so two moves each harmless alone can invert
a tetrahedron they share. Each tetrahedron is judged with **all four** of its
nodes at their proposed positions, and one that would invert has all of its
nodes put back. A node put back never moves again in that call, so the process
terminates and a fully-reverted tetrahedron has exactly its original
orientation. Only relevant when the surface bounds a volume mesh.

---

## 7. The 2D path — a different algorithm

- **Bowyer-Watson plus explicit constraint recovery** by iterative edge swapping
  (`Core/2D/ConstrainedDelaunay2D`, `Core/2D/MeshOperations2D`). This is the
  opposite of the 3D approach, where constraints emerge from restriction and are
  never recovered.
- **Ruppert/Shewchuk refinement** (`Core/2D/ShewchukRefiner2D`): encroached
  segments first, then circumcenters of poor-quality triangles; a circumcentre
  that would encroach a segment splits the segment instead (demotion), which is
  the termination argument.
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
change was not a refactor.

**External reference: CGAL Mesh_3.** The harness `~/tmp/cgal-saddle` (outside
the repository) meshes the saddle with CGAL from the same OCC geometry. With
`protected=<file>` it starts from *our* protected point set instead of CGAL's
own protection; `max_vertices=N` stops it after N vertices; `export=` writes
each facet with CGAL's own quality verdict and insertion point. Together these
compare the two refiners from identical inputs, step by step.

---

## 9. Retired (OPE-186)

The path `meshSurface()` and `meshVolume()` ran before 2026-09-28. Its code is in
git history up to `873c9bf`; `doc/RCDT_Restriction_Audit.md` inventories its
restriction decisions.

- **`CurveProtectionScheme` / `CurveProtectionSubdivider`** — protection sized
  from the longer adjacent segment, subdivided where it could not satisfy its
  own properties. Broke CGAL's conditions on every model.
- **`RestrictedTriangulation` / `DualEdgeRestrictionOracle`** — the unweighted
  dual-edge test behind a vertex-surface gate, plus accept-only shortcut routes
  (a protected-edge uniqueness rule, a centroid phase test through OCC's solid
  classifier, `PointPhase`) and a seam-surface carve-out.
- **`RCDTRefiner`** and its priorities — curve-segment encroachment splits,
  bad-triangle refinement with Shewchuk demotion, skinny-tetrahedron refinement,
  non-manifold-edge repair — with a size floor, proximity guard, protecting-ball
  refusals and permanently unrefinable sets.
- **`RestrictedFaceAudit`'s removal passes** — same-curve chord faces and
  over-acceptance flaps removed after refinement.

---

## See also

- `doc/Meshing_Core_Overview.md` — module layout and ownership
- `doc/Terminology.md` — CAD and mesh glossary
- `doc/Theory/RCDT.md`, `doc/Flowcharts/` — the paper's algorithm and the
  retired implementation of it
