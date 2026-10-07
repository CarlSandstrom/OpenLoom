> **Current (since 2026-09-28, OPE-186).** This is the live pipeline both `meshSurface()` and `meshVolume()` use. The algorithm it replaced is `doc/History/RCDT_Historical.md`; the full technique-by-technique inventory, with measured numbers, is `doc/RCDT_Techniques.md`.

Here is the algorithm as currently implemented in OpenLoom, built to follow CGAL Mesh_3's design.

---

## What problem RCDT solves

The old approach (UV-space mesher) triangulated each CAD face independently in its parametric (u,v) space, then stitched faces together via `TwinManager`. This broke on curved/periodic surfaces (cylinders, tori) because OCC's seam parameterisation creates coordinate discontinuities.

RCDT sidesteps this entirely. **It never works in UV space.** It builds a single 3D Delaunay tetrahedralization over all boundary vertices, and surface triangles *emerge* from it implicitly — no explicit constraint recovery, no stitching, no UV pathologies. This is unchanged from the historical path; what changed is everything about how the tetrahedralization is built and refined.

---

## Relationship to CGAL Mesh_3

CGAL Mesh_3 protects 0D/1D features (corners and curves) with weighted balls (Boissonnat-Oudot), seeds a weighted ("regular") Delaunay tetrahedralization with them, then refines 2D facets and 3D cells until each satisfies a set of shape/size criteria — `Refine_facets_3` and `Refine_cells_3`.

**OpenLoom does not use the CGAL library.** It reimplements this design from scratch — `ProtectingBallPlacer` ports `Protect_edges_sizing_field`; `SurfaceDelaunayRefiner`/`TetrahedronDelaunayRefiner` port `Refine_facets_3`/`Refine_cells_3` — built as one unit rather than ported piece by piece. Five partial ports (weighted duals alone, refining straddling faces alone, worst-first order alone, weighted insertion points alone, JIGSAW-style repair) were each measured worse than the unified design, because protection, the quality-criteria exemption, the refusal policy, restriction, and the refinement order all assume each other are in place. See the Linear document "Why CGAL succeeded where our RCDT refiner did not (OPE-186)".

This is a different lineage from the Khoury–Shewchuk portals paper the historical path was built from. CGAL's protecting-ball guarantee is older (Boissonnat-Oudot) and rests on different footing — it depends on the balls satisfying three preconditions (below), not a formal portals-based guarantee that every constraining segment appears as an edge. But it is what the retired path's crease defects actually traced to: CGAL's own protection, injected into the new path, measured 0 holes / 0 excess on every model once the rest of the design came with it.

---

## Key definitions

### Protecting ball
A weighted point `(p, w)` placed on every corner and curve sample, `w` the squared ball radius. Insertion into the ambient tetrahedralization is **regular** (weighted Delaunay), not ordinary: a candidate point `q` is **hidden** — refused — by `(p, w)` if `|q − p|² ≤ w`. `ProtectingBallPlacer` sizes every ball so three conditions hold everywhere:
1. no protected point is hidden by another ball;
2. balls that are not neighbours on a curve are disjoint;
3. neighbours along a curve overlap, so the whole curve is covered.

These are the preconditions the restricted-Delaunay guarantees near features actually depend on.

### Restricted face
A triangular face of the weighted tetrahedralization is **restricted to surface S** if its **dual edge in the power diagram** — the segment between the **orthocentres** (weighted circumcentres) of its two tetrahedra — crosses S. The face belongs to whichever surface its dual edge crosses.

There is no vertex-surface-membership gate and no inside/outside test here: restriction is the dual-edge crossing test, nothing else. A convex-hull face (one adjacent tetrahedron — see the bounding supertet, below) has a half-infinite dual ray instead of a segment, handled the same way by extension.

### Curve segment
A `CurveSegment [a, b]` is a straight mesh edge whose endpoints are consecutive protected points along a topology edge, recorded once by `CurveSegmentBuilder` right after seeding. Unlike the historical path, **a curve segment is never split**: protection is fixed after seeding, and any point that would fall inside a ball is refused rather than redirected into the curve.

---

## Phase 1 — Seed (`RCDTMesher::seedTriangulation`)

**Goal:** a weighted Delaunay tetrahedralization with every corner and curve point protected, and none of its faces classified yet.

**R1.1 — Discretize boundary** (`BoundaryDiscretizer3D`)
- Walk every corner and edge in the CAD topology — no surface interior points are pre-seeded, same as the historical path.
- Sample parametrically, by a tangent-angle bound (scale-free: bounds how far a segment may *turn*, not how long it may be).
- Surface interiors are populated entirely by Phase 2.

**R1.2 — Place protecting balls** (`ProtectingBallPlacer`) — *new relative to the historical path*
- Size every corner and curve ball from the discretization's existing point spacing (the size function).
- Repair overlaps iteratively (`refine_balls`): shrink any pair of non-neighbour balls that intersect, to at most their distance / 2.1; repopulate gaps between neighbours; repeat to a fixed point, at most 29 rounds.
- Produces a weight (squared radius) per protected point, meeting the three conditions in "Protecting ball" above.

**R1.3 — Build weighted Delaunay tetrahedralization** (`Delaunay3D`, Bowyer-Watson, regular)
- Same incremental Bowyer-Watson as the historical path — conflict region, cavity boundary, remove and reconnect — but the in-sphere test is now the weighted (regular) predicate, and a point hidden by an existing ball is refused rather than inserted.
- The bounding supertet is kept, not removed, through all of refinement: every convex-hull face keeps a neighbour to fall back on, and every restricted face keeps tetrahedra on both sides so its dual edge is always defined. Removed at the very end by `AmbientTetrahedronRemover`.

**R1.4 — Build curve segment manager** (`CurveSegmentBuilder`)
- For each topology edge, read the ordered node sequence from the discretization and register one `CurveSegment` per consecutive pair.
- Unlike the historical path's R1.4, this is the **last** thing seeding does, and nothing after this point ever changes it.

Unlike the historical path, **no restricted faces are classified in Phase 1.** Classification is the first thing Phase 2 does.

---

## Phase 2 — Refine (`SurfaceDelaunayRefiner::refine`, driven by `TetrahedronDelaunayRefiner` for a volume)

Classify every face once (the weighted dual-edge test), then refine until no classification is wrong and no facet or tetrahedron fails CGAL's criteria.

---

### Priority 1 — Refine bad restricted facets
`SurfaceDelaunayRefiner`, `SurfaceFacetCriteria`

**When:** a restricted facet fails one of CGAL's three criteria, checked in this order:
0. **shape** — sin² of its smallest angle below sin²(minimum angle);
1. **distance** — its orthocentre further than the chord-deviation tolerance from its surface Delaunay ball centre;
2. **same patch** — its two surface-interior vertices lie on different CAD surfaces. This is a facet straddling a crease — the historical path refused to restrict these; this path refines them instead.

**Steps:**
1. Bad facets are ranked worst first, by the first failing criterion, then by how badly.
2. The refinement point is the facet's **surface Delaunay ball centre** — the dual-edge crossing point, bisected onto the exact CAD surface along the dual segment (not projected, which would move it sideways into a protecting ball).
3. Insert it via weighted Bowyer-Watson, **unless refused**: the point is hidden by an existing ball, coincides with an existing vertex, or no tetrahedron beside the facet actually conflicts with it (so inserting it would not remove the bad facet). A refused facet is left exactly as it was — there is no fallback split, unlike the historical path's curve-split demotion.
4. A facet with a weighted vertex is **exempt** from the shape criterion when it is small relative to the ball(s) it touches, and from the distance criterion when smaller still (`ProtectionExemption`) — refining it would only place a point inside the ball, where it would be refused anyway.

**Termination:** an empty bad-facet queue, or `maxRefinementIterations` (CGAL's `maximal_number_of_vertices`). This follows the standard Delaunay-refinement argument — insertions cannot accumulate arbitrarily close together away from protected features — rather than an explicit size-reduction rule like the historical path's segment splits. A persistently bad facet (one of the 51 CGAL itself leaves under 30° at a crease, on the saddle) simply stays bad in the final mesh; nothing retries it once refused.

---

### Priority 2 — Refine bad tetrahedra *(volume meshing only)*
`TetrahedronDelaunayRefiner`

**When:** no bad facet remains, and an interior tetrahedron's weighted circumradius / shortest edge exceeds the quality bound (default 2.5; Shewchuk's bound only guarantees termination above 2.0).

**Steps:**
1. Compute its orthocentre.
2. **Cascade to Priority 1:** if the candidate's conflict region contains a restricted facet whose surface Delaunay ball contains the candidate — CGAL's facet **encroachment** — refine that facet instead and return. Every restricted facet an insertion would otherwise destroy is encroached this way, so the surface can never be broken from below.
3. Otherwise insert the orthocentre via weighted Bowyer-Watson.
4. "Inside" is read from the ambient flood fill (Phase 3), recomputed once per round — not from a per-point domain query, which is where the historical path's retired phase test failed.

This is the same *shape* of cascade as the historical path's Priority 3 (curve → facet → insert), one level shallower: there is no curve-split rung, because curves are frozen.

---

### Loop terminates when
No bad facet and no bad tetrahedron remain, or the iteration cap is hit.

---

## Phase 3 — Extract and finish

1. **Closed-boundary check** (`RestrictedFaceAudit`, volume only) — *new relative to the historical path*. Reads a per-edge coverage invariant off the CAD topology (one incident face per bounded surface on a curve edge; exactly two, same-surface, on any other edge). `meshVolume()` throws if any edge is short a face, rather than letting the next step silently flood-fill through the hole.
2. **Ambient removal** (`AmbientTetrahedronClassifier`, `AmbientTetrahedronRemover`) — one flood fill from the bounding supertet, crossing only non-restricted faces, reaching the true exterior and any interior voids at once. Strips everything reached, plus the supertet's own four nodes.
3. **Extraction** (`RCDTMeshExtractor`) — collect the restricted faces (surface) or the surviving tetrahedra (volume) into `SurfaceMesh3D` / `VolumeMesh3D`. Adjacent CAD surfaces share boundary nodes automatically, same as the historical path, because they are faces of the one shared tetrahedralization.
4. **Smoothing** (`SurfaceMeshSmoother`, `TetrahedronInversionGuard`) — *new relative to the historical path*. Laplacian smoothing with re-projection onto the owning CAD surface; corner/curve vertices never move. For a volume, every proposed move is checked against inverting any incident tetrahedron — judged with all four of its nodes at their proposed positions at once — before being kept.

---

## Data structures that must stay consistent after every insertion

| Structure | What changes |
|-----------|-------------|
| `MeshData3D` | New node added; new tets added; old tets removed — same as the historical path |
| `SurfaceDelaunayRefiner`'s bad-facet queue | Faces the insertion created or destroyed are reclassified by the weighted dual-edge test; nothing else is re-touched |
| `CurveSegmentManager` | **Never changes after Phase 1.** This is the sharpest structural difference from the historical path, where it was rewritten on every curve split |

---

## Compared to the historical path

| | Historical (`doc/History/RCDT_Historical.md`) | Current |
|---|---|---|
| Protection | None as a precondition; curves refined by splitting on encroachment | Weighted balls placed to satisfy three explicit preconditions before any refinement starts |
| Restriction test | Unweighted circumcenter crossing, gated on vertex-surface membership, plus accept-only shortcut routes | Weighted dual-edge (orthocentre) crossing alone, no gate, no shortcuts |
| Curve segments | Split on encroachment, growing throughout refinement | Fixed once at seeding; never split again |
| A facet straddling a crease | Refused (the vertex-surface gate declines it) | Refined (the "same patch" criterion) |
| When refinement can't fix a bad element | Refused, with a size floor and proximity guard, then marked permanently unrefinable | Refused (hidden / coincident / non-conflicting only); left bad, no separate "unrefinable" bookkeeping |
| Closed-boundary check before the volume flood fill | None | `RestrictedFaceAudit` throws on any gap first |
| Post-refinement repair | `RestrictedFaceAudit`'s removal passes (same-curve chord faces, over-acceptance flaps) | None — "the restricted facets are the surface" |

---

## See also

- `doc/RCDT_Techniques.md` — full technique-by-technique inventory, with measured numbers
- `doc/History/RCDT_Historical.md` — the algorithm this one replaced (OPE-186)
- `doc/Meshing_Core_Overview.md` — module layout and ownership
- `doc/Terminology.md` — CAD and mesh glossary
