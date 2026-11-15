# Meshing Walkthrough

A short, step-by-step tour of the RCDT meshing algorithm, with how the steps depend on each other. The detailed reference is `doc/RCDT_Current.md`. CAD and export are out of scope here.

## Overview

One 3D tetrahedralization is built and refined. Surface triangles are not constructed: they *emerge* as the faces of that tetrahedralization that "touch" the surface. A volume mesh is the tetrahedra inside.

```
1 Sample features -> 2 Protecting balls -> 3 Weighted Delaunay -> 4 Classify faces
                                                                       |
                         +---------------------------------------------+
                         v
              5 Refine facets  <->  6 Refine tetrahedra (volume only)
                         |
                         v
              7 Remove ambient tets -> 8 Extract -> 9 Smooth
```

---

## 1. Sample the boundary

Walk only the **corners and curves** (where CAD faces meet) and place points along them. Spacing comes from a tangent-angle bound, which limits how far a curve may turn between two points, so it does not depend on model scale. Straight edges get only their endpoints. No points are placed in surface interiors; refinement (5) adds those.

**Feeds step 2:** the point spacing becomes the size of each ball.

## 2. Place protecting balls

Every corner and curve point gets a **ball**: a weight, the squared radius. The ball reserves space around the point so that no later refinement point can land near the curve. The balls are sized and repaired until they meet three conditions. After this step the curve points and weights are **final**: refinement never splits a curve, and smoothing never moves these points.

Full description, including why and where protection is needed and its known limitation: [Appendix A](#appendix-a-protecting-balls).

**Feeds step 3:** the weighted points are the seed. **Feeds 5:** the balls mark where refinement points are refused.

## 3. Build the weighted Delaunay tetrahedralization

Insert the weighted points one at a time (Bowyer-Watson, using the weighted "regular" in-sphere test). A new point lying inside an existing ball is **refused**. A bounding supertetrahedron stays in place through all of refinement, so every face keeps a neighbour on each side. The ordered points of each curve are then recorded as fixed **curve segments**.

**Feeds step 4:** a tetrahedralization with no surface knowledge yet.

## 4. Classify faces: which ones are the surface?

A face is **restricted** to a surface when its **dual edge** (the segment between the orthocentres, the weighted circumcentres, of its two tetrahedra) crosses that surface. The restricted faces are the surface mesh. Every face is tested once here. After that, each insertion re-tests only the faces whose dual edge it could have moved.

The crossing point is the centre of the face's **surface Delaunay ball**. It is also the point that gets inserted if the face is bad.

**Feeds step 5:** the set of restricted faces and their crossing points.

## 5. Refine bad surface facets

Facets are queued worst-first and fail on any of three criteria, checked in this order:

1. **Shape:** the smallest angle is too small.
2. **Distance:** the facet is too far from the surface (chord deviation).
3. **Same patch:** the facet's vertices lie on different CAD surfaces, so it straddles a crease.

The fix is to insert the facet's surface-ball centre, snapped onto the true surface. The insertion is refused if the point lies in a protecting ball, coincides with an existing vertex, or would not change the facet. A refused facet stays as it is. Facets next to a ball are exempt from shape and distance when small relative to that ball, since a refinement point there would be refused anyway.

Each insertion re-runs step 4 locally. The loop ends when no bad facet remains or the iteration cap is hit. For **surface meshing, this is the whole refinement.**

**Feeds step 6:** a surface that is good before the interior is touched.

## 6. Refine bad tetrahedra (volume only)

Once no bad facet remains, an interior tetrahedron whose circumradius-to-shortest-edge ratio is too large (default 2.5) is refined by inserting its orthocentre. If that point's conflict region would destroy a restricted facet it encroaches on, that **facet is refined instead** (back to step 5). The surface therefore cannot be broken from below. "Interior" comes from a flood fill that is recomputed each round.

Steps 5 and 6 alternate until neither finds anything bad.

## 7. Remove the ambient tetrahedra

For a volume, a closed-boundary check runs first. It throws if any surface edge is missing a face, because the flood fill would otherwise leak through the hole. Then one flood fill runs from the supertetrahedron and crosses only non-restricted faces. It reaches the exterior and any inner voids. Everything it reaches is removed.

## 8. Extract

Collect the restricted faces (surface mesh) or the surviving tetrahedra (volume mesh). Adjacent CAD surfaces share nodes automatically, because their facets are faces of the one tetrahedralization.

## 9. Smooth

Laplacian smoothing with each point re-projected onto its CAD surface. Corner and curve points never move. For a volume, a move is rejected if it would invert any incident tetrahedron.

---

## How the steps depend on each other

| Dependency | Why |
|---|---|
| 1 → 2 | Ball size is derived from the sample spacing. |
| 2 → 3 | The weighted points seed the triangulation. |
| 2 → 5, 6 | Balls decide which refinement points are refused and which facets are exempt. |
| 3 → 4 | Classification needs tetrahedra with a dual edge on every face, which the supertetrahedron guarantees. |
| 4 ↔ 5 | Every insertion in 5 changes dual edges, so classification is incremental and local. |
| 6 → 5 | A tetrahedron insertion that encroaches on a facet is turned into a facet refinement. |
| 5, 6 → 7 | The flood fill is only meaningful once the restricted faces form a closed surface. |
| 2 → 9 | Curve points are frozen from step 2 to the end. |

Protection, restriction, the exemption, the refusal policy and the refinement order all assume each other. Adding only some of them was measured worse than the unified design.

---

## Appendix A: Protecting balls

### What a ball is

Every corner and curve point gets a weight `w`, the squared radius of its ball. Points are inserted into the triangulation by the weighted (regular) Delaunay rule. A candidate point `q` is **refused** if it lies inside an existing ball, that is, if `|q - p|² <= w`.

### Why curves need protection, and where

Protection is needed wherever the surface is not smooth: along the **curves** where two CAD faces meet or where a face ends, and at the **corners**. Smooth surface interiors need none.

The surface mesh emerges from the triangulation, which works only when the points are dense compared to the surface's local feature size. At a crease that size goes to zero, so no finite sampling is dense enough. Near a crease a face can straddle two surfaces, have its dual-edge crossing land outside its patch, or be missed entirely. The older, unprotected path produced holes at creases for this reason.

The ball keeps every other vertex out of the zone around a curve segment. With nothing inside that zone, the segment can't be cut by a closer point, so it stays an **edge** of the tetrahedralization. The facets of each surface then end on that edge, and neighbouring surfaces share it. The closed-boundary check in step 7 relies on this.

### Sizing and repair

Each ball starts at a size taken from the point spacing of step 1. The placer then repairs the sizes, for at most 29 rounds, until:

1. No protected point lies inside another ball.
2. Balls of non-neighbours are disjoint. Two that overlap both shrink to at most their distance / 2.1.
3. Neighbours along a curve overlap, so the curve is fully covered. A gap gets a new point, so the point count can grow during this step.

### Known limitation

Ball size comes from the step-1 point spacing, not from the element size the surfaces need, and it is fixed afterwards, so a ball can be larger than the local mesh wants. A ball also does more than refuse points inside its radius. Its weight pushes the orthocentres of nearby tetrahedra away, which moves their dual edges and can change the restriction test for faces near it, including faces on a surface the curve doesn't belong to.

Measured on `PartDesignExample-Body.step` (OPENL-14): a ball sized from a spacing of about 10 where the element size was about 4 made a facet lying flush on an R35 cylinder come out unrestricted (dual edge off the cylinder by 0.108). Three facets reaching 10 units inward to a protected curve point were restricted instead, leaving a dent. The sizing field h(x) (OPE-181) exists but does not drive the balls yet (planned: OPE-212).
