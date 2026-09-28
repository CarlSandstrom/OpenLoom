# Surface Mesh Quality

This document describes how triangle quality is evaluated and enforced during 3D surface mesh refinement. RCDT applies CGAL Mesh_3's facet criteria in `SurfaceDelaunayRefiner`. The 2D mesher's quality controller is summarised at the end.

---

## RCDT facet criteria

`SurfaceFacetCriteria` (`src/Meshing/Core/3D/RCDT/SurfaceFacetCriteria.h`) reads its bounds from `SurfaceMesh3DQualitySettings`:

| Setting | Criterion |
|-------|-------------|
| `minAngleDegrees` (30) | **shape**: a restricted facet is bad when sin² of its smallest angle is below sin²(`minAngleDegrees`) — CGAL's `facet_angle` |
| `chordDeviationTolerance` (0.1) | **distance**: bad when the facet's orthocentre lies further than this from its surface Delaunay ball centre (where its dual edge crosses the surface) — CGAL's `facet_distance` |
| — | **same patch**: bad when two of its surface-interior vertices lie on different surfaces — CGAL's `FACET_VERTICES_ON_SAME_SURFACE_PATCH` |

The first failing criterion, then how badly it fails, orders the refinement queue: worst first.

**Exemption at protecting balls** (`ProtectionExemption`, CGAL's `Facet_criterion_visitor_with_features`): a facet touching protected (weighted) vertices is exempt from the shape criterion, and when much smaller than the balls also from the distance criterion. Protection already guarantees the mesh at a crease; refining there would only place points inside the balls, where they are refused. Triangles under 30° therefore remain next to creases, as in CGAL's own output.

---

## How SurfaceDelaunayRefiner uses them

`SurfaceDelaunayRefiner` (`src/Meshing/Core/3D/RCDT/SurfaceDelaunayRefiner.h`):

1. Classify every face of the seeded tetrahedralization with `WeightedDualRestriction`; queue the restricted facets that fail a criterion.
2. Take the worst facet and insert its surface Delaunay ball centre by Bowyer-Watson — unless no tetrahedron beside the facet conflicts with it, or it is hidden by or coincident with an existing vertex; then drop the facet.
3. Reclassify only the faces the insertion created or destroyed.
4. Repeat until the queue is empty or `maxRefinementIterations` insertions have been made.

There is no size floor and no curve splitting: the protecting balls are fixed after seeding. For volumes, `TetrahedronDelaunayRefiner` runs below this level and bounds `tetCircumradiusToShortestEdgeRatio` (default 2.5); see `doc/RCDT_Techniques.md`.

After refinement, `SurfaceMeshSmoother` moves interior vertices toward their neighbours' centroid and re-projects them onto their surface; it improves most of the exempt triangles at the creases.

---

## 2D quality controller

`ShewchukRefiner2D` (the 2D mesher) refines against `IQualityController2D` (`src/Meshing/Interfaces/IQualityController2D.h`), implemented by `Shewchuk2DQualityController` (`src/Meshing/Core/2D/Shewchuk2DQualityController.h/.cpp`). A triangle is acceptable if its circumradius-to-shortest-edge ratio and minimum interior angle both satisfy the bounds in `Mesh2DQualitySettings`; refinement also stops once the mesh reaches `elementLimit` triangles.

The per-face UV-space surface mesher, which refined each CAD face in parameter space with this controller and a chord-deviation variant (`SurfaceMeshQualityController`), was deleted in OPE-192.
