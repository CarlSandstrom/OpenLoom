# BoxWithHoleSurface: meshing flow

Files in the order `BoxWithHoleSurface` runs them, for working through the meshing code one step at a time. CAD import (`Readers/`, OpenCascade), export and the 2D mesher are left out. Each file is listed once, at the step where it is first used. `[x]` marks files already brought in line with `.claude/rules/cpp-coding-standards.md`.

Line counts are `.h` / `.cpp` or `.cc`.

## 1. Entry: settings and the public mesher

`BoxWithHoleSurface` builds the settings, constructs `SurfaceMesher3D` and calls `mesh()`, which forwards to `RCDTMesher::meshSurface()`.

| Done | Class | .h | source | Lines | Role |
|---|---|---|---|---|---|
| [ ] | `Geometry/3D/Base/DiscretizationSettings3D` | 73 | - | 73 | Edge angle / segment count / surface samples |
| [ ] | `Meshing/Data/3D/SurfaceMesh3DQualitySettings` | 53 | - | 53 | Refinement bounds, smoothing iterations, size floor |
| [ ] | `Meshing/Core/3D/Surface/SurfaceMesher3D` | 73 | 36 | 109 | Thin public wrapper; owns an `ISurfaceMesher3D` |
| [ ] | `Meshing/Interfaces/ISurfaceMesher3D` | 24 | - | 24 | Strategy interface |
| [x] | `Meshing/Core/3D/RCDT/RCDTMesher` | 85 | 279 | 364 | The pipeline: orchestrates every step below |

## 2. `meshSurface()` creates the context

Owns the geometry and topology references plus the live mesh the whole pipeline mutates.

| Done | Class | .h | source | Lines | Role |
|---|---|---|---|---|---|
| [ ] | `Meshing/Core/3D/General/MeshingContext3D` | 76 | 77 | 153 | Bundles geometry, topology, mesh data, mutator and operations |
| [ ] | `Meshing/Data/3D/MeshData3D` | 65 | 105 | 170 | The live tetrahedral mesh |
| [ ] | `Meshing/Data/3D/Node3D` | 30 | 31 | 61 | A mesh node (coordinates and weight) |
| [ ] | `Meshing/Data/3D/TetrahedralElement` | 27 | 49 | 76 | A tetrahedron |
| [ ] | `Meshing/Data/3D/MeshMutator3D` | 55 | 133 | 188 | Controlled mutation of `MeshData3D` |
| [ ] | `Meshing/Data/Base/MeshConnectivity` | 36 | 112 | 148 | Element adjacency snapshot |

## 3. `buildInitialTriangulation`: discretize the boundary

Samples corners, edge interiors and surface interiors into points. Reads the CAD geometry and topology through their base interfaces.

| Done | Class | .h | source | Lines | Role |
|---|---|---|---|---|---|
| [x] | `Meshing/Core/3D/General/BoundaryDiscretizer3D` | 78 | 236 | 314 | Samples the boundary into points |
| [ ] | `Meshing/Data/3D/DiscretizationResult3D` | 38 | - | 38 | Points plus the edge/surface they belong to |
| [ ] | `Geometry/3D/Base/GeometryCollection3D` | 39 | 48 | 87 | Lookup of corners, edges and surfaces |
| [ ] | `Geometry/3D/Base/ICorner3D` | 22 | - | 22 | Corner interface |
| [ ] | `Geometry/3D/Base/IEdge3D` | 39 | - | 39 | Edge (curve) interface |
| [ ] | `Geometry/3D/Base/ISurface3D` | 101 | 22 | 123 | Surface interface |
| [ ] | `Topology/Topology3D` | 43 | 80 | 123 | Connectivity between the geometric entities |
| [ ] | `Topology/SeamCollection` | 44 | 43 | 87 | Periodic-surface seam twins |

## 4. `buildInitialTriangulation`: size floor

No explicit floor and no sizing field here, so the floor comes from the point spacing.

| Done | Class | .h | source | Lines | Role |
|---|---|---|---|---|---|
| [x] | `Meshing/Core/3D/RCDT/MinimumEdgeLengthEstimator` | 34 | 67 | 101 | Median nearest-neighbour spacing / 10 |

## 5. `buildAmbientTriangulation`: protecting balls

Gives every corner and curve point a weight (protecting-ball radius) so creases survive in the triangulation.

| Done | Class | .h | source | Lines | Role |
|---|---|---|---|---|---|
| [ ] | `Meshing/Core/3D/RCDT/ProtectingBallPlacer` | 62 | 505 | 567 | Places the protecting balls |

## 6. `buildAmbientTriangulation`: weighted Delaunay tetrahedralization

Inserts every point into the live mesh. `MeshOperations3D` and its predicates do the geometric work.

| Done | Class | .h | source | Lines | Role |
|---|---|---|---|---|---|
| [ ] | `Meshing/Core/3D/Volume/Delaunay3D` | 67 | 56 | 123 | Builds the initial weighted tetrahedralization |
| [ ] | `Meshing/Core/3D/General/MeshOperations3D` | 168 | 377 | 545 | Insertion and cavity operations on the mesh |
| [ ] | `Meshing/Core/3D/General/MeshQueries3D` | 83 | 152 | 235 | Point location and adjacency queries |
| [ ] | `Meshing/Core/3D/General/RegularPredicates3D` | 61 | 248 | 309 | Weighted (regular) in-sphere / orientation tests |
| [ ] | `Meshing/Core/3D/General/RobustPredicates3D` | 78 | 108 | 186 | Robust floating-point predicates |
| [ ] | `Meshing/Core/3D/General/ExactArithmetic3D` | 74 | 132 | 206 | Exact-arithmetic fallback |

## 7. `buildAmbientTriangulation`: curve segments

Records which mesh nodes form each CAD curve.

| Done | Class | .h | source | Lines | Role |
|---|---|---|---|---|---|
| [ ] | `Meshing/Core/3D/RCDT/CurveSegmentBuilder` | 45 | 96 | 141 | Builds the curve segments from the discretization |
| [ ] | `Meshing/Data/CurveSegmentManager` | 61 | 123 | 184 | Stores the curve segments |
| [ ] | `Meshing/Data/ConstraintRole` | 12 | - | 12 | Role tag for a curve segment |

## 8. `refineSurface`: facet refinement

The heart of the mesher. Classifies every face as restricted or not, then inserts points until no facet is bad.

| Done | Class | .h | source | Lines | Role |
|---|---|---|---|---|---|
| [ ] | `Meshing/Core/3D/RCDT/SurfaceDelaunayRefiner` | 130 | 223 | 353 | Classification, worst-first queue and insertion |
| [ ] | `Meshing/Connectivity/FaceKey` | 79 | - | 79 | Face identity |
| [ ] | `Meshing/Connectivity/EdgeKey` | 48 | - | 48 | Edge identity |
| [ ] | `Meshing/Core/3D/RCDT/RestrictedFaceTypes` | 51 | - | 51 | Restricted facet records and the map of them |
| [ ] | `Meshing/Core/3D/RCDT/WeightedDualRestriction` | 83 | 140 | 223 | Restriction test: does the weighted dual edge cross a surface |
| [ ] | `Meshing/Core/3D/RCDT/SurfaceTessellation` | 67 | 289 | 356 | One CAD surface sampled onto a UV grid |
| [ ] | `Meshing/Core/3D/RCDT/TriangleSoupIndex` | 88 | 214 | 302 | Uniform grid answering segment-crossing queries |
| [ ] | `Meshing/Core/3D/RCDT/SurfaceProjector` | 65 | 127 | 192 | Projects points onto a surface |
| [ ] | `Meshing/Core/3D/General/ElementGeometry3D` | 38 | 77 | 115 | Circumcenters, orthocenters and similar |
| [ ] | `Meshing/Data/2D/TriangleElement` | 30 | 57 | 87 | Triangle record (shared with 2D) |
| [ ] | `Meshing/Core/3D/RCDT/SurfaceFacetCriteria` | 54 | 88 | 142 | The three facet criteria |
| [ ] | `Meshing/Core/3D/RCDT/ProtectionExemption` | 28 | 102 | 130 | Waives the criteria next to protecting balls |
| [ ] | `Meshing/Core/3D/RCDT/RegularConflictRegion` | 45 | 81 | 126 | The conflict region an insertion removes |

## 9. `buildSurfaceMesh`: remove the ambient tetrahedra

Strips the tetrahedra that are not part of the model, keeping the restricted facets.

| Done | Class | .h | source | Lines | Role |
|---|---|---|---|---|---|
| [ ] | `Meshing/Core/3D/RCDT/AmbientTetrahedronRemover` | 26 | 43 | 69 | Removes the ambient tetrahedra |
| [ ] | `Meshing/Core/3D/RCDT/AmbientTetrahedronClassifier` | 46 | 71 | 117 | Flood fill deciding which tetrahedra are inside |

## 10. `buildSurfaceMesh`: extract the surface mesh

| Done | Class | .h | source | Lines | Role |
|---|---|---|---|---|---|
| [ ] | `Meshing/Core/3D/RCDT/RCDTMeshExtractor` | 44 | 115 | 159 | Turns the restricted facets into a `SurfaceMesh3D` |
| [ ] | `Meshing/Data/3D/SurfaceMesh3D` | 37 | - | 37 | The result handed back to the example |

## 11. `buildSurfaceMesh`: smooth

Moves nodes along the CAD surfaces. No tetrahedra to protect in surface mode.

| Done | Class | .h | source | Lines | Role |
|---|---|---|---|---|---|
| [ ] | `Meshing/Core/3D/RCDT/SurfaceMeshSmoother` | 53 | 138 | 191 | Smooths the surface mesh |
| [ ] | `Meshing/Core/3D/RCDT/TetrahedronInversionGuard` | 46 | 69 | 115 | Keeps tetrahedra from inverting (inactive in surface mode) |

## Used throughout

| Done | Class | .h | source | Lines | Role |
|---|---|---|---|---|---|
| [ ] | `Common/Types` | 19 | - | 19 | `Point3D`, `Vector3D` aliases |
| [ ] | `Common/BoundingBox2D` | 23 | 34 | 57 | UV bounds |
| [ ] | `Common/DebugFlags` | 50 | - | 50 | Debug switches |
| [ ] | `Common/Exceptions/Exception` | 77 | - | 77 | Exception base |
| [ ] | `Common/Exceptions/MeshException` | 159 | - | 159 | `OPENLOOM_THROW_MESH` |
| [ ] | `Common/Exceptions/GeometryException` | 112 | - | 112 | Geometry errors |

## Meshing files this example does not run

Not on the `BoxWithHoleSurface` path, so not part of this walk-through.

| Done | Class | .h | source | Lines | Role |
|---|---|---|---|---|---|
| [ ] | `Meshing/Core/3D/RCDT/TetrahedronDelaunayRefiner` | 71 | 152 | 223 | Volume mode only |
| [ ] | `Meshing/Core/3D/RCDT/RestrictedFaceAudit` | 80 | 165 | 245 | Volume closed-boundary check only |
| [ ] | `Meshing/Core/3D/Volume/VolumeMesher3D` | 74 | 36 | 110 | Volume wrapper |
| [ ] | `Meshing/Interfaces/IVolumeMesher3D` | 24 | - | 24 | Volume interface |
| [ ] | `Meshing/Data/3D/VolumeMesh3D` | 43 | - | 43 | Volume result |
| [ ] | `Meshing/Core/3D/General/SizingField3D` | 105 | 54 | 159 | Only with sizing-field settings |
| [ ] | `Meshing/Core/3D/General/SizingFieldBuilder3D` | 88 | 164 | 252 | Only with sizing-field settings |
| [ ] | `Meshing/Core/3D/General/LocalFeatureSize3D` | 106 | 292 | 398 | Only with sizing-field settings |
| [ ] | `Meshing/Core/3D/General/MeshDebugUtilities3D` | 49 | 41 | 90 | End-of-run debug export, not meshing |
