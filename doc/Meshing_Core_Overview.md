# Meshing Core Module Overview

**Last Updated:** 2026-09-28

## Purpose

The `src/Meshing/Core` module implements constrained Delaunay triangulation for 2D and 3D meshes. 2D meshing uses Bowyer-Watson incremental insertion with constraint edge recovery. 3D surface and volume meshing use the Restricted Constrained Delaunay Triangulation (RCDT) algorithm operating in ambient 3D space.

## Directory Organization

```
src/Meshing/Core/
├── 2D/                                  # 2D meshing algorithms
│   ├── MeshingContext2D.{h,cpp}
│   ├── ConstrainedDelaunay2D.{h,cpp}
│   ├── Delaunay2D.{h,cpp}
│   ├── MeshOperations2D.{h,cpp}
│   ├── MeshQueries2D.{h,cpp}
│   ├── MeshVerifier.{h,cpp}
│   ├── ElementGeometry2D.{h,cpp}
│   ├── ElementQuality2D.{h,cpp}
│   ├── GeometryUtilities2D.{h,cpp}
│   ├── GeometryStructures2D.h
│   ├── EdgeDiscretizer2D.{h,cpp}
│   ├── DiscretizationResult2D.h
│   ├── ConstraintChecker2D.{h,cpp}
│   ├── BoundarySplitSynchronizer.{h,cpp}
│   ├── ShewchukRefiner2D.{h,cpp}
│   ├── Shewchuk2DQualityController.{h,cpp}
│   └── MeshDebugUtils2D.{h,cpp}
├── 3D/
│   ├── General/                         # Shared 3D context, operations, geometry
│   │   ├── MeshingContext3D.{h,cpp}
│   │   ├── MeshOperations3D.{h,cpp}
│   │   ├── MeshQueries3D.{h,cpp}
│   │   ├── MeshVerifier3D.{h,cpp}
│   │   ├── ElementGeometry3D.{h,cpp}
│   │   ├── ElementQuality3D.{h,cpp}
│   │   ├── GeometryUtilities3D.{h,cpp}
│   │   ├── GeometryStructures3D.h
│   │   ├── BoundaryDiscretizer3D.{h,cpp}
│   │   ├── DiscretizationResult3D.h
│   │   ├── ConstraintChecker3D.{h,cpp}
│   │   ├── EdgeTwinTable.h
│   │   ├── FacetDiscretization2DBuilder.{h,cpp}
│   │   ├── FacetTriangulation.{h,cpp}
│   │   ├── FacetTriangulationManager.{h,cpp}
│   │   ├── TwinTableGenerator.{h,cpp}
│   │   └── MeshDebugUtils3D.{h,cpp}
│   ├── Surface/                         # UV-space surface mesher (superseded by RCDT)
│   │   ├── SurfaceMesher3D.{h,cpp}
│   │   ├── SurfaceMeshingContext3D.{h,cpp}
│   │   └── SurfaceMeshQuality.{h,cpp}
│   ├── Volume/                          # Top-level volume mesher + initial tetrahedralization
│   │   ├── Delaunay3D.{h,cpp}
│   │   └── VolumeMesher3D.{h,cpp}
│   └── RCDT/                            # Ambient-space RCDT mesher — the only algorithm
│       │                                # behind ISurfaceMesher3D/IVolumeMesher3D today
│       ├── RCDTMesher.{h,cpp}                # the pipeline
│       ├── ProtectingBallPlacer.{h,cpp}      # protecting balls on corners and curves
│       ├── SurfaceDelaunayRefiner.{h,cpp}    # facet refinement
│       ├── TetrahedronDelaunayRefiner.{h,cpp} # tetrahedron refinement (volume)
│       ├── WeightedDualRestriction.{h,cpp}   # the restriction test
│       ├── SurfaceFacetCriteria.{h,cpp}, ProtectionExemption.{h,cpp}
│       ├── RegularConflictRegion.{h,cpp}
│       ├── SurfaceTessellation.{h,cpp}, TriangleSoupIndex.{h,cpp}, SurfaceProjector.{h,cpp}
│       ├── AmbientTetrahedronClassifier/Remover, RCDTMeshExtractor, RestrictedFaceAudit
│       ├── SurfaceMeshSmoother.{h,cpp}, TetrahedronInversionGuard.{h,cpp}
│       └── CurveSegmentBuilder.{h,cpp}, MinimumEdgeLengthEstimator.{h,cpp}
└── ConstraintStructures.h
```

## Key Components

### Contexts
- **MeshingContext2D** (`2D/`): Manages 2D geometry, topology, and mesh data; supports standalone or surface-based usage
- **MeshingContext3D** (`3D/General/`): Manages 3D geometry, topology, mesh data, and connectivity; `RCDTMesher` owns one per call

### 2D Algorithms
- **ConstrainedDelaunay2D**: Full 2D constrained Delaunay with dual-mode operation (context-based or standalone)
- **Delaunay2D**: Simple unconstrained 2D Delaunay triangulation
- **MeshOperations2D**: High-level operations (Bowyer-Watson insertion, cavity finding, edge enforcement)
- **MeshQueries2D**: Spatial queries on 2D meshes
- **ElementGeometry2D**: Geometric computations (circumcircles, orientations)
- **ElementQuality2D**: Quality metrics for triangle elements
- **EdgeDiscretizer2D**: Samples constraint edges into discrete points
- **BoundarySplitSynchronizer**: Keeps boundary splits consistent across surfaces
- **ShewchukRefiner2D**: Quality-driven refinement (Ruppert's algorithm) for 2D meshes
- **Shewchuk2DQualityController**: Quality controller for 2D refinement
- **ConstraintChecker2D**: Encroachment checking for constrained edges
- **MeshVerifier**: Validates mesh orientation and detects overlaps

### 3D General (shared infrastructure)
- **MeshOperations3D**: High-level operations (Bowyer-Watson insertion, cavity finding)
- **MeshQueries3D**: Spatial queries on 3D meshes
- **ElementGeometry3D**: Geometric computations for tetrahedral elements (circumspheres, volumes)
- **ElementQuality3D**: Quality metrics for tetrahedral elements
- **GeometryUtilities3D**: Pure geometric utilities (sphere tests, edge length, etc.)
- **BoundaryDiscretizer3D**: Samples boundary geometry into discrete points
- **ConstraintChecker3D**: Encroachment checking for constrained segments and facets
- **MeshVerifier3D**: Validates mesh integrity (degenerate elements, orphan nodes)
- **FacetDiscretization2DBuilder**: Builds UV-space discretizations of CAD facets; used by the legacy UV-space surface mesher
- **FacetTriangulation**: Triangulates a single CAD facet in UV space; used by the legacy UV-space surface mesher
- **FacetTriangulationManager**: Orchestrates UV-space triangulation across all CAD facets; used by the legacy UV-space surface mesher
- **TwinTableGenerator**: Builds twin (half-edge neighbor) tables for surface meshes

### 3D Surface (legacy UV-space mesher)
- **SurfaceMesher3D**: High-level API for the UV-space surface mesher; superseded by `RCDTMesher`
- **SurfaceMeshingContext3D**: Per-face UV-space triangulation context; superseded by `RCDTMesher`
- **SurfaceMeshQuality**: Quality controller for the legacy two-phase UV-space refinement

### 3D Volume
- **Delaunay3D**: Unconstrained 3D Delaunay tetrahedralization; used by `RCDTMesher` to build its initial ambient tetrahedralization
- **VolumeMesher3D**: Top-level, pluggable volume-mesh entry point (mirrors `SurfaceMesher3D`, no strategy enum — only `RCDTMesher` implements `IVolumeMesher3D` today); returns a `VolumeMesh3D`

### RCDT (ambient-space mesher)
CGAL Mesh_3's design (OPE-186); `doc/RCDT_Techniques.md` explains each piece.
- **RCDTMesher**: Implements both `ISurfaceMesher3D` and `IVolumeMesher3D` on one pipeline: seed → refine → check (volume) → remove ambient tetrahedra → extract → smooth. `meshSurface()` and `meshVolume()` differ in whether tetrahedra are refined and in the extraction step
- **ProtectingBallPlacer**: Places the weighted protecting balls on corners and curves (CGAL's `Protect_edges_sizing_field`), sized from the boundary discretization's spacing
- **SurfaceDelaunayRefiner**: Facet refinement (CGAL's `Refine_facets_3`): restricted facets that fail `SurfaceFacetCriteria` are refined worst first at their surface Delaunay ball centre
- **TetrahedronDelaunayRefiner**: Tetrahedron refinement for volumes (CGAL's `Refine_cells_3`), the level below the facet refiner: inside tetrahedra over the radius-edge bound are refined at their orthocentre, or the facet they encroach is refined instead
- **WeightedDualRestriction**: A face is restricted iff its weighted dual edge crosses a surface, found on `SurfaceTessellation` / `TriangleSoupIndex` and refined onto the CAD surface with `SurfaceProjector`
- **RestrictedFaceAudit**: Counts the edges whose restricted faces don't match the CAD topology; `meshVolume()` refuses a boundary with holes
- **AmbientTetrahedronClassifier / Remover**: Flood fill from the bounding tetrahedron across non-restricted faces; used to label inside tetrahedra and to strip the outside
- **RCDTMeshExtractor**, **SurfaceMeshSmoother**, **TetrahedronInversionGuard**: Output assembly and Laplacian smoothing that never inverts a tetrahedron
- **CurveSegmentBuilder**: Records the protected point chains along each CAD curve in `CurveSegmentManager`
- **SurfaceMesh3DQualitySettings** (`Meshing/Data/3D/`): Quality settings shared by RCDT and the UV-space mesher (minimum angle, chord deviation, tetrahedron radius-edge bound, insertion cap)

## Design Patterns

### Strategy Pattern
`IQualityController2D` interface with implementations (`Shewchuk2DQualityController`, `SurfaceMeshQualityController`) enables pluggable quality metrics for the 2D mesher and the legacy UV-space surface mesher. RCDT reads its bounds directly from `SurfaceMesh3DQualitySettings`. `SurfaceMesher3D`/`VolumeMesher3D` are backed by `ISurfaceMesher3D`/`IVolumeMesher3D` — the extensibility point for a future non-RCDT algorithm.

### Context Pattern
Contexts (`MeshingContext2D`, `MeshingContext3D`) centralize access to geometry, topology, and mutable mesh data with clear ownership semantics.

### Dual-Mode Design
`ConstrainedDelaunay2D` supports both context-based (integrated with topology) and standalone (raw coordinates) operation.

## Typical Workflow

### 2D Mesh Generation
1. Create `MeshingContext2D` (standalone or from surface)
2. Instantiate `ConstrainedDelaunay2D` with context
3. Call `generateConstrained()` to sample topology and build constraints
4. Bowyer-Watson insertion via `MeshOperations2D`
5. Constraint edge recovery
6. Results stored in `MeshData2D`

### 3D Surface Mesh Generation (RCDT)
1. Construct `SurfaceMesher3D` (or `RCDTMesher` directly) with geometry, topology, discretization settings, and quality settings
2. Call `mesher.mesh()` (`RCDTMesher::meshSurface()` under the hood), which runs:
   - **seed**: discretize the boundary, place the protecting balls, build the weighted `Delaunay3D` tetrahedralization and the `CurveSegmentManager`
   - **refine**: `SurfaceDelaunayRefiner` until no restricted facet is bad
   - **extract**: remove the ambient tetrahedra, assemble `SurfaceMesh3D` from the restricted facets, smooth
3. Result is a `SurfaceMesh3D` ready for export via `VtkExporter::writeSurfaceMesh`

### 3D Volume Mesh Generation (RCDT)
1. Construct `VolumeMesher3D` (or `RCDTMesher` directly) with geometry, topology, discretization settings, and quality settings
2. Call `mesher.mesh()` (`RCDTMesher::meshVolume()` under the hood): the same pipeline, with `TetrahedronDelaunayRefiner` driving the facet refiner, a closed-boundary check before the ambient tetrahedra are removed, and smoothing that never inverts a tetrahedron
3. Result is a `VolumeMesh3D` (tetrahedra + labeled boundary triangles, sharing one node array) ready for export via `VtkExporter::writeVolumeMesh`

## Access Patterns

- Access mesh data through contexts using `getMeshData()` and `getConnectivity()`
- Never cache raw pointers; always go through the context
- Use `MeshMutator2D/3D` for low-level mutations
- Rebuild connectivity after bulk operations: `context.rebuildConnectivity()`

## Key Algorithms

### Bowyer-Watson Incremental Insertion
1. Create super-element (super triangle for 2D, super tetrahedron for 3D)
2. For each point:
   - Find conflicting elements (those whose circumsphere/circle contains the point)
   - Extract cavity boundary
   - Retriangulate cavity by connecting point to boundary
3. Remove super-element and connected elements

### Constraint Recovery (2D)
- **Edges**: Find intersecting triangles, swap edges iteratively until the constraint edge appears

### RCDT — Restricted Surface Triangulation
Surface constraints are not explicitly recovered; they emerge from the restricted Delaunay triangulation:
- A face of the 3D tetrahedralization is **restricted** to surface S if its weighted dual edge — the segment between the orthocentres of its two tetrahedra — crosses S within its trimmed patch (`WeightedDualRestriction`)
- The set of restricted faces for surface S forms its surface triangulation without a separate constraint recovery pass
- Protecting balls around the curves (`ProtectingBallPlacer`) make every crease an edge chain of the regular triangulation; refinement keeps inserting surface points until the restricted facets meet the criteria

## References

- See `.claude/rules/cpp-coding-standards.md` for coding conventions
- See test files in `tests/Meshing/Core/` for usage examples
