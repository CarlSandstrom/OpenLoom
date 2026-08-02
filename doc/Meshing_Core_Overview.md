# Meshing Core Module Overview

**Last Updated:** 2026-08-02

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
│       ├── RCDTMesher.{h,cpp}
│       ├── RCDTContext.{h,cpp}
│       ├── RCDTRefiner.{h,cpp}
│       ├── RCDTTetQualityController.{h,cpp}
│       ├── RestrictedTriangulation.{h,cpp}
│       ├── SurfaceProjector.{h,cpp}
│       └── CurveSegmentOperations.{h,cpp}
└── ConstraintStructures.h
```

## Key Components

### Contexts
- **MeshingContext2D** (`2D/`): Manages 2D geometry, topology, and mesh data; supports standalone or surface-based usage
- **MeshingContext3D** (`3D/General/`): Manages 3D geometry, topology, mesh data, and connectivity; shared by the RCDT and volume meshers
- **RCDTContext** (`3D/RCDT/`): Orchestrates the three RCDT phases (`buildInitial` / `refine` / `buildSurfaceMesh`) in ambient 3D space

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
- **SurfaceMeshingContext3D**: Per-face UV-space triangulation context; superseded by `RCDTContext`
- **SurfaceMeshQuality**: Quality controller for the legacy two-phase UV-space refinement

### 3D Volume
- **Delaunay3D**: Unconstrained 3D Delaunay tetrahedralization; used by `RCDTMesher` to build its initial ambient tetrahedralization
- **VolumeMesher3D**: Top-level, pluggable volume-mesh entry point (mirrors `SurfaceMesher3D`, no strategy enum — only `RCDTMesher` implements `IVolumeMesher3D` today); returns a `VolumeMesh3D`

### RCDT (ambient-space mesher)
- **RCDTMesher**: Implements both `ISurfaceMesher3D` and `IVolumeMesher3D`. `meshSurface()` and `meshVolume()` share a build → refine → remove-supertet → smooth pipeline, differing only in their final extraction step and in whether tet-quality refinement (priority 3) runs
- **RCDTContext**: Orchestrates the phases: `buildInitial` (Delaunay + restriction), `refine` (quality refinement), `buildSurfaceMesh`/`buildVolumeMesh` (assembly)
- **RCDTRefiner**: Three-priority refinement loop in ambient 3D space: split encroached curve segments, then split bad restricted triangles, then (volume meshing only, via an injected `RCDTTetQualityController`) split bad tetrahedra
- **RCDTTetQualityController**: Tetrahedron circumradius-to-shortest-edge quality controller (`IQualityController3D`) driving priority 3; only constructed when `meshVolume()` is called, so `meshSurface()` never pays for volume-quality refinement it doesn't need
- **RestrictedTriangulation**: Identifies and incrementally maintains the set of Delaunay faces restricted to input surfaces; a face is restricted to surface S if all three nodes lie on S and the two adjacent tetrahedra are on opposite sides of S
- **SurfaceProjector**: Computes signed distances and surface-crossing tests; used by `RestrictedTriangulation` to classify tetrahedral faces
- **CurveSegmentOperations**: Populates `CurveSegmentManager` from topology edges and computes arc-length midpoints for segment splitting
- **SurfaceMesh3DQualitySettings** (`Meshing/Data/3D/`): Unified quality settings for both surface and volume RCDT refinement (circumradius-to-shortest-edge ratios, chord deviation, element limits)

## Design Patterns

### Strategy Pattern
`IQualityController2D` interface with implementations (`Shewchuk2DQualityController`, `SurfaceMeshQualityController`) enables pluggable quality metrics for the 2D mesher and the legacy UV-space surface mesher. RCDT's surface-triangle refinement reads bounds directly from `SurfaceMesh3DQualitySettings`; its tet-quality refinement goes through `IQualityController3D` (`RCDTTetQualityController`), the same interface `SurfaceMesher3D`/`VolumeMesher3D` are backed by via `ISurfaceMesher3D`/`IVolumeMesher3D` — the extensibility point for a future non-RCDT algorithm.

### Context Pattern
Contexts (`MeshingContext2D`, `MeshingContext3D`, `RCDTContext`) centralize access to geometry, topology, and mutable mesh data with clear ownership semantics.

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
   - **buildInitial**: Discretize boundary geometry; insert all vertices into `Delaunay3D`; build `RestrictedTriangulation` and `CurveSegmentManager`
   - **refine**: Two-priority loop via `RCDTRefiner`: split encroached curve segments first, then split restricted triangles that fail `SurfaceMesh3DQualitySettings`
   - **buildSurfaceMesh**: Assemble `SurfaceMesh3D` from the restricted faces
3. Result is a `SurfaceMesh3D` ready for export via `VtkExporter::writeSurfaceMesh`

### 3D Volume Mesh Generation (RCDT)
1. Construct `VolumeMesher3D` (or `RCDTMesher` directly) with geometry, topology, discretization settings, and quality settings
2. Call `mesher.mesh()` (`RCDTMesher::meshVolume()` under the hood), which runs the same `buildInitial` → `refine` pipeline as surface meshing, except `refine` now also runs **priority 3** (split skinny tetrahedra, via `RCDTTetQualityController`) before removing the bounding supertet and smoothing
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
- A face of the 3D tetrahedralization is **restricted** to surface S if all three of its nodes lie on S (or on edges/corners adjacent to S) and the two adjacent tetrahedra are on opposite sides of S (`SurfaceProjector.crossesSurface`)
- The set of restricted faces for surface S forms its surface triangulation without a separate constraint recovery pass
- Curve segments along topology edges are maintained in `CurveSegmentManager` and split when encroached, driving the restricted triangulation to converge to the input geometry

## References

- See `.claude/rules/cpp-coding-standards.md` for coding conventions
- See test files in `tests/Meshing/Core/` for usage examples
