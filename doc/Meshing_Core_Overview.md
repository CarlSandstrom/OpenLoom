# Meshing Core Module Overview

**Last Updated:** 2026-02-28

## Purpose

The `src/Meshing/Core` module implements constrained Delaunay triangulation for both 2D and 3D meshes using the Bowyer-Watson incremental insertion algorithm with constraint recovery.

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
│   │   ├── ConstraintRegistrar3D.{h,cpp}
│   │   ├── EdgeTwinTable.h
│   │   └── MeshDebugUtils3D.{h,cpp}
│   ├── Surface/                         # Surface (triangle) mesh generation
│   │   ├── SurfaceMeshingContext3D.{h,cpp}
│   │   ├── SurfaceMesh3D.h
│   │   ├── FacetTriangulation.{h,cpp}
│   │   ├── FacetTriangulationManager.{h,cpp}
│   │   └── TwinTableGenerator.{h,cpp}
│   └── Volume/                          # Volume (tetrahedral) mesh generation
│       ├── ConstrainedDelaunay3D.{h,cpp}
│       ├── Delaunay3D.{h,cpp}
│       ├── ShewchukRefiner3D.{h,cpp}
│       └── Shewchuk3DQualityController.{h,cpp}
└── ConstraintStructures.h
```

## Key Components

### Contexts
- **MeshingContext2D** (`2D/`): Manages 2D geometry, topology, and mesh data; supports standalone or surface-based usage
- **MeshingContext3D** (`3D/General/`): Manages 3D geometry, topology, mesh data, and connectivity for volume meshing
- **SurfaceMeshingContext3D** (`3D/Surface/`): Context for surface (triangle) mesh generation on 3D faces

### 2D Algorithms
- **ConstrainedDelaunay2D**: Full 2D constrained Delaunay with dual-mode operation (context-based or standalone)
- **Delaunay2D**: Simple unconstrained 2D Delaunay triangulation
- **MeshOperations2D**: High-level operations (Bowyer-Watson insertion, cavity finding, edge enforcement)
- **MeshQueries2D**: Spatial queries on 2D meshes
- **ElementGeometry2D**: Geometric computations (circumcircles, orientations)
- **ElementQuality2D**: Quality metrics for triangle elements
- **EdgeDiscretizer2D**: Samples constraint edges into discrete points
- **BoundarySplitSynchronizer**: Keeps boundary splits consistent across surfaces
- **ShewchukRefiner2D**: Quality-driven refinement (Shewchuk's algorithm)
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
- **ConstraintRegistrar3D**: Registers constraint segments and facets into the mesh context
- **MeshVerifier3D**: Validates mesh integrity (degenerate elements, orphan nodes)

### 3D Surface
- **SurfaceMeshingContext3D**: Context for triangulating 3D surfaces; owns surface mesh data
- **FacetTriangulation**: Triangulates a single CAD facet in UV space
- **FacetTriangulationManager**: Orchestrates triangulation across all facets
- **TwinTableGenerator**: Builds twin (half-edge neighbor) tables for surface meshes

### 3D Volume
- **ConstrainedDelaunay3D**: Full 3D constrained Delaunay tetrahedralization
- **Delaunay3D**: Unconstrained 3D Delaunay tetrahedralization
- **ShewchukRefiner3D**: Quality-driven refinement (Shewchuk's 3D algorithm)
- **Shewchuk3DQualityController**: Quality controller implementing Shewchuk's criteria

## Design Patterns

### Strategy Pattern
`IQualityController` interface with implementations (`Shewchuk2DQualityController`, `Shewchuk3DQualityController`) enables pluggable quality metrics.

### Context Pattern
Contexts (`MeshingContext2D`, `MeshingContext3D`, `SurfaceMeshingContext3D`) centralize access to geometry, topology, and mutable mesh data with clear ownership semantics.

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

### 3D Surface Mesh Generation
1. Create `SurfaceMeshingContext3D` from CAD geometry
2. Use `FacetTriangulationManager` to triangulate each CAD face
3. Build twin table via `TwinTableGenerator`
4. Final mesh accessible via `SurfaceMeshingContext3D`

### 3D Volume Mesh Generation
1. Create `MeshingContext3D` with geometry and topology
2. Initialize Delaunay with bounding tetrahedron via `MeshOperations3D`
3. Insert vertices using Bowyer-Watson algorithm
4. Register constraint subsegments and subfacets via `ConstraintRegistrar3D`
5. Use `ShewchukRefiner3D` for quality-driven refinement
6. Final mesh accessible via `context.getMeshData()`

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

### Constraint Recovery
- **Segments (3D edges)**: Find intersecting tetrahedra, extract cavity, retriangulate with constraint
- **Facets (3D surfaces)**: Find intersecting tetrahedra, extract cavity, retriangulate with facet triangulation
- **Edges (2D)**: Find intersecting triangles, swap edges iteratively until constraint appears

## References

- See `.claude/rules/cpp-coding-standards.md` for coding conventions
- See test files in `tests/Meshing/Core/` for usage examples
