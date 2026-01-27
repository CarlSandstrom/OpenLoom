# Meshing Core Module Overview

**Last Updated:** 2025-12-26

## Purpose

The `src/Meshing/Core` module implements constrained Delaunay triangulation for both 2D and 3D meshes using the Bowyer-Watson incremental insertion algorithm with constraint recovery.

## Directory Organization

```
src/Meshing/Core/
├── 2D/                      # 2D meshing algorithms
│   ├── ConstrainedDelaunay2D.{h,cpp}
│   ├── Delaunay2D.{h,cpp}
│   ├── Computer2D.{h,cpp}
│   ├── MeshOperations2D.{h,cpp}
│   └── MeshVerifier.{h,cpp}
├── 3D/                      # 3D meshing algorithms
│   ├── MeshingContext3D.{h,cpp}
│   ├── MeshOperations3D.{h,cpp}
│   ├── ShewchukRefiner3D.{h,cpp}
│   ├── Shewchuk3DQualityController.{h,cpp}
│   ├── ElementGeometry3D.{h,cpp}
│   ├── ElementQuality3D.{h,cpp}
│   ├── ConstraintChecker3D.{h,cpp}
│   ├── GeometryUtilities3D.{h,cpp}
│   ├── GeometryStructures3D.h
│   ├── MeshVerifier3D.{h,cpp}
│   └── QualityComputer.{h,cpp}
├── Interfaces/              # Abstract interfaces
│   ├── IMesher.h
│   └── IQualityController.h
├── MeshingContext2D.{h,cpp}
├── MeshingContext3D.{h,cpp}
└── ConstraintStructures.h
```

## Key Components

### Contexts
- **MeshingContext2D**: Manages 2D geometry, topology, and mesh data; supports standalone or surface-based usage
- **MeshingContext3D**: Manages 3D geometry, topology, mesh data, and connectivity

### Interfaces
- **IMesher**: Abstract interface for mesh generation strategies
- **IQualityController**: Abstract interface for mesh quality evaluation

### 2D Algorithms
- **ConstrainedDelaunay2D**: Full 2D constrained Delaunay with dual-mode operation (context-based or standalone)
- **Delaunay2D**: Simple unconstrained 2D Delaunay triangulation
- **MeshOperations2D**: High-level operations (Bowyer-Watson insertion, cavity finding, edge enforcement)
- **Computer2D**: Geometric computations for 2D (circumcircles, orientations, quality)
- **MeshVerifier**: Validates mesh orientation and detects overlaps

### 3D Algorithms
- **MeshOperations3D**: High-level operations (Bowyer-Watson insertion, cavity finding, constraint operations)
- **ShewchukRefiner3D**: Implements Shewchuk's 3D Delaunay refinement algorithm
- **Shewchuk3DQualityController**: Quality controller implementing Shewchuk's criteria
- **ElementGeometry3D**: Geometric computations for tetrahedral elements (circumspheres, volumes)
- **ElementQuality3D**: Quality metrics for tetrahedral elements
- **ConstraintChecker3D**: Encroachment checking for constrained segments and facets
- **GeometryUtilities3D**: Pure geometric utilities (sphere tests, edge length, etc.)
- **MeshVerifier3D**: Validates mesh integrity (degenerate elements, orphan nodes)
- **QualityComputer**: Helper functions for quality computations

### Data Structures
- **ConstraintStructures.h**: Defines constraint segments and facets from CAD geometry

## Design Patterns

### Strategy Pattern
IQualityController interface with implementations (Shewchuk2DQualityController, Shewchuk3DQualityController) enables pluggable quality metrics.

### Context Pattern
MeshingContext2D/3D centralize access to geometry, topology, and mutable mesh data with clear ownership semantics.

### Dual-Mode Design
ConstrainedDelaunay2D supports both context-based (integrated with topology) and standalone (raw coordinates) operation.

## Typical Workflow

### 3D Mesh Generation
1. Create `MeshingContext3D` with geometry and topology
2. Initialize Delaunay with bounding tetrahedron via `MeshOperations3D`
3. Insert vertices using Bowyer-Watson algorithm
4. Add constraint subsegments and subfacets to context
5. Use `ShewchukRefiner3D` for quality-driven refinement
6. Final mesh accessible via `context.getMeshData()`

### 2D Mesh Generation
1. Create `MeshingContext2D` (standalone or from surface)
2. Instantiate `ConstrainedDelaunay2D` with context
3. Call `generateConstrained()` to sample topology and build constraints
4. Bowyer-Watson insertion via `MeshOperations2D`
5. Constraint edge recovery
6. Results stored in `MeshData2D`

## Access Patterns

- Access mesh data through contexts using `getMeshData()` and `getConnectivity()`
- Never cache raw pointers; always go through the context
- Use `MeshMutator2D/3D` for low-level mutations
- Use `MeshOperations2D` for high-level 2D algorithms
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

## Future Considerations

- Fully integrate quality-driven refinement using IQualityController
- Consider merging Computer and ElementGeometry to reduce duplication
- Optimize surface triangulation context creation/pooling
- Clarify naming between MeshOperations and MeshMutator

## References

- See `doc/Coding standards.md` for coding conventions
- See test files in `tests/Meshing/Core/` for usage examples
- Consult original geometry papers for algorithm details
