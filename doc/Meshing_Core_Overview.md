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
│   ├── ConstrainedDelaunay3D.{h,cpp}
│   ├── ConstrainedMesher.{h,cpp}
│   ├── SimpleMesher.{h,cpp}
│   ├── Computer3D.{h,cpp}
│   ├── QualityComputer.{h,cpp}
│   └── ConstrainedDelaunayHelper.h
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
- **ConstrainedDelaunay3D**: Full 3D constrained Delaunay with segment and facet recovery
- **ConstrainedMesher**: High-level wrapper implementing IMesher interface
- **SimpleMesher**: Basic placeholder mesher for testing
- **Computer3D**: Geometric computations for 3D (circumspheres, volumes, quality)
- **QualityComputer**: Quality metrics for tetrahedral elements
- **ConstrainedDelaunayHelper**: Static utilities for constraint recovery

### Data Structures
- **ConstraintStructures.h**: Defines constraint segments and facets from CAD geometry

## Design Patterns

### Strategy Pattern
IMesher interface with multiple implementations (ConstrainedMesher, SimpleMesher) enables pluggable meshing algorithms.

### Context Pattern
MeshingContext2D/3D centralize access to geometry, topology, and mutable mesh data with clear ownership semantics.

### Dual-Mode Design
ConstrainedDelaunay2D supports both context-based (integrated with topology) and standalone (raw coordinates) operation.

## Typical Workflow

### 3D Mesh Generation
1. Create `MeshingContext3D` with geometry and topology
2. Instantiate mesher (e.g., `ConstrainedMesher`)
3. Call `mesher.generate(context)`
4. Mesher extracts constraints from topology
5. For each surface, creates `MeshingContext2D` for parametric triangulation
6. `ConstrainedDelaunay2D` triangulates surface
7. Surface triangles become constraint facets in 3D mesh
8. Bowyer-Watson insertion of remaining points
9. Constraint recovery forces segments and facets
10. Final mesh accessible via `context.getMeshData()`

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
