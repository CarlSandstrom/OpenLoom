# Meshing Library Design Document

## Overview

A C++ meshing library using OpenCascade 7.9.2 for geometry handling. The library supports mesh generation with initial focus on unstructured tetrahedral meshes, designed with scalability in mind for future element types and structured mesh support.

## Design Philosophy

- **Separation of concerns**: Clear boundaries between topology, geometry, and mesh data
- **Scalability**: Abstract interfaces to support multiple geometry backends and element types
- **Maintainability**: Clean code structure suitable for a private project with limited development time
- **Initial scope**: Unstructured tetrahedral meshes
- **Future-proof**: Design choices that accommodate expansion to other element types

---

## Meshing Process

The mesh generation process is divided into four main stages:

### 1. Read Source Information
- Accept OpenCascade TopoDS_Shape object as input
- User is responsible for loading CAD files (STEP, IGES, etc.) into TopoDS_Shape
- Extract geometric entities from the shape

### 2. Transform to Topology and Geometry Objects

**Topology:**
- Contains information on how surfaces, edges, and corners are connected
- Represents the connectivity and relationships between geometric entities
- Immutable after creation (generated on reading, not changed during meshing)

**Geometry:**
- Contains geometry objects with methods to calculate:
  - Points on surfaces
  - Normal vectors
  - Curvature information
  - Distance calculations
  - Point projections
- Abstract base classes for surfaces, edges, and corners
- Concrete implementations for OpenCascade geometry types
- Designed for extensibility to support additional geometry kernels in the future

**Entity Identification:**
- Each entity (surface, edge, corner) has a unique identifier (typically a string)
- Identifiers map entities between topology and geometry representations
- Enables tracking of boundary conditions and physical groups

### 3. Perform Meshing Operations
- Generate tetrahedral mesh using selected algorithms
- Apply mesh quality improvements
- Handle size functions and refinement criteria

### 4. Export to Mesh Format
- Write mesh to various file formats
- Support for visualization formats (VTK)
- Support for solver formats (Gmsh, etc.)

---

## File Structure

```
meshing_library/
├── doc/
│   ├── coding_standards.md
│   ├── architecture.md
│   └── algorithms.md
│
├── examples/
│   └── basic_tetrahedral_mesh/
│
├── tests/
│   ├── unit/
│   └── integration/
│
├── cmake/
│   └── modules/
│
├── tools/
│
└── src/
    ├── Common/
    │   ├── Logger
    │   ├── Exceptions
    │   └── Types
    │
    ├── Readers/
    │   └── ShapeReader
    │
    ├── Topology/
    │   ├── Data
    │   └── Queries
    │
    ├── Geometry/
    │   ├── Base
    │   ├── OpenCascade
    │   ├── Data
    │   ├── Queries
    │   └── Healing
    │
    ├── Mesh/
    │   ├── Data/
    │   │   ├── Core
    │   │   ├── Connectivity
    │   │   └── Elements
    │   │
    │   ├── Sizing/
    │   │
    │   ├── Generators/
    │   │   ├── Surface
    │   │   └── Volume
    │   │
    │   ├── Operations/
    │   │   ├── EdgeSwap
    │   │   ├── NodeInsertion
    │   │   ├── EdgeCollapse
    │   │   └── Smoothing
    │   │
    │   ├── Quality/
    │   ├── Validation/
    │   └── Refinement/
    │
    └── Exporters/
        ├── Base
        ├── VTK
        ├── Gmsh
        └── Native
```

### Directory Descriptions

**doc/**
- Documentation including coding standards, architecture overview, and algorithm descriptions
- Design decisions and rationale
- User guides and API documentation

**examples/**
- Example usage and test cases
- Demonstration of basic workflows

**tests/**
- Unit tests for individual components
- Integration tests for complete workflows

**cmake/**
- CMake modules and find scripts
- Dependency management (OpenCascade, VTK, etc.)

**tools/**
- Command-line utilities
- Helper scripts for development

**src/Common/**
- Shared utilities across all modules
- Logging system for debugging
- Exception handling
- Common type definitions

**src/Readers/**
- ShapeReader: Takes OpenCascade TopoDS_Shape as input
- Extracts topology and geometry from the shape
- Generates topology and geometry data with proper mapping
- Note: User is responsible for loading STEP/IGES files into TopoDS_Shape

**src/Topology/**
- Data structures for topological information
- Immutable after creation
- Query operations for topology relationships

**src/Geometry/**
- Base: Abstract geometry classes (Surface, Edge, Corner)
- OpenCascade: OCC-specific implementations
- Data: Geometric data structures
- Queries: Point projection, distance calculations, parametric operations
- Healing: Geometry repair and cleanup utilities

**src/Mesh/Data/**
- Core: Basic mesh data structures (nodes, elements)
- Connectivity: Element-to-node, node-to-element, face-to-element relationships
- Elements: Tetrahedral element definitions (designed to support other types later)

**src/Mesh/Sizing/**
- Size functions controlling element sizes throughout domain
- Uniform, proximity-based, curvature-based sizing
- User-defined size fields

**src/Mesh/Generators/**
- Surface: 2D surface meshing algorithms (triangulation)
- Volume: 3D volume meshing algorithms (tetrahedral generation)
- Initially: Delaunay-based or advancing front methods

**src/Mesh/Operations/**
- Local mesh modification operations
- Edge swapping for quality improvement
- Node insertion and deletion
- Edge collapse operations
- Smoothing algorithms (Laplacian, optimization-based)

**src/Mesh/Quality/**
- Quality metrics for tetrahedral elements
  - Aspect ratio
  - Skewness
  - Jacobian determinant
  - Dihedral angles
- Mesh statistics and reporting

**src/Mesh/Validation/**
- Mesh validity checking
- Detection of inverted elements
- Topology consistency verification
- Overlap detection

**src/Mesh/Refinement/**
- Adaptive mesh refinement capabilities
- Element subdivision for tetrahedra
- Coarsening algorithms (future)

**src/Exporters/**
- Base: Abstract exporter class
- VTK: VTK/VTU format for ParaView visualization
- Gmsh: .msh format
- Native: Fast save/load format for library

---

## Key Design Decisions

### 1. Abstraction for Scalability

**Geometry Backend:**
```cpp
// Abstract base class
class Surface {
public:
    virtual Point3D evaluatePoint(double u, double v) const = 0;
    virtual Vector3D evaluateNormal(double u, double v) const = 0;
    virtual double evaluateCurvature(double u, double v) const = 0;
    // ...
};

// OpenCascade implementation
class OCCSurface : public Surface {
    // OCC-specific implementation
};
```

**Element Types:**
```cpp
// Abstract element class
class MeshElement {
public:
    virtual ElementType getType() const = 0;
    virtual int getNodeCount() const = 0;
    virtual double computeQuality() const = 0;
    // ...
};

// Tetrahedral implementation
class TetrahedralElement : public MeshElement {
    // Tet-specific implementation
};

// Future: HexahedralElement, PrismaticElement, etc.
```

### 2. Immutable Topology and Geometry

- Topology and geometry data are created during the reading phase
- These structures remain unchanged throughout the meshing process
- Mesh data is mutable and evolves during generation and optimization
- Clear separation prevents accidental modification of input data

### 3. Entity Identification System

- String-based identifiers for surfaces, edges, and corners
- Enables mapping between:
  - CAD entities and mesh entities
  - Boundary conditions and mesh faces
  - Physical groups for solver applications

### 4. Memory and Performance Considerations

- Spatial data structures (octrees/kd-trees) for efficient nearest-neighbor searches
- Smart pointer strategy for ownership management
- Design with future parallelization in mind (thread-safe data structures)

### 5. Connectivity Storage

For tetrahedral meshes:
- Element-to-node connectivity (4 nodes per tet)
- Node-to-element connectivity (for local operations)
- Face-to-element connectivity (for boundary tracking)
- Efficient storage using contiguous arrays where possible

---

## Initial Scope: Tetrahedral Meshes

### Supported Features (Version 1.0)

**Mesh Generation:**
- Unstructured tetrahedral mesh generation
- Surface triangulation as prerequisite
- Volume meshing from surface mesh

**Quality Control:**
- Quality metrics specific to tetrahedra
- Element quality-based refinement
- Basic smoothing operations

**Input:**
- OpenCascade TopoDS_Shape object
- User handles file I/O (STEP, IGES, etc.) using OpenCascade tools

**Output Formats:**
- VTK/VTU for visualization in ParaView
- Gmsh .msh format
- Native binary format

**Meshing Algorithms:**
- Delaunay-based tetrahedral generation OR
- Advancing front method
- (Decision to be made during implementation)

### Explicitly Out of Scope (Version 1.0)

- Structured meshes
- Hexahedral elements
- Prismatic/pyramidal elements
- Hybrid meshes
- Parallel mesh generation
- Distributed memory support
- Boundary layer generation
- Anisotropic mesh adaptation

---

## Critical Design Questions

### Answered for V1.0

1. **Element types?** Tetrahedra only, but designed for future expansion
2. **Meshing algorithm?** Delaunay or advancing front (TBD during implementation)
3. **Quality metrics?** Standard tetrahedral metrics (aspect ratio, volume, dihedral angles)
4. **File formats?** VTK for visualization, Gmsh for compatibility

### To Be Decided During Implementation

1. **Mesh entity IDs:** Global vs local numbering scheme
2. **Size function implementation:** How to specify and interpolate size fields
3. **Refinement strategy:** Subdivision patterns for tetrahedra
4. **Memory vs speed tradeoff:** Level of adjacency information to store
5. **Error handling:** Exceptions vs error codes approach

---

## Dependencies

### Required

- **OpenCascade 7.9.2**: Geometry kernel
- **C++17 or later**: Modern C++ features
- **CMake 3.15+**: Build system

### Optional

- **VTK**: For advanced visualization export
- **Google Test**: For unit testing
- **Doxygen**: For documentation generation

---

## TODO List

### Phase 1: Foundation (Core Infrastructure)

**Common Module:**
- [ ] Set up CMake build system with OpenCascade integration
- [ ] Implement basic logging system (log levels, file output)
- [ ] Define exception hierarchy for error handling
- [ ] Create common type definitions (Point3D, Vector3D, etc.)

**Documentation:**
- [ ] Write coding standards document
- [ ] Document architecture decisions
- [ ] Set up Doxygen configuration

**Testing Infrastructure:**
- [ ] Set up Google Test framework
- [ ] Create test utilities for generating simple TopoDS_Shapes
- [ ] Write example unit test structure

### Phase 2: Geometry and Topology (Input Processing)

**Readers:**
- [ ] Implement ShapeReader class
  - [ ] Accept TopoDS_Shape as input
  - [ ] Extract faces, edges, vertices from shape
  - [ ] Handle topology extraction using OCC topology iterators
  - [ ] Generate unique identifiers for entities
- [ ] Write unit tests with simple shapes (box, cylinder)

**Topology:**
- [ ] Design topology data structure
  - [ ] Surface connectivity information
  - [ ] Edge connectivity information
  - [ ] Corner (vertex) information
- [ ] Implement topology queries (adjacent surfaces, boundary edges, etc.)
- [ ] Write unit tests for topology operations

**Geometry:**
- [ ] Implement abstract geometry base classes (Surface, Edge, Corner)
- [ ] Implement OpenCascade geometry wrappers
  - [ ] OCCSurface (point evaluation, normal, curvature)
  - [ ] OCCEdge (point evaluation, tangent, length)
  - [ ] OCCCorner (point location)
- [ ] Implement geometry query operations
  - [ ] Point projection onto surfaces/edges
  - [ ] Distance calculations
  - [ ] Closest point queries
- [ ] Write unit tests for geometry operations

**Integration Test:**
- [ ] Create simple TopoDS_Shape (box/cylinder) and verify topology/geometry extraction

### Phase 3: Surface Meshing (2D Triangulation)

**Mesh Data Structures:**
- [ ] Implement Node class (3D coordinates, identifier, boundary flag)
- [ ] Implement Triangle class (3 node indices)
- [ ] Implement surface mesh connectivity (node-to-triangle, triangle-to-node)
- [ ] Write unit tests for data structures

**Surface Mesh Generator:**
- [ ] Research and select algorithm (Delaunay 2D, advancing front, or use existing library)
- [ ] Implement edge discretization
  - [ ] Uniform spacing
  - [ ] Size function-based spacing
- [ ] Implement surface triangulation
- [ ] Handle multiple surfaces with shared edges
- [ ] Write unit tests and integration tests

**Mesh Sizing:**
- [ ] Implement uniform size function
- [ ] Implement curvature-based size function
- [ ] Design interface for custom size functions

### Phase 4: Volume Meshing (3D Tetrahedralization)

**Mesh Data Structures:**
- [ ] Implement TetrahedralElement class (4 node indices)
- [ ] Implement volume mesh connectivity
  - [ ] Element-to-node
  - [ ] Node-to-element (for operations)
  - [ ] Face-to-element (for boundary tracking)
- [ ] Write unit tests

**Volume Mesh Generator:**
- [ ] Research and select algorithm (Delaunay 3D, advancing front, or TetGen wrapper)
- [ ] Implement tetrahedral mesh generation from surface mesh
- [ ] Handle interior points insertion based on size function
- [ ] Write unit tests with simple geometries

**Integration Test:**
- [ ] Generate complete tetrahedral mesh from STEP file

### Phase 5: Mesh Quality and Operations

**Quality Metrics:**
- [ ] Implement aspect ratio calculation for tetrahedra
- [ ] Implement volume/Jacobian calculation
- [ ] Implement dihedral angle calculation
- [ ] Implement quality statistics (min, max, average, histogram)
- [ ] Write unit tests

**Validation:**
- [ ] Implement inverted element detection
- [ ] Implement topology consistency checks
- [ ] Implement boundary face verification
- [ ] Write unit tests

**Basic Operations:**
- [ ] Implement edge swap operation
- [ ] Implement node insertion
- [ ] Implement Laplacian smoothing
- [ ] Write unit tests for each operation

**Quality Improvement:**
- [ ] Implement iterative quality improvement loop
  - [ ] Identify poor quality elements
  - [ ] Apply appropriate operations
  - [ ] Re-evaluate quality
- [ ] Define quality thresholds and stopping criteria

### Phase 6: Export (Visualization and Output)

**Exporters:**
- [ ] Implement base Exporter abstract class
- [ ] Implement VTK/VTU exporter
  - [ ] Node coordinates
  - [ ] Element connectivity
  - [ ] Boundary face marking
  - [ ] Cell data (quality metrics)
- [ ] Implement Gmsh .msh exporter (version 4.1 format)
- [ ] Implement simple native format (for testing/debugging)
- [ ] Write unit tests (validate output files)

**Verification:**
- [ ] Test VTK output in ParaView
- [ ] Test Gmsh output in Gmsh viewer

### Phase 7: Integration and Examples

**End-to-End Pipeline:**
- [ ] Create simple command-line tool or example program
  - [ ] Load STEP file into TopoDS_Shape (using OCC)
  - [ ] Pass shape to meshing library
  - [ ] Configure mesh parameters
  - [ ] Output: Mesh files
- [ ] Test complete workflow with various geometries

**Examples:**
- [ ] Create example: Simple cube
- [ ] Create example: Cylinder
- [ ] Create example: More complex CAD part
- [ ] Document each example with README

**Documentation:**
- [ ] Write user guide
- [ ] Document API with Doxygen
- [ ] Create tutorial for basic usage

### Phase 8: Refinement and Polish

**Mesh Refinement:**
- [ ] Implement tetrahedral subdivision (1-to-8 or edge-based)
- [ ] Implement adaptive refinement based on size function
- [ ] Handle hanging nodes or ensure conforming mesh
- [ ] Write unit tests

**Advanced Sizing:**
- [ ] Implement proximity-based sizing
- [ ] Implement gradation control (smooth size transitions)
- [ ] Allow user-defined point sources for local refinement

**Performance:**
- [ ] Profile code and identify bottlenecks
- [ ] Optimize critical sections
- [ ] Add progress reporting for long operations

**Error Handling:**
- [ ] Review and improve error messages
- [ ] Add input validation
- [ ] Handle edge cases gracefully

### Future Enhancements (Post V1.0)

**Not in immediate scope, but design should accommodate:**

- [ ] Additional element types (hex, prism, pyramid)
- [ ] Structured mesh support
- [ ] Boundary layer mesh generation
- [ ] Parallel mesh generation
- [ ] Anisotropic mesh adaptation
- [ ] Other geometry kernels beyond OpenCascade
- [ ] Additional file format support (Exodus, CGNS)
- [ ] Mesh partitioning for parallel solvers
- [ ] Remeshing capabilities
- [ ] Moving mesh support

---

## Development Workflow

### Recommended Order

1. Start with **Phase 1** to establish solid foundation
2. Proceed to **Phase 2** to handle geometry input
3. Implement **Phase 3** for surface meshing (validates geometry handling)
4. Move to **Phase 4** for volume meshing (core functionality)
5. Add **Phase 5** for mesh quality (essential for usable meshes)
6. Implement **Phase 6** for export (ability to visualize results)
7. Complete **Phase 7** for integration and usability
8. Refine with **Phase 8** based on experience from earlier phases

### Testing Strategy

- Write unit tests for each component as it's developed
- Create integration tests after completing each phase
- Use simple geometric shapes for initial testing
- Progress to complex geometries as confidence builds
- Validate visual output in ParaView regularly

### Version Control Milestones

- **v0.1**: Foundation and infrastructure
- **v0.2**: Geometry and topology handling
- **v0.3**: Surface meshing working
- **v0.4**: Volume meshing working
- **v0.5**: Quality metrics and basic operations
- **v0.6**: Export functionality
- **v0.7**: Integration and examples
- **v1.0**: Refinement, documentation, and polish

---

## Notes and Considerations

### Time Management for Private Project

- Focus on getting a minimum viable product working first
- Tetrahedral meshing is sufficient for many applications
- Abstract interfaces allow future expansion without major rewrites
- Don't over-engineer initially; refactor as needs become clear
- Use existing libraries where sensible (e.g., TetGen for tetrahedralization if needed)

### Key Success Criteria for V1.0

1. Successfully read STEP/IGES files
2. Generate quality tetrahedral meshes
3. Export to VTK for visualization
4. Reasonable performance for moderate-sized models (10K-100K elements)
5. Clean, maintainable code structure
6. Basic documentation and examples

### Design Principles to Maintain

- **Simplicity**: Keep interfaces simple and intuitive
- **Modularity**: Each component should be independently testable
- **Extensibility**: Design for future expansion (other element types)
- **Performance**: Profile before optimizing; clarity first, then speed
- **Documentation**: Code should be self-documenting; comments for "why" not "what"

---

## References and Resources

### Meshing Algorithms

- Delaunay triangulation and tetrahedralization theory
- Advancing front methods
- Quality metrics for tetrahedral elements

### Libraries and Tools

- OpenCascade documentation: https://dev.opencascade.org/
- VTK file formats: https://vtk.org/wp-content/uploads/2015/04/file-formats.pdf
- Gmsh file format: https://gmsh.info/doc/texinfo/gmsh.html#File-formats
- TetGen (potential dependency): http://tetgen.org/

### Books

- "Mesh Generation" by Pascal Frey and Paul-Louis George
- "Computational Geometry: Algorithms and Applications" by de Berg et al.

---

**Document Version:** 1.0  
**Last Updated:** 2025-10-22  
**Author:** Carl Sandström
**Project Status:** Design Phase