# src/Meshing/Core Directory Overview

**Generated:** 2025-12-19
**Purpose:** Comprehensive documentation of all files, classes, and methods to facilitate refactoring

---

## Summary

The `src/Meshing/Core` directory contains **22 files** (11 header files and 11 implementation files) that implement the core mesh generation infrastructure for the cMesh project. The primary responsibility is providing Constrained Delaunay Triangulation in both 2D and 3D, along with supporting geometric computations, quality control interfaces, and mesh context management.

**Key Capabilities:**
- 3D Constrained Delaunay Triangulation respecting CAD topology
- 2D Constrained Delaunay Triangulation in parametric space
- Bowyer-Watson incremental insertion algorithm
- Constraint recovery (segments and facets)
- Mesh quality evaluation and geometric computations
- Context-based mesh management and operations

---

## File Listing

### Context Management (4 files)
1. `MeshingContext3D.h` / `MeshingContext3D.cpp`
2. `MeshingContext2D.h` / `MeshingContext2D.cpp`

### Abstract Interfaces (2 files)
3. `IMesher.h`
4. `IQualityController.h`

### Mesher Implementations (4 files)
5. `ConstrainedMesher.h` / `ConstrainedMesher.cpp`
6. `SimpleMesher.h` / `SimpleMesher.cpp`

### Constrained Delaunay Algorithms (5 files)
7. `ConstrainedDelaunay3D.h` / `ConstrainedDelaunay3D.cpp`
8. `ConstrainedDelaunay2D.h` / `ConstrainedDelaunay2D.cpp`
9. `ConstrainedDelaunayHelper.h`

### Operations and Geometry (6 files)
10. `MeshOperations2D.h` / `MeshOperations2D.cpp`
11. `Computer.h` / `Computer.cpp`
12. `ElementGeometry.h` / `ElementGeometry.cpp`

### Data Structures (1 file)
13. `ConstraintStructures.h`

---

## Detailed Class Documentation

### 1. MeshingContext3D

**Files:** `MeshingContext3D.h`, `MeshingContext3D.cpp`

**Purpose:** Central orchestrator for 3D meshing operations. Holds references to immutable geometry and topology, and owns mutable mesh data, connectivity, and operations.

**Class Declaration:**
```cpp
class MeshingContext3D
```

**Public Interface:**

*Constructor & Destructor:*
- `MeshingContext3D(const GeometryCollection3D& geometry, const Topology3D& topology)`
- `~MeshingContext3D()`

*Access Methods:*
- `const GeometryCollection3D& getGeometry() const`
- `const Topology3D& getTopology() const`
- `MeshData& getMeshData()`
- `MeshConnectivity& getConnectivity()`
- `MeshMutator3D& getOperations()`

*Mesh Management:*
- `void rebuildConnectivity()` - Rebuild connectivity after large structural changes
- `void clearMesh()` - Clear all mesh data and connectivity

**Private Members:**
- `const GeometryCollection3D& geometry_` - Reference to geometry
- `const Topology3D& topology_` - Reference to topology
- `std::unique_ptr<MeshData> meshData_` - Owned mesh data
- `std::unique_ptr<MeshConnectivity> connectivity_` - Owned connectivity
- `std::unique_ptr<MeshMutator3D> operations_` - Owned mutation operations

**Private Methods:**
- `void ensureInitialized()` - Lazy initialization helper

**Key Relationships:**
- Used by: `ConstrainedMesher`, `ConstrainedDelaunay3D`
- References: `GeometryCollection3D`, `Topology3D`
- Owns: `MeshData`, `MeshConnectivity`, `MeshMutator3D`

---

### 2. MeshingContext2D

**Files:** `MeshingContext2D.h`, `MeshingContext2D.cpp`

**Purpose:** Central orchestrator for 2D meshing in parametric space. Can be used standalone or created from a 3D surface for surface triangulation.

**Class Declaration:**
```cpp
class MeshingContext2D
```

**Public Interface:**

*Static Factory Method:*
- `static MeshingContext2D fromSurface(const ISurface3D& surface, const Surface3D& surfaceTopology, const Topology3D& topology, const GeometryCollection3D& geometry)`

*Constructor & Destructor:*
- `MeshingContext2D(std::unique_ptr<GeometryCollection2D> geometry, std::unique_ptr<Topology2D> topology)`
- `~MeshingContext2D()`

*Copy/Move Semantics:*
- Copy constructor/assignment: Deleted
- Move constructor/assignment: Allowed

*Access Methods:*
- `const GeometryCollection2D& getGeometry() const`
- `const Topology2D& getTopology() const`
- `MeshData2D& getMeshData()`
- `MeshMutator2D& getOperations()`

*Mesh Management:*
- `void clearMesh()` - Clear mesh data while preserving geometry and topology

**Private Members:**
- `std::unique_ptr<GeometryCollection2D> geometry_` - Owned geometry
- `std::unique_ptr<Topology2D> topology_` - Owned topology
- `std::unique_ptr<MeshData2D> meshData_` - Owned mesh data
- `std::unique_ptr<MeshMutator2D> operations_` - Owned operations

**Private Methods:**
- `void ensureInitialized()` - Lazy initialization helper

**Key Relationships:**
- Used by: `ConstrainedDelaunay2D`, `ConstrainedDelaunay3D` (for surface triangulation)
- Owns: `GeometryCollection2D`, `Topology2D`, `MeshData2D`, `MeshMutator2D`

---

### 3. IMesher

**File:** `IMesher.h`

**Purpose:** Abstract interface for mesh generation strategies. Defines the contract for all meshing algorithms.

**Class Declaration:**
```cpp
class IMesher
```

**Public Interface:**

*Constructor:*
- `IMesher(const IQualityController* qualityController = nullptr)`

*Pure Virtual Methods:*
- `virtual void generate(MeshingContext3D& context) = 0` - Generate or regenerate mesh
- `virtual std::string getName() const = 0` - Return mesher name/identifier

**Protected Members:**
- `const IQualityController* qualityController_` - Optional quality controller

**Key Relationships:**
- Implemented by: `ConstrainedMesher`, `SimpleMesher`
- Uses: `IQualityController` (optional)

---

### 4. IQualityController

**File:** `IQualityController.h`

**Purpose:** Abstract interface for mesh quality evaluation. Allows pluggable quality assessment strategies.

**Class Declaration:**
```cpp
class IQualityController
```

**Public Interface:**

*Pure Virtual Methods:*
- `virtual bool isMeshAcceptable(const MeshData& mesh, const MeshConnectivity& connectivity) const = 0` - Evaluate overall mesh quality
- `virtual double getTargetElementQuality() const = 0` - Return target quality metric
- `virtual std::size_t getElementLimit() const = 0` - Return maximum element count to prevent runaway

**Key Relationships:**
- Used by: `IMesher` implementations
- Provides: Quality-driven meshing control

---

### 5. ConstrainedMesher

**Files:** `ConstrainedMesher.h`, `ConstrainedMesher.cpp`

**Purpose:** High-level mesher that generates 3D constrained Delaunay meshes respecting CAD edges and surfaces.

**Class Declaration:**
```cpp
class ConstrainedMesher : public IMesher
```

**Public Interface:**

*Constructor & Destructor:*
- `ConstrainedMesher(const IQualityController* qualityController = nullptr, size_t samplesPerEdge = 10, size_t samplesPerSurface = 5)`
- `~ConstrainedMesher() override = default`

*Core Method:*
- `void generate(MeshingContext3D& context) override` - Generate constrained Delaunay mesh

*Configuration Methods:*
- `void setSamplesPerEdge(size_t samples)`
- `size_t getSamplesPerEdge() const`
- `void setSamplesPerSurface(size_t samples)`
- `size_t getSamplesPerSurface() const`
- `std::string getName() const override` - Returns "ConstrainedMesher"

**Private Members:**
- `size_t samplesPerEdge_` - Number of samples per CAD edge
- `size_t samplesPerSurface_` - Number of samples per CAD surface

**Key Relationships:**
- Inherits from: `IMesher`
- Uses: `ConstrainedDelaunay3D` internally

---

### 6. SimpleMesher

**Files:** `SimpleMesher.h`, `SimpleMesher.cpp`

**Purpose:** Basic placeholder mesher for testing or simple mesh seeding operations.

**Class Declaration:**
```cpp
class SimpleMesher : public IMesher
```

**Public Interface:**

*Constructor & Destructor:*
- `SimpleMesher(const IQualityController* qualityController = nullptr)`
- `~SimpleMesher() override = default`

*Methods:*
- `void generate(MeshingContext3D& context) override` - Generate simple mesh
- `std::string getName() const override` - Returns "SimpleMesher"

**Key Relationships:**
- Inherits from: `IMesher`

---

### 7. ConstrainedDelaunay3D

**Files:** `ConstrainedDelaunay3D.h`, `ConstrainedDelaunay3D.cpp`

**Purpose:** Implementation of 3D Constrained Delaunay Triangulation using classical CDT approach with Bowyer-Watson insertion and constraint recovery.

**Class Declaration:**
```cpp
class ConstrainedDelaunay3D
```

**Public Interface:**

*Constructor & Destructor:*
- `ConstrainedDelaunay3D(MeshingContext3D& context)`
- `~ConstrainedDelaunay3D() = default`

*Core Methods:*
- `void initialize(const std::vector<Point3D>& points)` - Initialize with point cloud
- `void insertVertex(const Point3D& point)` - Insert single vertex using Bowyer-Watson
- `void generateConstrained(size_t samplesPerEdge = 10, size_t samplesPerSurface = 5)` - Full constrained generation from topology
- `void forceSegment(size_t startNodeId, size_t endNodeId)` - Force constraint segment into mesh
- `void forceFacet(size_t n0, size_t n1, size_t n2)` - Force triangular facet into mesh

*Access Methods:*
- `const MeshData& getMeshData() const`
- `MeshData& getMeshData()`
- `const std::unordered_set<size_t>& getActiveTetrahedronIds() const`
- `bool isElementActive(size_t elementId) const`
- `Computer& getComputer()`
- `const Computer& getComputer() const`
- `const TetrahedralElement* getTetrahedralElement(size_t elementId) const`
- `bool isDelaunay() const` - Check if mesh satisfies Delaunay property

**Private Members:**
- `ConstraintSet constraints_` - Constraint segments and facets from topology
- `std::unordered_map<std::string, size_t> topologyToNodeId_` - Maps topology IDs to mesh nodes
- `std::unordered_set<std::pair<size_t, size_t>, PairHash> satisfiedSegments_` - Recovered segments
- `std::unordered_set<std::array<size_t, 3>, TriangleHash> satisfiedFacets_` - Recovered facets
- `Computer computer_` - Geometric computation helper
- `std::vector<size_t> superNodeIds_` - Super tetrahedron node IDs
- `std::unordered_set<size_t> activeTetrahedra_` - Currently active elements
- `MeshingContext3D& context_` - Reference to context
- `MeshData& meshData_` - Reference to mesh data
- `MeshMutator3D& operations_` - Reference to operations

**Private Methods:**
- `void extractConstraints(const Topology3D&, const GeometryCollection3D&, size_t samplesPerEdge, size_t samplesPerSurface)`
- `void insertCornerNodes(const Topology3D&, const GeometryCollection3D&)`
- `void insertEdgeNodes(const Topology3D&, const GeometryCollection3D&, size_t samplesPerEdge)`
- `void triangulateSurfaces(const Topology3D&, const GeometryCollection3D&, size_t samplesPerSurface)`
- `void recoverSegments()` - Recover all constraint segments
- `void recoverFacets()` - Recover all constraint facets
- `std::vector<std::array<size_t, 3>> triangulateSurface2D(const std::vector<size_t>& boundaryNodes, const std::vector<size_t>& interiorNodes, const ISurface3D* surface)`
- `std::vector<std::array<size_t, 3>> triangulateSurfaceWithContext(const ISurface3D*, const Surface3D&, size_t samplesPerEdge)`
- `void createSuperTetrahedron(const std::vector<Point3D>&)`
- `void removeSuperTetrahedron()`
- `std::vector<size_t> findConflictingTetrahedra(const Point3D&) const`
- `std::vector<std::array<size_t, 3>> findCavityBoundary(const std::vector<size_t>&) const`
- `void retriangulate(size_t vertexNodeId, const std::vector<std::array<size_t, 3>>&)`

**Key Relationships:**
- Uses: `MeshingContext3D`, `MeshingContext2D` (for surface triangulation)
- Uses: `Computer` for geometric calculations
- Uses: `ConstrainedDelaunayHelper` for utility operations
- Uses: `ConstraintStructures` for constraint storage

---

### 8. ConstrainedDelaunay2D

**Files:** `ConstrainedDelaunay2D.h`, `ConstrainedDelaunay2D.cpp`

**Purpose:** Implementation of 2D Constrained Delaunay Triangulation in parametric space using Bowyer-Watson incremental insertion.

**Class Declaration:**
```cpp
class ConstrainedDelaunay2D
```

**Public Interface:**

*Constructors (Dual Mode):*
- `ConstrainedDelaunay2D(MeshingContext2D& context)` - Context-based mode (enables generateConstrained)
- `ConstrainedDelaunay2D(const std::unordered_map<size_t, Point2D>& nodeCoords)` - Standalone mode with raw coordinates

*Destructor:*
- `~ConstrainedDelaunay2D()`

*Core Methods:*
- `void addConstraintEdge(size_t nodeId1, size_t nodeId2)` - Add constraint edge
- `std::vector<std::array<size_t, 3>> triangulate()` - Compute triangulation (returns node ID triplets)
- `void generateConstrained(size_t samplesPerEdge = 10)` - Generate from topology (requires context constructor)

*Access Methods:*
- `MeshData2D& getMeshData2D()`
- `const MeshData2D& getMeshData2D() const`
- `MeshData getMeshData() const` - Backward compatibility (3D version)

**Private Members:**
- `MeshingContext2D* context_` - Optional context pointer
- `MeshData2D* meshData2D_` - Reference to mesh data
- `MeshMutator2D* operations_` - Reference to operations
- `std::unique_ptr<MeshData2D> ownedMeshData_` - For standalone mode
- `std::unique_ptr<MeshOperations2D> meshOps_` - For standalone mode
- `std::unordered_map<std::string, size_t> topologyToNodeId_` - Topology ID to node mapping
- `std::unordered_map<size_t, Point2D> nodeCoords_` - 2D parametric coordinates
- `std::vector<std::pair<size_t, size_t>> constraintEdges_` - List of constraint edges
- `std::vector<TriangleElement> activeTriangles_` - Active triangles during triangulation
- `std::vector<size_t> superNodeIds_` - Super triangle node IDs

**Private Methods:**
- `void createSuperTriangle()` - Create super triangle containing all points
- `void insertVertex(size_t nodeId)` - Insert vertex into triangulation
- `void removeSuperTriangle()` - Remove super triangle and its elements
- `void forceEdge(size_t nodeId1, size_t nodeId2)` - Force constraint edge into mesh
- `bool edgeExists(size_t nodeId1, size_t nodeId2) const` - Check if edge exists
- `static std::pair<size_t, size_t> makeEdgeKey(size_t a, size_t b)` - Create ordered edge key
- `void insertCornerNodes()` - Insert topology corner nodes
- `void insertEdgeNodes(size_t samplesPerEdge)` - Sample and insert edge nodes
- `void buildTriangulation()` - Create initial Delaunay triangulation
- `void recoverConstraintEdges()` - Force all constraint edges into mesh
- `void storeResultsInMeshData()` - Store final results in mesh data

**Key Relationships:**
- Uses: `MeshingContext2D` (optional)
- Uses: `MeshOperations2D` for high-level operations
- Uses: `MeshData2D`, `MeshMutator2D`

---

### 9. ConstrainedDelaunayHelper

**File:** `ConstrainedDelaunayHelper.h`

**Purpose:** Static helper functions for 3D constrained Delaunay operations including segment/facet checking and cavity retriangulation.

**Class Declaration:**
```cpp
class ConstrainedDelaunay3DHelper
```

**Public Static Methods:**

*Key Generation:*
- `static std::pair<size_t, size_t> makeSegmentKey(size_t a, size_t b)` - Create ordered segment key
- `static std::array<size_t, 3> makeTriangleKey(size_t a, size_t b, size_t c)` - Create sorted triangle key

*Existence Checking:*
- `static bool segmentExists(const ConstrainedDelaunay3D& mesh, size_t nodeId1, size_t nodeId2, const std::unordered_set<size_t>& activeTetrahedra, const std::unordered_set<std::pair<size_t, size_t>, PairHash>& satisfiedSegments)`
- `static bool facetExists(const ConstrainedDelaunay3D& mesh, size_t n0, size_t n1, size_t n2, const std::unordered_set<size_t>& activeTetrahedra, const std::unordered_set<std::array<size_t, 3>, TriangleHash>& satisfiedFacets)`

*Intersection Detection:*
- `static std::vector<size_t> findIntersectingTetrahedra(const ConstrainedDelaunay3D& mesh, const MeshData& meshData, size_t nodeId1, size_t nodeId2, const std::unordered_set<size_t>& activeTetrahedra)`
- `static std::vector<size_t> findIntersectingTetrahedraForFacet(const ConstrainedDelaunay3D& mesh, const MeshData& meshData, size_t n0, size_t n1, size_t n2, const std::unordered_set<size_t>& activeTetrahedra)`

*Cavity Retriangulation:*
- `static void retriangulateCavityWithSegment(MeshMutator3D& operations, std::unordered_set<size_t>& activeTetrahedra, size_t n1, size_t n2, const std::vector<std::array<size_t, 3>>& cavityBoundary)`
- `static void retriangulateCavityWithFacet(MeshMutator3D& operations, std::unordered_set<size_t>& activeTetrahedra, size_t n0, size_t n1, size_t n2, const std::vector<std::array<size_t, 3>>& cavityBoundary)`

**Private Static Methods:**
- `static bool segmentIntersectsTet(const MeshData& meshData, const Point3D& p1, const Point3D& p2, const TetrahedralElement& tet)` - Segment-tetrahedron intersection test
- `static bool triangleIntersectsTet(const MeshData& meshData, const Point3D& v0, const Point3D& v1, const Point3D& v2, const TetrahedralElement& tet)` - Triangle-tetrahedron intersection test

**Key Relationships:**
- Used by: `ConstrainedDelaunay3D`
- Provides: Utility operations for constraint recovery

---

### 10. MeshOperations2D

**Files:** `MeshOperations2D.h`, `MeshOperations2D.cpp`

**Purpose:** High-level 2D mesh operations built on top of MeshMutator2D, implementing Bowyer-Watson algorithm, cavity finding, and triangle intersection detection.

**Class Declaration:**
```cpp
class MeshOperations2D
```

**Inner Struct:**
```cpp
struct CircumCircle {
    Point2D center;
    double radiusSquared;
};
```

**Public Interface:**

*Constructor:*
- `MeshOperations2D(MeshData2D& meshData)`

*Core Methods:*
- `void insertVertexBowyerWatson(size_t nodeId, const std::unordered_map<size_t, Point2D>& nodeCoords, std::vector<TriangleElement>& activeTriangles) const` - Bowyer-Watson vertex insertion
- `std::vector<size_t> findConflictingTriangles(const Point2D& point, const std::unordered_map<size_t, Point2D>& nodeCoords, const std::vector<TriangleElement>& triangles) const` - Find triangles whose circumcircle contains point
- `std::vector<std::array<size_t, 2>> findCavityBoundary(const std::vector<size_t>& conflictingIndices, const std::vector<TriangleElement>& triangles) const` - Find boundary edges of cavity
- `std::vector<size_t> findIntersectingTriangles(size_t nodeId1, size_t nodeId2, const std::unordered_map<size_t, Point2D>& nodeCoords, const std::vector<TriangleElement>& triangles) const` - Find triangles intersecting an edge

*Access Methods:*
- `MeshMutator2D& getMutator()`
- `const MeshMutator2D& getMutator() const`

**Private Members:**
- `MeshData2D& meshData_` - Reference to mesh data
- `std::unique_ptr<MeshMutator2D> mutator_` - Owned mutator

**Private Methods:**
- `void retriangulate(size_t vertexNodeId, const std::vector<std::array<size_t, 2>>& cavityBoundary, std::vector<TriangleElement>& activeTriangles) const`
- `bool segmentsIntersect(const Point2D& a1, const Point2D& a2, const Point2D& b1, const Point2D& b2) const` - 2D segment intersection test
- `static std::pair<size_t, size_t> makeEdgeKey(size_t a, size_t b)` - Create ordered edge key

**Key Relationships:**
- Uses: `MeshMutator2D` for primitive operations
- Used by: `ConstrainedDelaunay2D`

---

### 11. Computer

**Files:** `Computer.h`, `Computer.cpp`

**Purpose:** Geometric computation helper that owns a reference to mesh data and exposes computations requiring node coordinates. Wraps ElementGeometry functions with mesh-aware convenience methods.

**Class Declaration:**
```cpp
class Computer
```

**Inner Struct:**
```cpp
struct CircumCircle2D {
    Point2D center;
    double radiusSquared;
};
```

**Public Interface:**

*Constructor:*
- `Computer(const MeshData& mesh)` - Holds reference to mesh data

*Static Methods (Point-based):*
- `static double computeVolume(Point3DRef v0, Point3DRef v1, Point3DRef v2, Point3DRef v3)` - Compute tetrahedral volume
- `static double computeArea(Point3DRef v0, Point3DRef v1, Point3DRef v2)` - Compute triangle area
- `static bool getIsPointInsideCircumscribingSphere(const CircumscribedSphere& sphere, Point3DRef point, double tolerance = 1e-10)` - Test point-sphere containment
- `static std::optional<CircumCircle2D> computeCircumcircle(const TriangleElement& tri, const std::unordered_map<size_t, Point2D>& coords)` - 2D circumcircle
- `static bool isPointInsideCircumcircle(const CircumCircle2D& circle, const Point2D& point)` - 2D point-circle test

*Instance Methods (Element-based):*
- `double computeVolume(const TetrahedralElement& tet) const`
- `std::optional<CircumscribedSphere> computeCircumscribingSphere(const TetrahedralElement& tet) const`
- `bool getIsPointInsideCircumscribingSphere(const TetrahedralElement& tet, const Point3D& point) const`
- `double computeArea(const TriangleElement& tri) const`
- `double computeQuality(const TetrahedralElement& tet) const`
- `double computeQuality(const TriangleElement& tri) const`
- `double getShortestEdgeLength(const TetrahedralElement& tet) const`
- `double getCircumradiusToShortestEdgeRatio(const TetrahedralElement& tet) const`
- `bool isSkinny(const TetrahedralElement& tet, double threshold) const`

**Key Relationships:**
- Wraps: `ElementGeometry` namespace functions
- Used by: `ConstrainedDelaunay3D`, `MeshOperations2D`

---

### 12. ElementGeometry

**Files:** `ElementGeometry.h`, `ElementGeometry.cpp`

**Purpose:** Namespace of static utility functions for computing geometric properties of mesh elements (circumspheres, quality metrics).

**Namespace Declaration:**
```cpp
namespace Meshing::ElementGeometry
```

**Inner Struct:**
```cpp
struct CircumscribedSphere {
    Point3D center;
    double radius;
};
```

**Public Functions:**

*Circumsphere:*
- `std::optional<CircumscribedSphere> computeCircumscribingSphere(Point3DRef v0, Point3DRef v1, Point3DRef v2, Point3DRef v3)` - From points
- `std::optional<CircumscribedSphere> computeCircumscribingSphere(const MeshData& mesh, const TetrahedralElement& tet)` - From element

*Quality Metrics:*
- `std::optional<double> computeQuality(const MeshData& mesh, const TetrahedralElement& tet)` - Tetrahedral quality
- `std::optional<double> computeQuality(const MeshData& mesh, const TriangleElement& tri)` - Triangle quality

**Key Relationships:**
- Wrapped by: `Computer`
- Provides: Pure geometric computation functions

---

### 13. ConstraintStructures

**File:** `ConstraintStructures.h`

**Purpose:** Data structures defining constraints from CAD geometry including segments (edges) and facets (surfaces).

**Hash Functions:**

```cpp
struct PairHash {
    std::size_t operator()(const std::pair<size_t, size_t>& p) const;
};

struct TriangleHash {
    std::size_t operator()(const std::array<size_t, 3>& arr) const;
};
```

**Structs:**

**ConstrainedSegment** - CAD edge constraint
```cpp
struct ConstrainedSegment {
    size_t startNodeId;
    size_t endNodeId;
    std::string topologyEdgeId;

    ConstrainedSegment(size_t start, size_t end, const std::string& edgeId);
};
```

**ConstrainedSubfacet** - Triangular piece of a CAD surface
```cpp
struct ConstrainedSubfacet {
    std::array<size_t, 3> nodeIds;
    std::string topologySurfaceId;

    ConstrainedSubfacet(size_t n0, size_t n1, size_t n2, const std::string& surfaceId);
    ConstrainedSubfacet(const std::array<size_t, 3>& nodes, const std::string& surfaceId);
};
```

**ConstrainedFacet** - Complete CAD surface with its triangulation
```cpp
struct ConstrainedFacet {
    std::string topologySurfaceId;
    std::vector<size_t> boundaryNodeIds;  // Ordered boundary loop
    std::vector<ConstrainedSubfacet> subfacets;  // Triangulation

    void addSubfacet(size_t n0, size_t n1, size_t n2);
    size_t getSubfacetCount() const;
};
```

**ConstraintSet** - Collection of all constraints
```cpp
struct ConstraintSet {
    std::vector<ConstrainedSegment> segments;
    std::vector<ConstrainedFacet> facets;

    void clear();
    size_t getSegmentCount() const;
    size_t getFacetCount() const;
    size_t getTotalSubfacetCount() const;
};
```

**Key Relationships:**
- Used by: `ConstrainedDelaunay3D`
- Provides: Constraint storage and organization

---

## Architecture Overview

### Design Patterns

#### 1. Strategy Pattern
- **Interface:** `IMesher`
- **Implementations:** `ConstrainedMesher`, `SimpleMesher`
- **Purpose:** Allows pluggable meshing algorithms

#### 2. Context Pattern
- **Classes:** `MeshingContext3D`, `MeshingContext2D`
- **Purpose:** Centralize access to geometry, topology, and mutable mesh data
- **Benefits:** Single point of access, lazy initialization, clear ownership

#### 3. Helper/Utility Pattern
- **Classes:** `ConstrainedDelaunayHelper`, `ElementGeometry` (namespace)
- **Purpose:** Provide static utility functions
- **Benefits:** Separation of concerns, reusability

#### 4. Dual-Mode Design
- **Class:** `ConstrainedDelaunay2D`
- **Modes:**
  - Context-based: Full integration with topology and geometry
  - Standalone: Works with raw coordinate maps
- **Purpose:** Flexibility for different use cases

### Class Relationships

```
┌─────────────────────────────────────────────────────────────────┐
│                         User Code                                │
└─────────────────────────────┬───────────────────────────────────┘
                              │
                              ▼
                    ┌──────────────────┐
                    │     IMesher      │ (Abstract)
                    └────────┬─────────┘
                             │ implements
              ┌──────────────┴──────────────┐
              ▼                             ▼
    ┌───────────────────┐        ┌──────────────────┐
    │ ConstrainedMesher │        │   SimpleMesher   │
    └─────────┬─────────┘        └──────────────────┘
              │ uses
              ▼
    ┌──────────────────────┐
    │ ConstrainedDelaunay3D │
    └─────────┬─────────────┘
              │ uses
              ├──────────────────────────────┐
              ▼                              ▼
    ┌───────────────────┐         ┌───────────────────┐
    │ MeshingContext3D  │         │ MeshingContext2D  │
    └─────────┬─────────┘         └─────────┬─────────┘
              │ owns                         │ owns
              ├──────────┬────────┐         ├──────────────┐
              ▼          ▼        ▼         ▼              ▼
         MeshData  Connectivity  │    MeshData2D   MeshMutator2D
                                 │                       │
                        MeshMutator3D            ┌──────┴──────┐
                                                  │             │
                                                  ▼             ▼
                                         MeshOperations2D   ConstrainedDelaunay2D
                                                  │
                                                  ▼
                                            Computer
                                                  │
                                                  ▼
                                          ElementGeometry
```

### Dependency Flow

**3D Mesh Generation Flow:**
1. User creates `MeshingContext3D` with geometry and topology
2. User instantiates `ConstrainedMesher` (or other `IMesher`)
3. `ConstrainedMesher::generate()` called with context
4. Internally creates `ConstrainedDelaunay3D`
5. `ConstrainedDelaunay3D` extracts constraints from topology
6. For each surface, creates `MeshingContext2D` for 2D triangulation
7. `ConstrainedDelaunay2D` triangulates surface in parametric space
8. Results integrated back into 3D mesh as constraint facets
9. 3D Bowyer-Watson insertion of interior points
10. Constraint recovery forces segments and facets
11. Final mesh stored in `MeshingContext3D::meshData_`

**2D Mesh Generation Flow:**
1. User creates `MeshingContext2D` (standalone or from surface)
2. User instantiates `ConstrainedDelaunay2D` with context
3. `generateConstrained()` samples topology and builds constraint edges
4. Bowyer-Watson insertion via `MeshOperations2D`
5. Constraint edge recovery
6. Results stored in `MeshData2D`

---

## Potential Refactoring Opportunities

### 1. Computer vs ElementGeometry Duplication
**Issue:** `Computer` essentially wraps `ElementGeometry` functions with mesh-aware convenience.

**Options:**
- Merge into single class with static and instance methods
- Keep separation but clarify when to use each
- Consider if `Computer` adds enough value to justify duplication

**Impact:** Medium - affects geometric computation call sites

---

### 2. Naming Inconsistency: Operations vs Mutator
**Issue:** We have both `MeshOperations2D` and `MeshMutator2D/3D` with overlapping concerns.

**Analysis:**
- `MeshMutator2D`: Low-level primitive operations
- `MeshOperations2D`: High-level algorithmic operations (Bowyer-Watson, cavity finding)

**Options:**
- Rename to clarify hierarchy (e.g., `MeshMutator2D` → `MeshPrimitives2D`)
- Keep current naming but document distinction clearly
- Consider merging if overlap is too great

**Impact:** Low to Medium - mostly naming/documentation

---

### 3. Hybrid 2D/3D Surface Triangulation
**Issue:** `ConstrainedDelaunay3D` creates temporary `MeshingContext2D` objects for surface triangulation.

**Analysis:**
- Elegant separation of 2D and 3D concerns
- Additional overhead from context creation
- Two code paths for surface triangulation (`triangulateSurface2D` and `triangulateSurfaceWithContext`)

**Options:**
- Keep current approach (clean separation)
- Optimize context creation/pooling
- Unify surface triangulation paths

**Impact:** Low - works well, optimization potential

---

### 4. Helper Class Organization
**Issue:** `ConstrainedDelaunayHelper` is a separate header with only static methods.

**Options:**
- Move helpers into `ConstrainedDelaunay3D` as private static methods
- Keep separate but namespace it
- Create a more general `DelaunayUtilities` namespace

**Impact:** Low - organizational clarity

---

### 5. Dual-Mode ConstrainedDelaunay2D
**Issue:** `ConstrainedDelaunay2D` supports two construction modes (context-based and standalone) leading to conditional logic.

**Analysis:**
- Flexibility is valuable
- Internal complexity with multiple ownership models
- Could be split into two classes

**Options:**
- Keep current design (flexible and working)
- Split into `ConstrainedDelaunay2DContext` and `ConstrainedDelaunay2DStandalone`
- Make standalone mode a factory method on context version

**Impact:** Medium - affects API design

---

### 6. Constraint Structure Organization
**Issue:** All constraint structures in single header file.

**Analysis:**
- Currently clean and cohesive
- Hash functions specific to constraint keys

**Recommendation:** Keep as-is, well-organized

---

### 7. Quality Controller Integration
**Issue:** `IQualityController` is defined but integration is minimal (optional parameter).

**Analysis:**
- Interface ready for quality-driven refinement
- Not currently used by `ConstrainedMesher`

**Options:**
- Fully integrate quality-driven refinement loops
- Remove if not planned
- Document integration plan

**Impact:** Depends on future requirements

---

## Code Metrics

### Complexity Estimates

**High Complexity:**
- `ConstrainedDelaunay3D` (~15-20 methods, complex constraint recovery)
- `ConstrainedDelaunay2D` (~12-15 methods, dual-mode design)
- `MeshOperations2D` (algorithmic complexity)

**Medium Complexity:**
- `MeshingContext3D`, `MeshingContext2D` (orchestration)
- `Computer` (many similar methods)
- `ElementGeometry` (geometric algorithms)

**Low Complexity:**
- `IMesher`, `IQualityController` (interfaces)
- `ConstraintStructures` (data structures)
- `SimpleMesher` (placeholder)
- `ConstrainedDelaunayHelper` (utilities)

### Lines of Code Estimates (Header + Implementation)

| Component | Est. LOC | Complexity |
|-----------|----------|------------|
| ConstrainedDelaunay3D | ~800-1000 | High |
| ConstrainedDelaunay2D | ~600-800 | High |
| MeshOperations2D | ~400-500 | Medium |
| Computer | ~300-400 | Medium |
| ElementGeometry | ~300-400 | Medium |
| MeshingContext3D | ~150-200 | Low |
| MeshingContext2D | ~150-200 | Low |
| ConstrainedDelaunayHelper | ~200-300 | Medium |
| ConstraintStructures | ~100-150 | Low |
| ConstrainedMesher | ~80-120 | Low |
| SimpleMesher | ~40-60 | Low |
| IMesher | ~30-40 | Low |
| IQualityController | ~30-40 | Low |

**Total Estimate:** ~3,200-4,200 LOC

---

## Testing Recommendations

### Critical Test Coverage Needed

1. **ConstrainedDelaunay3D:**
   - Bowyer-Watson insertion correctness
   - Constraint recovery (segments and facets)
   - Delaunay property validation
   - Edge cases (degenerate geometry, coplanar points)

2. **ConstrainedDelaunay2D:**
   - Both construction modes
   - Constraint edge recovery
   - Super triangle removal
   - Edge intersection handling

3. **MeshOperations2D:**
   - Circumcircle calculations
   - Cavity boundary finding
   - Triangle intersection detection

4. **Computer / ElementGeometry:**
   - Volume and area calculations
   - Quality metric accuracy
   - Circumsphere calculations
   - Degenerate element handling

5. **Context Classes:**
   - Lazy initialization
   - Resource ownership
   - Connectivity rebuilding

---

## Migration/Refactoring Strategy

If refactoring is pursued, recommended order:

### Phase 1: Documentation and Tests (Low Risk)
1. Add comprehensive unit tests for existing functionality
2. Document expected behavior and edge cases
3. Establish quality metrics baseline

### Phase 2: Low-Impact Refactoring (Low Risk)
1. Rename classes/methods for consistency (Operations vs Mutator)
2. Organize helper functions (namespace or integrate)
3. Clean up dual-mode design if needed

### Phase 3: Structural Changes (Medium Risk)
1. Consider Computer/ElementGeometry consolidation
2. Evaluate 2D/3D integration optimization
3. Implement or remove quality controller

### Phase 4: API Evolution (Higher Risk)
1. Simplify public interfaces based on usage
2. Consider breaking changes if worthwhile
3. Update dependent code

---

## Conclusion

The `src/Meshing/Core` directory is a well-structured implementation of constrained Delaunay triangulation in both 2D and 3D. The code exhibits clear separation of concerns with context management, algorithmic implementations, and geometric utilities properly separated.

**Strengths:**
- Clear architectural patterns (Strategy, Context, Helper)
- Good separation between 2D and 3D
- Flexible dual-mode design for 2D triangulation
- Comprehensive geometric utilities

**Areas for Consideration:**
- Some naming inconsistencies
- Potential duplication between Computer and ElementGeometry
- Helper class organization could be clearer
- Quality controller integration incomplete

**Overall Assessment:** The codebase is functional and well-organized. Any refactoring should be incremental and test-driven, focusing first on low-risk improvements like naming consistency and documentation before tackling structural changes.
