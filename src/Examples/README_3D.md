# 3D Mesh Refinement Examples

This directory contains examples demonstrating the implementation of 3D Delaunay refinement components based on Shewchuk's algorithm.

## Examples

### 1. Basic 3D Constraint Example (`basic_3d_constraint_example.cpp`)

Demonstrates the fundamental constraint checking operations:

- **Subsegment Encroachment Testing**: Uses diametral sphere to check if points encroach on edge constraints
- **Subfacet Encroachment Testing**: Uses equatorial sphere to check if non-coplanar points encroach on triangular face constraints
- **Bowyer-Watson Vertex Insertion**: Demonstrates the 3D cavity-based vertex insertion algorithm

**Key Concepts:**
- A subsegment is encroached if a point lies inside its diametral sphere
- A subfacet is encroached if a non-coplanar point lies inside its equatorial sphere (coplanar points don't encroach)
- Bowyer-Watson insertion maintains the Delaunay property by removing conflicting tetrahedra and retriangulating

**Run:**
```bash
cd build/examples
./basic_3d_constraint_example
```

### 2. 3D Mesh with Holes Example (`3d_mesh_with_holes_example.cpp`)

Demonstrates handling complex domains with interior voids:

- **Constrained Subsegments**: Defining edge constraints for domain boundaries
- **Constrained Subfacets**: Defining triangular face constraints for surfaces
- **Subsegment Splitting**: Splitting constrained edges at their midpoints
- **Encroachment in Complex Domains**: Testing constraints in domains with holes

**Key Concepts:**
- Outer boundary and inner cavity (hole) vertices
- Constraint enforcement for both segments and facets
- Edge splitting creates two new subsegments from one
- Interior voids must be handled by classifying tetrahedra as interior/exterior

**Run:**
```bash
cd build/examples
./3d_mesh_with_holes_example
```

## Implementation Status

### ✅ Completed Components

1. **GeometryStructures3D.h** - Data structures for constraints:
   - `ConstrainedSubsegment3D` - Edge constraints
   - `ConstrainedSubfacet3D` - Face constraints
   - `DiametralSphere` - For subsegment encroachment
   - `EquatorialSphere` - For subfacet encroachment

2. **GeometryUtilities3D** - Geometric computations:
   - `createDiametralSphere()` - Computes diametral sphere of a segment
   - `isPointInDiametralSphere()` - Tests subsegment encroachment
   - `createEquatorialSphere()` - Computes equatorial sphere of a triangle
   - `isPointInEquatorialSphere()` - Tests subfacet encroachment (excludes coplanar points)
   - `computeEdgeLength()` - Helper for distance calculations

3. **ConstraintChecker3D** - Encroachment testing:
   - `isSubsegmentEncroached()` - Checks if point encroaches edge
   - `isSubfacetEncroached()` - Checks if point encroaches face

4. **MeshOperations3D** - High-level mesh operations:
   - `insertVertexBowyerWatson()` - 3D Delaunay vertex insertion
   - `findConflictingTetrahedra()` - Finds tets whose circumsphere contains point
   - `findCavityBoundary()` - Extracts triangular faces of cavity
   - `splitConstrainedSubsegment()` - Splits edge at midpoint
   - `splitConstrainedSubfacet()` - Splits face at circumcenter
   - `classifyTetrahedraInteriorExterior()` - Placeholder for flood fill (TODO)

### 🚧 Next Steps for Full Shewchuk Refinement

To complete the full Shewchuk 3D refinement algorithm, implement:

1. **ShewchukRefiner3D** - Main refinement class:
   - `refine()` - Main refinement loop
   - `refineStep()` - Single refinement iteration with 3-priority system:
     - Priority 1: Encroached subsegments
     - Priority 2: Encroached subfacets
     - Priority 3: Skinny tetrahedra
   - `findEncroachedSubsegments()` - Find all encroached edges
   - `findEncroachedSubfacets()` - Find all encroached faces
   - `handlePoorQualityTetrahedron()` - Insert circumcenter of skinny tet

2. **IQualityController3D** - Quality interface:
   - `isMeshAcceptable()` - Check overall mesh quality
   - `isTetrahedronAcceptable()` - Check individual tet quality

3. **Shewchuk3DQualityController** - Concrete quality controller:
   - Implements circumradius-to-shortest-edge ratio bound (B > 2.0)
   - Configurable quality thresholds

## Algorithm Overview

The 3D Delaunay refinement follows Shewchuk's three-priority system:

### Priority 1: Encroached Subsegments (Edges)
- Find all edges with vertices in their diametral sphere
- Split at midpoint using parent edge geometry
- Higher priority ensures edges are recovered first

### Priority 2: Encroached Subfacets (Triangular Faces)
- Find all triangular faces with non-coplanar vertices in their equatorial sphere
- Insert vertex at circumcenter
- If circumcenter would encroach subsegment → split that subsegment instead
- Each facet maintains independent 2D Delaunay triangulation

### Priority 3: Skinny Tetrahedra
- Find tetrahedra with circumradius-to-shortest-edge ratio > B (typically 2.0)
- Insert vertex at circumcenter
- If circumcenter would encroach subsegment → split subsegment
- If circumcenter would encroach subfacet → split subfacet

## Key Differences from 2D

| **2D (ShewchukRefiner2D)** | **3D (ShewchukRefiner3D)** |
|----------------------------|----------------------------|
| 2 priorities | 3 priorities |
| ConstrainedSegment2D | ConstrainedSubsegment3D + ConstrainedSubfacet3D |
| Circumcircle | Circumsphere |
| Diametral circle | Diametral sphere |
| - | Equatorial sphere (new!) |
| Colinear check | Coplanar check |

## Building and Running

Build all examples:
```bash
cd build
cmake --build . --target basic_3d_constraint_example
cmake --build . --target 3d_mesh_with_holes_example
```

Or build everything:
```bash
cmake --build . -j8
```

## References

- Shewchuk, J.R. (1998). "Tetrahedral Mesh Generation by Delaunay Refinement"
- See [doc/delref3d.pdf](../doc/delref3d.pdf) for the full paper
