# Constrained Delaunay Triangulation - Terminology and Definitions

## Overview

This glossary defines terms used in 3D constrained Delaunay triangulation. Terms are organized by domain to avoid confusion between CAD geometry, mesh topology, and constraint concepts.

---

## CAD Geometry Terms

### Corner
**Definition:** A 0D geometric entity representing a vertex in the CAD model.

**Example:** A cube has 8 corners.

**In Code:**
```cpp
string cornerId = "corner_0";
Point3D point = geometry.getCorner(cornerId).getPoint();
```

**Aliases:** Vertex, point

---

### Edge (CAD)
**Definition:** A 1D geometric entity representing a curve connecting two corners in the CAD model.

**Properties:**
- Has a start corner and end corner
- Represented parametrically: `Point3D = Edge.getPoint(t)` where `t ∈ [tMin, tMax]`
- Can be straight (line) or curved (arc, spline, NURBS)

**Example:** A cube has 12 edges.

**In Code:**
```cpp
string edgeId = "edge_0";
Edge* edge = geometry.getEdge(edgeId);
Point3D midpoint = edge->getPoint(0.5);  // Parametric midpoint
```

**Important:** This is **NOT** the same as a mesh edge!

**Aliases:** Curve, CAD edge

---

### Surface (CAD)
**Definition:** A 2D geometric entity representing a face of the CAD model.

**Properties:**
- Bounded by a loop of edges
- Represented parametrically: `Point3D = Surface.getPoint(u, v)` where `(u,v) ∈ [uMin,uMax] × [vMin,vMax]`
- Can be planar or curved (cylinder, sphere, NURBS patch)

**Example:** A cube has 6 surfaces (all planar).

**In Code:**
```cpp
string surfaceId = "surface_0";
Surface* surface = geometry.getSurface(surfaceId);
Point3D point = surface->getPoint(0.3, 0.7);  // Point at parametric (u,v)
```

**Important:** This is **NOT** the same as a mesh face!

**Aliases:** Face (CAD), patch

---

## Mesh Topology Terms

### Node
**Definition:** A 0D mesh entity representing a point in 3D space.

**Properties:**
- Has a unique ID
- Has coordinates (x, y, z)
- Multiple nodes can exist on the same CAD edge or surface

**Example:** A cube mesh might have 212 nodes (8 corners + edge samples + surface samples).

**In Code:**
```cpp
size_t nodeId = operations.addNode(Point3D(1.5, 2.3, 0.7));
Point3D coords = meshData.getNode(nodeId).getCoordinates();
```

**Aliases:** Vertex (mesh), point (mesh)

---

### Edge (Mesh)
**Definition:** A 1D mesh entity connecting two nodes.

**Properties:**
- Defined by two node IDs
- Straight line segment in 3D space (not curved)
- Part of one or more tetrahedra

**Example:** A tetrahedron has 6 edges.

**Visual:**
```
node_5 ●───────────● node_12
       mesh edge
```

**Important:** This is **NOT** the same as a CAD edge!

**In Code:**
```cpp
// A mesh edge is implicit - defined by two nodes
pair<size_t, size_t> meshEdge = {nodeId1, nodeId2};

// Check if tet has this edge
bool hasEdge = tet.hasEdge(nodeId1, nodeId2);
```

**Aliases:** Mesh segment, link

---

### Face (Mesh)
**Definition:** A 2D mesh entity - a triangle defined by three nodes.

**Properties:**
- Defined by three node IDs
- Planar (flat) triangle in 3D space
- Shared by exactly two tetrahedra (interior) or one tetrahedron (boundary)

**Example:** A tetrahedron has 4 triangular faces.

**Visual:**
```
       node_8
       ●
      /|\
     / | \
    /  |  \
   ●───●───●
node_2 node_5 node_11

Mesh face (triangle)
```

**Important:** This is **NOT** the same as a CAD surface!

**In Code:**
```cpp
// A mesh face is implicit - defined by three nodes
array<size_t, 3> meshFace = {node0, node1, node2};

// Get face from tetrahedron
Triangle face = tet.getFace(0);  // Face opposite to vertex 0
```

**Aliases:** Triangle, facet (ambiguous - see below)

---

### Tetrahedron
**Definition:** A 3D mesh entity - a volumetric element with 4 vertices, 6 edges, and 4 triangular faces.

**Properties:**
- Defined by four node IDs
- Simplest 3D element
- Has signed volume (can be inverted if nodes are in wrong order)

**Visual:**
```
       v3
       ●
      /|\
     / | \
    /  |  \
   /   |   \
  ●────●────●
 v0    v1   v2

Vertices: v0, v1, v2, v3
Edges: (v0,v1), (v0,v2), (v0,v3), (v1,v2), (v1,v3), (v2,v3)
Faces: (v1,v2,v3), (v0,v3,v2), (v0,v1,v3), (v0,v2,v1)
```

**In Code:**
```cpp
Tetrahedron tet(node0, node1, node2, node3);
size_t tetId = meshData.addElement(tet);
```

**Aliases:** Tet, element, cell

---

## Constraint Terms

### Segment (Constraint)
**Definition:** A constraint requiring that a specific mesh edge exist between two nodes.

**Purpose:** Ensures a CAD edge appears as a connected sequence of mesh edges.

**Properties:**
- Defined by two node IDs (start and end)
- References the source CAD edge ID
- Must be enforced during meshing

**Relationship:**
```
CAD Edge (curve) 
    → sampled at N points
    → creates N-1 constraint segments
    → each segment forces a mesh edge
```

**Example:**
```
CAD Edge: corner_0 to corner_1 (length 10.0)
Sampling: 11 points → 10 segments

node_0 ●─●─●─●─●─●─●─●─●─●─● node_10
       │ │ │ │ │ │ │ │ │ │
       └─────────────────────── 10 constraint segments
       each segment forces the mesh edge to exist
```

**In Code:**
```cpp
struct ConstrainedSegment {
    size_t startNodeId;     // e.g., node_5
    size_t endNodeId;       // e.g., node_6
    string topologyEdgeId;  // e.g., "edge_0" (source CAD edge)
};

// This forces mesh edge (node_5, node_6) to exist
```

**Key Point:** 
- A **segment** is a **constraint** that forces a **mesh edge** to exist
- A segment is NOT itself an edge - it's a requirement

**Aliases:** Constrained edge, edge constraint

---

### Subfacet (Constraint)
**Definition:** A constraint requiring that a specific mesh face (triangle) exist.

**Purpose:** Represents one triangle in the discretization of a CAD surface.

**Properties:**
- Defined by three node IDs
- References the source CAD surface ID
- Part of a larger facet constraint

**Relationship:**
```
CAD Surface (patch)
    → sampled at N×M points
    → triangulated into K triangles
    → each triangle is a subfacet constraint
    → each subfacet forces a mesh face
```

**Example:**
```
CAD Surface: rectangular face (10×10)
Sampling: 5×5 grid → 25 points
Triangulation: ~24 triangles

Each triangle is a subfacet:
  subfacet_0: (node_5, node_6, node_12)
  subfacet_1: (node_6, node_13, node_12)
  ...
```

**Visual:**
```
●─────●─────●─────●
│ \ 0 │ \ 2 │ \ 4 │  Each numbered region
●─────●─────●─────●  is a subfacet (triangle)
│ 1 \ │ 3 \ │ 5 \ │  
●─────●─────●─────●
│ \ 6 │ \ 8 │ \10 │
●─────●─────●─────●

Subfacet 0: top-left triangle
Subfacet 1: bottom-left triangle
etc.
```

**In Code:**
```cpp
struct ConstrainedSubfacet {
    array<size_t, 3> nodeIds;  // e.g., {node_5, node_6, node_12}
    string topologySurfaceId;  // e.g., "surface_0" (source CAD surface)
};

// This forces mesh face (node_5, node_6, node_12) to exist
```

**Key Point:**
- A **subfacet** is a **constraint** that forces a **mesh face (triangle)** to exist
- A subfacet is NOT itself a face - it's a requirement

**Aliases:** Triangular constraint, face constraint

---

### Facet (Constraint)
**Definition:** A collection of subfacets representing the complete triangulation of one CAD surface.

**Purpose:** Groups all triangles that discretize a single CAD surface.

**Properties:**
- References one CAD surface ID
- Contains boundary node IDs (the loop)
- Contains multiple subfacets (the interior triangulation)

**Relationship:**
```
1 CAD Surface → 1 Facet → Many Subfacets → Many Mesh Faces
```

**Example:**
```
Facet for "surface_0" (bottom face of cube):
  - Boundary: nodes {0, 1, 2, 3} forming a square loop
  - Contains 24 subfacets (triangles)
  - Each subfacet forces one mesh triangle to exist
```

**In Code:**
```cpp
struct ConstrainedFacet {
    string topologySurfaceId;           // e.g., "surface_0"
    vector<size_t> boundaryNodeIds;     // e.g., {0, 8, 16, ..., 1}
    vector<ConstrainedSubfacet> subfacets;  // e.g., 24 triangles
};
```

**Hierarchy:**
```
ConstraintSet
├── segments (vector)
│   └── ConstrainedSegment
│       ├── startNodeId
│       ├── endNodeId
│       └── topologyEdgeId
│
└── facets (vector)
    └── ConstrainedFacet
        ├── topologySurfaceId
        ├── boundaryNodeIds
        └── subfacets (vector)
            └── ConstrainedSubfacet
                ├── nodeIds[3]
                └── topologySurfaceId
```

**Key Point:**
- A **facet** is a **collection of subfacets**
- NOT to be confused with a mesh face (triangle)

**Terminology Warning:** 
- In some literature, "facet" means a mesh face (triangle)
- In this implementation, "facet" means a constraint for a CAD surface
- This can be confusing!

**Aliases:** Surface constraint, face constraint group

---

## Geometric Algorithm Terms

### Delaunay Triangulation
**Definition:** A triangulation where no point lies inside the circumsphere of any tetrahedron.

**Property:** Maximizes the minimum angle, producing high-quality elements.

**Visual (2D analogy):**
```
Non-Delaunay:           Delaunay:
    ●                      ●
   /|\                    / \
  / | \                  /   \
 /  |  \                /     \
●───●───●              ●───────●
 \  |  /                \     /
  \ | /                  \   /
   \|/                    \ /
    ●                      ●

Circle passes through    Circle is empty
other points (bad)       (Delaunay property)
```

**Constraint:** In CDT, Delaunay property is violated near constraints.

---

### Constrained Delaunay Triangulation (CDT)
**Definition:** A Delaunay triangulation where specific edges and faces are forced to exist.

**Property:** Delaunay where possible, but respects constraints.

**Trade-off:**
- ✅ Boundaries respected exactly
- ❌ Delaunay property violated locally near constraints
- ❌ Element quality not guaranteed

---

### Bowyer-Watson Algorithm
**Definition:** An incremental algorithm for constructing Delaunay triangulation.

**Steps:**
1. Insert point p
2. Find all tets whose circumsphere contains p
3. Delete those tets (creates a cavity)
4. Retriangulate cavity with p

---

### Cavity
**Definition:** A connected region of space created by removing tetrahedra.

**Properties:**
- Has a boundary composed of triangular faces
- Interior is empty (no tets)
- Must be retriangulated

**Visual:**
```
Before removal:        After removal (cavity):      After retriangulation:
    ●                      ●                             ●
   /|\                    / \                           /|\
  / | \                  /   \   ← cavity              / | \
 /  |  \                /     \    boundary          /  p  \
●───●───●              ●───────●                    ●───●───●
 \  |  /                \     /                      \  |  /
  \ | /                  \   /                        \ | /
   \|/                    \ /                          \|/
    ●                      ●                             ●

Tets removed         Empty cavity            New tets from p
```

---

### Cavity Boundary
**Definition:** The set of triangular faces that bound a cavity.

**Property:** Each boundary face appears in exactly one removed tetrahedron.

**How to Find:**
```
1. For each removed tet, list its 4 faces
2. Count occurrences of each face
3. Faces appearing once are on the boundary
4. Faces appearing twice were interior (removed)
```

---

### Retriangulation
**Definition:** Creating new tetrahedra to fill a cavity.

**Unconstrained:** Connect a point to all boundary faces.

**Constrained:** Connect boundary faces to constraints (segments/facets).

---

### Circumsphere
**Definition:** The unique sphere passing through all 4 vertices of a tetrahedron.

**Property:** For Delaunay tets, no other points lie inside this sphere.

**How to Compute:**
```cpp
Sphere circumsphere = computer_.computeCircumsphere(
    tet.getVertex(0),
    tet.getVertex(1),
    tet.getVertex(2),
    tet.getVertex(3)
);

Point3D center = circumsphere.center;
double radius = circumsphere.radius;
```

---

### Barycentric Coordinates
**Definition:** A coordinate system for expressing a point's location relative to a simplex (triangle or tetrahedron).

**For tetrahedron with vertices v0, v1, v2, v3:**
```
Point p = λ0·v0 + λ1·v1 + λ2·v2 + λ3·v3

where λ0 + λ1 + λ2 + λ3 = 1
```

**Property:** Point p is inside tet if all λi ≥ 0.

**Use:** Testing if a point is inside a tetrahedron.

---

## Sampling Terms

### Parametric Sampling
**Definition:** Sampling points on a geometric entity using its parametric representation.

**For CAD Edge:**
```cpp
// t varies from tMin to tMax
for (t = tMin; t <= tMax; t += dt) {
    Point3D point = edge->getPoint(t);
    // point is on the curve
}
```

**For CAD Surface:**
```cpp
// (u,v) varies in rectangular domain
for (u = uMin; u <= uMax; u += du) {
    for (v = vMin; v <= vMax; v += dv) {
        Point3D point = surface->getPoint(u, v);
        // point is on the surface
    }
}
```

---

### Samples Per Edge
**Definition:** Number of points (including endpoints) to sample along a CAD edge.

**Example:** 
- `samplesPerEdge = 10` → 10 points → 9 constraint segments
- `samplesPerEdge = 2` → 2 points (endpoints only) → 1 constraint segment

**In Code:**
```cpp
mesher.setSamplesPerEdge(20);  // Finer edge discretization
```

---

### Samples Per Surface
**Definition:** Number of points to sample in each parametric direction (u and v) on a CAD surface.

**Example:**
- `samplesPerSurface = 5` → 5×5 = 25 points → ~24 subfacets (triangles)
- `samplesPerSurface = 10` → 10×10 = 100 points → ~98 subfacets

**In Code:**
```cpp
mesher.setSamplesPerSurface(10);  // Finer surface discretization
```

---

## Quality Terms

### Aspect Ratio
**Definition:** Ratio of longest edge to shortest edge (or largest to smallest dimension).

**Range:** [1, ∞) where 1 is perfect (equilateral).

**Example:**
- Equilateral tet: aspect ratio ≈ 1.0 (good)
- Sliver tet: aspect ratio > 100 (bad)

---

### Dihedral Angle
**Definition:** Angle between two faces of a tetrahedron.

**Range:** (0°, 180°)

**Quality:**
- Good: 60° - 120°
- Bad: < 10° or > 170° (creates slivers)

---

### Sliver
**Definition:** A tetrahedron with near-zero volume but non-degenerate edges.

**Appearance:** Nearly flat, like four points almost on a plane.

**Problem:** Poor numerical conditioning in FEA.

**Visual:**
```
Good tet:           Sliver tet:
    ●                  ●  ← all 4 points
   /|\                 ● ← almost on
  / | \                ● ← same
 /  |  \               ● ← plane
●───●───●
```

---

## Topology Terms

### Active Tetrahedra
**Definition:** Set of tetrahedra currently in the mesh (not deleted).

**In Code:**
```cpp
set<size_t> activeTetrahedra_;

// Add tet
activeTetrahedra_.insert(tetId);

// Remove tet
activeTetrahedra_.erase(tetId);

// Check if active
bool isActive = activeTetrahedra_.contains(tetId);
```

---

### Element Removal
**Definition:** Marking a tetrahedron as deleted without physically removing it from memory.

**Why:** Preserves element IDs, allows undo, faster than reallocation.

**In Code:**
```cpp
operations_.removeElement(tetId);  // Marks as inactive
activeTetrahedra_.erase(tetId);    // Removes from active set
```

---

## Summary Tables

### CAD vs Mesh vs Constraint

| Dimension | CAD Entity | Mesh Entity | Constraint Type |
|-----------|------------|-------------|-----------------|
| 0D | Corner | Node | (none) |
| 1D | Edge (curve) | Edge (line segment) | Segment |
| 2D | Surface (patch) | Face (triangle) | Subfacet / Facet |
| 3D | Solid (volume) | Tetrahedron | (none) |

### What Forces What

| Constraint | Forces | Purpose |
|------------|--------|---------|
| ConstrainedSegment | Mesh edge exists | CAD edge conformance |
| ConstrainedSubfacet | Mesh face exists | CAD surface conformance (one triangle) |
| ConstrainedFacet | Multiple mesh faces | CAD surface conformance (complete) |

### Terminology Conflicts

| Term | Meaning 1 (CAD) | Meaning 2 (Mesh) | Meaning 3 (Constraint) |
|------|-----------------|------------------|------------------------|
| Edge | CAD curve | Mesh line segment | (none) |
| Face | CAD surface | Mesh triangle | (none) |
| Facet | (none) | Mesh triangle | Surface constraint |
| Segment | (none) | Mesh edge | Edge constraint |

**Rule of thumb:**
- **CAD terms**: Refer to geometric entities (can be curved)
- **Mesh terms**: Refer to discrete elements (straight, flat)
- **Constraint terms**: Refer to requirements to be enforced

---

## Common Confusions - Clarified

### "What is a segment?"

**Answer:** A segment is a **constraint** that requires a specific mesh edge to exist.

- It is NOT a CAD edge (that's curved, continuous)
- It is NOT a mesh edge (that's the result of satisfying the constraint)
- It IS a requirement that two specific nodes must be connected

### "What is a facet?"

**Answer:** A facet is a **collection of subfacets** representing one CAD surface.

- It is NOT a CAD surface (that's parametric, possibly curved)
- It is NOT a mesh face/triangle (those are the result of satisfying subfacets)
- It IS a grouping of triangular constraints for one surface

### "What's the difference between an edge and a segment?"

| Property | CAD Edge | Mesh Edge | Constraint Segment |
|----------|----------|-----------|-------------------|
| Nature | Geometric curve | Discrete line | Requirement |
| Representation | Parametric | Two node IDs | Two node IDs + source edge ID |
| Can be curved? | Yes | No (always straight) | N/A (not geometric) |
| Count (cube) | 12 | ~1000 | ~120 |

### "What's the difference between a surface and a facet?"

| Property | CAD Surface | Mesh Face | Constraint Facet |
|----------|-------------|-----------|-----------------|
| Nature | Geometric patch | Discrete triangle | Requirement group |
| Representation | Parametric (u,v) | Three node IDs | Multiple subfacets |
| Can be curved? | Yes | No (always planar) | N/A (not geometric) |
| Count (cube) | 6 | ~2000 | 6 (containing ~144 subfacets) |

---

## Quick Reference

**I want to...**

- Sample a curve → Use **CAD Edge** with parametric `t`
- Sample a patch → Use **CAD Surface** with parametric `(u,v)`
- Require a mesh edge exists → Create **Constraint Segment**
- Require a mesh triangle exists → Create **Constraint Subfacet**
- Group constraints for a surface → Use **Constraint Facet**
- Test if point is in tet → Use **Barycentric Coordinates**
- Rebuild after deletion → **Retriangulate Cavity**

---

## Conclusion

The key to understanding constrained Delaunay is recognizing **three separate domains**:

1. **CAD Geometry**: Continuous, curved, parametric (edges, surfaces)
2. **Mesh Topology**: Discrete, straight, flat (nodes, edges, faces, tets)
3. **Constraints**: Requirements linking CAD to mesh (segments, subfacets, facets)

The algorithm **samples** CAD geometry to create mesh nodes, then **forces** constraints to ensure the mesh respects CAD boundaries.
