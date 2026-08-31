---
name: meshing-patterns
description: How to drive OpenLoom's meshers and mesh data - entry points for 2D triangulation, 3D surface and volume meshing, the context/mutator ownership rules, and export. Load before writing or changing any code that constructs a mesher, touches MeshData/MeshMutator, or adds an example under src/Examples/.
---

# Meshing Patterns for OpenLoom

## Entry points

There is one entry point per dimension/kind. RCDT is the only volume/surface
algorithm implemented today; it sits behind `ISurfaceMesher3D` /
`IVolumeMesher3D` and is reached through the wrappers below, not directly.

### 2D, unconstrained (raw points)

```cpp
MeshData2D meshData;
Delaunay2D triangulator(points, &meshData);
triangulator.triangulate();
```

### 2D, constrained (geometry + topology)

```cpp
MeshingContext2D context(std::move(geometry), std::move(topology));
ConstrainedDelaunay2D mesher(context, discretizationSettings);
mesher.triangulate();
const MeshData2D& mesh = context.getMeshData();
```

### 3D surface

```cpp
SurfaceMesher3D mesher(converter.getGeometryCollection(),
                       converter.getTopology(),
                       discretizationSettings,
                       SurfaceMesh3DQualitySettings{},
                       SurfaceMeshingStrategy::Auto);
```

### 3D volume

```cpp
Geometry3D::DiscretizationSettings3D discretizationSettings(3, 2);

Meshing::VolumeMesher3D mesher(converter.getGeometryCollection(),
                               converter.getTopology(),
                               discretizationSettings,
                               Meshing::SurfaceMesh3DQualitySettings{});

auto volumeMesh = mesher.mesh();   // VolumeMesh3D by value
```

`VolumeMesh3D` exposes `.nodes`, `.tetrahedra`, `.boundaryTriangles`.

## Context Pattern

Contexts own geometry + topology + mesh lifetime. Access mesh data through the
context; never cache raw pointers into it across operations that may
reallocate.

```cpp
const MeshData2D& mesh = context.getMeshData();
```

## Friend Classes (controlled mutation)

- `MeshData2D` <-> `MeshMutator2D`
- `MeshData3D` <-> `MeshMutator3D`

All low-level insertion/deletion goes through the mutator. `MeshMutator3D`
hands out element ids from a monotonic `nextElementId_++`; ids are **not**
reused after deletion, and `restoreElement()` puts back the identical element.
Node coordinates are fixed for the whole of refinement (`moveNode` is only
called during post-refinement smoothing). Caches in `RestrictedTriangulation`
depend on both facts.

## Connectivity rebuilding

Rebuild after bulk operations:

```cpp
context.rebuildConnectivity();
```

## Export

`VtkExporter::exportMesh` is the real method; `writeVtu` is a convenience alias
for the 2D and 3D mesh-data overloads. Volume meshes have their own entry.

```cpp
Export::VtkExporter exporter;
exporter.writeVtu(meshData, "output.vtu");            // MeshData2D / MeshData3D
exporter.writeVolumeMesh(volumeMesh, "box_mesh.vtu"); // VolumeMesh3D
```

View with `paraview output.vtu`.

## When investigating

Run examples and tests with `CHECK_MESH_EACH_ITERATION=1` to catch mesh
corruption early. `EXPORT_MESH_EACH_ITERATION=1` dumps per-iteration meshes.
